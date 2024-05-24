import numpy as np

import biorbd

from bioptim import (
    Axis,
    BiorbdModel,
    BoundsList,
    DynamicsFcn,
    DynamicsList,
    InitialGuessList,
    InterpolationType,
    ObjectiveFcn,
    ObjectiveList,
    OdeSolver,
    OptimalControlProgram,
    PhaseDynamics,
    SolutionMerge,
    Solver
)

from ..dynamics.inverse_kinematics_and_dynamics import get_circle_coord, inverse_kinematics_cycling


def get_initial_guess(biorbd_model_path: str, final_time: int, n_stim: int, n_shooting: int, objective: dict) -> dict:
    """
    Get the initial guess for the ocp

    Parameters
    ----------
    biorbd_model_path: str
        The path to the model
    final_time: int
        The ocp final time
    n_stim: int
        The number of stimulation
    n_shooting: list
        The shooting points number
    objective: dict
        The ocp objective

    Returns
    -------
    dict
        The initial guess for the ocp

    """
    # Checking if the objective is a cycling objective
    if objective["cycling"] is None:
        raise ValueError("Only a cycling objective is implemented for the warm start")

    # Getting q and qdot from the inverse kinematics
    ocp, q, qdot = prepare_muscle_driven_ocp(biorbd_model_path, n_shooting[0] * n_stim, final_time, objective)

    # Solving the ocp to get muscle controls
    sol = ocp.solve(Solver.IPOPT(_tol=1e-4))
    muscles_control = sol.decision_controls(to_merge=[SolutionMerge.PHASES, SolutionMerge.NODES])
    model = biorbd.Model(biorbd_model_path)

    # Reorganizing the q and qdot shape for the warm start
    q_init = [q[:, n_shooting[0] * i:n_shooting[0] * (i + 1) + 1] for i in range(n_stim)]
    qdot_init = [qdot[:, n_shooting[0] * i:n_shooting[0] * (i + 1) + 1] for i in range(n_stim)]

    # Building the initial guess dictionary
    states_init = {"q": q_init, "qdot": qdot_init}

    # Creating initial muscle forces guess from the muscle controls and the muscles max iso force characteristics
    for i in range(muscles_control['muscles'].shape[0]):
        fmax = model.muscle(i).characteristics().forceIsoMax()  # Get the max iso force of the muscle
        states_init[model.muscle(i).name().to_string()] = [
            np.array([muscles_control["muscles"][i][n_shooting[0] * j:n_shooting[0] * (j + 1) + 1]]) * fmax for j in
            range(n_stim)]  # Multiply the muscle control by the max iso force to get the muscle force for each shooting point
        states_init[model.muscle(i).name().to_string()][-1] = np.array([np.append(
            states_init[model.muscle(i).name().to_string()][-1], states_init[model.muscle(i).name().to_string()][-1][0][
                -1])])  # Adding a last value to the end for each interpolation frames

    return states_init


def prepare_muscle_driven_ocp(
        biorbd_model_path: str,
        n_shooting: int,
        final_time: int,
        objective: dict,
) -> tuple:
    """
    Prepare the muscle driven ocp with a cycling objective

    Parameters
    ----------
    biorbd_model_path: str
        The path to the model
    n_shooting: int
        The number of shooting points
    final_time: int
        The ocp final time
    objective: dict
        The ocp objective

    Returns
    -------
    OptimalControlProgram
        The muscle driven ocp
    np.array
        The joints angles
    np.array
        The joints velocities
    """

    # Adding the models to the same phase
    bio_model = BiorbdModel(biorbd_model_path, )

    # Add objective functions
    x_center = objective["cycling"]["x_center"]
    y_center = objective["cycling"]["y_center"]
    radius = objective["cycling"]["radius"]
    get_circle_coord_list = np.array([get_circle_coord(theta, x_center, y_center, radius)[:-1] for theta in
                                      np.linspace(0, -2 * np.pi, n_shooting)])
    objective_functions = ObjectiveList()
    for i in range(n_shooting):
        objective_functions.add(
            ObjectiveFcn.Mayer.TRACK_MARKERS,
            weight=100,
            axes=[Axis.X, Axis.Y],
            marker_index=0,
            target=np.array(get_circle_coord_list[i]),
            node=i,
            phase=0,
            quadratic=True,
        )

    # Dynamics
    dynamics = DynamicsList()
    dynamics.add(DynamicsFcn.MUSCLE_DRIVEN, expand_dynamics=True,
                 phase_dynamics=PhaseDynamics.SHARED_DURING_THE_PHASE)

    # Path constraint
    x_bounds = BoundsList()
    q_x_bounds = bio_model.bounds_from_ranges("q")
    qdot_x_bounds = bio_model.bounds_from_ranges("qdot")
    x_bounds.add(key="q", bounds=q_x_bounds, phase=0)
    x_bounds.add(key="qdot", bounds=qdot_x_bounds, phase=0)

    # Define control path constraint
    u_bounds = BoundsList()
    u_bounds["muscles"] = [0] * bio_model.nb_muscles, [1] * bio_model.nb_muscles

    # Initial q guess
    x_init = InitialGuessList()
    u_init = InitialGuessList()

    q_guess, qdot_guess, qddotguess = inverse_kinematics_cycling(biorbd_model_path, n_shooting, x_center,
                                                                 y_center, radius, ik_method="trf")
    x_init.add("q", q_guess, interpolation=InterpolationType.EACH_FRAME)
    x_init.add("qdot", qdot_guess, interpolation=InterpolationType.EACH_FRAME)

    return OptimalControlProgram(
        bio_model,
        dynamics,
        n_shooting,
        final_time,
        x_bounds=x_bounds,
        u_bounds=u_bounds,
        x_init=x_init,
        u_init=u_init,
        objective_functions=objective_functions,
        ode_solver=OdeSolver.RK4(),
    ), q_guess, qdot_guess
