"""
This example will do a 10 stimulation example with Ding's 2007 frequency model.
This ocp was build to produce a elbow motion from 5 to 120 degrees.
The stimulation frequency will be optimized between 10 and 100 Hz and pulse width between minimal sensitivity
threshold and 600us to satisfy the flexion and minimizing required elbow torque control.
"""

from bioptim import Solver
from cocofest import DingModelPulseWidthFrequency, OcpFesMsk, FesMskModel

muscles_model = DingModelPulseWidthFrequency(muscle_name="BIClong")
model = FesMskModel(
    name=None,
    biorbd_path="../msk_models/arm26_biceps_1dof.bioMod",  # Modèle de Léa
    muscles_model=[muscles_model],  # Nom du muscle qui est dans le modèle
    activate_force_length_relationship=True,
    activate_force_velocity_relationship=True,
    activate_residual_torque=True,  # Tous les DoFs
)
# TODO: add root mapping ou retirer les tau bounds

custom_objectives = ObjectiveList()
custom_constraints = ConstraintsList()
final_time = 1  # À voir
minimum_pulse_width = muscles_model.pd0
ocp = OcpFesMsk.prepare_ocp(
    model=model,
    stim_time=np.linspace(0, final_time, 33)[:-1],  # Généralement 33 Hz
    final_time=final_time,
    pulse_width={
        "min": minimum_pulse_width,
        "max": 0.0006,  # Max du stimulateur
        "bimapping": False,
    },
    objective={"minimize_residual_torque": True, "custom": custom_objectives},
    msk_info={
        "with_residual_torque": True,  # Mandatory
        "custom_constraint": custom_constraints
    },
)

if __name__ == "__main__":
    sol = ocp.solve(Solver.IPOPT(show_online_optim=False, _max_iter=2000))
    sol.animate()
    sol.graphs(show_bounds=False)
