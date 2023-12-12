"""
This example demonstrates the way of identifying the Ding 2003 model parameter using noisy simulated data.
First we integrate the model with a given parameter set. Then we add noise to the previously calculated force output.
Finally, we use the noisy data to identify the model parameters. It is possible to lock a_rest to an arbitrary value but
you need to remove it from the key_parameter_to_identify.
"""

import pickle
import os
import matplotlib.pyplot as plt
import numpy as np

from bioptim import Solution, Shooting, SolutionIntegrator

from cocofest import (
    DingModelFrequency,
    DingModelFrequencyForceParameterIdentification,
    IvpFes,
)


# --- Setting simulation parameters --- #
n_stim = 10
n_shooting = 10
final_time = 1
extra_phase_time = 1


# --- Creating the simulated data to identify on --- #
# Building the Initial Value Problem
ivp = IvpFes(
    model=DingModelFrequency(),
    n_stim=n_stim,
    n_shooting=n_shooting,
    final_time=final_time,
    use_sx=True,
    extend_last_phase=extra_phase_time,
)

# Creating the solution from the initial guess
sol_from_initial_guess = Solution.from_initial_guess(ivp, [ivp.x_init, ivp.u_init, ivp.p_init, ivp.s_init])

# Integrating the solution
result = sol_from_initial_guess.integrate(
    shooting_type=Shooting.SINGLE, integrator=SolutionIntegrator.OCP, merge_phases=True
)

# Adding noise to the force
noise = np.random.normal(0, 5, len(result.states["F"][0]))
force1 = result.states["F"] + noise
force = force1.tolist()
time = [result.time.tolist()]
stim_temp = [0 if i == 0 else result.ocp.nlp[i].tf for i in range(len(result.ocp.nlp))]
stim = [sum(stim_temp[: i + 1]) for i in range(len(stim_temp))]

# Saving the data in a pickle file
dictionary = {
    "time": time,
    "force": force,
    "stim_time": stim,
}

pickle_file_name = "../data/temp_identification_simulation.pkl"
with open(pickle_file_name, "wb") as file:
    pickle.dump(dictionary, file)


# --- Identifying the model parameters --- #
ocp = DingModelFrequencyForceParameterIdentification(
    model=DingModelFrequency(),
    data_path=[pickle_file_name],
    identification_method="full",
    identification_with_average_method_initial_guess=False,
    key_parameter_to_identify=["a_rest", "km_rest", "tau1_rest", "tau2"],
    additional_key_settings={},
    n_shooting=n_shooting,
    # a_rest=2500,
    use_sx=True,
)

identified_parameters = ocp.force_model_identification()
print(identified_parameters)

# --- Plotting noisy simulated data and simulation from model with the identified parameter --- #
identified_model = ocp.model

identified_force_list = []
identified_time_list = []

ivp_from_identification = IvpFes(
    model=identified_model,
    n_stim=n_stim,
    n_shooting=n_shooting,
    final_time=final_time,
    use_sx=True,
    extend_last_phase=extra_phase_time,
)

# Creating the solution from the initial guess
identified_sol_from_initial_guess = Solution.from_initial_guess(
    ivp_from_identification,
    [
        ivp_from_identification.x_init,
        ivp_from_identification.u_init,
        ivp_from_identification.p_init,
        ivp_from_identification.s_init,
    ],
)

# Integrating the solution
identified_result = identified_sol_from_initial_guess.integrate(
    shooting_type=Shooting.SINGLE, integrator=SolutionIntegrator.OCP, merge_phases=True
)

identified_time = identified_result.time.tolist()
identified_force = identified_result.states["F"][0]

(
    pickle_time_data,
    pickle_stim_apparition_time,
    pickle_muscle_data,
    pickle_discontinuity_phase_list,
) = DingModelFrequencyForceParameterIdentification.full_data_extraction([pickle_file_name])

# Plotting the identification result
plt.title("Force state result")
plt.plot(pickle_time_data, pickle_muscle_data, color="blue", label="simulated")
plt.plot(identified_time, identified_force, color="red", label="identified")
plt.xlabel("time (s)")
plt.ylabel("force (N)")

plt.annotate("a_rest : ", xy=(0.7, 0.85), xycoords="axes fraction", color="black")
plt.annotate("km_rest : ", xy=(0.7, 0.80), xycoords="axes fraction", color="black")
plt.annotate("tau1_rest : ", xy=(0.7, 0.75), xycoords="axes fraction", color="black")
plt.annotate("tau2 : ", xy=(0.7, 0.70), xycoords="axes fraction", color="black")

plt.annotate(str(round(identified_model.a_rest, 5)), xy=(0.78, 0.85), xycoords="axes fraction", color="red")
plt.annotate(str(round(identified_model.km_rest, 5)), xy=(0.78, 0.80), xycoords="axes fraction", color="red")
plt.annotate(str(round(identified_model.tau1_rest, 5)), xy=(0.78, 0.75), xycoords="axes fraction", color="red")
plt.annotate(str(round(identified_model.tau2, 5)), xy=(0.78, 0.70), xycoords="axes fraction", color="red")

plt.annotate(str(DingModelFrequency().a_rest), xy=(0.85, 0.85), xycoords="axes fraction", color="blue")
plt.annotate(str(DingModelFrequency().km_rest), xy=(0.85, 0.80), xycoords="axes fraction", color="blue")
plt.annotate(str(DingModelFrequency().tau1_rest), xy=(0.85, 0.75), xycoords="axes fraction", color="blue")
plt.annotate(str(DingModelFrequency().tau2), xy=(0.85, 0.70), xycoords="axes fraction", color="blue")

# --- Delete the temp file ---#
os.remove(f"../data/temp_identification_simulation.pkl")

plt.legend()
plt.show()
