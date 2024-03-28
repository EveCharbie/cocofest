import time as time_package

from bioptim import Solver, Objective, OdeSolver

from ..models.ding2003 import DingModelFrequency
from ..optimization.fes_identification_ocp import OcpFesId
from .identification_method import (full_data_extraction,
                                    average_data_extraction,
                                    sparse_data_extraction,
                                    node_shooting_list_creation,
                                    force_at_node_in_ocp)
from .identification_abstract_class import ParameterIdentification


class DingModelFrequencyForceParameterIdentification(ParameterIdentification):
    """
    The main class to define an ocp. This class prepares the full program and gives all
    the needed parameters to solve a functional electrical stimulation ocp

    Attributes
    ----------
    model: DingModelFrequency,
        The model to use for the ocp
    data_path: str | list[str],
        The path to the force model data
    force_model_identification_method: str,
        The method to use for the force model identification,
         "full" for objective function on all data,
         "average" for objective function on average data,
         "sparse" for objective function at the beginning and end of the data
    n_shooting: int,
        The number of shooting points for the ocp
    use_sx: bool
        The nature of the casadi variables. MX are used if False.
    """

    def __init__(
        self,
        model: DingModelFrequency,
        data_path: str | list[str] = None,
        identification_method: str = "full",
        double_step_identification: bool = False,
        key_parameter_to_identify: list = None,
        additional_key_settings: dict = None,
        n_shooting: int = 5,
        custom_objective: list[Objective] = None,
        use_sx: bool = True,
        ode_solver: OdeSolver = OdeSolver.RK4(n_integration_steps=1),
        n_threads: int = 1,
        **kwargs,
    ):
        self.default_values = self._set_default_values(model=model)

        dict_parameter_to_configure = model.identifiable_parameters
        model_parameters_value = [
            None if key in key_parameter_to_identify else dict_parameter_to_configure[key]
            for key in dict_parameter_to_configure
        ]
        self.model = self._set_model_parameters(model, model_parameters_value)

        self.input_sanity(
            model,
            data_path,
            identification_method,
            double_step_identification,
            key_parameter_to_identify,
            additional_key_settings,
            n_shooting,
        )

        self.key_parameter_to_identify = key_parameter_to_identify
        self.additional_key_settings = self.key_setting_to_dictionary(key_settings=additional_key_settings)

        self.data_path = data_path
        self.force_model_identification_method = identification_method
        self.double_step_identification = double_step_identification

        self.force_ocp = None
        self.force_identification_result = None
        self.n_shooting = n_shooting
        self.custom_objective = custom_objective
        self.use_sx = use_sx
        self.ode_solver = ode_solver
        self.n_threads = n_threads
        self.kwargs = kwargs

    def _set_default_values(self, model):
        return {
            "a_rest": {
                "initial_guess": 1000,
                "min_bound": 1,
                "max_bound": 10000,
                "function": model.set_a_rest,
                "scaling": 1,
            },
            "km_rest": {
                "initial_guess": 0.5,
                "min_bound": 0.001,
                "max_bound": 1,
                "function": model.set_km_rest,
                "scaling": 1,  # 1000
            },
            "tau1_rest": {
                "initial_guess": 0.5,
                "min_bound": 0.0001,
                "max_bound": 1,
                "function": model.set_tau1_rest,
                "scaling": 1,  # 1000
            },
            "tau2": {
                "initial_guess": 0.5,
                "min_bound": 0.0001,
                "max_bound": 1,
                "function": model.set_tau2,
                "scaling": 1,  # 1000
            },
        }

    def _set_default_parameters_list(self):
        self.numeric_parameters = [self.model.a_rest, self.model.km_rest, self.model.tau1_rest, self.model.tau2]
        self.key_parameters = ["a_rest", "km_rest", "tau1_rest", "tau2"]

    def input_sanity(
        self,
        model,
        data_path,
        identification_method,
        double_step_identification,
        key_parameter_to_identify,
        additional_key_settings,
        n_shooting,
    ):
        if model._with_fatigue:
            raise ValueError(
                f"The given model is not valid and should not be including the fatigue equation in the model"
            )
        self.check_experiment_force_format(data_path)

        if identification_method not in ["full", "average", "sparse"]:
            raise ValueError(
                f"The given model identification method is not valid,"
                f"only 'full', 'average' and 'sparse' are available,"
                f" the given value is {identification_method}"
            )

        if not isinstance(double_step_identification, bool):
            raise TypeError(
                f"The given double_step_identification must be bool type,"
                f" the given value is {type(double_step_identification)} type"
            )

        if isinstance(key_parameter_to_identify, list):
            for key in key_parameter_to_identify:
                if key not in self.default_values:
                    raise ValueError(
                        f"The given key_parameter_to_identify is not valid,"
                        f" the given value is {key},"
                        f" the available values are {list(self.default_values.keys())}"
                    )
        else:
            raise TypeError(
                f"The given key_parameter_to_identify must be list type,"
                f" the given value is {type(key_parameter_to_identify)} type"
            )

        if isinstance(additional_key_settings, dict):
            for key in additional_key_settings:
                if key not in self.default_values:
                    raise ValueError(
                        f"The given additional_key_settings is not valid,"
                        f" the given value is {key},"
                        f" the available values are {list(self.default_values.keys())}"
                    )
                for setting_name in additional_key_settings[key]:
                    if setting_name not in self.default_values[key]:
                        raise ValueError(
                            f"The given additional_key_settings is not valid,"
                            f" the given value is {setting_name},"
                            f" the available values are {list(self.default_values[key].keys())}"
                        )
                    if not isinstance(
                        additional_key_settings[key][setting_name], type(self.default_values[key][setting_name])
                    ):
                        raise TypeError(
                            f"The given additional_key_settings value is not valid,"
                            f" the given value is {type(additional_key_settings[key][setting_name])},"
                            f" the available type is {type(self.default_values[key][setting_name])}"
                        )
        else:
            raise TypeError(
                f"The given additional_key_settings must be dict type,"
                f" the given value is {type(additional_key_settings)} type"
            )

        if not isinstance(n_shooting, int):
            raise TypeError(f"The given n_shooting must be int type," f" the given value is {type(n_shooting)} type")

        self._set_default_parameters_list()
        if not all(isinstance(param, None | int | float) for param in self.numeric_parameters):
            raise ValueError(f"The given model parameters are not valid, only None, int and float are accepted")

        for i in range(len(self.numeric_parameters)):
            if self.numeric_parameters[i] and self.key_parameters[i] in key_parameter_to_identify:
                raise ValueError(
                    f"The given {self.key_parameters[i]} parameter can not be given and identified at the same time."
                    f"Consider either giving {self.key_parameters[i]} and removing it from the key_parameter_to_identify list"
                    f" or the other way around"
                )
            elif not self.numeric_parameters[i] and self.key_parameters[i] not in key_parameter_to_identify:
                raise ValueError(
                    f"The given {self.key_parameters[i]} parameter is not valid, it must be given or identified"
                )

    def key_setting_to_dictionary(self, key_settings):
        """
        This method is used to set the identified parameter optimization values (initial guesses, bounds,
        scaling and function). The default values can be modified by the user when sending the "additional_key_settings"
        input into class. If the user does not provide a value for a specific parameter, the default value will be used.

        Parameters
        ----------
        key_settings: dict,
            The settings attributed from user for parameter to identify

        Returns
        -------
        This function will return a dictionary of dictionaries which contains the identified keys with its associated settings.

        """
        settings_dict = {}
        for key in self.key_parameter_to_identify:
            settings_dict[key] = {}
            for setting_name in self.default_values[key]:
                settings_dict[key][setting_name] = (
                    key_settings[key][setting_name]
                    if (key in key_settings and setting_name in key_settings[key])
                    else self.default_values[key][setting_name]
                )
        return settings_dict

    @staticmethod
    def check_experiment_force_format(data_path):
        if isinstance(data_path, list):
            for i in range(len(data_path)):
                if not isinstance(data_path[i], str):
                    raise TypeError(
                        f"In the given list, all model_data_path must be str type," f" path index n°{i} is not str type"
                    )
                if not data_path[i].endswith(".pkl"):
                    raise TypeError(
                        f"In the given list, all model_data_path must be pickle type and end with .pkl,"
                        f" path index n°{i} is not ending with .pkl"
                    )
        elif isinstance(data_path, str):
            data_path = [data_path]
            if not data_path[0].endswith(".pkl"):
                raise TypeError(
                    f"In the given list, all model_data_path must be pickle type and end with .pkl,"
                    f" path index is not ending with .pkl"
                )
        else:
            raise TypeError(
                f"In the given path, model_data_path must be str or list[str] type, the input is {type(data_path)} type"
            )

    @staticmethod
    def _set_model_parameters(model, model_parameters_value):
        model.a_rest = model_parameters_value[0]
        model.km_rest = model_parameters_value[1]
        model.tau1_rest = model_parameters_value[2]
        model.tau2 = model_parameters_value[3]
        return model

    def _force_model_identification_for_initial_guess(self):
        self.input_sanity(
            self.model,
            self.data_path,
            self.force_model_identification_method,
            self.double_step_identification,
            self.key_parameter_to_identify,
            self.additional_key_settings,
            self.n_shooting,
        )
        self.check_experiment_force_format(self.data_path)
        # --- Data extraction --- #
        # --- Force model --- #
        stimulated_n_shooting = self.n_shooting
        force_curve_number = None

        time, stim, force, discontinuity = average_data_extraction(self.data_path)
        n_shooting, final_time_phase = node_shooting_list_creation(stim, stimulated_n_shooting)
        force_at_node = force_at_node_in_ocp(time, force, n_shooting, final_time_phase, force_curve_number)

        # --- Building force ocp --- #
        self.force_ocp = OcpFesId.prepare_ocp(
            model=self.model,
            n_shooting=n_shooting,
            final_time_phase=final_time_phase,
            force_tracking=force_at_node,
            key_parameter_to_identify=self.key_parameter_to_identify,
            additional_key_settings=self.additional_key_settings,
            custom_objective=self.custom_objective,
            discontinuity_in_ocp=discontinuity,
            use_sx=self.use_sx,
            ode_solver=self.ode_solver,
            n_threads=self.n_threads,
        )

        self.force_identification_result = self.force_ocp.solve(
            Solver.IPOPT()
        )  # _hessian_approximation="limited-memory"

        initial_guess = {}
        for key in self.key_parameter_to_identify:
            initial_guess[key] = self.force_identification_result.parameters[key][0][0]

        return initial_guess

    def force_model_identification(self):
        if not self.double_step_identification:
            self.input_sanity(
                self.model,
                self.data_path,
                self.force_model_identification_method,
                self.double_step_identification,
                self.key_parameter_to_identify,
                self.additional_key_settings,
                self.n_shooting,
            )
            self.check_experiment_force_format(self.data_path)

        # --- Data extraction --- #
        # --- Force model --- #
        stimulated_n_shooting = self.n_shooting
        force_curve_number = None
        time, stim, force, discontinuity = None, None, None, None

        if self.force_model_identification_method == "full":
            time, stim, force, discontinuity = full_data_extraction(self.data_path)

        elif self.force_model_identification_method == "average":
            time, stim, force, discontinuity = average_data_extraction(self.data_path)

        elif self.force_model_identification_method == "sparse":
            force_curve_number = self.kwargs["force_curve_number"] if "force_curve_number" in self.kwargs else 5
            time, stim, force, discontinuity = sparse_data_extraction(self.data_path, force_curve_number)

        n_shooting, final_time_phase = node_shooting_list_creation(stim, stimulated_n_shooting)
        force_at_node = force_at_node_in_ocp(time, force, n_shooting, final_time_phase, force_curve_number)

        if self.double_step_identification:
            initial_guess = self._force_model_identification_for_initial_guess()

            for key in self.key_parameter_to_identify:
                self.additional_key_settings[key]["initial_guess"] = initial_guess[key]

        # --- Building force ocp --- #
        start_time = time_package.time()
        self.force_ocp = OcpFesId.prepare_ocp(
            model=self.model,
            n_shooting=n_shooting,
            final_time_phase=final_time_phase,
            force_tracking=force_at_node,
            key_parameter_to_identify=self.key_parameter_to_identify,
            additional_key_settings=self.additional_key_settings,
            custom_objective=self.custom_objective,
            discontinuity_in_ocp=discontinuity,
            use_sx=self.use_sx,
            ode_solver=self.ode_solver,
            n_threads=self.n_threads,
        )

        print(f"OCP creation time : {time_package.time() - start_time} seconds")

        self.force_identification_result = self.force_ocp.solve(Solver.IPOPT(_max_iter=1000))

        identified_parameters = {}
        for key in self.key_parameter_to_identify:
            identified_parameters[key] = self.force_identification_result.parameters[key][0]

        return identified_parameters
