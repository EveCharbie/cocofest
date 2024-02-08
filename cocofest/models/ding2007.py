from typing import Callable

from casadi import MX, vertcat, exp

from bioptim import (
    ConfigureProblem,
    DynamicsEvaluation,
    NonLinearProgram,
    OptimalControlProgram,
    ParameterList,
)
from .ding2003 import DingModelFrequency


class DingModelPulseDurationFrequency(DingModelFrequency):
    """
    This is a custom models that inherits from bioptim. CustomModel.
    As CustomModel is an abstract class, some methods are mandatory and must be implemented.
    Such as serialize, name_dof, nb_state.

    This is the Ding 2007 model using the stimulation frequency and pulse duration in input.

    Ding, J., Chou, L. W., Kesar, T. M., Lee, S. C., Johnston, T. E., Wexler, A. S., & Binder‐Macleod, S. A. (2007).
    Mathematical model that predicts the force–intensity and force–frequency relationships after spinal cord injuries.
    Muscle & Nerve: Official Journal of the American Association of Electrodiagnostic Medicine, 36(2), 214-222.
    """

    def __init__(self, model_name: str = "ding_2007", muscle_name: str = None, sum_stim_truncation: int = None):
        super(DingModelPulseDurationFrequency, self).__init__(
            model_name=model_name, muscle_name=muscle_name, sum_stim_truncation=sum_stim_truncation
        )
        self._with_fatigue = False
        self.impulse_time = None
        # ---- Custom values for the example ---- #
        # ---- Force models ---- #
        self.a_scale = 4920  # Value from Ding's 2007 article (N/s)
        self.pd0 = 0.000131405  # Value from Ding's 2007 article (s)
        self.pdt = 0.000194138  # Value from Ding's 2007 article (s)
        self.tau1_rest = 0.060601  # Value from Ding's 2003 article (s)
        self.tau2 = 0.001  # Value from Ding's 2007 article (s)
        self.km = 0.137  # Value from Ding's 2007 article (unitless)
        self.tauc = 0.011  # Value from Ding's 2007 article (s)

    def set_a_scale(self, model, a_scale: MX | float):
        # models is required for bioptim compatibility
        self.a_scale = a_scale

    def set_pd0(self, model, pd0: MX | float):
        self.pd0 = pd0

    def set_pdt(self, model, pdt: MX | float):
        self.pdt = pdt

    def serialize(self) -> tuple[Callable, dict]:
        # This is where you can serialize your models
        # This is useful if you want to save your models and load it later
        return (
            DingModelPulseDurationFrequency,
            {
                "tauc": self.tauc,
                "a_rest": self.a_rest,
                "tau1_rest": self.tau1_rest,
                "km_rest": self.km_rest,
                "tau2": self.tau2,
                "a_scale": self.a_scale,
                "pd0": self.pd0,
                "pdt": self.pdt,
            },
        )

    def system_dynamics(
        self,
        cn: MX,
        f: MX,
        t: MX = None,
        t_stim_prev: list[MX] | list[float] = None,
        impulse_time: MX = None,
        force_length_relationship: MX | float = 1,
        force_velocity_relationship: MX | float = 1,
    ) -> MX:
        """
        The system dynamics is the function that describes the models.

        Parameters
        ----------
        cn: MX
            The value of the ca_troponin_complex (unitless)
        f: MX
            The value of the force (N)
        t: MX
            The current time at which the dynamics is evaluated (ms)
        t_stim_prev: list[MX]
            The time list of the previous stimulations (ms)
        impulse_time: MX
            The pulsation duration of the current stimulation (ms)
        force_length_relationship: MX | float
            The force length relationship value (unitless)
        force_velocity_relationship: MX | float
            The force velocity relationship value (unitless)

        Returns
        -------
        The value of the derivative of each state dx/dt at the current time t
        """
        r0 = self.km_rest + self.r0_km_relationship  # Simplification
        cn_dot = self.cn_dot_fun(cn, r0, t, t_stim_prev=t_stim_prev)  # Equation n°1 from Ding's 2003 article
        a = self.a_calculation(a_scale=self.a_scale, impulse_time=impulse_time)  # Equation n°3 from Ding's 2007 article
        f_dot = self.f_dot_fun(
            cn,
            f,
            a,
            self.tau1_rest,
            self.km_rest,
            force_length_relationship=force_length_relationship,
            force_velocity_relationship=force_velocity_relationship,
        )  # Equation n°2 from Ding's 2003 article
        return vertcat(cn_dot, f_dot)

    def a_calculation(self, a_scale: float | MX, impulse_time: MX) -> MX:
        """
        Parameters
        ----------
        a_scale: float | MX
            The scaling factor of the current stimulation (unitless)
        impulse_time: MX
            The pulsation duration of the current stimulation (s)

        Returns
        -------
        The value of scaling factor (unitless)
        """
        return a_scale * (1 - exp(-(impulse_time - self.pd0) / self.pdt))

    def set_impulse_duration(self, value: list[MX]):
        """
        Sets the impulse time for each pulse (phases) according to the ocp parameter "impulse_time"

        Parameters
        ----------
        value: list[MX]
            The pulsation duration list (s)
        """
        self.impulse_time = value

    @staticmethod
    def get_pulse_duration_parameters(nlp_parameters: ParameterList, muscle_name: str = None) -> MX:
        """
        Get the nlp list of pulse_duration parameters

        Parameters
        ----------
        nlp_parameters: ParameterList
            The nlp list parameter
        muscle_name: str
            The muscle name

        Returns
        -------
        The list of list of pulse_duration parameters
        """
        pulse_duration_parameters = vertcat()
        for j in range(nlp_parameters.mx.shape[0]):
            if muscle_name:
                if "pulse_duration_" + muscle_name in str(nlp_parameters.mx[j]):
                    pulse_duration_parameters = vertcat(pulse_duration_parameters, nlp_parameters.mx[j])
            elif "pulse_duration" in str(nlp_parameters.mx[j]):
                pulse_duration_parameters = vertcat(pulse_duration_parameters, nlp_parameters.mx[j])

        return pulse_duration_parameters

    @staticmethod
    def dynamics(
        time: MX,
        states: MX,
        controls: MX,
        parameters: MX,
        stochastic_variables: MX,
        nlp: NonLinearProgram,
        stim_apparition: list[float] = None,
        fes_model=None,
        force_length_relationship: MX | float = 1,
        force_velocity_relationship: MX | float = 1,
    ) -> DynamicsEvaluation:
        """
        Functional electrical stimulation dynamic

        Parameters
        ----------
        time: MX
            The system's current node time
        states: MX
            The state of the system CN, F, A, Tau1, Km
        controls: MX
            The controls of the system, none
        parameters: MX
            The parameters acting on the system, final time of each phase
        stochastic_variables: MX
            The stochastic variables of the system, none
        nlp: NonLinearProgram
            A reference to the phase
        stim_apparition: list[float]
            The time list of the previous stimulations (s)
        fes_model: DingModelPulseDurationFrequency
            The current phase fes model
        force_length_relationship: MX | float
            The force length relationship value (unitless)
        force_velocity_relationship: MX | float
            The force velocity relationship value (unitless)
        Returns
        -------
        The derivative of the states in the tuple[MX] format
        """
        pulse_duration_parameters = (
            nlp.model.get_pulse_duration_parameters(nlp.parameters)
            if fes_model is None
            else fes_model.get_pulse_duration_parameters(nlp.parameters, muscle_name=fes_model.muscle_name)
        )

        if pulse_duration_parameters.shape[0] == 1:  # check if pulse duration is mapped
            impulse_time = pulse_duration_parameters[0]
        else:
            impulse_time = pulse_duration_parameters[nlp.phase_idx]

        dxdt_fun = fes_model.system_dynamics if fes_model else nlp.model.system_dynamics

        return DynamicsEvaluation(
            dxdt=dxdt_fun(
                cn=states[0],
                f=states[1],
                t=time,
                t_stim_prev=stim_apparition,
                impulse_time=impulse_time,
                force_length_relationship=force_length_relationship,
                force_velocity_relationship=force_velocity_relationship,
            ),
            defects=None,
        )

    def declare_ding_variables(self, ocp: OptimalControlProgram, nlp: NonLinearProgram):
        """
        Tell the program which variables are states and controls.
        The user is expected to use the ConfigureProblem.configure_xxx functions.
        Parameters
        ----------
        ocp: OptimalControlProgram
            A reference to the ocp
        nlp: NonLinearProgram
            A reference to the phase
        """
        self.configure_ca_troponin_complex(
            ocp=ocp, nlp=nlp, as_states=True, as_controls=False, muscle_name=self.muscle_name
        )
        self.configure_force(ocp=ocp, nlp=nlp, as_states=True, as_controls=False, muscle_name=self.muscle_name)
        stim_apparition = self.get_stim_prev(ocp, nlp)
        ConfigureProblem.configure_dynamics_function(ocp, nlp, dyn_func=self.dynamics, stim_apparition=stim_apparition)
