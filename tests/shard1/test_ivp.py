import numpy as np
import pytest
import re

from cocofest import (
    IvpFes,
    DingModelFrequencyIntegrate,
    DingModelFrequencyWithFatigueIntegrate,
    DingModelPulseDurationFrequencyIntegrate,
    DingModelPulseDurationFrequencyWithFatigueIntegrate,
    DingModelIntensityFrequencyIntegrate,
    DingModelIntensityFrequencyWithFatigueIntegrate,
)


@pytest.mark.parametrize("model", [DingModelFrequencyIntegrate(), DingModelFrequencyWithFatigueIntegrate()])
def test_ding2003_ivp(model):
    fes_parameters = {"model": model, "stim_time": [0, 0.1, 0.2]}
    ivp_parameters = {"n_shooting": 300, "final_time": 0.3, "use_sx": True}

    ivp = IvpFes(fes_parameters, ivp_parameters)

    # Integrating the solution
    result = ivp.integrate(return_time=False)

    if model._with_fatigue:
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][100], 92.47272524474053)
        np.testing.assert_almost_equal(result["F"][0][-1], 140.08443077501863)
    else:
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][100], 91.80567367377697)
        np.testing.assert_almost_equal(result["F"][0][-1], 131.36141448855284)


@pytest.mark.parametrize(
    "model",
    [DingModelPulseDurationFrequencyIntegrate(), DingModelPulseDurationFrequencyWithFatigueIntegrate()],
)
@pytest.mark.parametrize("pulse_duration", [0.0003, [0.0003, 0.0004, 0.0005]])
def test_ding2007_ivp(model, pulse_duration):
    fes_parameters = {"model": model, "stim_time": [0, 0.1, 0.2], "pulse_duration": pulse_duration}
    ivp_parameters = {"n_shooting": 30, "final_time": 0.3, "use_sx": True}

    ivp = IvpFes(fes_parameters, ivp_parameters)

    # Integrating the solution
    result = ivp.integrate(return_time=False)

    if model._with_fatigue and isinstance(pulse_duration, list):
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][10], 28.3477940849177)
        np.testing.assert_almost_equal(result["F"][0][-1], 54.99264277880504)
    elif model._with_fatigue is False and isinstance(pulse_duration, list):
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][10], 28.116838973337046)
        np.testing.assert_almost_equal(result["F"][0][-1], 51.68572030372867)
    elif model._with_fatigue and isinstance(pulse_duration, float):
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][10], 28.3477940849177)
        np.testing.assert_almost_equal(result["F"][0][-1], 38.25981953994852)
    elif model._with_fatigue is False and isinstance(pulse_duration, float):
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][10], 28.116838973337046)
        np.testing.assert_almost_equal(result["F"][0][-1], 36.263299814887766)


@pytest.mark.parametrize("model", [DingModelIntensityFrequencyIntegrate(), DingModelIntensityFrequencyWithFatigueIntegrate()])
@pytest.mark.parametrize("pulse_intensity", [50, [50, 60, 70]])
def test_hmed2018_ivp(model, pulse_intensity):
    fes_parameters = {"model": model, "stim_time": [0, 0.1, 0.2], "pulse_intensity": pulse_intensity}
    ivp_parameters = {"n_shooting": 30, "final_time": 0.3, "use_sx": True}

    ivp = IvpFes(fes_parameters, ivp_parameters)

    # Integrating the solution
    result = ivp.integrate(return_time=False)

    if model._with_fatigue and isinstance(pulse_intensity, list):
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][10], 42.18211764372109)
        np.testing.assert_almost_equal(result["F"][0][-1], 94.48614428838563)
    elif model._with_fatigue is False and isinstance(pulse_intensity, list):
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][10], 41.91914906078192)
        np.testing.assert_almost_equal(result["F"][0][-1], 90.43032549879167)
    elif model._with_fatigue and isinstance(pulse_intensity, float):
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][10], 42.18211764372109)
        np.testing.assert_almost_equal(result["F"][0][-1], 58.26448576796251)
    elif model._with_fatigue is False and isinstance(pulse_intensity, float):
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][10], 41.91914906078192)
        np.testing.assert_almost_equal(result["F"][0][-1], 55.57471909903151)


@pytest.mark.parametrize("pulse_mode", ["single", "doublet", "triplet"])
def test_pulse_mode_ivp(pulse_mode):
    fes_parameters = {
        "model": DingModelFrequencyWithFatigueIntegrate(),
        "stim_time": [0, 0.1, 0.2],
        "pulse_mode": pulse_mode,
    }
    ivp_parameters = {"n_shooting": 300, "final_time": 0.3, "use_sx": True}

    ivp = IvpFes(fes_parameters, ivp_parameters)

    # Integrating the solution
    result = ivp.integrate(return_time=False)

    if pulse_mode == "single":
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][100], 92.47272524474053)
        np.testing.assert_almost_equal(result["F"][0][-1], 140.08443077501863)
    elif pulse_mode == "doublet":
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][100], 129.20173193528873)
        np.testing.assert_almost_equal(result["F"][0][-1], 204.32341945760993)

    elif pulse_mode == "triplet":
        np.testing.assert_almost_equal(result["F"][0][0], 0)
        np.testing.assert_almost_equal(result["F"][0][100], 148.0019331088371)
        np.testing.assert_almost_equal(result["F"][0][-1], 238.6839192670876)


def test_ivp_methods():
    fes_parameters = {
        "model": DingModelFrequencyIntegrate(),
        "frequency": 30,
        "round_down": True,
    }
    ivp_parameters = {"n_shooting": 1250, "final_time": 1.25, "use_sx": True}
    ivp = IvpFes.from_frequency_and_final_time(fes_parameters, ivp_parameters)

    fes_parameters = {"model": DingModelFrequencyIntegrate(), "n_stim": 3, "frequency": 10}
    ivp_parameters = {"n_shooting": 300, "use_sx": True}
    ivp = IvpFes.from_frequency_and_n_stim(fes_parameters, ivp_parameters)


def test_all_ivp_errors():
    with pytest.raises(
        ValueError,
        match=re.escape(
            "The number of stimulation needs to be integer within the final time t, set round down "
            "to True or set final_time * frequency to make the result an integer."
        ),
    ):
        IvpFes.from_frequency_and_final_time(
            fes_parameters={
                "model": DingModelFrequencyIntegrate(),
                "frequency": 30,
                "round_down": False,
            },
            ivp_parameters={"n_shooting": 100, "final_time": 1.25},
        )

    with pytest.raises(ValueError, match="Pulse mode not yet implemented"):
        IvpFes(
            fes_parameters={
                "model": DingModelFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_mode": "Quadruplet",
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    pulse_duration = 0.00001
    with pytest.raises(
        ValueError,
        match=re.escape("Pulse duration must be greater than minimum pulse duration"),
    ):
        IvpFes(
            fes_parameters={
                "model": DingModelPulseDurationFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_duration": pulse_duration,
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    with pytest.raises(ValueError, match="pulse_duration list must have the same length as n_stim"):
        IvpFes(
            fes_parameters={
                "model": DingModelPulseDurationFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_duration": [0.0003, 0.0004],
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    pulse_duration = [0.001, 0.0001, 0.003]
    with pytest.raises(
        ValueError,
        match=re.escape("Pulse duration must be greater than minimum pulse duration"),
    ):
        IvpFes(
            fes_parameters={
                "model": DingModelPulseDurationFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_duration": pulse_duration,
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    with pytest.raises(TypeError, match="pulse_duration must be int, float or list type"):
        IvpFes(
            fes_parameters={
                "model": DingModelPulseDurationFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_duration": True,
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    pulse_intensity = 0.1
    with pytest.raises(
        ValueError,
        match=re.escape("Pulse intensity must be greater than minimum pulse intensity"),
    ):
        IvpFes(
            fes_parameters={
                "model": DingModelIntensityFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_intensity": pulse_intensity,
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    with pytest.raises(ValueError, match="pulse_intensity list must have the same length as n_stim"):
        IvpFes(
            fes_parameters={
                "model": DingModelIntensityFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_intensity": [20, 30],
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    pulse_intensity = [20, 30, 0.1]
    with pytest.raises(
        ValueError,
        match=re.escape("Pulse intensity must be greater than minimum pulse intensity"),
    ):
        IvpFes(
            fes_parameters={
                "model": DingModelIntensityFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_intensity": pulse_intensity,
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    with pytest.raises(TypeError, match="pulse_intensity must be int, float or list type"):
        IvpFes(
            fes_parameters={
                "model": DingModelIntensityFrequencyIntegrate(),
                "stim_time": [0, 0.1, 0.2],
                "pulse_intensity": True,
            },
            ivp_parameters={"n_shooting": 100, "final_time": 0.3},
        )

    with pytest.raises(ValueError, match="ode_solver must be a OdeSolver type"):
        IvpFes(
            fes_parameters={"model": DingModelFrequencyIntegrate(), "stim_time": [0, 0.1, 0.2],},
            ivp_parameters={"n_shooting": 100, "final_time": 0.3, "ode_solver": None},
        )

    with pytest.raises(ValueError, match="use_sx must be a bool type"):
        IvpFes(
            fes_parameters={"model": DingModelFrequencyIntegrate(), "stim_time": [0, 0.1, 0.2],},
            ivp_parameters={"n_shooting": 100, "final_time": 0.3, "use_sx": None},
        )

    with pytest.raises(ValueError, match="n_thread must be a int type"):
        IvpFes(
            fes_parameters={"model": DingModelFrequencyIntegrate(), "stim_time": [0, 0.1, 0.2],},
            ivp_parameters={"n_shooting": 100, "final_time": 0.3, "n_threads": None},
        )
