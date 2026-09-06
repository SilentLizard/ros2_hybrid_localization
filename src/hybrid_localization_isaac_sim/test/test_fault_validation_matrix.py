import importlib.util
import json
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def _load(name, file):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


matrix = _load("fault_validation_matrix_under_test", "fault_validation_matrix.py")
runtime = _load("scenario_runtime_for_stage6_matrix", "scenario_runtime.py")


def test_matrix_covers_s0_through_s9_once_and_matches_catalog():
    cases = matrix.validation_cases()
    assert [case.family for case in cases] == [f"S{i}" for i in range(10)]
    matrix.validate_matrix(runtime.load_catalog(), runtime.scenario_faults)


def test_s9_combined_case_requires_both_channels_to_complete():
    case = matrix.case_for_scenario("S9_COMBINED")
    assert dict(case.expected_fault_states) == {
        "combined_odom_1": "completed",
        "combined_lidar_1": "completed",
    }


def test_fault_status_progression_keeps_most_advanced_state():
    observed = {}
    for state in ("scheduled", "active", "completed", "active"):
        matrix.update_observed_statuses(
            observed,
            json.dumps({"scenario_id": "S4_ODOMETRY_DRIFT", "fault_id": "odom_drift_1", "state": state}),
            "S4_ODOMETRY_DRIFT",
        )
    assert observed["odom_drift_1"]["state"] == "completed"


def test_status_validation_rejects_error_missing_and_unexpected_faults():
    case = matrix.case_for_scenario("S4_ODOMETRY_DRIFT")
    with pytest.raises(matrix.FaultValidationError, match="no status"):
        matrix.validate_observed_statuses(case, {})
    with pytest.raises(matrix.FaultValidationError, match="entered error"):
        matrix.validate_observed_statuses(
            case, {"odom_drift_1": {"state": "error", "detail": "boom"}}
        )
    with pytest.raises(matrix.FaultValidationError, match="unexpected"):
        matrix.validate_observed_statuses(
            case,
            {
                "odom_drift_1": {"state": "completed"},
                "other": {"state": "completed"},
            },
        )


def test_nominal_case_requires_no_fault_statuses():
    case = matrix.case_for_scenario("S0_BASELINE")
    matrix.validate_observed_statuses(case, {})
    with pytest.raises(matrix.FaultValidationError, match="unexpected"):
        matrix.validate_observed_statuses(case, {"leak": {"state": "active"}})


def test_gateway_publishers_must_be_single_expected_owners():
    matrix.validate_gateway_publishers(
        {
            "/odom": ["hybrid_localization_odometry_fault_injector"],
            "/scan": ["/hybrid_localization_lidar_fault_injector"],
        }
    )
    with pytest.raises(matrix.FaultValidationError, match="exactly one publisher"):
        matrix.validate_gateway_publishers(
            {
                "/odom": ["hybrid_localization_odometry_fault_injector", "isaac"],
                "/scan": ["hybrid_localization_lidar_fault_injector"],
            }
        )


def test_ground_truth_requires_multiple_strictly_increasing_samples():
    matrix.validate_ground_truth_stamps([1, 2, 3])
    with pytest.raises(matrix.FaultValidationError, match="at least"):
        matrix.validate_ground_truth_stamps([1])
    with pytest.raises(matrix.FaultValidationError, match="strictly"):
        matrix.validate_ground_truth_stamps([1, 1])
