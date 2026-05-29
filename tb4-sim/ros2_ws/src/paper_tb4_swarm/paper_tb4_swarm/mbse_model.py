"""Small executable MBSE model for the two-TurtleBot4 experiment."""

from enum import Enum


TASK_MODEL_ID = "paper_mbse_two_turtlebot4_swarm"
SCENARIO_ID = "two_turtlebot4_leader_follower"
VALIDATION_CASE = "VC-SIM-2TB4-001"


class Role(str, Enum):
    IDLE = "idle"
    LEADER = "leader"
    FOLLOWER = "follower"


class Phase(str, Enum):
    WAITING_FOR_TARGET = "waiting_for_target"
    ESTIMATE_STATE = "estimate_state"
    ALLOCATE_ROLES = "allocate_roles"
    NAVIGATE = "navigate"
    TARGET_REACHED = "target_reached"


REQUIREMENTS = {
    "REQ-001": "Two robots shall cooperate on one target-acquisition task.",
    "REQ-002": "Robot1 shall act as the leader and navigate to the operator-selected target.",
    "REQ-003": "Robot2 shall act as the follower and track a dynamic pose behind robot1.",
    "REQ-004": "The runtime shall publish observable roles, goals, task state, and events.",
    "REQ-005": "The simulation shall run before physical deployment and write validation metrics.",
    "REQ-006": "The physical deployment shall document DDS/domain mapping and shall not command TurtleBot4 motion unless explicitly enabled.",
}


PHASE_REQUIREMENTS = {
    Phase.WAITING_FOR_TARGET.value: ["REQ-004"],
    Phase.ESTIMATE_STATE.value: ["REQ-001", "REQ-004"],
    Phase.ALLOCATE_ROLES.value: ["REQ-001", "REQ-002", "REQ-003"],
    Phase.NAVIGATE.value: ["REQ-001", "REQ-003", "REQ-004", "REQ-006"],
    Phase.TARGET_REACHED.value: ["REQ-005", "REQ-006"],
}


def metadata(phase: Phase | str) -> dict:
    phase_value = phase.value if isinstance(phase, Phase) else str(phase)
    return {
        "task_model_id": TASK_MODEL_ID,
        "scenario_id": SCENARIO_ID,
        "validation_case": VALIDATION_CASE,
        "phase": phase_value,
        "requirement_refs": PHASE_REQUIREMENTS.get(phase_value, []),
    }
