#!/usr/bin/env python3
"""ROSMonitoring property for the Nav2/MoveIt interlock."""

import math

import oracle


PROPERTY = (
    "(({nav_request: true} -> ({arm_stowed: true} or {arm_carry: true})) and "
    "({arm_request: true} -> {navigation_active: false}) and "
    "({gripper_open_request: true} -> {navigation_active: false}))"
)

NAV_ACTION = "/a200_0000/navigate_to_pose"
ARM_ACTION = (
    "/a200_0000/arm_0_joint_trajectory_controller/"
    "follow_joint_trajectory"
)
GRIPPER_ACTION = "/a200_0000/arm_0_gripper_controller/gripper_cmd"
JOINT_STATE_TOPIC = "/a200_0000/monitor_joint_states"
ARM_JOINTS = tuple(f"arm_0_joint_{index}" for index in range(1, 7))
STOW_POSITIONS = (
    0.0,
    math.pi / 4,
    5 * math.pi / 6,
    math.pi / 2,
    math.pi / 4,
    -math.pi / 2,
) #posizione stow -> braccio richiuso
CARRY_POSITIONS = (
    0.0,
    0.0,
    1.0,
    math.pi / 2,
    1.8,
    -math.pi / 2,
) # posa di trasporto compatta, simile a stow

predicates = dict(
    time=0,
    arm_stowed=False,
    arm_carry=False,
    navigation_active=False,
    nav_request=False,
    arm_request=False,
    gripper_open_request=False,
)


def abstract_message(message):
    """Translate one ROSMonitor JSON event into property predicates."""
    predicates["nav_request"] = False # reset a false
    predicates["arm_request"] = False
    predicates["gripper_open_request"] = False

    topic = message.get("topic") #lettura campi messaggio
    kind = message.get("event_kind")
    action = message.get("action_name")

    if topic == JOINT_STATE_TOPIC:
        positions = dict(zip(message.get("name", []), message.get("position", [])))
        def arm_is_at(expected_positions):
            return all(
                name in positions and abs(positions[name] - expected) < 0.05
                for name, expected in zip(ARM_JOINTS, expected_positions)
            )

        predicates["arm_stowed"] = arm_is_at(STOW_POSITIONS)
        predicates["arm_carry"] = arm_is_at(CARRY_POSITIONS)

    if kind == "status" and action == NAV_ACTION:
        statuses = [item.get("status", 0) for item in message.get("status_list", [])]
        if any(status in (1, 2, 3) for status in statuses): #status 1 accepted 2 execturing 3 cancelling 4 succeded 5 canceled 6 aborted
            predicates["navigation_active"] = True
        elif any(status in (4, 5, 6) for status in statuses):
            predicates["navigation_active"] = False

    if kind == "start_action_response" and action == NAV_ACTION: #se ricevuto la risposta di Nav2 alla richiesta di un nuovo goal
        response = message.get("response", {}) #response contiene esito richiesta
        goal_was_accepted = response.get("accepted", False) # cerca il campo response, false se non esiste

        if goal_was_accepted:
            predicates["navigation_active"] = True

    if kind == "start_action_request": # se evento di richiesta avvio action
        if action == NAV_ACTION: #se richiesta di navigate_to_pose
            predicates["nav_request"] = True
        elif action == ARM_ACTION: # se richiesta di follow_joint_trajectory
            predicates["arm_request"] = True
        elif action == GRIPPER_ACTION:
            request = message.get("request", {})
            goal = request.get("goal", {})
            command = goal.get("command", {})
            predicates["gripper_open_request"] = command.get("position", 1.0) < 0.1

    predicates["time"] = message.get("time", predicates["time"]) #se il messaggio contiene time aggiorniamo con valore presente oppure conserva quello prec
    return predicates
