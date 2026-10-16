"""
OPC-UA address space definition for the CNC lathe loading arm.
Namespace: urn:CITChennai:LatheArm
"""

NAMESPACE_URI = "urn:CITChennai:LatheArm"

NODES = {
    "RobotState": {
        "node_id": "ns=2;s=RobotState",
        "type": "String",
        "description": "IDLE / RUNNING / FAULT / E_STOP",
        "access": "Read",
    },
    "CycleCount": {
        "node_id": "ns=2;s=CycleCount",
        "type": "UInt32",
        "description": "Total load/unload cycles completed since last reset",
        "access": "Read",
    },
    "LastCycleTime_s": {
        "node_id": "ns=2;s=LastCycleTime_s",
        "type": "Double",
        "description": "Duration of the last completed cycle in seconds",
        "access": "Read",
    },
    "BatchProgress": {
        "node_id": "ns=2;s=BatchProgress",
        "type": "String",
        "description": "Completed/Total parts in current batch (e.g. '3/10')",
        "access": "Read",
    },
    "EStop": {
        "node_id": "ns=2;s=EStop",
        "type": "Boolean",
        "description": "Emergency stop state, write True to trigger e-stop",
        "access": "ReadWrite",
    },
    "JointPositions": {
        "node_id": "ns=2;s=JointPositions",
        "type": "DoubleArray",
        "description": "Current joint angles [j1..j5] in radians",
        "access": "Read",
    },
    "EEPosition": {
        "node_id": "ns=2;s=EEPosition",
        "type": "DoubleArray",
        "description": "End-effector Cartesian position [x, y, z] in metres",
        "access": "Read",
    },
    "Faults": {
        "node_id": "ns=2;s=Faults",
        "type": "String",
        "description": "Comma-separated active fault codes (e.g. 'E01,E04')",
        "access": "Read",
    },
}

FAULT_CODES = {
    "E01": "Joint position limit violation",
    "E02": "Joint velocity exceeded rated limit",
    "E03": "Joint tracking error exceeded 1°",
    "E04": "Near-singular configuration (μ < 0.01)",
    "E05": "Safety zone violation, operator proximity",
}
