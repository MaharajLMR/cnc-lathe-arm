"""Tests for lathe_arm_opcua node definitions, no ROS, no asyncua required."""

import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))
from lathe_arm_opcua.opcua_nodes import NODES, FAULT_CODES, NAMESPACE_URI
from lathe_arm_opcua.opcua_server import LatheArmOpcUAServer


def test_namespace_uri_defined():
    assert NAMESPACE_URI.startswith("urn:"), "Namespace must be a URN"
    assert "LatheArm" in NAMESPACE_URI


def test_all_required_nodes_present():
    required = {"RobotState", "CycleCount", "EStop", "JointPositions", "EEPosition", "Faults"}
    assert required.issubset(NODES.keys()), f"Missing nodes: {required - NODES.keys()}"


def test_node_structure_valid():
    for name, defn in NODES.items():
        assert "node_id" in defn, f"{name} missing node_id"
        assert "type" in defn, f"{name} missing type"
        assert "access" in defn, f"{name} missing access"
        assert defn["access"] in ("Read", "ReadWrite"), f"{name} invalid access"


def test_estop_is_readwrite():
    assert NODES["EStop"]["access"] == "ReadWrite"


def test_five_fault_codes():
    assert len(FAULT_CODES) == 5
    for code in ["E01", "E02", "E03", "E04", "E05"]:
        assert code in FAULT_CODES


def test_server_update_state():
    srv = LatheArmOpcUAServer()
    assert srv.robot_state == "IDLE"
    srv.update_state(state="RUNNING", joints=[0.1, 0.2, 0.3, 0.4, 0.5],
                     ee=[0.4, 0.0, 0.15], cycle_delta=1)
    assert srv.robot_state == "RUNNING"
    assert srv.cycle_count == 1
    assert srv.joint_positions == [0.1, 0.2, 0.3, 0.4, 0.5]


def test_server_estop_default_false():
    srv = LatheArmOpcUAServer()
    assert srv.estop is False


def test_server_ee_position_default():
    srv = LatheArmOpcUAServer()
    assert len(srv.ee_position) == 3
    assert abs(srv.ee_position[2] - 0.985) < 0.01


def test_live_server_serves_all_nodes():
    """Start the real asyncua server and read every node back with a client.

    Regression: CycleCount was created as Int64 and the first UInt32 write
    crashed the server loop. Skipped only when asyncua is not installed.
    """
    pytest.importorskip("asyncua")
    import asyncio
    from asyncua import Client

    port = 48555

    async def scenario():
        srv = LatheArmOpcUAServer(port=port)
        srv.update_state(cycle_delta=3)
        task = asyncio.create_task(srv.run())
        try:
            for _ in range(50):
                try:
                    async with Client(f"opc.tcp://localhost:{port}/LatheArm/") as c:
                        arm = await c.nodes.objects.get_child(["2:LatheArm"])
                        values = {}
                        for name in NODES:
                            child = await arm.get_child([f"2:{name}"])
                            values[name] = await child.read_value()
                        assert not task.done(), "server loop exited"
                        return values
                except (OSError, asyncio.TimeoutError):
                    await asyncio.sleep(0.1)
            raise AssertionError("server never accepted a connection")
        finally:
            task.cancel()

    values = asyncio.run(scenario())
    assert set(values) == set(NODES)
    assert values["CycleCount"] == 3
