"""
OPC-UA server for the CNC lathe loading arm.

Exposes robot state, cycle count, joint positions, e-stop, and fault codes to
any OPC-UA client (UA Expert, Ignition, Python opcua-asyncio, etc.).

Start:
  python3 -m lathe_arm_opcua.opcua_server --port 4840

Requires: pip install asyncua
Degrades gracefully when asyncua is not installed, prints stub info and exits.
"""

import asyncio
import argparse

from .opcua_nodes import NODES, NAMESPACE_URI

try:
    from asyncua import Server, ua
    _ASYNCUA_OK = True
except ImportError:
    _ASYNCUA_OK = False


class LatheArmOpcUAServer:
    """
    Stateful OPC-UA server wrapper.

    Call update_state() from the replay_node or batch_scheduler to push new
    values; the asyncio loop refreshes all OPC-UA variables every 100 ms.
    """

    def __init__(self, port: int = 4840):
        self.port = port
        self.robot_state: str = "IDLE"
        self.cycle_count: int = 0
        self.last_cycle_time_s: float = 0.0
        self.batch_progress: str = "0/0"
        self.estop: bool = False
        self.joint_positions: list[float] = [0.0] * 5
        self.ee_position: list[float] = [0.0, 0.0, 0.985]
        self.faults: str = ""

    def update_state(
        self,
        state: str | None = None,
        joints: list[float] | None = None,
        ee: list[float] | None = None,
        cycle_delta: int = 0,
        last_cycle_time_s: float | None = None,
        batch_progress: str | None = None,
        faults: str | None = None,
    ):
        """Thread-safe (GIL) update of server state before the next 100 ms poll."""
        if state is not None:
            self.robot_state = state
        if joints is not None:
            self.joint_positions = list(joints)
        if ee is not None:
            self.ee_position = list(ee)
        if cycle_delta:
            self.cycle_count += cycle_delta
        if last_cycle_time_s is not None:
            self.last_cycle_time_s = last_cycle_time_s
        if batch_progress is not None:
            self.batch_progress = batch_progress
        if faults is not None:
            self.faults = faults

    async def run(self):
        """Start the OPC-UA server. Blocks until cancelled."""
        if not _ASYNCUA_OK:
            print("asyncua not installed, OPC-UA server running as stub.")
            print(f"  Endpoint (would be): opc.tcp://0.0.0.0:{self.port}/LatheArm/")
            print("  Install with: pip install asyncua")
            print()
            print("  Defined nodes:")
            for name, defn in NODES.items():
                print(f"    [{defn['node_id']}]  {name}  ({defn['type']}, {defn['access']})")
            return

        server = Server()
        await server.init()
        server.set_endpoint(f"opc.tcp://0.0.0.0:{self.port}/LatheArm/")
        server.set_server_name("CIT Chennai, 5-DOF CNC Lathe Arm")

        idx = await server.register_namespace(NAMESPACE_URI)
        objects = server.get_objects_node()
        arm_obj = await objects.add_object(idx, "LatheArm")

        # Create variables
        v = {}
        defaults = {
            "String": "",
            "UInt32": 0,
            "Double": 0.0,
            "Boolean": False,
            "DoubleArray": [0.0],
        }
        # Explicit variant types so later writes match the declared node type
        # (a bare Python int would otherwise be created as Int64).
        vtypes = {
            "String": ua.VariantType.String,
            "UInt32": ua.VariantType.UInt32,
            "Double": ua.VariantType.Double,
            "Boolean": ua.VariantType.Boolean,
            "DoubleArray": ua.VariantType.Double,
        }
        for name, defn in NODES.items():
            var = await arm_obj.add_variable(
                idx, name, defaults[defn["type"]], varianttype=vtypes[defn["type"]]
            )
            if defn["access"] == "ReadWrite":
                await var.set_writable()
            v[name] = var

        async with server:
            print(f"OPC-UA server running at opc.tcp://0.0.0.0:{self.port}/LatheArm/")
            while True:
                await v["RobotState"].write_value(self.robot_state)
                await v["CycleCount"].write_value(
                    ua.Variant(self.cycle_count, ua.VariantType.UInt32)
                )
                await v["LastCycleTime_s"].write_value(float(self.last_cycle_time_s))
                await v["BatchProgress"].write_value(self.batch_progress)
                await v["EStop"].write_value(self.estop)
                await v["JointPositions"].write_value(list(self.joint_positions))
                await v["EEPosition"].write_value(list(self.ee_position))
                await v["Faults"].write_value(self.faults)
                await asyncio.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="CNC Lathe Arm OPC-UA server")
    parser.add_argument("--port", type=int, default=4840)
    args = parser.parse_args()

    srv = LatheArmOpcUAServer(port=args.port)
    asyncio.run(srv.run())


if __name__ == "__main__":
    main()
