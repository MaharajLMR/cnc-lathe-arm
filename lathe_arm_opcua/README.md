# OPC-UA Interface — CNC Lathe Loading Arm

## Why OPC-UA?

OPC-UA (IEC 62541) is the de-facto standard for industrial device connectivity. It allows the lathe arm controller to be monitored and controlled from:

- **SCADA / HMI systems** (Ignition, WonderWare, Simatic WinCC)
- **MES / ERP systems** (SAP ME, Tulip) — cycle count and batch progress feed the production tracking database
- **Digital twin platforms** (Azure Digital Twins, AWS IoT TwinMaker) — real-time joint positions and EE pose
- **Python test clients** — `opcua-asyncio` for automated quality checks

## Address Space

Namespace URI: `urn:CITChennai:LatheArm`

| Node name        | Node ID             | Type        | Access     | Description |
|---|---|---|---|---|
| RobotState       | `ns=2;s=RobotState` | String      | Read       | `IDLE` / `RUNNING` / `FAULT` / `E_STOP` |
| CycleCount       | `ns=2;s=CycleCount` | UInt32      | Read       | Total cycles since last reset |
| LastCycleTime_s  | `ns=2;s=LastCycleTime_s` | Double | Read      | Last cycle duration (seconds) |
| BatchProgress    | `ns=2;s=BatchProgress` | String   | Read       | `"3/10"` format |
| EStop            | `ns=2;s=EStop`      | Boolean     | ReadWrite  | Write `True` to trigger e-stop |
| JointPositions   | `ns=2;s=JointPositions` | Double[] | Read    | [j1..j5] in radians |
| EEPosition       | `ns=2;s=EEPosition` | Double[]    | Read       | [x, y, z] in metres |
| Faults           | `ns=2;s=Faults`     | String      | Read       | Comma-separated fault codes |

## Fault Codes

| Code | Meaning |
|---|---|
| E01 | Joint position limit violation |
| E02 | Joint velocity exceeded rated limit |
| E03 | Joint tracking error exceeded 1° |
| E04 | Near-singular configuration (μ < 0.01) |
| E05 | Safety zone violation — operator proximity |

## Quick Start

```bash
pip install asyncua

# Start server
python3 -m lathe_arm_opcua.opcua_server --port 4840

# Connect with UA Expert (free OPC-UA browser):
#   Endpoint: opc.tcp://localhost:4840/LatheArm/
#   Browse to: Objects > LatheArm > RobotState

# Python client example:
# from asyncua import Client
# async with Client("opc.tcp://localhost:4840/LatheArm/") as c:
#     node = c.get_node("ns=2;s=CycleCount")
#     print(await node.read_value())
```

## Integration with batch_scheduler

```python
from lathe_arm_opcua.opcua_server import LatheArmOpcUAServer
from analysis.batch_scheduler import BatchScheduler

srv = LatheArmOpcUAServer()
sched = BatchScheduler()
# ... add parts ...

for i, record in enumerate(sched.simulate_batch("priority_fifo")["records"]):
    srv.update_state(
        state="RUNNING",
        batch_progress=f"{i+1}/{len(sched.parts)}",
        last_cycle_time_s=record["end_s"] - record["start_s"],
        cycle_delta=1,
    )
```
