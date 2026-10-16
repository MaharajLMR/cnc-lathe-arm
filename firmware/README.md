# Firmware — 5-DOF Lathe Loading Arm

Proposed hardware and firmware architecture for the industrial version of the arm. This is a design document with a costed BOM; it is not the firmware from the hackathon prototype.

## Hardware Overview

| Component | Details |
|---|---|
| MCU | STM32F446RE Nucleo-64 (ARM Cortex-M4 @ 180 MHz) |
| Servos | 5× Dynamixel XM430-W350-T (Protocol 2.0, 14-bit encoder) |
| Interface | U2D2 USB-to-Dynamixel (RS-485 @ 1 Mbps) |
| Power | 24 V / 10 A Meanwell LRS-240-24 |
| E-stop | Normally-closed push button on GPIO PA0 (TIM1 disable interrupt) |

## Servo Configuration

| Servo ID | Joint | Lower limit | Upper limit | Rated torque |
|---|---|---|---|---|
| 1 | joint_1_base_shoulder | −90° | +90° | 2.6 Nm |
| 2 | joint_2_shoulder_upper | −57° | +109° | 2.6 Nm |
| 3 | joint_3_upper_elbow | −120° | +11.5° | 2.6 Nm |
| 4 | joint_4_elbow_wrist | −90° | +90° | 2.6 Nm |
| 5 | joint_5_wrist_roll | −180° | +180° | 2.6 Nm |

## Communication

```
Host PC (ROS 2)
  │  USB
  ▼
U2D2 — USB ↔ RS-485 half-duplex
  │  RS-485 daisy-chain @ 1 Mbps
  ▼
XM430 #1 → #2 → #3 → #4 → #5
```

Protocol: Dynamixel Protocol 2.0  
Sync-Write for simultaneous position commands (100 Hz), Bulk-Read for feedback.

## Safety

- **Hardware e-stop**: PA0 shorted to GND → TIM1 update ISR zeroes all PWM outputs.
- **Software joint limits**: enforced in `servo_control.cpp` before every write.
- **Overcurrent**: XM430 reports `present_current`; MCU faults if > 2.0 A sustained > 200 ms.

## Firmware Structure

```
firmware/
├── src/
│   ├── main.cpp            (init + 100 Hz control loop)
│   ├── servo_control.cpp   (DXL P2.0 sync write, fault detect)
│   └── estop_handler.cpp   (PA0 interrupt → servo disable)
├── include/
│   └── config.h            (servo IDs, limits, Kp/Kd gains)
├── bom.csv                 (Bill of Materials)
└── pinout.md               (STM32 → U2D2 → Dynamixel wiring)
```

*Planned layout; source not yet written.*
