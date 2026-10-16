# Pinout: STM32F446RE → U2D2 → Dynamixel

## STM32 ↔ U2D2

| STM32 Nucleo Pin | Signal | U2D2 Pin |
|---|---|---|
| PA9 (USART1_TX) | Data TX | TXD |
| PA10 (USART1_RX) | Data RX | RXD |
| PA11 | TX Enable (DIR) | 485_DIR |
| GND | Ground | GND |
| 5V (CN7 pin 18) | Logic power | 5V |

UART settings: 1 Mbps, 8N1, half-duplex via 485_DIR pin.

## U2D2 ↔ Dynamixel XM430 chain

```
U2D2 [DATA+/DATA-] ─── XM430 #1 ─── XM430 #2 ─── XM430 #3 ─── XM430 #4 ─── XM430 #5
                   │                                                                  │
                  24V                                                                GND
           (Meanwell PSU)
```

Dynamixel ID assignment: 1=base_shoulder, 2=shoulder_upper, 3=upper_elbow,
4=elbow_wrist, 5=wrist_roll.

## E-stop

| STM32 Pin | Signal | Description |
|---|---|---|
| PA0 (WKUP) | E-STOP_IN | Pulled to 3.3V via 10kΩ; button shorts to GND |
| PC13 (LED) | STATUS_LED | Lit green during normal operation; red on fault |

EXTI0 interrupt on PA0 falling edge → disables all servo torque via emergency broadcast.
