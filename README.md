# Dual-Axis Rocket Thrust Vector Control (TVC) Prototype

An embedded dual-axis Thrust Vector Control (TVC) stabilization prototype built on the ATmega328P (Arduino Uno). Using a custom 3D-printed concentric gimbal mount, an MPU-6050 Inertial Measurement Unit (IMU), and two MG90S metal-gear micro servos, the system executes a real-time discrete PID control loop at 66 Hz ($\Delta t \approx 15\text{ ms}$) to actively maintain a vertical $0^\circ$ thrust vector setpoint across both pitch and roll axes.

All firmware is implemented in bare-metal, register-level C—bypassing standard Arduino libraries (`Servo.h`, `Wire.h`, and `millis()`) to guarantee deterministic execution timing and eliminate control loop jitter.

## Key Features

* **Bare-Metal Register Architecture:** Directly configures ATmega328P hardware registers for GPIO (`DDRD`), system timing (`Timer0`), PWM actuation (`Timer1`), and $I^2C$ communication (`TWI`).
* **Deterministic 1 ms System Timer (`Timer0`):** Configured in Clear Timer on Compare Match (CTC) mode (`prescaler = 64`, `OCR0A = 249`) to generate a deterministic 1 kHz interrupt for the PID time delta ($\Delta t$).
* **Hardware Fast PWM Servo Drive (`Timer1`):** Uses 16-bit `Timer1` (`prescaler = 8`, `ICR1 = 39999`) to generate a 50 Hz (20 ms) frame with $0.5\text{ }\mu\text{s}$ resolution, mapping $0^\circ\text{--}180^\circ$ across an extended $544\text{ }\mu\text{s}\text{--}2400\text{ }\mu\text{s}$ ($1088\text{--}4800$ ticks) pulse range.
* **Direct TWI ($I^2C$) IMU Polling:** Communicates with the MPU-6050 at 100 kHz (`TWBR = 72`), waking the sensor via register `0x6B` and reading 6-byte raw accelerometer vectors starting at `0x3B` to compute pitch and roll via `atan2()`.
* **Dual Independent PID Controllers:** Runs two decoupled discrete-time PID control loops in the ZX and ZY planes with software-enforced mechanical travel limits to prevent gimbal ring collisions.

## Hardware & Pin Mapping

| Component | Pin / Terminal | Arduino Pin | Functionality / Register |
| :--- | :--- | :--- | :--- |
| **MPU-6050 IMU** | `VCC` / `GND` | `5V` / `GND` | Internal 5V Logic Rail |
| | `SCL` / `SDA` | `A5` / `A4` | Hardware TWI ($I^2C$ @ 100 kHz) |
| **X-Axis Servo (MG90S)** | Signal (PWM) | `D3` (`PD3`) | Fast PWM (`OCR1A` Compare Match) |
| | `VCC` / `GND` | Ext. 5V / Common GND | Isolated 5V Servo Power Rail |
| **Y-Axis Servo (MG90S)** | Signal (PWM) | `D5` (`PD5`) | Fast PWM (`OCR1B` Compare Match) |
| | `VCC` / `GND` | Ext. 5V / Common GND | Isolated 5V Servo Power Rail |

**Power Isolation Note:** To prevent microcontroller brown-outs during rapid servo torque reversals, the MG90S actuators are powered by an independent 5V supply tied to the Arduino's common ground reference.

## Discrete-Time PID Control Law

Executed every $\Delta t \approx 15\text{ ms}$ (66 Hz), the controller computes corrective gimbal angles using Riemann sum integration and backward finite differences:

$$u[k] = K_p e[k] + K_i \sum_{n=0}^{k} e[n]\Delta t + K_d \frac{e[k] - e[k-1]}{\Delta t}$$

* $K_p$: Proportional gain providing immediate restorative torque proportional to tilt error.
* $K_i$: Integral gain accumulating persistent error over time to eliminate steady-state mechanical bias.
* $K_d$: Derivative gain damping the rate of angular change to prevent overshoot around the $0^\circ$ setpoint.


## Author

Fazakas David-Antal
