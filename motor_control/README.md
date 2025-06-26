## STM32F411 Blackpill - Motor Control `.ioc` Configuration

This project uses STM32CubeMX to configure motor control with PWM, encoder inputs, UART communication, and a buzzer output for a **2-motor differential drive AGV**.

---

### Pin Configuration

#### Motor A (Left Motor)

| Pin     | Function                       | Purpose                     |
| ------- | ------------------------------ | --------------------------- |
| **PA0** | TIM2\_CH1 (Alternate Function) | PWM output to control speed |
| **PA4** | GPIO Output                    | Motor direction pin DIR1    |
| **PA5** | GPIO Output                    | Motor direction pin DIR2    |
| **PB0** | GPIO Input with EXTI interrupt | Encoder signal A (ENCA)     |
| **PB1** | GPIO Input with EXTI interrupt | Encoder signal B (ENCB)     |

#### Motor B (Right Motor)

| Pin      | Function                       | Purpose                     |
| -------- | ------------------------------ | --------------------------- |
| **PB6**  | TIM4\_CH1 (Alternate Function) | PWM output to control speed |
| **PA6**  | GPIO Output                    | Motor direction pin DIR1    |
| **PA7**  | GPIO Output                    | Motor direction pin DIR2    |
| **PB10** | GPIO Input with EXTI interrupt | Encoder signal A (ENCA)     |
| **PB8**  | GPIO Input with EXTI interrupt | Encoder signal B (ENCB)     |

#### UART + Buzzer

| Pin      | Function    | Purpose                    |
| -------- | ----------- | -------------------------- |
| **PA2**  | USART2\_TX  | Serial communication (TX)  |
| **PA3**  | USART2\_RX  | Serial communication (RX)  |
| **PB13** | GPIO Output | Buzzer or status indicator |

---

### Peripheral Configuration

#### TIM2 - PWM (Motor A)

* **Channel**: CH1 (linked to **PA0**)
* **Mode**: PWM Generation CH1
* **Purpose**: Controls speed of Motor A

#### TIM4 - PWM (Motor B)

* **Channel**: CH1 (linked to **PB6**)
* **Mode**: PWM Generation CH1
* **Purpose**: Controls speed of Motor B

#### USART2 - UART Communication

* **Mode**: Asynchronous
* **Baud Rate**: 115200
* **Pins**: PA2 (TX), PA3 (RX)

#### EXTI - Encoder Interrupts

* Enable EXTI interrupts for:

  * **PB0** → EXTI0\_IRQn
  * **PB1** → EXTI1\_IRQn
  * **PB10** → EXTI15\_10\_IRQn
  * **PB8** → EXTI9\_5\_IRQn

* Purpose: Track encoder pulses in real time

---

### Code Generator Settings

* Enable:

  * **Generate peripheral initialization as a pair of `.c/.h` files per peripheral**
  * **Keep user code when re-generating**
