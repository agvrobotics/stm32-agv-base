// motor.c
#include "motor.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

char serialBuffer[100];
int serialIndex = 0;
bool commandReady = false;
uint32_t last_serial_command_time = 0;
const uint32_t command_timeout_ms = 1000;

void motor_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // --- Left Motor Direction Pins (PA4, PA5) ---
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- Right Motor Direction Pins (PA6, PA7) ---
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    // Mode, Pull, Speed same as above
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- Start PWM for Left Motor (TIM2 CH1) ---
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

    // --- Start PWM for Right Motor (TIM4 CH1) ---
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

    left_motor.target_rpm = 150;
    right_motor.target_rpm = 150;
    // (Optional) Initialize encoder pins or interrupts here if needed
}


// motor.c (add this below motor_init)

void motor_set(encoded_motor_info *motor, uint8_t direction, uint16_t speed)
{
    // direction: 0 = stop, 1 = forward, 2 = backward
    // speed: PWM duty cycle (0 to max timer period, e.g., 0-1000)

    switch(direction)
    {
        case 0: // Stop
            HAL_GPIO_WritePin(motor->DIR1_Port, motor->DIR1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->DIR2_Port, motor->DIR2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, 0);
            break;

        case 1: // Forward
            HAL_GPIO_WritePin(motor->DIR1_Port, motor->DIR1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->DIR2_Port, motor->DIR2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, speed);
            break;

        case 2: // Backward
            HAL_GPIO_WritePin(motor->DIR1_Port, motor->DIR1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->DIR2_Port, motor->DIR2_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, speed);
            break;

        default:
            // Invalid direction: stop motor for safety
            HAL_GPIO_WritePin(motor->DIR1_Port, motor->DIR1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->DIR2_Port, motor->DIR2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, 0);
            break;
    }
}


// Update encoder count based on new A and B signals
void encoder_update(encoded_motor_info *motor, int A_state, int B_state) {
    int last_A = motor->last_A_state;
    int last_B = motor->last_B_state;

    if (A_state != last_A || B_state != last_B) {
        // Determine direction based on quadrature logic
        if (last_A == B_state) {
            motor->encoder_count++;
        } else {
            motor->encoder_count--;
        }
    }

    motor->last_A_state = A_state;
    motor->last_B_state = B_state;
}

// Calculate RPM based on encoder count difference over time interval
int calculate_rpm(encoded_motor_info *motor) {
    uint32_t current_time_us = HAL_GetTick() * 1000;  // convert ms to us
    uint32_t dt = current_time_us - motor->prev_rpm_calc_time_us;

    if (dt == 0) return motor->current_rpm; // Avoid divide by zero

    long delta_counts = motor->encoder_count - motor->prev_encoder_count;
    float revs = (float)delta_counts / motor->counts_per_rev;

    float rpm = (revs * 60.0f * 1000000.0f) / dt;

    motor->prev_encoder_count = motor->encoder_count;
    motor->prev_rpm_calc_time_us = current_time_us;
    motor->current_rpm = (int)rpm;

    return motor->current_rpm;
}

static void update_encoder_count(encoded_motor_info *motor)
{
    int A_state = HAL_GPIO_ReadPin(motor->ENCA_Port, motor->ENCA_Pin);
    int B_state = HAL_GPIO_ReadPin(motor->ENCB_Port, motor->ENCB_Pin);

    // Quadrature decoding logic based on previous states
    if (A_state != motor->last_A_state) {
        // A changed
        if (A_state == B_state)
            motor->encoder_count++;
        else
            motor->encoder_count--;
    } else if (B_state != motor->last_B_state) {
        // B changed
        if (A_state != B_state)
            motor->encoder_count++;
        else
            motor->encoder_count--;
    }

    motor->last_A_state = A_state;
    motor->last_B_state = B_state;
}

// Override HAL GPIO EXTI callback for encoder pins
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == left_motor.ENCA_Pin || GPIO_Pin == left_motor.ENCB_Pin)
    {
        update_encoder_count(&left_motor);
    }
    if(GPIO_Pin == right_motor.ENCA_Pin || GPIO_Pin == right_motor.ENCB_Pin)
    {
        update_encoder_count(&right_motor);
    }
}


// motor.c

int motor_update_pid(encoded_motor_info* motor)
{
    // 1. Calculate error
    float error = motor->target_rpm - motor->current_rpm;

    // 2. Integral accumulation
    motor->integral += error;

    // Optional: limit integral to prevent wind-up
    if (motor->integral > 1000) motor->integral = 1000;
    if (motor->integral < -1000) motor->integral = -1000;

    // 3. Derivative
    float derivative = error - motor->previous_error;
    motor->previous_error = error;

    // 4. PID formula
    float output = (motor->kp * error) +
                   (motor->ki * motor->integral) +
                   (motor->kd * derivative);

    // 5. Scale and clamp
    motor->pwm_value = (int)(output * motor->PID_scaling_factor);
    if (motor->pwm_value < 0) motor->pwm_value = 0;
    if (motor->pwm_value > 1000) motor->pwm_value = 1000;

    return motor->pwm_value;
}

// --- Step 2: Function to reset PID state (equivalent to Arduino resetPID()) ---
void reset_all_pid() {
    left_motor.integral = right_motor.integral = 0;
    left_motor.previous_error = right_motor.previous_error = 0;
}
// --- Step 3: Functions to parse "p10i0.5d0.1" style commands ---
void update_pid_from_serial(char *cmd) {
    float kp = 0, ki = 0, kd = 0;
    if (sscanf(cmd, "p%fi%fd%f", &kp, &ki, &kd) == 3) {
    	left_motor.kp = kp; left_motor.ki = ki; left_motor.kd = kd;
    	right_motor.kp = kp; right_motor.ki = ki; right_motor.kd = kd;
        reset_all_pid();
        printf("PID updated: kp=%.2f ki=%.2f kd=%.2f\n", kp, ki, kd);
    } else {
        printf("Invalid PID format. Use p10i1d0.1\n");
    }
}

// --- Step 4: Serial parser ---
void handle_serial_input() {
    while (HAL_UART_Receive(&huart2, (uint8_t*)&serialBuffer[serialIndex], 1, 0) == HAL_OK) {
        char c = serialBuffer[serialIndex];
        if (c == '\n' || c == '\r') {
            serialBuffer[serialIndex] = '\0';
            commandReady = true;
            break;
        } else {
            serialIndex++;
            if (serialIndex >= sizeof(serialBuffer) - 1) {
                serialIndex = 0; // overflow safety
            }
        }
    }
}

// --- Step 5: Command dispatcher ---
void process_command() {
    if (!commandReady) return;

    if (serialBuffer[0] == 'p') {
        update_pid_from_serial(serialBuffer); // PID tuning
    } else if (serialBuffer[0] == 'e') {
        printf("Encoders: Left=%ld Right=%ld\n",
               left_motor.encoder_count, right_motor.encoder_count);
    } else {
        printf("Unknown command: %s\n", serialBuffer);
    }

    commandReady = false;
    serialIndex = 0;
    last_serial_command_time = HAL_GetTick();
}



// Define motor instances

encoded_motor_info left_motor = {
    .DIR1_Port = GPIOA,
    .DIR1_Pin = GPIO_PIN_4,       // DIR1 Left motor (PA4)
    .DIR2_Port = GPIOA,
    .DIR2_Pin = GPIO_PIN_5,       // DIR2 Left motor (PA5)
    .htim_pwm = &htim2,
    .pwm_channel = TIM_CHANNEL_1, // PWM Left motor (TIM2_CH1 on PA0)
    .ENCA_Port = GPIOB,
    .ENCA_Pin = GPIO_PIN_0,       // Encoder A Left motor (PB0)
    .ENCB_Port = GPIOB,
    .ENCB_Pin = GPIO_PIN_1,       // Encoder B Left motor (PB1)
    .pulses_per_rev = 20.0f,
    .counts_per_rev = 80.0f,
    .is_quadrature = true,

    .kp = 1.0f,
    .ki = 0.5f,
    .kd = 0.1f,
    .PID_scaling_factor = 1.0f,

    .encoder_count = 0,
    .prev_encoder_count = 0,
    .last_pulse_time_us = 0,
    .last_A_state = 0,
    .last_B_state = 0,
    .prev_rpm_calc_time_us = 0,
    .current_rpm = 0,
    .target_rpm = 0,
    .pwm_value = 0,
    .direction_actual = true,
};

encoded_motor_info right_motor = {
    .DIR1_Port = GPIOA,
    .DIR1_Pin = GPIO_PIN_6,       // DIR1 Right motor (PA6)
    .DIR2_Port = GPIOA,
    .DIR2_Pin = GPIO_PIN_7,       // DIR2 Right motor (PA7)
    .htim_pwm = &htim4,
    .pwm_channel = TIM_CHANNEL_1, // PWM Right motor (TIM4_CH1 on PB6)
    .ENCA_Port = GPIOB,
    .ENCA_Pin = GPIO_PIN_10,      // Encoder A Right motor (PB10)
    .ENCB_Port = GPIOB,
    .ENCB_Pin = GPIO_PIN_8,       // Encoder B Right motor (PB8)
    .pulses_per_rev = 20.0f,
    .counts_per_rev = 80.0f,
    .is_quadrature = true,

    .kp = 1.0f,
    .ki = 0.5f,
    .kd = 0.1f,
    .PID_scaling_factor = 1.0f,

    .encoder_count = 0,
    .prev_encoder_count = 0,
    .last_pulse_time_us = 0,
    .last_A_state = 0,
    .last_B_state = 0,
    .prev_rpm_calc_time_us = 0,
    .current_rpm = 0,
    .target_rpm = 0,
    .pwm_value = 0,
    .direction_actual = true,
};


