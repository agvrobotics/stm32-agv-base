// motor.c
#include "motor.h"
#include "stm32f4xx_hal.h"

void motor_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Configure Direction pins PA4 and PA5 as output push-pull
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. Start PWM on TIM2 Channel 1 for speed control
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // 3. Set initial PWM duty cycle to 0 (motor stopped)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

    // (Optional) You can initialize encoder pins here if needed
}

// motor.c (add this below motor_init)

void motor_set(uint8_t direction, uint16_t speed)
{
    // direction: 0 = stop, 1 = forward, 2 = backward
    // speed: PWM duty cycle (0 to max timer period, e.g., 0-1000)

    switch(direction)
    {
        case 0: // Stop
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            break;

        case 1: // Forward
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
            break;

        case 2: // Backward
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
            break;

        default:
            // Invalid direction: stop motor for safety
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
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
    // Handle encoder interrupts for all motors (example with 4 motors)
    if(GPIO_Pin == frontLeft_motor.ENCA_Pin || GPIO_Pin == frontLeft_motor.ENCB_Pin)
    {
        update_encoder_count(&frontLeft_motor);
    }
    if(GPIO_Pin == frontRight_motor.ENCA_Pin || GPIO_Pin == frontRight_motor.ENCB_Pin)
    {
        update_encoder_count(&frontRight_motor);
    }
    if(GPIO_Pin == backLeft_motor.ENCA_Pin || GPIO_Pin == backLeft_motor.ENCB_Pin)
    {
        update_encoder_count(&backLeft_motor);
    }
    if(GPIO_Pin == backRight_motor.ENCA_Pin || GPIO_Pin == backRight_motor.ENCB_Pin)
    {
        update_encoder_count(&backRight_motor);
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

// Define motor instances
encoded_motor_info frontLeft_motor = {
    .DIR1_Port = GPIOA,
    .DIR1_Pin = GPIO_PIN_4,
    .DIR2_Port = GPIOA,
    .DIR2_Pin = GPIO_PIN_5,
    .htim_pwm = &htim2,
    .pwm_channel = TIM_CHANNEL_1,
    .ENCA_Port = GPIOB,
    .ENCA_Pin = GPIO_PIN_0,
    .ENCB_Port = GPIOB,
    .ENCB_Pin = GPIO_PIN_1,
    .pulses_per_rev = 20.0f,  // Adjust based on your encoder
    .counts_per_rev = 80.0f,  // If using quadrature ×4
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

encoded_motor_info frontRight_motor = { /* duplicate and change pins */ };
encoded_motor_info backLeft_motor   = { /* duplicate and change pins */ };
encoded_motor_info backRight_motor = { /* duplicate and change pins */ };

