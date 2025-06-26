#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

char serialBuffer[100];
uint8_t serialIndex = 0;
bool commandReady = false;
uint32_t last_serial_command_time = 0;
uint32_t command_timeout_ms = 1000;
void motor_init(void);
void motor_set(uint8_t direction, uint16_t speed);
void encoder_update(encoded_motor_info *motor, int A_state, int B_state);
int calculate_rpm(encoded_motor_info *motor);
int motor_update_pid(encoded_motor_info* motor);

typedef struct {
    // Pin configuration
    GPIO_TypeDef* DIR1_Port;
    uint16_t DIR1_Pin;

    GPIO_TypeDef* DIR2_Port;
    uint16_t DIR2_Pin;

    TIM_HandleTypeDef* htim_pwm;  // PWM Timer
    uint32_t pwm_channel;

    GPIO_TypeDef* ENCA_Port;
    uint16_t ENCA_Pin;
    GPIO_TypeDef* ENCB_Port;
    uint16_t ENCB_Pin;

    // Encoder configuration
    float pulses_per_rev;
    float counts_per_rev;
    bool is_quadrature;

    // PID parameters
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float PID_scaling_factor;

    // Runtime values
    volatile long encoder_count;
    volatile uint32_t last_pulse_time_us;
    volatile int last_A_state;
    volatile int last_B_state;

    long prev_encoder_count;
    uint32_t prev_rpm_calc_time_us;

    int current_rpm;
    int target_rpm;
    int pwm_value;
    bool direction_actual;
} encoded_motor_info;

// Declare 2 motor objects for your AGV
extern encoded_motor_info left_motor;
extern encoded_motor_info right_motor;

#endif // MOTOR_H
