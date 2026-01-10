#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#define MOTOR_1_EN_L 2
#define MOTOR_1_EN_R 4
#define MOTOR_1_PWM_L 5
#define MOTOR_1_PWM_R 6

#define MOTOR_2_PWM_R 10
#define MOTOR_2_PWM_L 9
#define MOTOR_2_EN_L A1
#define MOTOR_2_EN_R A2
void motor_1_setup(int status);

void motor_2_setup(int status);

void drive(int8_t speed, int8_t steering);
#endif