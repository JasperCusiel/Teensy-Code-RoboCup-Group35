//
// Created by Jasper Cusiel on 20/07/2026.
//

#ifndef ROBOCUP_SMART_SERVO_H
#define ROBOCUP_SMART_SERVO_H

// Initalizes servos and checks they start with no errors.
bool smart_servo_init();
void set_front_servo_up();
void set_front_servo_down();
void set_back_servo_up();
void set_back_servo_down();
bool is_front_servo_in_position();
bool is_back_servo_in_position();


#endif // ROBOCUP_SMART_SERVO_H
