#pragma once
void payload_release_servo_setup(int pin);
void paraglider_servo_setup(int pin_l, int pin_r);
void egg_release_servo_setup(int pin);
void payload_release_actuate();
void egg_release_actuate();
void update_paraglider_steering(float yaw_rate_dps);
