#include <ch.h>
#include <hal.h>
#include <motors.h>
#include <sensors/proximity.h>
#include "leds.h"
#include "motors1.h"
#include "proximity1.h"




void turn_left(uint16_t angle) {
    angle = DEGREE_TO_STEPS * angle;    // conversion de degree a steps
    
    int32_t left_start = left_motor_get_pos();
    int32_t right_start = right_motor_get_pos();

    left_motor_set_speed(-SPEED);
    right_motor_set_speed(SPEED);

    while (1) {
        int32_t left_now = left_motor_get_pos();
        int32_t right_now = right_motor_get_pos();

        int32_t left_diff = left_start - left_now;
        int32_t right_diff = right_now - right_start;

        if (left_diff >= angle && right_diff >= angle) {
            break;
        }

        chThdSleepMilliseconds(SMALL_SLEEP_MS);
    }

    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

   

void turn_right(uint16_t angle) {
    angle = DEGREE_TO_STEPS * angle;
    
    int32_t left_start = left_motor_get_pos();
    int32_t right_start = right_motor_get_pos();

    left_motor_set_speed(SPEED);
    right_motor_set_speed(-SPEED);

    while (1) {
        int32_t left_now = left_motor_get_pos();
        int32_t right_now = right_motor_get_pos();

        int32_t left_diff = left_now - left_start;
        int32_t right_diff = right_start - right_now; 


        if (left_diff >= angle && right_diff >= angle) {
            break;
        }

        chThdSleepMilliseconds(SMALL_SLEEP_MS);
    }

    left_motor_set_speed(0);
    right_motor_set_speed(0);

}


void move_forward_extra(int32_t position) {     //moves for a certain distance, good for taking
    int32_t left_start = left_motor_get_pos();  //extra steps after obstacle deflection
    int32_t right_start = right_motor_get_pos();

    left_motor_set_speed(SPEED);
    right_motor_set_speed(SPEED);

    while (1) {
        int32_t left_now = left_motor_get_pos();
        int32_t right_now = right_motor_get_pos();

        if(get_sensor(FRONT_LEFT) < DISTANCE_VERY_CLOSE || get_sensor(FRONT_RIGHT) < DISTANCE_VERY_CLOSE || 
        get_sensor(FRONT_LEFT_45) < DISTANCE_VERY_CLOSE || get_sensor(FRONT_RIGHT_45) < DISTANCE_VERY_CLOSE){
            break;
        }

        if ((left_now - left_start) >= position &&
            (right_now - right_start) >= position) {
            break;
        }

        chThdSleepMilliseconds(SMALL_SLEEP_MS);
    }

    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

void move_backward_extra(int32_t position) {    // moves backward mainly for when obstacle deflection
    int32_t left_start = left_motor_get_pos();  // is not possible
    int32_t right_start = right_motor_get_pos();

    left_motor_set_speed(-SPEED);
    right_motor_set_speed(-SPEED);

    while (1) {
        int32_t left_now = left_motor_get_pos();
        int32_t right_now = right_motor_get_pos();

        if ((left_start - left_now) >= position &&
            (right_start - right_now) >= position) {
            break;
        }

        chThdSleepMilliseconds(SMALL_SLEEP_MS);
    }

    left_motor_set_speed(0);
    right_motor_set_speed(0);
}


