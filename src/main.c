/**
 * Authors: Marc Aboujaoude, William El Mouhtar
 */

#include <stdbool.h>
#include <stdint.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>

#include <motors.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>

#include "main.h"
#include "audio_processing.h"
#include "motors1.h"
#include "proximity1.h"
#include "process_image.h"


/* Control constants */
#define WALL_FOLLOWING_KP           17
#define BASE_SPEED                  600
#define ERROR_TOLERANCE             3
#define DISTANCE_TO_WALL            70

/* Movement constants */
#define AVOIDANCE_TURN_ANGLE        120
#define ESCAPE_TURN_ANGLE           135
#define FRONT_CORRECTION_ANGLE      10

/* Timing constants */
#define MAIN_LOOP_DELAY_MS          30
#define RED_STOP_DELAY_MS           100

/* Audio command thresholds used to leave obstacle-avoidance mode */
#define CMD_SIDE_THRESHOLD          1.5f
#define CMD_FRONT_THRESHOLD         0.3f
#define CMD_SIDE_ASSIST_THRESHOLD   0.5f


static bool obstacle_mode = false;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


/* Static function declarations */


static void serial_start(void);
static void system_start(void);
static void stop_robot(void);

static bool obstacle_on_left(void);
static bool obstacle_on_right(void);
static bool blocked_front_turn_right(void);
static bool blocked_front_turn_left(void);

static void avoid_obstacle_from_left(void);
static void avoid_obstacle_from_right(void);
static void escape_by_turning_right(void);
static void escape_by_turning_left(void);

static bool should_leave_left_obstacle_mode(void);
static bool should_leave_right_obstacle_mode(void);


/* Obstacle mode accessors*/

bool get_obstacle_mode(void) {
    return obstacle_mode;
}

void set_obstacle_mode(bool val) {
    obstacle_mode = val;
}

/* Initialization */

static void serial_start(void) {
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg);
}


static void system_start(void) {
    halInit();
    chSysInit();
    mpu_init();

    serial_start();
    usb_start();

    dcmi_start();
    po8030_start();

    motors_init();

    mic_start(&processAudioData);

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    proximity_start();
    calibrate_ir();
    process_image_start();
}


static void stop_robot(void) {
    right_motor_set_speed(0);
    left_motor_set_speed(0);
}

/*
 * Leave left-obstacle avoidance when the audio command indicates that the
 * target is sufficiently to the right or forward-right.
 */

static bool should_leave_left_obstacle_mode(void) {
    return (get_cmd_side() > CMD_SIDE_THRESHOLD) ||
           ((get_cmd_front() > CMD_FRONT_THRESHOLD) &&
            (get_cmd_side() > CMD_SIDE_ASSIST_THRESHOLD));
}

/*
 * Leave right-obstacle avoidance when the audio command indicates that the
 * target is sufficiently to the left or forward-left.
 */
static bool should_leave_right_obstacle_mode(void) {
    return (get_cmd_side() < -CMD_SIDE_THRESHOLD) ||
           ((get_cmd_front() > CMD_FRONT_THRESHOLD) &&
            (get_cmd_side() < -CMD_SIDE_ASSIST_THRESHOLD));
}

/* Obstacle detection conditions */

static bool obstacle_on_left(void) {   //cas a et b dans le rapport
    return (get_sensor(FRONT_LEFT) < DISTANCE_CLOSE ||
            get_sensor(FRONT_LEFT_45) < DISTANCE_FAIRLY_CLOSE) &&
           get_sensor(FRONT_RIGHT_45) > DISTANCE_CLOSE &&
           get_sensor(RIGHT) > DISTANCE_LONG;
}


static bool obstacle_on_right(void) {   //cas a et b dans le rapport
    return (get_sensor(FRONT_RIGHT) < DISTANCE_CLOSE ||
            get_sensor(FRONT_RIGHT_45) < DISTANCE_FAIRLY_CLOSE) &&
           get_sensor(FRONT_LEFT_45) > DISTANCE_CLOSE &&
           get_sensor(LEFT) > DISTANCE_LONG;
}


static bool blocked_front_turn_right(void) {  //cas c, d et e dans le rapport
    return (get_sensor(FRONT_LEFT_45) < DISTANCE_CLOSE &&
            get_sensor(FRONT_RIGHT_45) < DISTANCE_CLOSE) ||

           (get_sensor(FRONT_LEFT_45) < DISTANCE_CLOSE &&
            get_sensor(FRONT_RIGHT) < DISTANCE_CLOSE &&
            get_sensor(FRONT_LEFT) > DISTANCE_LONG) ||

           (get_sensor(FRONT_LEFT_45) < DISTANCE_VERY_CLOSE &&
            get_sensor(RIGHT) < DISTANCE_VERY_CLOSE);
}


static bool blocked_front_turn_left(void) {  //cas c, d et e dans le rapport
    return (get_sensor(FRONT_RIGHT_45) < DISTANCE_CLOSE &&
            get_sensor(FRONT_LEFT_45) < DISTANCE_CLOSE) ||

           (get_sensor(FRONT_RIGHT_45) < DISTANCE_CLOSE &&
            get_sensor(FRONT_LEFT) < DISTANCE_CLOSE &&
            get_sensor(FRONT_RIGHT) > DISTANCE_LONG) ||

           (get_sensor(FRONT_RIGHT_45) < DISTANCE_VERY_CLOSE &&
            get_sensor(LEFT) < DISTANCE_VERY_CLOSE);
}



/* Obstacle avoidance actions */

/*
 * Avoid an obstacle detected on the left side.
 *
 * The robot first turns right until the front-left sensors are clear. Then it
 * follows the obstacle boundary until the audio command indicates that it can
 * leave obstacle-avoidance mode, or when it finished traversing the obstacle.
 */

static void avoid_obstacle_from_left(void) {
    set_obstacle_mode(true);

    while (get_sensor(FRONT_LEFT) < DISTANCE_MEDIUM ||
           get_sensor(FRONT_LEFT_45) < DISTANCE_CLOSE) {
        turn_right(EPSILON_ANGLE);
    }

    while (get_sensor(LEFT) < DISTANCE_LONG &&
           get_sensor(FRONT_LEFT_45) < DISTANCE_LONG) {

        int16_t err = get_sensor(FRONT_LEFT_45) - (DISTANCE_TO_WALL);

        if (err > ERROR_TOLERANCE || err < -ERROR_TOLERANCE) {
            left_motor_set_speed(BASE_SPEED - WALL_FOLLOWING_KP * err);
            right_motor_set_speed(BASE_SPEED + WALL_FOLLOWING_KP * err);
        } else {
            left_motor_set_speed(BASE_SPEED);
            right_motor_set_speed(BASE_SPEED);
        }

        if (get_sensor(FRONT_RIGHT_45) < DISTANCE_VERY_CLOSE) {
            turn_right(AVOIDANCE_TURN_ANGLE);
        }

        if (get_sensor(FRONT_RIGHT) < DISTANCE_CLOSE) {
            turn_right(FRONT_CORRECTION_ANGLE);
            move_forward_extra(SMALL_STEP);
        }

        if (should_leave_left_obstacle_mode()) {
            set_obstacle_mode(false);
            break;
        }
        chThdSleepMilliseconds(SMALL_SLEEP_MS);
    }

    if (get_obstacle_mode()) {
        move_forward_extra(BIG_STEP);
        set_obstacle_mode(false);
    }
}

/*
 * Avoid an obstacle detected on the right side.
 *
 * This behavior is symmetrical to avoid_obstacle_from_left().
 */

static void avoid_obstacle_from_right(void) {
    set_obstacle_mode(true);

    while (get_sensor(FRONT_RIGHT) < DISTANCE_MEDIUM ||
           get_sensor(FRONT_RIGHT_45) < DISTANCE_CLOSE) {
        turn_left(EPSILON_ANGLE);
    }

    while (get_sensor(RIGHT) < DISTANCE_LONG &&
           get_sensor(FRONT_RIGHT_45) < DISTANCE_LONG) {

        int16_t err = get_sensor(FRONT_RIGHT_45) - DISTANCE_TO_WALL;

        if (err > ERROR_TOLERANCE || err < -ERROR_TOLERANCE) {
            left_motor_set_speed(BASE_SPEED + WALL_FOLLOWING_KP * err);
            right_motor_set_speed(BASE_SPEED - WALL_FOLLOWING_KP * err);
        } else {
            left_motor_set_speed(BASE_SPEED);
            right_motor_set_speed(BASE_SPEED);
        }

        if (get_sensor(FRONT_LEFT_45) < DISTANCE_VERY_CLOSE) {
            turn_left(AVOIDANCE_TURN_ANGLE);
        }

        if (get_sensor(FRONT_LEFT) < DISTANCE_CLOSE) {
            turn_left(FRONT_CORRECTION_ANGLE);
            move_forward_extra(SMALL_STEP);
        }

        if (should_leave_right_obstacle_mode()) {
            set_obstacle_mode(false);
            break;
        }
        chThdSleepMilliseconds(SMALL_SLEEP_MS);
    }

    if (get_obstacle_mode()) {
        move_forward_extra(BIG_STEP);
        set_obstacle_mode(false);
    }
}


/*
 * Escape from a blocked-front situation by moving backward, turning right,
 * then moving forward again.  cases c, d and e in the report
 */
static void escape_by_turning_right(void) {
    set_obstacle_mode(true);

    move_backward_extra(BIG_STEP);

    turn_right(ESCAPE_TURN_ANGLE);
    move_forward_extra(BIG_STEP);

    set_obstacle_mode(false);
}


/*
 * Escape from a blocked-front situation by moving backward, turning left,
 * then moving forward again. cases c, d and e in the report
 */
static void escape_by_turning_left(void) {
    set_obstacle_mode(true);

    move_backward_extra(BIG_STEP);


    turn_left(ESCAPE_TURN_ANGLE);
    move_forward_extra(BIG_STEP);

    set_obstacle_mode(false);
}


int main(void) {
    system_start();

    while (1) {

        if (get_detection_rouge()) {
            stop_robot();
            chThdSleepMilliseconds(RED_STOP_DELAY_MS);
            continue;
        }

        if (obstacle_on_left()) {
            avoid_obstacle_from_left();
        }

        else if (obstacle_on_right()) {
            avoid_obstacle_from_right();
        }

        else if (blocked_front_turn_right()) {
            escape_by_turning_right();
        }

        else if (blocked_front_turn_left()) {
            escape_by_turning_left();
        }

        chThdSleepMilliseconds(MAIN_LOOP_DELAY_MS);
    }
}

#define STACK_CHK_GUARD 0xe2dee396

uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}