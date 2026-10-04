#include <ch.h>
#include <hal.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <stdint.h>

#include "arm_math.h"
#include "audio_processing.h"
#include "main.h"
#include "process_image.h"


#define NOISE_LEVEL_FRONT        7000000  //threshold pour le bruit de fond
#define NOISE_LEVEL_SIDE         7000000

#define MIC_RIGHT                0
#define MIC_LEFT                 1
#define MIC_BACK                 2
#define MIC_FRONT                3

#define NB_MICROPHONES           4
#define CORRELATION_NB_SAMPLES   64
#define MIN_PHASE_SHIFT          -3
#define MAX_PHASE_SHIFT          3

#define FILTER_B0                0.03720f  
#define FILTER_B1                0.0f
#define FILTER_B2               -0.03720f
#define FILTER_A1               -1.8319f
#define FILTER_A2                0.92562f

#define INT16_MAX_FLOAT          32767.0f
#define INT16_MIN_FLOAT         -32768.0f

#define CMD_MEMORY_FACTOR        0.8f
#define CMD_UPDATE_FACTOR        0.2f

#define FORWARD_SPEED_GAIN       700.0f
#define TURN_SPEED_GAIN          250.0f

#define BACK_SOUND_THRESHOLD    -0.3f
#define BACK_TURN_SPEED          350
#define BACK_BASE_SPEED          450


static float cmd_front = 0.0f; //consigne de controle des moteurs responsable de faire avancer le robot
static float cmd_side = 0.0f; //consigne de controle des moteurs responsable de faire tourner le robot


float get_cmd_side(void) {
    return cmd_side;
}


float get_cmd_front(void) {
    return cmd_front;
}


static inline uint32_t pack_two_int16(int16_t lo, int16_t hi) {  // utilise par une instruction DSP pour le produit scalaire de la cross correlation
    return ((uint32_t)(uint16_t)lo) |
           ((uint32_t)(uint16_t)hi << 16);
}


int64_t cross_correlation_mic(int16_t *signals,
                              uint16_t signal_size,
                              int8_t phase,
                              int8_t direction1,
                              int8_t direction2) {
    //int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
    //so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]

    int64_t sum = 0;

    for (int i = 0; i < CORRELATION_NB_SAMPLES; i += 2) {
        uint32_t a_pack = pack_two_int16(
            signals[signal_size / 2 + direction1 + NB_MICROPHONES * i],
            signals[signal_size / 2 + direction1 + NB_MICROPHONES * (i + 1)]
        );

        uint32_t b_pack = pack_two_int16(
            signals[signal_size / 2 + direction2 + NB_MICROPHONES * i + NB_MICROPHONES * phase],
            signals[signal_size / 2 + direction2 + NB_MICROPHONES * (i + 1) + NB_MICROPHONES * phase]
        );

        sum = __SMLALD(a_pack, b_pack, sum);   // sum = a_pack_low + b_back_low + a_pack_high + b_back_high
    }

    return sum;
}

//Biquad Filter design
typedef struct {
    float x1, x2;
    float y1, y2;
} BiquadState;


// one filter state per mic
static BiquadState bp_right = {0};
static BiquadState bp_left  = {0};
static BiquadState bp_back  = {0};
static BiquadState bp_front = {0};


// Band-pass around 800 Hz, fs = 16000 Hz
static inline float bandpass_800hz_step(BiquadState *s, float x) {   //y[n] + a1y[n-1] + a2y[n-2] = b0x[n] + b1x[n-1] + b2x[n-2], filtre passe bande 2eme ordre
    float y = FILTER_B0 * x
            + FILTER_B1 * s->x1
            + FILTER_B2 * s->x2
            - FILTER_A1 * s->y1
            - FILTER_A2 * s->y2;

    s->x2 = s->x1;
    s->x1 = x;
    s->y2 = s->y1;
    s->y1 = y;

    return y;
}


// clip float to int16_t pour eviter l'overflow
static inline int16_t float_to_int16(float x) {
    if (x > INT16_MAX_FLOAT) {
        return INT16_MAX;
    }

    if (x < INT16_MIN_FLOAT) {
        return INT16_MIN;
    }

    return (int16_t)x;
}


// data = [R1, L1, B1, F1, R2, L2, B2, F2, ...]
// num_samples = total number of int16_t values in data

void bandpass_filter_all_mics(int16_t *data, uint16_t num_samples) {  // filtrer les 4 micros avec un filtre passbande autour de 800hz, pour eliminier le bruit de fond
    for (uint16_t i = 0; i + (NB_MICROPHONES - 1) < num_samples; i += NB_MICROPHONES) {
        float xr = (float)data[i + MIC_RIGHT];
        float xl = (float)data[i + MIC_LEFT];
        float xb = (float)data[i + MIC_BACK];
        float xf = (float)data[i + MIC_FRONT];

        float yr = bandpass_800hz_step(&bp_right, xr);
        float yl = bandpass_800hz_step(&bp_left,  xl);
        float yb = bandpass_800hz_step(&bp_back,  xb);
        float yf = bandpass_800hz_step(&bp_front, xf);

        data[i + MIC_RIGHT] = float_to_int16(yr);
        data[i + MIC_LEFT]  = float_to_int16(yl);
        data[i + MIC_BACK]  = float_to_int16(yb);
        data[i + MIC_FRONT] = float_to_int16(yf);
    }
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/


void processAudioData(int16_t *data, uint16_t num_samples) {

    bandpass_filter_all_mics(data, num_samples);

    int64_t max_cross_corr_front = INT64_MIN;	//-infinity, plus petite valeur possible dans un int 64
    int64_t max_cross_corr_side = INT64_MIN;	//-infinity, plus petite valeur possible dans un int 64
    int8_t corresponding_delay_front = 0; // dephasage pour les micros devant arriere
    int8_t corresponding_delay_side = 0; // dephasage pour les micros gauche droite

    //trouver le dephasage qui maximise la corss correlation:
    for (int8_t k = MIN_PHASE_SHIFT; k <= MAX_PHASE_SHIFT; k++) {
        //front and back mics
        int64_t cross_k = cross_correlation_mic(data, num_samples, k, MIC_FRONT, MIC_BACK);

        if (cross_k > max_cross_corr_front) {
            max_cross_corr_front = cross_k;
            corresponding_delay_front = k;
        }

        //left and right mics
        cross_k = cross_correlation_mic(data, num_samples, k, MIC_RIGHT, MIC_LEFT);

        if (cross_k > max_cross_corr_side) {
            max_cross_corr_side = cross_k;
            corresponding_delay_side = k;
        }
    }

    int8_t direction_avant = 0;  // = signe(dephasage entre les micros avant arriere) , si >0 alors avance  vers devant

    // validity test: Intensite doit etre suffisament grand, pour eliminer le bruit de fond
    bool valid_front = (max_cross_corr_front > NOISE_LEVEL_FRONT);
    bool valid_side = (max_cross_corr_side > NOISE_LEVEL_SIDE);

    if (valid_front) {
        if (corresponding_delay_front > 0) {
            direction_avant = 1;
        } else if (corresponding_delay_front < 0) {
            direction_avant = -1;
        } else {
            direction_avant = 0;
        }
    } else {
        direction_avant = 0;
    }

    // Pour eviter un mouvement irregulier du robot on moyenne les valeurs obtenues (ie : sinon son mouvement ne serait pas parfaitement continu)
    cmd_front = CMD_MEMORY_FACTOR * cmd_front + CMD_UPDATE_FACTOR * direction_avant;

    if (valid_side) {
        cmd_side = CMD_MEMORY_FACTOR * cmd_side + CMD_UPDATE_FACTOR * corresponding_delay_side;
    } else {
        cmd_side = CMD_MEMORY_FACTOR * cmd_side;
    }

    int16_t base = 0;

    //cas ou le son vient de l'avant du robot
    if (cmd_front > 0) {
        base = (int16_t)(FORWARD_SPEED_GAIN * cmd_front);
    }

    int16_t turn = (int16_t)(TURN_SPEED_GAIN * cmd_side);

    //cas ou le son vient de l'avant du robot: le robot fait demi-tour
    if (cmd_front < BACK_SOUND_THRESHOLD) {
        if (cmd_side > 0) {
            turn = BACK_TURN_SPEED;
        } else {
            turn = -BACK_TURN_SPEED;
        }

        base = BACK_BASE_SPEED;
    }

    //si un obstacle n'est pas en face , alors le robot suit le son
    if (!get_obstacle_mode() && !get_detection_rouge()) {
        left_motor_set_speed(base + turn);
        right_motor_set_speed(base - turn);
    }
}