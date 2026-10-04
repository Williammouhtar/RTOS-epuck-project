#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <camera/po8030.h>
#include <motors.h>

#include "process_image.h"
#include "main.h"

#define USED_LINE 				200   // Must be inside [0..478]
#define IMAGE_BUFFER_SIZE		640
#define RED_RATIO               100
#define RED_WAIT_MS				30
#define RED_MARGIN				20
#define MIN_RED					80

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the lines USED_LINE and USED_LINE + 1 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, USED_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

static bool detection_rouge = false; // condition d'arret par detection	du rouge

bool get_detection_rouge(){
    return detection_rouge;
}



static THD_WORKING_AREA(waProcessImage, 512);
static THD_FUNCTION(ProcessImage, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t *img_buff_ptr;

    while(1){
        chBSemWait(&image_ready_sem);	// venant du CaptureImage thread
        img_buff_ptr = dcmi_get_last_image_ptr();
        
        uint32_t sum_red = 0;	// counts number of red pixels in a line

		for(uint16_t i = 0; i < 2 * IMAGE_BUFFER_SIZE; i += 2){ // for every other image for less space consumption
			uint8_t r = img_buff_ptr[i] & 0xF8; // color red
			uint8_t g = ((img_buff_ptr[i] & 0x07) << 5) | ((img_buff_ptr[i+1] & 0xE0) >> 3); // color green
			uint8_t b = (img_buff_ptr[i+1] & 0x1F) << 3; // color blue

			if ((r > MIN_RED) && (r > g + RED_MARGIN) && (r > b + RED_MARGIN)) {
				sum_red++;	// if red is the dominating color in a pixel, sum of red pixels increases
			}
		}
	if (sum_red > RED_RATIO){
		detection_rouge = true; // stop condition in main with get_detection_rouge()
	}
	
	else{
		detection_rouge = false;
	}
	chThdSleepMilliseconds(RED_WAIT_MS);

	}
	
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

