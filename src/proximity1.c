#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <sensors/proximity.h>
#include "proximity1.h"
#include <main.h>

#define IR_CONVERTER_FACTOR 8200

int32_t IR_converter(uint32_t distance){
	return (IR_CONVERTER_FACTOR/distance) ;		//valeur trouve experimentallement
}

int32_t get_sensor(int8_t corresponding_sensor){	//return the distance of a IR proximity sensor in millimeters
	uint32_t distance_sensor = get_calibrated_prox(corresponding_sensor);
	distance_sensor = IR_converter(distance_sensor);
	return distance_sensor;
}


