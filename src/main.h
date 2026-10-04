#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* The following includes and declarations are necessary as this main overrides the main of the library,
 and some files in the library require these includes/declarations to be in the main.h. */

#include <camera/dcmi_camera.h>
#include <msgbus/messagebus.h>
#include <parameter/parameter.h>

bool get_obstacle_mode(void);
void set_obstacle_mode(bool val);

#define USED_RGB_LED            LED4
#define INTENSITY_RGB_LED       10

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
