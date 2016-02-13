#ifndef MTOUCH_H
#define MTOUCH_H

#include <stdint.h>

#define MTOUCH_GESTURE_INERTIA 2

typedef enum {
	NONE,
	TOUCH,
	MOVE_UP,
	MOVE_DOWN,
	MOVE_LEFT,
	MOVE_RIGHT,
	ZOOM_IN_X,
	ZOOM_OUT_X,
	ZOOM_IN_Y,
	ZOOM_OUT_Y,
} MTOUCH_GestureID;

typedef struct {
	uint8_t points;
	uint16_t x[2];
	uint16_t y[2];
} MTOUCH_TouchData_s;

typedef MTOUCH_TouchData_s* MTOUCH_TouchData_p;

typedef struct {
	MTOUCH_GestureID gesture;
	uint16_t origin_x;
	uint16_t origin_y;
} MTOUCH_GestureData_s;

typedef MTOUCH_GestureData_s* MTOUCH_GestureData_p;

void MTOUCH_AddTouchData(MTOUCH_TouchData_p touchData);
void MTOUCH_GetGesture(MTOUCH_GestureData_p gestureData);

#endif /* MTOUCH_H */
