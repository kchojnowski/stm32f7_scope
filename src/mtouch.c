/**
 ******************************************************************************
 * @file    src/mtouch.c
 * @author  Krzysztof Chojnowski
 * @brief   Simple multi-touch gesture recognition library
 ******************************************************************************
 **/

#include "mtouch.h"

static MTOUCH_TouchData_s touchDataPrev;
static MTOUCH_TouchData_s touchDataCurr;
static uint16_t origin[2];

static inline void MTOUCH_CalculateDelta(uint16_t x1, uint16_t x2,
		int16_t* delta, uint16_t* deltaAbs) {
	*delta = x1 - x2;
	*deltaAbs = *delta > 0 ? *delta : *delta * -1;
}

MTOUCH_GestureID MTOUCH_DecodeGesture_1Point() {
	int16_t dx, dy;
	uint16_t dxAbs, dyAbs;

	MTOUCH_CalculateDelta(touchDataCurr.x[0], touchDataPrev.x[0], &dx, &dxAbs);
	MTOUCH_CalculateDelta(touchDataCurr.y[0], touchDataPrev.y[0], &dy, &dyAbs);

	if (dxAbs > dyAbs) {
		if (dxAbs < MTOUCH_GESTURE_INERTIA)
			return TOUCH;
		else if (dx > 0)
			return MOVE_RIGHT;
		else
			return MOVE_LEFT;
	} else {
		if (dyAbs < MTOUCH_GESTURE_INERTIA)
			return TOUCH;
		else if (dy > 0)
			return MOVE_DOWN;
		else
			return MOVE_UP;
	}
}

MTOUCH_GestureID MTOUCH_DecodeGesture_2Point() {
	int16_t dxPrev, dxCurr, dyPrev, dyCurr, dx, dy;
	uint16_t dxPrevAbs, dxCurrAbs, dyPrevAbs, dyCurrAbs, dxAbs, dyAbs;

	MTOUCH_CalculateDelta(touchDataPrev.x[1], touchDataPrev.x[0], &dxPrev, &dxPrevAbs);
	MTOUCH_CalculateDelta(touchDataCurr.x[1], touchDataCurr.x[0], &dxCurr, &dxCurrAbs);
	MTOUCH_CalculateDelta(touchDataPrev.y[1], touchDataPrev.y[0], &dyPrev, &dyPrevAbs);
	MTOUCH_CalculateDelta(touchDataCurr.y[1], touchDataCurr.y[0], &dyCurr, &dyCurrAbs);

	MTOUCH_CalculateDelta(dxCurrAbs, dxPrevAbs, &dx, &dxAbs);
	MTOUCH_CalculateDelta(dyCurrAbs, dyPrevAbs, &dy, &dyAbs);

	if (dxAbs > dyAbs) {
		if (dxAbs < MTOUCH_GESTURE_INERTIA)
			return TOUCH;
		else if (dx > 0)
			return ZOOM_IN_X;
		else
			return ZOOM_OUT_X;
	} else {
		if (dyAbs < MTOUCH_GESTURE_INERTIA)
			return TOUCH;
		else if (dy > 0)
			return ZOOM_IN_Y;
		else
			return ZOOM_OUT_Y;
	}
}

static void MTOUCH_SetGestureOrigin(MTOUCH_GestureData_p gestureData) {
	gestureData->origin_x = origin[0];
	gestureData->origin_y = origin[1];
}

void MTOUCH_GetGesture(MTOUCH_GestureData_p gestureData) {

	if (touchDataCurr.points == 0) {
		gestureData->gesture = NONE;
	} else if (touchDataCurr.points != touchDataPrev.points) {
		if (touchDataCurr.points == 1) {
			origin[0] = touchDataCurr.x[0];
			origin[1] = touchDataCurr.y[0];
		} else if (touchDataCurr.points == 2) {
			origin[0] = (touchDataCurr.x[0] + touchDataCurr.x[1]) / 2;
			origin[1] = (touchDataCurr.y[0] + touchDataCurr.y[1]) / 2;
		}
		gestureData->gesture = TOUCH;
		MTOUCH_SetGestureOrigin(gestureData);
	} else if (touchDataCurr.points == 1) {
		gestureData->gesture = MTOUCH_DecodeGesture_1Point();
		MTOUCH_SetGestureOrigin(gestureData);
	} else if (touchDataCurr.points == 2) {
		gestureData->gesture = MTOUCH_DecodeGesture_2Point();
		MTOUCH_SetGestureOrigin(gestureData);
	} else {
		gestureData->gesture = NONE;
	}
}

void MTOUCH_AddTouchData(MTOUCH_TouchData_p touchData) {
	touchDataPrev = touchDataCurr;
	touchDataCurr = *touchData;
}

