/*
 * BallBalancing - Copyright (C) 2013 Luca D'Onofrio.
 *
 * This file is part of BallBalancing Project
 *
 * BallBalancing is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * BallBalancing is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This firmware is based on ChibiOS-RT 2.5.0, see <http://www.chibios.org>.
 */

/**
 * @file    main.c
 * @brief   Main servo control algorithms for Ball Balancing System.
 *
 * @addtogroup BallBalancing
 * @{
 */

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/

#include "ch.h"
#include "hal.h"
#include "serial.h"

/*===========================================================================*/
/* Variables.                                                                */
/*===========================================================================*/

/**
 * @brief 	PWM configuration structure.
 * @details Channels 3 and 4 enabled without callbacks, the active state is
 * 			logic high.
 */
static PWMConfig pwmcfg = {
		1000000, /* 10KHz PWM clock frequency.   */
		20000, /* PWM period 20ms (in ticks).    */
		NULL,
		{
			{ PWM_OUTPUT_ACTIVE_LOW, NULL },
			{ PWM_OUTPUT_ACTIVE_LOW, NULL },
			{ PWM_OUTPUT_ACTIVE_HIGH, NULL },
			{ PWM_OUTPUT_ACTIVE_HIGH, NULL }
		},
		/* HW dependent part.*/
		0,
#if STM32_PWM_USE_ADVANCED
		0
#endif
};

/*===========================================================================*/
/* Defines and macros.                                                       */
/*===========================================================================*/

/**
 * @brief	HS325HB servo min pulse.
 */
#define MIN_SERVO  700

/**
 * @brief	HS325HB servo max pulse.
 */
#define MAX_SERVO  2400

/**
 * @brief	HS325HB servo max absolute angle
 */
#define MAX_ANGLE 	90

/**
 * @brief		Scale an input value within a given range.
 *
 * @param[in]	in		the value to scale.
 * @param[in]	inMin	the lower bound of input value.
 * @param[in]	inMax	the upper bound of input value.
 * @param[in]	outMin	the lower bound of output value.
 * @param[in]	outMax	the upper bound of utput value.
 *
 * @return		the scaled value.
 */
#define map(in, inMin, inMax, outMin, outMax) \
	(((float)in - (float)inMin) / \
	((float)inMax - (float)inMin) * \
	((float)outMax - (float)outMin) + (float)outMin)


/*===========================================================================*/
/* Internal Functions.                                                       */
/*===========================================================================*/

/**
 * @brief 		Rotate a servo with the given angle
 *
 * @param[in]	servo	the servo to command (0 or 1).
 * @param[in]	angle	the rotation angle in the range [-MAX_ANGLE,MAX_ANGLE].
 *
 * @note		the servo id will be mapped to the associated PWM channel.
 */
void commandServo(uint8_t servo, int8_t angle) {
	if (angle >= -MAX_ANGLE && angle <= MAX_ANGLE)
		pwmEnableChannel(&PWMD3, servo + 2,
				map(angle,MAX_ANGLE,-MAX_ANGLE,MIN_SERVO,MAX_SERVO));
}

/**
 * @brief 		The servo control thread.
 * @details		It reads the angle used to control the servos from serial
 * 				driver, then it commands the servo.
 * @param[in]	arg		the thread argument.
 *
 * @return		the thread exit status.
 *
 * @note		serial 1° byte --> servo 0 angle,
 * 				serial 2° byte --> servo 1 angle.
 */
static WORKING_AREA(ctrlThreadWA, 128);
static msg_t CtrlThread(void *arg) {
	uint8_t data[2];

	(void) arg;
	chRegSetThreadName("Control");
	while (TRUE) {
		sdRead(&SD1,&data[0],2);

		int8_t a0 = (int8_t)data[0];
		int8_t a1 = (int8_t)data[1];

		if (a0 >= -MAX_ANGLE && a0 <= MAX_ANGLE)
			commandServo(0, a0);

		if (a1 >= -MAX_ANGLE && a1 <= MAX_ANGLE)
			commandServo(1, a1);
	}

	return 0;
}

/*===========================================================================*/
/* External Functions.                                                       */
/*===========================================================================*/

/**
 * @brief	Application entry point.
 */
int main(void) {

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	/*
	 * Activates the serial driver with the default configuration
	 */
	sdStart(&SD1, NULL);
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(0));
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(0));

	/*
	 * Initializes the PWM driver 3.
	 * GPIOC 9 and GPIOC 8 are the PWM output (CH2-3).
	 */
	pwmStart(&PWMD3, &pwmcfg);
	palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(0));
	palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(0));

	/*
	 * Test each servo at min angle
	 */
	commandServo(0, -90);
	commandServo(1, -90);
	chThdSleepMilliseconds(1000);

	/*
	 * Test each servo at max angle
	 */
	commandServo(0, 90);
	commandServo(1, 90);
	chThdSleepMilliseconds(1000);

	/*
	 * Set servos to 0 angle
	 */
	commandServo(0, 0);
	commandServo(1, 0);
	chThdSleepMilliseconds(1000);

	/*
	 * Create control thread
	 */
	chThdCreateStatic(ctrlThreadWA, sizeof(ctrlThreadWA), NORMALPRIO,
			CtrlThread, NULL);

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop.
	 */
	while (TRUE) {
		chThdSleepMilliseconds(100);
	}
}

/** @} */
