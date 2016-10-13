/* silent-stepper-brick
 * Copyright (C) 2015 Olaf Lüke <olaf@tinkerforge.com>
 *
 * silent-stepper.c: Implementation of Silent Stepper Brick specific functions
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "silent-stepper.h"

#include <stdio.h>

#include "config.h"
#include "communication.h"
#include "tcm2130.h"

#include "bricklib/com/com.h"
#include "bricklib/com/com_common.h"
#include "bricklib/logging/logging.h"
#include "bricklib/bricklet/bricklet_init.h"
#include "bricklib/drivers/dacc/dacc.h"
#include "bricklib/drivers/adc/adc.h"
#include "bricklib/drivers/tc/tc.h"
#include "bricklib/drivers/pio/pio.h"
#include "bricklib/drivers/usart/usart.h"
#include "bricklib/drivers/pwmc/pwmc.h"
#include "bricklib/drivers/usb/USBD_HAL.h"
#include "bricklib/utility/util_definitions.h"
#include "bricklib/utility/led.h"
#include "bricklib/utility/init.h"

Pin pin_voltage_switch = VOLTAGE_STACK_SWITCH_PIN;

Pin pin_enable =  PIN_ENABLE;
Pin pin_step = PIN_STEP;
Pin pin_direction = PIN_DIRECTION;
Pin pin_vref = PIN_VREF;


uint32_t stepper_velocity_goal = STEPPER_VELOCITY_DEFAULT;
uint32_t stepper_velocity = 0;
uint16_t stepper_acceleration = STEPPER_ACCELERATION_DEFAULT;
uint16_t stepper_acceleration_sqrt = STEPPER_ACCELERATION_SQRT_DEFAULT;
uint16_t stepper_deceleration = STEPPER_DECELERATION_DEFAUL;
uint16_t stepper_minimum_voltage = STEPPER_MINIMUM_VOLTAGE_DEFAULT;

int32_t stepper_position = 0;
int32_t stepper_target_position = 0;
int32_t stepper_steps = 0;
uint16_t stepper_output_current = 0;
int8_t stepper_state = STEPPER_STATE_OFF;
int8_t stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
int8_t stepper_direction = STEPPER_DIRECTION_FORWARD;

int32_t stepper_step_counter = 0;
int32_t stepper_acceleration_counter = 0;
uint32_t stepper_acceleration_steps = 0;
int32_t stepper_deceleration_steps = 0;
int32_t stepper_delay = 0;
int32_t stepper_last_delay = 0;
int32_t stepper_delay_rest = 0;
int32_t stepper_deceleration_start = 0;
uint32_t stepper_tick_counter = 0;
uint32_t stepper_tick_calc_counter = 0;

uint32_t stepper_time_base = 1;
uint32_t stepper_time_base_counter = 1;
uint32_t stepper_all_data_period = 0;
uint32_t stepper_all_data_period_counter = 0;

bool stepper_running = false;
bool stepper_position_reached = false;

uint32_t stepper_current_sum = 0;
uint32_t stepper_current = 0;

uint8_t stepper_api_state = STEPPER_API_STATE_STOP;
uint8_t stepper_api_prev_state = STEPPER_API_STATE_STOP;
bool stepper_api_state_send = false;

bool silent_mode_switched = false;

const uint32_t stepper_timer_frequency[] = {BOARD_MCK/2,
                                            BOARD_MCK/8,
                                            BOARD_MCK/32,
                                            BOARD_MCK/128,
                                            32768};
const uint32_t stepper_timer_velocity[]  = {BOARD_MCK/2   / MAX_TIMER_VALUE,
                                            BOARD_MCK/8   / MAX_TIMER_VALUE,
                                            BOARD_MCK/32  / MAX_TIMER_VALUE,
                                            BOARD_MCK/128 / MAX_TIMER_VALUE,
                                            32768         / MAX_TIMER_VALUE};

extern ComInfo com_info;
extern bool usb_first_connection;

void stepper_position_reached_signal(void) {
	PositionReachedSignal prs;
	com_make_default_header(&prs, com_info.uid, sizeof(PositionReachedSignal), FID_POSITION_REACHED);
	prs.position = stepper_position;

	send_blocking_with_timeout(&prs,
	                           sizeof(PositionReachedSignal),
	                           com_info.current);
}

void stepper_check_error_signals(void) {
	if(stepper_tick_counter % 1000 != 0 || stepper_state == STEPPER_STATE_OFF) {
		return;
	}

	const uint16_t external_voltage = stepper_get_external_voltage();
	const uint16_t stack_voltage    = stepper_get_stack_voltage();

	// Under Voltage if external voltage is below minimum voltage (regardless
	// of stack voltage), or if external voltage is zero and stack velotage is
	// below minimum voltage

	if((external_voltage > STEPPER_VOLTAGE_EPSILON &&
	    external_voltage < stepper_minimum_voltage) ||
	   (external_voltage < STEPPER_VOLTAGE_EPSILON &&
	    stack_voltage > STEPPER_VOLTAGE_EPSILON &&
	    stack_voltage < stepper_minimum_voltage)) {
		UnderVoltageSignal uvs;
		com_make_default_header(&uvs, com_info.uid, sizeof(UnderVoltageSignal), FID_UNDER_VOLTAGE);
		uvs.voltage = external_voltage < STEPPER_VOLTAGE_EPSILON ? stack_voltage : external_voltage;

		send_blocking_with_timeout(&uvs,
		                           sizeof(UnderVoltageSignal),
		                           com_info.current);
		led_on(LED_STD_RED);
	} else {
		led_off(LED_STD_RED);
	}
}

void stepper_make_drive_speedramp(const uint8_t state) {
	stepper_state = STEPPER_STATE_DRIVE;
	stepper_speedramp_state = state;

	if(state == STEPPER_SPEEDRAMP_STATE_STOP) {
		stepper_set_new_api_state(STEPPER_API_STATE_DECELERATION);
	} else if(state == STEPPER_SPEEDRAMP_STATE_BACKWARD ||
	          state == STEPPER_SPEEDRAMP_STATE_FORWARD) {
		stepper_set_new_api_state(STEPPER_API_STATE_ACCELERATION);
	}

	if(!stepper_is_currently_running()) {
		// call drive speedramp one time, to get it going
		// (i.e. make velocity != 0)
		stepper_drive_speedramp();
		TC0_IrqHandler();
	}
}

void stepper_make_step_speedramp(const int32_t steps) {
	if(stepper_velocity_goal == 0) {
		return;
	}

	int32_t use_steps = steps;

	if(use_steps == 0) {
		stepper_position_reached = true;
		stepper_state = STEPPER_STATE_STOP;
		return;
	}
	if(use_steps < 0) {
		stepper_set_direction(STEPPER_DIRECTION_BACKWARD);
		use_steps = -steps;
	} else {
		stepper_set_direction(STEPPER_DIRECTION_FORWARD);
	}

	if(use_steps == 1) {
		// Just make the single step, no need for TC IRQ
		stepper_make_step();
		stepper_position_reached = true;
		stepper_set_new_api_state(STEPPER_API_STATE_RUN);
		stepper_set_new_api_state(STEPPER_API_STATE_STOP);
		return;
	}

	uint16_t acceleration;
	uint16_t acceleration_sqrt;
	uint16_t deceleration;

	if(stepper_acceleration == 0) {
		acceleration = 0xFFFF;
		acceleration_sqrt = 256; // sqrt(0xFFFF)
	} else {
		acceleration = stepper_acceleration;
		acceleration_sqrt = stepper_acceleration_sqrt;
	}

	if(stepper_deceleration == 0) {
		deceleration = 0xFFFF;
	} else {
		deceleration = stepper_deceleration;
	}

	stepper_acceleration_steps = (stepper_velocity_goal*stepper_velocity_goal)/
	                             (2*acceleration);
	if(stepper_acceleration_steps == 0) {
		stepper_acceleration_steps = 1;
	}

	int32_t acceleration_limit = (((int64_t)use_steps)*((int64_t)deceleration))/
	                             (acceleration + deceleration);
	if(acceleration_limit == 0) {
		acceleration_limit = 1;
	}

	if(acceleration_limit <= stepper_acceleration_steps) {
		stepper_deceleration_steps = acceleration_limit - use_steps;
	} else {
		stepper_deceleration_steps = -(((int64_t)stepper_acceleration_steps) *
									   ((int64_t)acceleration) /
		                               deceleration);
	}
	if(stepper_deceleration_steps == 0) {
		stepper_deceleration_steps = -1;
	}

	stepper_velocity = acceleration_sqrt;
	stepper_delay = VELOCITY_TO_DELAY(acceleration_sqrt);

	stepper_deceleration_start = use_steps + stepper_deceleration_steps;
	stepper_step_counter = 0;
	stepper_acceleration_counter = 0;
	stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_ACCELERATION;
	stepper_set_new_api_state(STEPPER_API_STATE_ACCELERATION);

	TC0_IrqHandler();
}

void tick_task(const uint8_t tick_type) {
	static int8_t message_counter = 0;

	if(tick_type == TICK_TASK_TYPE_CALCULATION) {
		tcm2130_handle_register_write();

		stepper_tick_calc_counter++;
		stepper_current_sum += adc_channel_get_data(STEPPER_CURRENT_CHANNEL);
		if(stepper_tick_calc_counter % 100 == 0) {
//			printf("before tstep\n\r");
//			uint32_t value = tcm2130_read_register(TMC2130_REG_TSTEP);
//			printf("tstep: %d\n\r", value);
			stepper_current = stepper_current_sum/100;
			stepper_current_sum = 0;
		}

		// Switch Output Voltage between extern and stack
		if(stepper_get_external_voltage() < STEPPER_VOLTAGE_EPSILON) {
			PIO_Set(&pin_voltage_switch);
		} else {
			PIO_Clear(&pin_voltage_switch);
		}

		// Power TMC2130 if external or stack voltage is connected and above voltage minimum
		if((stepper_get_external_voltage() > STEPPER_VOLTAGE_EPSILON) || (stepper_get_stack_voltage() > STEPPER_VOLTAGE_EPSILON)) {
			tcm2130_set_active(true);
		} else {
			tcm2130_set_active(true); // TODO: FIXME: Set to false
		}

		stepper_all_data_period_counter++;
	} else if(tick_type == TICK_TASK_TYPE_MESSAGE) {
		if(usb_first_connection && !usbd_hal_is_disabled(IN_EP)) {
			message_counter++;
			if(message_counter >= 100) {
				message_counter = 0;
				if(brick_init_enumeration(COM_USB)) {
					com_info.current = COM_USB;
					message_counter = 0;
					usb_first_connection = false;
				}
			}
		}

		stepper_tick_counter++;

		if(stepper_position_reached) {
			stepper_position_reached = false;
			stepper_position_reached_signal();
		}

		if(stepper_api_state_send) {
			stepper_api_state_send = false;
			stepper_state_signal();
		}

		stepper_check_error_signals();

		if((stepper_all_data_period != 0) &&
		   (stepper_all_data_period <= stepper_all_data_period_counter)) {
			// Test if we are totally out of time (lost a whole
			// period), in this case we don't send the signal again.
			// This is needed for the wireless extensions.
			if(stepper_all_data_period*2 <= stepper_all_data_period_counter) {
				stepper_all_data_period_counter = stepper_all_data_period;
			}
			stepper_all_data_signal();
		}
	}
}

void stepper_state_signal(void) {
	NewStateSignal nss;
	com_make_default_header(&nss, com_info.uid, sizeof(NewStateSignal), FID_NEW_STATE);
	nss.state_new      = stepper_api_state;
	nss.state_previous = stepper_api_prev_state;

	send_blocking_with_timeout(&nss, sizeof(NewStateSignal), com_info.current);
}

void stepper_all_data_signal(void) {
	stepper_all_data_period_counter -= stepper_all_data_period;

	AllDataSignal ads;
	com_make_default_header(&ads, com_info.uid, sizeof(AllDataSignal), FID_ALL_DATA);
	ads.current_velocity = stepper_velocity > 0xFFFF ? 0xFFFF : stepper_velocity;
	ads.current_position = stepper_position;
	ads.remaining_steps = stepper_get_remaining_steps();
	ads.stack_voltage = stepper_get_stack_voltage();
	ads.external_voltage = stepper_get_external_voltage();
	ads.current_consumption = stepper_get_current();

	send_blocking_with_timeout(&ads, sizeof(AllDataSignal), com_info.current);
}


void stepper_init(void) {
	Pin stepper_power_management_pins[] = {VOLTAGE_STACK_PIN,
	                                       VOLTAGE_EXTERN_PIN,
	                                       VOLTAGE_STACK_SWITCH_PIN,
	                                       STEPPER_CURRENT_PIN};
	PIO_Configure(stepper_power_management_pins, PIO_LISTSIZE(stepper_power_management_pins));

	adc_channel_enable(VOLTAGE_EXTERN_CHANNEL);
	adc_channel_enable(VOLTAGE_STACK_CHANNEL);
	adc_channel_enable(STEPPER_CURRENT_CHANNEL);

	tcm2130_set_active(true); // TODO: FIXME: Set to false here!
	SLEEP_MS(40);
}


void stepper_set_next_timer(const uint32_t velocity) {
	uint32_t velocity_use = velocity;
	if(velocity == 0) {
		if(stepper_state == STEPPER_STATE_DRIVE && stepper_velocity_goal != 0) {
			// In drive mode we have a transition from backward to forward here.
			// Wait 10ms in that case
			velocity_use = 100;
		} else {
			// In step mode this should not happen, stop tc
			stepper_running = false;
			tc_channel_stop(&STEPPER_TC_CHANNEL);
			return;
		}
	}

	int8_t i;
	for(i = 4; i > 0; i--) {
		if(velocity_use <= stepper_timer_velocity[i-1]) {
			break;
		}
	}

	STEPPER_TC_CHANNEL.TC_CMR = i | TC_CMR_CPCTRG;
	STEPPER_COUNTER = stepper_timer_frequency[i] / velocity_use;
	if(!stepper_is_currently_running()) {
		stepper_running = true;
		tc_channel_start(&STEPPER_TC_CHANNEL);
	}
}

bool stepper_is_currently_running(void) {
	return stepper_running;
}

void stepper_make_step(void) {
	// We change step pin back and force for one step each (dedge = 1)
	if(pin_step.pio->PIO_ODSR & pin_step.mask) {
		pin_step.pio->PIO_CODR = pin_step.mask;
	} else {
		pin_step.pio->PIO_SODR = pin_step.mask;
	}

	stepper_position += stepper_direction;
}

void stepper_step_speedramp(void) {
	int32_t new_delay = 0;

	switch(stepper_speedramp_state) {
		case STEPPER_SPEEDRAMP_STATE_STOP: {
			stepper_velocity = 0;
			stepper_step_counter = 0;
			stepper_acceleration_counter = 0;
			stepper_delay_rest = 0;

			stepper_running = false;
			tc_channel_stop(&STEPPER_TC_CHANNEL);
			return;
		}

		case STEPPER_SPEEDRAMP_STATE_ACCELERATION: {
			stepper_step_counter++;
			stepper_acceleration_counter++;

			int32_t a = (2*stepper_delay + stepper_delay_rest);
			int32_t b = (4*stepper_acceleration_counter + 1);
			new_delay = stepper_delay - a/b;
			stepper_delay_rest = a % b;

			if(stepper_step_counter >= stepper_deceleration_start) {
				stepper_acceleration_counter = stepper_deceleration_steps;
				stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_DECELERATION;
				stepper_set_new_api_state(STEPPER_API_STATE_DECELERATION);
			} else if(new_delay <= VELOCITY_TO_DELAY(stepper_velocity_goal)) {
				stepper_last_delay = new_delay;
				new_delay = VELOCITY_TO_DELAY(stepper_velocity_goal);
				stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_RUN;
				stepper_set_new_api_state(STEPPER_API_STATE_RUN);
			}
			break;
		}

		case STEPPER_SPEEDRAMP_STATE_RUN: {
			stepper_step_counter++;
			if(stepper_step_counter >= stepper_deceleration_start) {
				stepper_acceleration_counter = stepper_deceleration_steps;
				new_delay = stepper_last_delay;
				stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_DECELERATION;
				stepper_set_new_api_state(STEPPER_API_STATE_DECELERATION);
			} else {
				new_delay = VELOCITY_TO_DELAY(stepper_velocity_goal);
			}
			break;
		}

		case STEPPER_SPEEDRAMP_STATE_DECELERATION: {
			stepper_step_counter++;
			stepper_acceleration_counter++;

			int32_t a = (2*stepper_delay + stepper_delay_rest);
			int32_t b = (4*stepper_acceleration_counter + 1);
			new_delay = stepper_delay - a/b;
			stepper_delay_rest = a % b;

			if(stepper_acceleration_counter >= 0) {
				stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
				stepper_state = STEPPER_STATE_STOP;
				stepper_set_new_api_state(STEPPER_API_STATE_STOP);
				stepper_position_reached = true;
				stepper_running = false;
				tc_channel_stop(&STEPPER_TC_CHANNEL);
				stepper_velocity = 0;
				return;
			}
			break;
		}
	}

	stepper_delay = new_delay;
	stepper_velocity = DELAY_TO_VELOCITY(stepper_delay);
}

void stepper_full_brake(void) {
	stepper_running = false;
	tc_channel_stop(&STEPPER_TC_CHANNEL);
	stepper_state = STEPPER_STATE_STOP;
	stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
	stepper_set_new_api_state(STEPPER_API_STATE_STOP);
	stepper_velocity = 0;
}

void stepper_drive_speedramp(void) {
	static uint32_t rest = 0;
	uint16_t goal = stepper_velocity_goal;

	if((stepper_speedramp_state == STEPPER_SPEEDRAMP_STATE_STOP) ||
	   (stepper_speedramp_state != stepper_direction)) {
		goal = 0;
	}
    
	if(goal == stepper_velocity) {
		if(stepper_speedramp_state == STEPPER_SPEEDRAMP_STATE_STOP) {
			stepper_running = false;
			tc_channel_stop(&STEPPER_TC_CHANNEL);
			stepper_state = STEPPER_STATE_STOP;
			stepper_velocity = 0;
			stepper_step_counter = 0;
			stepper_acceleration_counter = 0;
			stepper_delay_rest = 0;
			rest = 0;
			stepper_set_new_api_state(STEPPER_API_STATE_STOP);
			return;
		} else if(stepper_speedramp_state != stepper_direction) {
			goal = stepper_velocity_goal;
			rest = 0;
		} else {
			// If i am at stepper velocity goal and the direction is correct
			// -> We have nothing to do
			return;
		}
	}

	uint16_t acceleration;
	uint16_t acceleration_sqrt;
	uint16_t deceleration;
	int32_t delta;

	if(stepper_acceleration == 0) {
		acceleration = 0xFFFF;
		acceleration_sqrt = 256; // sqrt(0xFFFF)
	} else {
		acceleration = stepper_acceleration;
		acceleration_sqrt = stepper_acceleration_sqrt;
	}
	if(stepper_deceleration == 0) {
		deceleration = 0xFFFF;
	} else {
		deceleration = stepper_deceleration;
	}

	if(stepper_velocity == 0) {
		delta = acceleration_sqrt;
		rest = 0;
		if(stepper_speedramp_state == STEPPER_SPEEDRAMP_STATE_FORWARD) {
			stepper_set_direction(STEPPER_DIRECTION_FORWARD);
		} else if(stepper_speedramp_state == STEPPER_SPEEDRAMP_STATE_BACKWARD) {
			stepper_set_direction(STEPPER_DIRECTION_BACKWARD);
		}
	} else {
		if(stepper_velocity < goal) {
			delta = (acceleration + rest) / stepper_velocity;
			rest = (acceleration + rest) % stepper_velocity;
		} else {
			delta = (deceleration + rest) / stepper_velocity;
			rest = (deceleration + rest) % stepper_velocity;
		}
	}

	if(stepper_velocity < goal) {
		stepper_velocity = MIN(stepper_velocity + delta,
							   goal);

		if(stepper_velocity == goal) {
			stepper_set_new_api_state(STEPPER_API_STATE_RUN);
		}
	} else {
		stepper_velocity = MAX(((int32_t)stepper_velocity) - delta,
							   goal);
	}
}

void TC0_IrqHandler(void) {
	// Acknowledge interrupt
	tc_channel_interrupt_ack(&STEPPER_TC_CHANNEL);

	stepper_time_base_counter--;
	if(stepper_time_base_counter > 0) {
		tc_channel_start(&STEPPER_TC_CHANNEL);
		return;
	}

	stepper_time_base_counter = stepper_time_base;

	if(stepper_state != STEPPER_STATE_STOP) {
		stepper_set_next_timer(stepper_velocity);
		stepper_make_step();
	}

	if(stepper_state == STEPPER_STATE_STEPS ||
	   stepper_state == STEPPER_STATE_TARGET) {
		stepper_step_speedramp();
	} else if(stepper_state == STEPPER_STATE_DRIVE) {
		stepper_drive_speedramp();
	}
}


void stepper_set_direction(const int8_t direction) {
	if(direction == stepper_direction) {
		return;
	}

	stepper_direction = direction;
	if(direction == STEPPER_DIRECTION_FORWARD) {
		if(stepper_state == STEPPER_STATE_DRIVE) {
			stepper_set_new_api_state(STEPPER_API_STATE_DIR_CHANGE_FORWARD);
		}
		PIO_Clear(&pin_direction);
	} else {
		if(stepper_state == STEPPER_STATE_DRIVE) {
			stepper_set_new_api_state(STEPPER_API_STATE_DIR_CHANGE_BACKWARD);
		}
		PIO_Set(&pin_direction);
	}
}

void stepper_enable_pin_apply(void) {
	if(stepper_state == STEPPER_STATE_OFF) {
		pin_enable.type = PIO_OUTPUT_1;
	} else {
		pin_enable.type = PIO_OUTPUT_0;
	}

	PIO_Configure(&pin_enable, 1);
}

void stepper_enable(void) {
	stepper_state = STEPPER_STATE_STOP;
	stepper_enable_pin_apply();
	stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
	stepper_api_state = STEPPER_API_STATE_STOP;
	stepper_api_prev_state = STEPPER_API_STATE_STOP;
}

void stepper_disable(void) {
	stepper_state = STEPPER_STATE_OFF;
	stepper_speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
	stepper_enable_pin_apply();
	stepper_full_brake();
	stepper_state = STEPPER_STATE_OFF;
}

void stepper_set_output_current(const uint16_t current) {
	const uint16_t new_current = BETWEEN(VREF_MIN_CURRENT,
	                                     current,
	                                     VREF_MAX_CURRENT);

	DACC_SetConversionData(DACC, SCALE(new_current,
	                                   VREF_MIN_CURRENT,
	                                   VREF_MAX_CURRENT,
	                                   VOLTAGE_MIN_VALUE,
	                                   VOLTAGE_MAX_VALUE));

	stepper_output_current = new_current;
}

uint16_t stepper_get_external_voltage(void) {
    return adc_channel_get_data(VOLTAGE_EXTERN_CHANNEL) *
           VOLTAGE_EXTERN_REFERENCE *
           VOLTAGE_EXTERN_MULTIPLIER /
           VOLTAGE_MAX_VALUE;
}

uint16_t stepper_get_stack_voltage(void) {
    return adc_channel_get_data(VOLTAGE_STACK_CHANNEL) *
           VOLTAGE_STACK_REFERENCE *
           VOLTAGE_STACK_MULTIPLIER /
           VOLTAGE_MAX_VALUE;
}

uint16_t stepper_get_current(void) {
	return stepper_current *
	       STEPPER_CURRENT_REFERENCE *
	       STEPPER_CURRENT_MULTIPLIER /
	       VOLTAGE_MAX_VALUE;
}

int32_t stepper_get_remaining_steps(void) {
	if(stepper_state == STEPPER_STATE_STEPS) {
		if(stepper_steps > 0) {
			return stepper_steps - stepper_step_counter;
		} else {
			return stepper_steps + stepper_step_counter;
		}
	} else if(stepper_state == STEPPER_STATE_TARGET) {
		return stepper_target_position - stepper_position;
	}

	return 0;
}

void stepper_set_new_api_state(const uint8_t new_state) {
	stepper_api_prev_state = stepper_api_state;
	stepper_api_state = new_state;
	stepper_api_state_send = true;
}
