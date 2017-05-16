/* silent-stepper-brick
 * Copyright (C) 2015 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.c: Implementation of Silent Stepper Brick specific messages
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

#include "communication.h"

#include "tmc2130.h"
#include "silent-stepper.h"
#include "bricklib/logging/logging.h"
#include "bricklib/com/com_common.h"
#include "bricklib/drivers/pwmc/pwmc.h"
#include "bricklib/drivers/adc/adc.h"

#include "bricklib/utility/util_definitions.h"
#include "bricklib/utility/sqrt.h"

#include <stdint.h>
#include <stdio.h>

extern uint32_t stepper_velocity_goal;
extern uint32_t stepper_velocity;
extern uint16_t stepper_acceleration;
extern uint16_t stepper_acceleration_sqrt;
extern uint16_t stepper_deceleration;
extern uint16_t stepper_minimum_voltage;

extern int32_t stepper_position;
extern int32_t stepper_target_position;
extern int32_t stepper_steps;
extern uint16_t stepper_output_current;

extern int8_t stepper_state;
extern uint32_t stepper_time_base;
extern uint32_t stepper_all_data_period;

extern uint32_t tmc2130_register_to_write_mask;

extern TMC2130HighLevel tmc2130_high_level;

extern TMC2130RegGSTAT tmc2130_reg_gstat;
extern TMC2130RegTSTEP tmc2130_reg_tstep;
extern TMC2130RegDRV_STATUS tmc2130_reg_drv_status;
extern TMC2130RegPWM_SCALE tmc2130_reg_pwm_scale;

extern TMC2130RegIHOLD_IRUN tmc2130_reg_ihold_run;
extern TMC2130RegTPOWERDOWN tmc2130_reg_tpowerdown;
extern TMC2130RegTPWMTHRS tmc2130_reg_tpwmthrs;
extern TMC2130RegTCOOLTHRS tmc2130_reg_tcoolthrs;
extern TMC2130RegTHIGH tmc2130_reg_thigh;
extern TMC2130RegCOOLCONF tmc2130_reg_coolconf;
extern TMC2130RegPWMCONF tmc2130_reg_pwmconf;
extern TMC2130RegGCONF tmc2130_reg_gconf;
extern TMC2130RegCHOPCONF tmc2130_reg_chopconf;

void set_max_velocity(const ComType com, const SetMaxVelocity *data) {
	uint32_t old_velocity_goal = stepper_velocity_goal;
	stepper_velocity_goal = data->velocity;

	if(stepper_state == STEPPER_STATE_DRIVE && old_velocity_goal == 0) {
		TC0_IrqHandler();
	}

	com_return_setter(com, data);
}

void get_max_velocity(const ComType com, const GetMaxVelocity *data) {
	GetMaxVelocityReturn gmvr;

	gmvr.header        = data->header;
	gmvr.header.length = sizeof(GetMaxVelocityReturn);
	gmvr.velocity      = stepper_velocity_goal;

	send_blocking_with_timeout(&gmvr, sizeof(GetMaxVelocityReturn), com);
}

void get_current_velocity(const ComType com, const GetCurrentVelocity *data) {
	GetCurrentVelocityReturn gcvr;

	gcvr.header        = data->header;
	gcvr.header.length = sizeof(GetCurrentVelocityReturn);
	gcvr.velocity      = stepper_velocity > 0xFFFF ? 0xFFFF : stepper_velocity;

	send_blocking_with_timeout(&gcvr, sizeof(GetCurrentVelocityReturn), com);
}

void set_speed_ramping(const ComType com, const SetSpeedRamping *data) {
	stepper_acceleration_sqrt = sqrt_integer_precise(data->acceleration);
	stepper_acceleration      = data->acceleration;
	stepper_deceleration      = data->deceleration;

	com_return_setter(com, data);
}

void get_speed_ramping(const ComType com, const GetSpeedRamping *data) {
	GetSpeedRampingReturn gsrr;

	gsrr.header        = data->header;
	gsrr.header.length = sizeof(GetSpeedRampingReturn);
	gsrr.acceleration  = stepper_acceleration;
	gsrr.deceleration  = stepper_deceleration;

	send_blocking_with_timeout(&gsrr, sizeof(GetSpeedRampingReturn), com);
}

void full_brake(const ComType com, const FullBrake *data) {
	stepper_full_brake();

	com_return_setter(com, data);
}

void set_current_position(const ComType com, const SetCurrentPosition *data) {
	stepper_position = data->position;

	com_return_setter(com, data);
}

void get_current_position(const ComType com, const GetCurrentPosition *data) {
	GetCurrentPositionReturn gcpr;

	gcpr.header        = data->header;
	gcpr.header.length = sizeof(GetCurrentPositionReturn);
	gcpr.position      = stepper_position;

	send_blocking_with_timeout(&gcpr, sizeof(GetCurrentPositionReturn), com);
}

void set_target_position(const ComType com, const SetTargetPosition *data) {
	if(stepper_is_currently_running() || stepper_state == STEPPER_STATE_OFF) {
		com_return_setter(com, data);
		return;
	}

	stepper_state = STEPPER_STATE_TARGET;
	stepper_target_position = data->position;
	stepper_make_step_speedramp(stepper_target_position - stepper_position);

	com_return_setter(com, data);
}

void get_target_position(const ComType com, const GetTargetPosition *data) {
	GetTargetPositionReturn gtpr;

	gtpr.header        = data->header;
	gtpr.header.length = sizeof(GetTargetPositionReturn);
	gtpr.position      = stepper_target_position;

	send_blocking_with_timeout(&gtpr, sizeof(GetTargetPositionReturn), com);
}

void set_steps(const ComType com, const SetSteps *data) {
	if(stepper_is_currently_running() || stepper_state == STEPPER_STATE_OFF) {
		com_return_setter(com, data);
		return;
	}

	stepper_state = STEPPER_STATE_STEPS;
	stepper_steps = data->steps;
	stepper_make_step_speedramp(data->steps);

	com_return_setter(com, data);
}

void get_steps(const ComType com, const GetSteps *data) {
	GetStepsReturn gsr;

	gsr.header        = data->header;
	gsr.header.length = sizeof(GetStepsReturn);
	gsr.steps         = stepper_steps;

	send_blocking_with_timeout(&gsr, sizeof(GetStepsReturn), com);
}

void get_remaining_steps(const ComType com, const GetRemainingSteps *data) {
	GetRemainingStepsReturn grsr;

	grsr.header        = data->header;
	grsr.header.length = sizeof(GetRemainingStepsReturn);
	grsr.steps         = stepper_get_remaining_steps();
	send_blocking_with_timeout(&grsr, sizeof(GetRemainingStepsReturn), com);
}

void set_step_configuration(const ComType com, const SetStepConfiguration *data) {
	if(data->step_resolution > 8) {
		com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	tmc2130_reg_chopconf.bit.mres   = data->step_resolution;
	tmc2130_reg_chopconf.bit.intpol = data->interpolation;

	tmc2130_register_to_write_mask |= TMC2130_REG_CHOPCONF_BIT;
	tmc2130_handle_register_read_and_write();

	com_return_setter(com, data);
}

void get_step_configuration(const ComType com, const GetStepConfiguration *data) {
	GetStepConfigurationReturn gscr;

	gscr.header          = data->header;
	gscr.header.length   = sizeof(GetStepConfigurationReturn);
	gscr.step_resolution = tmc2130_reg_chopconf.bit.mres;
	gscr.interpolation   = tmc2130_reg_chopconf.bit.intpol;

	send_blocking_with_timeout(&gscr, sizeof(GetStepConfigurationReturn), com);
}

void drive_forward(const ComType com, const DriveForward *data) {
	if(stepper_state == STEPPER_STATE_STEPS ||
	   stepper_state == STEPPER_STATE_TARGET ||
	   stepper_state == STEPPER_STATE_OFF) {
		com_return_setter(com, data);
		return;
	}

	stepper_make_drive_speedramp(STEPPER_SPEEDRAMP_STATE_FORWARD);

	com_return_setter(com, data);
}

void drive_backward(const ComType com, const DriveBackward *data) {
	if(stepper_state == STEPPER_STATE_STEPS ||
	   stepper_state == STEPPER_STATE_TARGET ||
	   stepper_state == STEPPER_STATE_OFF) {
		com_return_setter(com, data);
		return;
	}

	stepper_make_drive_speedramp(STEPPER_SPEEDRAMP_STATE_BACKWARD);

	com_return_setter(com, data);
}

void stop(const ComType com, const Stop *data) {
	if(stepper_state == STEPPER_STATE_OFF) {
		com_return_setter(com, data);
		return;
	}

	stepper_make_drive_speedramp(STEPPER_SPEEDRAMP_STATE_STOP);
	com_return_setter(com, data);
}

void get_stack_input_voltage(const ComType com, const GetStackInputVoltage *data) {
	GetStackInputVoltageReturn gsivr;

	gsivr.header        = data->header;
	gsivr.header.length = sizeof(GetStackInputVoltageReturn);
    gsivr.voltage       = stepper_get_stack_voltage();

	send_blocking_with_timeout(&gsivr, sizeof(GetStackInputVoltageReturn), com);
}

void get_external_input_voltage(const ComType com, const GetExternalInputVoltage *data) {
	GetExternalInputVoltageReturn geivr;

	geivr.header        = data->header;
	geivr.header.length = sizeof(GetExternalInputVoltageReturn);
	geivr.voltage       = stepper_get_external_voltage();

	send_blocking_with_timeout(&geivr, sizeof(GetExternalInputVoltageReturn), com);
}

void get_current_consumption(const ComType com, const GetCurrentConsumption *data) {
	GetCurrentConsumptionReturn gccr;

	gccr.header        = data->header;
	gccr.header.length = sizeof(GetCurrentConsumptionReturn);

	// This function is here because of a copy and paste error from the old Stepper Brick,
	// the TMC2130 does not actually have an external current measurement mechanism.
	// See get_driver_status for an alternative.
	gccr.current       = 0;

	send_blocking_with_timeout(&gccr, sizeof(GetCurrentConsumptionReturn), com);
}

void set_motor_current(const ComType com, const SetMotorCurrent *data) {
	stepper_set_output_current(data->current);

	// update output current dependent registers
	tmc2130_reg_ihold_run.bit.ihold = MIN(SCALE(tmc2130_high_level.standstill_current, 0, stepper_output_current, 0, 31), 31);
	tmc2130_reg_ihold_run.bit.irun  = MIN(SCALE(tmc2130_high_level.motor_run_current, 0, stepper_output_current, 0, 31), 31);
	tmc2130_register_to_write_mask |= TMC2130_REG_IHOLD_IRUN_BIT;
	tmc2130_handle_register_read_and_write();

	com_return_setter(com, data);
}

void get_motor_current(const ComType com, const GetMotorCurrent *data) {
	GetMotorCurrentReturn gmcr;

	gmcr.header        = data->header;
	gmcr.header.length = sizeof(GetMotorCurrentReturn);
	gmcr.current       = stepper_output_current;

	send_blocking_with_timeout(&gmcr, sizeof(GetMotorCurrentReturn), com);
}

void enable(const ComType com, const Enable *data) {
	stepper_enable();
	com_return_setter(com, data);
}

void disable(const ComType com, const Disable *data) {
	stepper_disable();
	com_return_setter(com, data);
}

void is_enabled(const ComType com, const IsEnabled *data) {
	IsEnabledReturn ier;

	ier.header        = data->header;
	ier.header.length = sizeof(IsEnabledReturn);
	ier.enabled       = stepper_state != STEPPER_STATE_OFF;

	send_blocking_with_timeout(&ier, sizeof(IsEnabledReturn), com);
}

void set_basic_configuration(const ComType com, const SetBasicConfiguration *data) {
	if((data->power_down_time > 5222) ||
	   (data->standstill_delay_time > 307)) {
		com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	tmc2130_high_level.standstill_current    = data->standstill_current;
	tmc2130_high_level.motor_run_current     = data->motor_run_current;
	tmc2130_high_level.standstill_delay_time = data->standstill_delay_time;
	tmc2130_high_level.power_down_time       = data->power_down_time;
	tmc2130_high_level.stealth_threshold     = data->stealth_threshold;
	tmc2130_high_level.coolstep_threshold    = data->coolstep_threshold;
	tmc2130_high_level.classic_threshold     = data->classic_threshold;

	tmc2130_reg_ihold_run.bit.ihold       = MIN(SCALE(data->standstill_current, 0, stepper_output_current, 0, 31), 31);
	tmc2130_reg_ihold_run.bit.irun        = MIN(SCALE(data->motor_run_current, 0, stepper_output_current, 0, 31), 31);
	tmc2130_reg_ihold_run.bit.ihold_delay = MIN((data->standstill_delay_time*100)/2048, 15);
	tmc2130_reg_tpowerdown.bit.delay      = MIN((data->power_down_time*100)/2048, 255);
	tmc2130_reg_tpwmthrs.bit.velocity     = MIN(TCP2130_CLOCK_FREQUENCY/(data->stealth_threshold*256), 0xfffff);
	tmc2130_reg_tcoolthrs.bit.velocity    = MIN(TCP2130_CLOCK_FREQUENCY/(data->coolstep_threshold*256), 0xfffff);
	tmc2130_reg_thigh.bit.velocity        = MIN(TCP2130_CLOCK_FREQUENCY/(data->classic_threshold*256), 0xfffff);
	tmc2130_reg_chopconf.bit.vhighchm     = data->high_velocity_chopper_mode;

	tmc2130_register_to_write_mask |= (TMC2130_REG_IHOLD_IRUN_BIT | TMC2130_REG_TPOWERDOWN_BIT | TMC2130_REG_TPWMTHRS_BIT | TMC2130_REG_TCOOLTHRS_BIT | TMC2130_REG_THIGH_BIT | TMC2130_REG_CHOPCONF_BIT);
	tmc2130_handle_register_read_and_write();

	com_return_setter(com, data);
}

void get_basic_configuration(const ComType com, const GetBasicConfiguration *data) {
	GetBasicConfigurationReturn gbcr;

	gbcr.header                     = data->header;
	gbcr.header.length              = sizeof(GetBasicConfigurationReturn);
	gbcr.standstill_current         = tmc2130_high_level.standstill_current;
	gbcr.motor_run_current          = tmc2130_high_level.motor_run_current;
	gbcr.standstill_delay_time      = tmc2130_high_level.standstill_delay_time;
	gbcr.power_down_time            = tmc2130_high_level.power_down_time;
	gbcr.stealth_threshold          = tmc2130_high_level.stealth_threshold;
	gbcr.coolstep_threshold         = tmc2130_high_level.coolstep_threshold;
	gbcr.classic_threshold          = tmc2130_high_level.classic_threshold;
	gbcr.high_velocity_chopper_mode = tmc2130_reg_chopconf.bit.vhighchm;

	send_blocking_with_timeout(&gbcr, sizeof(GetBasicConfigurationReturn), com);
}

void set_spreadcycle_configuration(const ComType com, const SetSpreadcycleConfiguration *data) {
	if((data->slow_decay_duration > 15) ||
	   (data->fast_decay_duration > 15) ||
	   (data->hysteresis_start_value > 7) ||
	   (data->hysteresis_end_value < -3 || data->hysteresis_end_value > 12) ||
	   (data->sinewave_offset < -3 || data->sinewave_offset > 12) ||
	   (data->chopper_mode > 1) ||
	   (data->comparator_blank_time > 3)) {
		com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	tmc2130_reg_chopconf.bit.toff      = data->slow_decay_duration;
	tmc2130_reg_chopconf.bit.rndtf     = data->enable_random_slow_decay;
	if(data->chopper_mode) {
		tmc2130_reg_chopconf.bit.hstrt = data->fast_decay_duration & 0b111;
		tmc2130_reg_chopconf.bit.fd3   = (data->fast_decay_duration >> 3) & 0b1;
		tmc2130_reg_chopconf.bit.hend  = data->sinewave_offset; // TODO: handle signednes correctly!
	} else {
		tmc2130_reg_chopconf.bit.hstrt = data->hysteresis_start_value & 0b111;
		tmc2130_reg_chopconf.bit.fd3   = 0;
		tmc2130_reg_chopconf.bit.hend  = data->hysteresis_end_value; // TODO: handle signednes correctly!
	}
	tmc2130_reg_chopconf.bit.chm       = data->chopper_mode;
	tmc2130_reg_chopconf.bit.tbl       = data->comparator_blank_time;
	tmc2130_reg_chopconf.bit.disfdcc   = data->fast_decay_without_comparator;

	tmc2130_register_to_write_mask |= TMC2130_REG_CHOPCONF_BIT;
	tmc2130_handle_register_read_and_write();

	com_return_setter(com, data);
}

void get_spreadcycle_configuration(const ComType com, const GetSpreadcycleConfiguration *data) {
	GetSpreadcycleConfigurationReturn gscr;

	gscr.header                        = data->header;
	gscr.header.length                 = sizeof(GetSpreadcycleConfigurationReturn);
	gscr.slow_decay_duration           = tmc2130_reg_chopconf.bit.toff;
	gscr.enable_random_slow_decay      = tmc2130_reg_chopconf.bit.rndtf;
	if(tmc2130_reg_chopconf.bit.chm) {
		gscr.fast_decay_duration       = tmc2130_reg_chopconf.bit.hstrt | (tmc2130_reg_chopconf.bit.fd3 << 3);
		gscr.sinewave_offset           = tmc2130_reg_chopconf.bit.hend;
		gscr.hysteresis_start_value    = 0;
		gscr.hysteresis_end_value      = 0;
	} else {
		gscr.fast_decay_duration       = 0;
		gscr.sinewave_offset           = 0;
		gscr.hysteresis_start_value    = tmc2130_reg_chopconf.bit.hstrt;
		gscr.hysteresis_end_value      = tmc2130_reg_chopconf.bit.hend;
	}
	gscr.chopper_mode                  = tmc2130_reg_chopconf.bit.chm;
	gscr.comparator_blank_time         = tmc2130_reg_chopconf.bit.tbl;
	gscr.fast_decay_without_comparator = tmc2130_reg_chopconf.bit.disfdcc;

	send_blocking_with_timeout(&gscr, sizeof(GetSpreadcycleConfigurationReturn), com);
}

void set_stealth_configuration(const ComType com, const SetStealthConfiguration *data) {
	if(data->freewheel_mode > 3) {
		com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	tmc2130_reg_gconf.bit.en_pwm_mode     = data->enable_stealth;
	tmc2130_reg_pwmconf.bit.pwm_ampl      = data->amplitude;
	tmc2130_reg_pwmconf.bit.pwm_grad      = data->gradiant;
	tmc2130_reg_pwmconf.bit.pwm_autoscale = data->enable_autoscale;
	tmc2130_reg_pwmconf.bit.pwm_symmetric = data->force_symmetric;
	tmc2130_reg_pwmconf.bit.freewheel     = data->freewheel_mode;

	tmc2130_register_to_write_mask |= (TMC2130_REG_GCONF_BIT | TMC2130_REG_PWMCONF_BIT);
	tmc2130_handle_register_read_and_write();

	com_return_setter(com, data);
}

void get_stealth_configuration(const ComType com, const GetStealthConfiguration *data) {
	GetStealthConfigurationReturn gscr;

	gscr.header           = data->header;
	gscr.header.length    = sizeof(GetStealthConfigurationReturn);
	gscr.enable_stealth   = tmc2130_reg_gconf.bit.en_pwm_mode;
	gscr.amplitude        = tmc2130_reg_pwmconf.bit.pwm_ampl;
	gscr.gradiant         = tmc2130_reg_pwmconf.bit.pwm_grad;
	gscr.enable_autoscale = tmc2130_reg_pwmconf.bit.pwm_autoscale;
	gscr.force_symmetric  = tmc2130_reg_pwmconf.bit.pwm_symmetric;
	gscr.freewheel_mode   = tmc2130_reg_pwmconf.bit.freewheel;

	send_blocking_with_timeout(&gscr, sizeof(GetStealthConfigurationReturn), com);
}

void set_coolstep_configuration(const ComType com, const SetCoolstepConfiguration *data) {
	if((data->minimum_stallguard_value > 15) ||
	   (data->maximum_stallguard_value > 15) ||
	   (data->current_up_step_width > 3) ||
	   (data->current_down_step_width > 3) ||
	   (data->minimum_current > 1) ||
	   (data->stallguard_threshold_value < -64 || data->stallguard_threshold_value > 63) ||
	   (data->stallguard_mode > 1)) {
		com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	tmc2130_reg_coolconf.bit.semin  = data->minimum_stallguard_value;
	tmc2130_reg_coolconf.bit.semax  = data->maximum_stallguard_value;
	tmc2130_reg_coolconf.bit.seup   = data->current_up_step_width;
	tmc2130_reg_coolconf.bit.sedn   = data->current_down_step_width;
	tmc2130_reg_coolconf.bit.seimin = data->minimum_current;
	tmc2130_reg_coolconf.bit.sgt    = data->stallguard_threshold_value;
	tmc2130_reg_coolconf.bit.sfilt  = data->stallguard_mode;

	tmc2130_register_to_write_mask |= TMC2130_REG_COOLCONF_BIT;
	tmc2130_handle_register_read_and_write();

	com_return_setter(com, data);
}

void get_coolstep_configuration(const ComType com, const GetCoolstepConfiguration *data) {
	GetCoolstepConfigurationReturn gccr;

	gccr.header                     = data->header;
	gccr.header.length              = sizeof(GetCoolstepConfigurationReturn);
	gccr.minimum_stallguard_value   = tmc2130_reg_coolconf.bit.semin;
	gccr.maximum_stallguard_value   = tmc2130_reg_coolconf.bit.semax;
	gccr.current_up_step_width      = tmc2130_reg_coolconf.bit.seup;
	gccr.current_down_step_width    = tmc2130_reg_coolconf.bit.sedn;
	gccr.minimum_current            = tmc2130_reg_coolconf.bit.seimin;
	gccr.stallguard_threshold_value = tmc2130_reg_coolconf.bit.sgt;
	gccr.stallguard_mode            = tmc2130_reg_coolconf.bit.sfilt;

	send_blocking_with_timeout(&gccr, sizeof(GetCoolstepConfigurationReturn), com);
}

void set_misc_configuration(const ComType com, const SetMiscConfiguration *data) {
	if(data->synchronize_phase_frequency > 15) {
		com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	tmc2130_reg_chopconf.bit.diss2g = data->disable_short_to_ground_protection;
	tmc2130_reg_chopconf.bit.sync   = data->synchronize_phase_frequency;

	tmc2130_register_to_write_mask |= TMC2130_REG_CHOPCONF_BIT;
	tmc2130_handle_register_read_and_write();

	com_return_setter(com, data);
}

void get_misc_configuration(const ComType com, const GetMiscConfiguration *data) {
	GetMiscConfigurationReturn gmcr;

	gmcr.header                             = data->header;
	gmcr.header.length                      = sizeof(GetMiscConfigurationReturn);
	gmcr.disable_short_to_ground_protection = tmc2130_reg_chopconf.bit.diss2g;
	gmcr.synchronize_phase_frequency        = tmc2130_reg_chopconf.bit.sync;

	send_blocking_with_timeout(&gmcr, sizeof(GetMiscConfigurationReturn), com);
}

void get_driver_status(const ComType com, const GetDriverStatus *data) {
	GetDriverStatusReturn gdsr;

	gdsr.header                    = data->header;
	gdsr.header.length             = sizeof(GetDriverStatusReturn);
	gdsr.open_load                 = (tmc2130_reg_drv_status.bit.ola << 0) | (tmc2130_reg_drv_status.bit.olb << 1);
	gdsr.short_to_ground           = (tmc2130_reg_drv_status.bit.s2ga << 0) | (tmc2130_reg_drv_status.bit.s2gb << 1);
	gdsr.over_temperature          = 0;
	if(tmc2130_reg_drv_status.bit.otpw) {
		gdsr.over_temperature      = 1;
	}
	if(tmc2130_reg_drv_status.bit.ot) {
		gdsr.over_temperature      = 2;
	}
	gdsr.motor_stalled             = tmc2130_reg_drv_status.bit.stall_guard;
	gdsr.actual_motor_current      = tmc2130_reg_drv_status.bit.cs_actual;
	gdsr.full_step_active          = tmc2130_reg_drv_status.bit.fsactive;
	gdsr.stallguard_result         = tmc2130_reg_drv_status.bit.sg_result;
	gdsr.stealth_voltage_amplitude = tmc2130_reg_pwm_scale.bit.amplitude_scalar;

	send_blocking_with_timeout(&gdsr, sizeof(GetDriverStatusReturn), com);
}

void set_minimum_voltage(const ComType com, const SetMinimumVoltage *data) {
	stepper_minimum_voltage = data->voltage;
	com_return_setter(com, data);
}

void get_minimum_voltage(const ComType com, const GetMinimumVoltage *data) {
	GetMinimumVoltageReturn gmvr;

	gmvr.header        = data->header;
	gmvr.header.length = sizeof(GetMinimumVoltageReturn);
	gmvr.voltage       = stepper_minimum_voltage;

	send_blocking_with_timeout(&gmvr, sizeof(GetMinimumVoltageReturn), com);
}

void set_time_base(const ComType com, const SetTimeBase *data) {
	stepper_time_base = data->time_base;

	com_return_setter(com, data);
}

void get_time_base(const ComType com, const GetTimeBase *data) {
	GetTimeBaseReturn gtbr;

	gtbr.header        = data->header;
	gtbr.header.length = sizeof(GetTimeBaseReturn);
	gtbr.time_base     = stepper_time_base;

	send_blocking_with_timeout(&gtbr, sizeof(GetTimeBaseReturn), com);
}

void get_all_data(const ComType com, const GetAllData *data) {
	GetAllDataReturn gadr;

	gadr.header              = data->header;
	gadr.header.length       = sizeof(GetAllDataReturn);
	gadr.current_velocity    = stepper_velocity > 0xFFFF ? 0xFFFF : stepper_velocity;
	gadr.current_position    = stepper_position;
	gadr.remaining_steps     = stepper_get_remaining_steps();
	gadr.stack_voltage       = stepper_get_stack_voltage();
	gadr.external_voltage    = stepper_get_external_voltage();
	gadr.current_consumption = (tmc2130_high_level.motor_run_current * 32) / tmc2130_reg_drv_status.bit.cs_actual;

	send_blocking_with_timeout(&gadr, sizeof(GetAllDataReturn), com);
}

void set_all_data_period(const ComType com, const SetAllDataPeriod *data) {
	stepper_all_data_period = data->period;

	com_return_setter(com, data);
}

void get_all_data_period(const ComType com, const GetAllDataPeriod *data) {
	GetAllDataPeriodReturn gadpr;

	gadpr.header        = data->header;
	gadpr.header.length = sizeof(GetAllDataPeriodReturn);
	gadpr.period        = stepper_all_data_period;

	send_blocking_with_timeout(&gadpr, sizeof(GetAllDataPeriodReturn), com);
}
