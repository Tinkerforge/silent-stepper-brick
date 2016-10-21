/* silent-stepper-brick
 * Copyright (C) 2015 Olaf Lüke <olaf@tinkerforge.com>
 *
 * tcm2130.c: TCM2130 configuration
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

#include "tcm2130.h"

#include <stdio.h>

#include "config.h"

#include "silent-stepper.h"

#include "bricklib/logging/logging.h"

#include "bricklib/drivers/pio/pio.h"
#include "bricklib/drivers/usart/usart.h"
#include "bricklib/drivers/dacc/dacc.h"
#include "bricklib/drivers/tc/tc.h"
#include "bricklib/drivers/pwmc/pwmc.h"

Pin pins_active[] = {PINS_ACTIVE};
Pin pin_3v3 = PIN_PWR_SW_3V3;

// Unused registers

// Read Only
TMC2130RegIOEN tmc2130_reg_ioen; // Reads state of pins, we can get this without reading a register
TMC2130RegMSCNT tmc2130_reg_mscnt; // Not needed if we don't use MSLUT
TMC2130RegMSCURACT tmc2130_reg_mscuract; // Not needed if we don't use MSLUT
TMC2130RegLOST_STEPS tmc2130_reg_lost_steps; // Not used in API (needs dcStep)

// Write Only
TMC2130RegVDCMIN tmc2130_reg_vdcmin = { // Not used in API
	.bit = {
		.velocity = 0
	}
};

TMC2130RegMSLUT tmc2130_reg_mslut[TMC2130_REG_MSLUT_NUM] = { // Not used in API
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } }
};

TMC2130RegMSLUTSEL tmc2130_reg_mslutsel = { // Not used in API
	.bit = {
		.w0 = 0,
		.w1 = 0,
		.w2 = 0,
		.w3 = 0,
		.x1 = 0,
		.x2 = 0,
		.x3 = 0,
	}
};

TMC2130RegMSLUTSTART tmc2130_reg_mslutstart = { // Not used in API
	.bit = {
		.start_sin = 0,
		.start_sin90 = 247
	}
};

TMC2130RegDCCTRL tmc2130_reg_dcctrl = { // Can only be used together with dcStep
	.bit = {
		.dc_time = 0,
		.dc_sg = 0
	}
};

TMC2130RegENCM_CTRL tmc2130_reg_encm_ctrl = { // Unnecessary
	.bit = {
		.inv = 0,
		.maxspeed = 0
	}
};

// Read/Write
TMC2130RegXDIRECT tmc2130_reg_xdirect = { // Not used in API
	.bit = {
		.coil_a = 0,
		.coil_b = 0
	}
};



// Used registers

// Read Only
TMC2130RegGSTAT tmc2130_reg_gstat; // Read + Clear upon read
TMC2130RegTSTEP tmc2130_reg_tstep; // Only needed if decide to use internal clock
TMC2130RegDRV_STATUS tmc2130_reg_drv_status;
TMC2130RegPWM_SCALE tmc2130_reg_pwm_scale;  // Can be used to detect motor stalling


// Write Only
TMC2130RegIHOLD_IRUN tmc2130_reg_ihold_run = { // velocity_based_mode_configuration
	.bit = {
		.ihold = 8,
		.irun = 31,
		.ihold_delay = 0
	}
};

TMC2130RegTPOWERDOWN tmc2130_reg_tpowerdown = { // velocity_based_mode_configuration
	.bit = {
		.delay = 64 // ca 1 second
	}
};

TMC2130RegTPWMTHRS tmc2130_reg_tpwmthrs = { // velocity_based_mode_configuration
	.bit = {
		.velocity = TCP2130_CLOCK_FREQUENCY/(500*256)
	}
};

TMC2130RegTCOOLTHRS tmc2130_reg_tcoolthrs = { // velocity_based_mode_configuration
	.bit = {
		.velocity = TCP2130_CLOCK_FREQUENCY/(500*256)
	}
};

TMC2130RegTHIGH tmc2130_reg_thigh = { // velocity_based_mode_configuration
	.bit = {
		.velocity = TCP2130_CLOCK_FREQUENCY/(1000*256)
	}
};

TMC2130RegCOOLCONF tmc2130_reg_coolconf = {
	.bit = {
		.semin = 2, // coolstep_configuration
		.seup = 0, // coolstep_configuration
		.semax = 10, // coolstep_configuration
		.sedn = 0,  // coolstep_configuration
		.seimin = 0, // coolstep_configuration
		.sgt = 0, // stallguard_configuration
		.sfilt = 0 // stallguard_configuration
	}
};

TMC2130RegPWMCONF tmc2130_reg_pwmconf = { // stealth_configuration
	.bit = {
		.pwm_ampl = 0x80,
		.pwm_grad = 4,
		.pwm_freq = 1, // We have a fixed external clock of 12.8MHz and with pwm_freq=1 we are about at the optimum of 40kHz PWM frequency
		.pwm_autoscale = 1, // automatic current control enabled
		.pwm_symmetric = 0,
		.freewheel = 0
	}
};

// Read/Write
TMC2130RegGCONF tmc2130_reg_gconf = {
	.bit = {
		.i_scale_analog         = 1, // Fixed
		.internal_rsense        = 0, // Fixed
		.en_pwm_mode            = 1, // stealth_configuration (Change only in stand still)
		.enc_commutation        = 0, // Fixed
		.shaft                  = 0, // Fixed ???
		.diag0_error            = 0, // Fixed
		.diag0_otpw             = 0, // Fixed
		.diag0_stall            = 0, // Fixed
		.diag1_stall            = 0, // Fixed
		.diag1_index            = 0, // Fixed
		.diag1_diag1_onstate    = 0, // Fixed
		.diag1_steps_skipped    = 0, // Fixed
		.diag0_int_pushpull     = 0, // Fixed
		.diag1_pushpull         = 0, // Fixed
		.small_hysteresis       = 0, // velocity_based_mode_configuration
		.stop_enable            = 0, // Fixed
		.direct_mode            = 0, // Fixed
		.test_mode              = 0  // Fixed, never change
	}
};

TMC2130RegCHOPCONF tmc2130_reg_chopconf = {
	.bit = {
		.toff = 4, // spreadcycle_configuration
		.hstrt = 0, // spreadcycle_configuration
		.hend = 0, // spreadcycle_configuration
		.fd3 = 0, // ???
		.disfdcc = 0, // spreadcycle_configuration
		.rndtf = 0, // misc_configuration
		.chm = 0, // spreadcycle_configuration
		.tbl = 1, // spreadcycle_configuration
		.vsense = 0, // Always 0
		.vhighfs = 0, // Always 0 (is only used for dcStep)
		.vhighchm = 0, // velocity_based_mode_configuration
		.sync = 0, // chopsync_configuration
		.mres = 0, // step_configuration
		.intpol = 1, // step_configuration
		.dedge = 1, // Always 1.
		.diss2g = 0 // misc_configuration
	}
};


uint32_t tcm2130_register_to_write_mask = 0;
uint32_t tcm2130_register_to_read_mask = 0;

uint8_t tcm2130_write_buffer[5];
uint8_t tcm2130_read_buffer[5];

uint32_t tcm2130_current_read_bit = 0;

bool tcm2130_is_active = false;

typedef enum {
	TCM2130_STATUS_IDLE,
	TCM2130_STATUS_READ_WRITE_REG,
	TCM2130_STATUS_READ_WRITE_REG_NEXT,
	TCM2130_STATUS_READ,
	TCM2130_STATUS_READ_DONE,
	TCM2130_STATUS_WRITE
} TCM2130Status;

TCM2130Status tcm2130_status = TCM2130_STATUS_IDLE;

void tcm2130_select(void) {
	USART0->US_CR |= US_CR_RTSEN;
    SLEEP_NS(250);
}

void tcm2130_deselect(void) {
   	SLEEP_NS(250);
	USART0->US_CR |= US_CR_RTSDIS;
	USART0->US_PTCR = US_PTCR_TXTDIS | US_PTCR_RXTDIS;
}

uint8_t tcm2130_spi_transceive_byte(const uint8_t value) {
	// Wait for transfer buffer to be empty
	while((USART0->US_CSR & US_CSR_TXEMPTY) == 0);
	USART0->US_THR = value;

	// Wait until receive buffer has new data
	while((USART0->US_CSR & US_CSR_RXRDY) == 0);
	return USART0->US_RHR;
}


void tcm2130_write_register(const uint8_t address, const uint32_t value, const bool busy_waiting) {
	if(tcm2130_status != TCM2130_STATUS_IDLE) {
		return;
	}

	tcm2130_select();
	tcm2130_write_buffer[0] = address | TCM2130_WRITE;
	tcm2130_write_buffer[1] = 0xFF & (value >> 24);
	tcm2130_write_buffer[2] = 0xFF & (value >> 16);
	tcm2130_write_buffer[3] = 0xFF & (value >>  8);
	tcm2130_write_buffer[4] = 0xFF &  value;
    USART0->US_TPR = (uint32_t)tcm2130_write_buffer;
    USART0->US_TCR = 5;
    USART0->US_RPR = (uint32_t)tcm2130_read_buffer;
    USART0->US_RCR = 5;
    USART0->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN;

    if(busy_waiting) {
    	while((!(USART0->US_CSR & US_CSR_TXBUFE)) ||
    	      (!(USART0->US_CSR & US_CSR_ENDTX))  ||
    	      (!(USART0->US_CSR & US_CSR_RXBUFF)) ||
    	      (!(USART0->US_CSR & US_CSR_ENDRX))) {
    		__NOP();
    	}
    	tcm2130_deselect();
	} else {
		tcm2130_status = TCM2130_STATUS_WRITE;
	}
}

uint32_t tcm2130_read_register(const uint8_t address, const bool busy_waiting) {
	if(tcm2130_status == TCM2130_STATUS_IDLE) {
		tcm2130_select();
		tcm2130_write_buffer[0] = address | TCM2130_READ;
		tcm2130_write_buffer[1] = 0;
		tcm2130_write_buffer[2] = 0;
		tcm2130_write_buffer[3] = 0;
		tcm2130_write_buffer[4] = 0;
		USART0->US_TPR = (uint32_t)tcm2130_write_buffer;
		USART0->US_TCR = 5;
		USART0->US_RPR = (uint32_t)tcm2130_read_buffer;
		USART0->US_RCR = 5;
		USART0->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN;

		if(busy_waiting) {
			while((!(USART0->US_CSR & US_CSR_TXBUFE)) ||
				  (!(USART0->US_CSR & US_CSR_ENDTX))  ||
				  (!(USART0->US_CSR & US_CSR_RXBUFF)) ||
				  (!(USART0->US_CSR & US_CSR_ENDRX))) {
				__NOP();
			}
			tcm2130_deselect();
		} else {
			tcm2130_status = TCM2130_STATUS_READ_WRITE_REG;
		}
	} else if(tcm2130_status == TCM2130_STATUS_READ_WRITE_REG_NEXT) {
		tcm2130_select();
		tcm2130_write_buffer[0] = 0;
		tcm2130_write_buffer[1] = 0;
		tcm2130_write_buffer[2] = 0;
		tcm2130_write_buffer[3] = 0;
		tcm2130_write_buffer[4] = 0;
		USART0->US_TPR = (uint32_t)tcm2130_write_buffer;
		USART0->US_TCR = 5;
		USART0->US_RPR = (uint32_t)tcm2130_read_buffer;
		USART0->US_RCR = 5;
		USART0->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN;

		if(busy_waiting) {
			while((!(USART0->US_CSR & US_CSR_TXBUFE)) ||
				  (!(USART0->US_CSR & US_CSR_ENDTX))  ||
				  (!(USART0->US_CSR & US_CSR_RXBUFF)) ||
				  (!(USART0->US_CSR & US_CSR_ENDRX))) {
				__NOP();
			}
			uint32_t value  = tcm2130_read_buffer[1] << 24;
			         value |= tcm2130_read_buffer[2] << 16;
			         value |= tcm2130_read_buffer[3] <<  8;
			         value |= tcm2130_read_buffer[4] <<  0;
			return value;
		} else {
			tcm2130_status = TCM2130_STATUS_READ;
		}
	} else if(tcm2130_status == TCM2130_STATUS_READ) {
		uint32_t value  = tcm2130_read_buffer[1] << 24;
		         value |= tcm2130_read_buffer[2] << 16;
		         value |= tcm2130_read_buffer[3] <<  8;
		         value |= tcm2130_read_buffer[4] <<  0;

		tcm2130_status = TCM2130_STATUS_READ_DONE;
		return value;
	}

	return 0;
}


void tcm2130_spi_init(void) {
	// Enable peripheral clock
	PMC->PMC_PCER0 = 1 << ID_USART0;

	// Configure the USART0 as SPI
	USART_Configure(USART0,
					US_MR_USART_MODE_SPI_MASTER |
					US_MR_USCLKS_MCK            |
					US_MR_CHRL_8_BIT            |
					US_MR_PAR_NO                |
					US_MR_CHMODE_NORMAL         |
					US_MR_CLKO                  |
					US_SPI_CPOL_1               |
					US_SPI_CPHA_0,
					SPI_FREQ,
					BOARD_MCK);

	// Enable receiver and transmitter
	USART0->US_CR = US_CR_TXEN;
	USART0->US_CR = US_CR_RXEN;


    NVIC_DisableIRQ(USART0_IRQn);
    NVIC_ClearPendingIRQ(USART0_IRQn);
    NVIC_SetPriority(USART0_IRQn, PRIORITY_USART_DMA);
    NVIC_EnableIRQ(USART0_IRQn);

	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_IHOLD_IRUN, tmc2130_reg_ihold_run.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_TPOWERDOWN, tmc2130_reg_tpowerdown.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_TPWMTHRS, tmc2130_reg_tpwmthrs.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_TCOOLTHRS, tmc2130_reg_tcoolthrs.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_THIGH, tmc2130_reg_thigh.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_COOLCONF, tmc2130_reg_coolconf.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_PWMCONF, tmc2130_reg_pwmconf.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_ENCM_CTRL, tmc2130_reg_encm_ctrl.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_GCONF, tmc2130_reg_gconf.reg, true);
	SLEEP_US(10);
	tcm2130_write_register(TMC2130_REG_CHOPCONF, tmc2130_reg_chopconf.reg, true);
	SLEEP_US(10);
}

void tcm2130_set_active(const bool active) {
	if(active == tcm2130_is_active) {
		return;
	}
	tcm2130_is_active = active;

	if(active) {
		PIO_Set(&pin_3v3);

		// Initialize and enable DACC to set VREF and DECAY pins
		DACC_Initialize(DACC,
						ID_DACC,
						0, // Hardware triggers are disabled
						0, // External trigger
						0, // Half-Word Transfer
						0, // Normal Mode (not sleep mode)
						BOARD_MCK,
						1, // refresh period
						0, // Channel 0 selection
						1, // Tag Selection Mode enabled
						16); //  value of the start up time
		DACC_EnableChannel(DACC, VREF_CHANNEL);

		// Enable peripheral clock for TC
		PMC->PMC_PCER0 = 1 << ID_TC0;

		// Configure and enable TC interrupts
		NVIC_DisableIRQ(TC0_IRQn);
		NVIC_ClearPendingIRQ(TC0_IRQn);
		NVIC_SetPriority(TC0_IRQn, PRIORITY_STEPPER_TC0);
		NVIC_EnableIRQ(TC0_IRQn);

		tc_channel_init(&STEPPER_TC_CHANNEL, TC_CMR_TCCLKS_TIMER_CLOCK5 | TC_CMR_CPCTRG);

		// Interrupt in compare
		tc_channel_interrupt_set(&STEPPER_TC_CHANNEL, TC_IER_CPCS);

		// Configure PWMH3 to be external clock
		PMC->PMC_PCER0 = 1 << ID_PWM;
		PWMC_ConfigureChannel(PWM, 3, PWM_CMR_CPRE_MCK, /*PWM_CMR_CALG*/ 0, PWM_CMR_CPOL);
		PWMC_SetPeriod(PWM, 3, 5);
		PWMC_SetDutyCycle(PWM, 3, 2);
		PWM->PWM_IER1 = PWM_IER1_CHID3;

		pins_active[PWR_ENABLE].type = PIO_OUTPUT_1;
		pins_active[PWR_ENABLE].attribute = PIO_DEFAULT;
		pins_active[PWR_STEP].type = PIO_OUTPUT_1;
		pins_active[PWR_STEP].attribute = PIO_DEFAULT;
		pins_active[PWR_DIRECTION].type = PIO_OUTPUT_0;
		pins_active[PWR_DIRECTION].attribute = PIO_DEFAULT;
		pins_active[PWR_VREF].type = PIO_INPUT;
		pins_active[PWR_VREF].attribute = PIO_DEFAULT;

		pins_active[PWR_CLK].type = PIO_PERIPH_C;
		pins_active[PWR_CLK].attribute = PIO_DEFAULT;
		pins_active[PWR_SW_3V3].type = PIO_OUTPUT_1;
		pins_active[PWR_SW_3V3].attribute = PIO_DEFAULT;

		pins_active[PWR_SDO].type = PIO_PERIPH_A;				// MOSI
		pins_active[PWR_SDI].type = PIO_PERIPH_A;				// MISO
		pins_active[PWR_SCK].type = PIO_PERIPH_B;				// Clock
		pins_active[PWR_CS].type  = PIO_PERIPH_A;	//PIO_OUTPUT_1; //			// Chip Select
		pins_active[CFG_DIAG0].type = PIO_INPUT;
		pins_active[CFG_DIAG0].attribute = PIO_PULLUP;
		pins_active[CFG_DIAG1].type = PIO_INPUT;
		pins_active[CFG_DIAG1].attribute = PIO_PULLUP;

		PIO_Configure(pins_active, PIO_LISTSIZE(pins_active));


		stepper_set_output_current(VREF_DEFAULT_CURRENT);

		PWMC_EnableChannel(PWM, 3);
		while(!(PWM->PWM_SR & (1 << 3))); // Wait for PWM to be active

		tcm2130_spi_init();
	} else {
		PIO_Clear(&pin_3v3);

		pins_active[PWR_ENABLE].type =  PIO_INPUT;
		pins_active[PWR_ENABLE].attribute = PIO_DEFAULT;
		pins_active[PWR_STEP].type = PIO_INPUT;
		pins_active[PWR_STEP].attribute = PIO_DEFAULT;
		pins_active[PWR_DIRECTION].type = PIO_INPUT;
		pins_active[PWR_DIRECTION].attribute = PIO_DEFAULT;
		pins_active[PWR_VREF].type = PIO_INPUT;
		pins_active[PWR_VREF].attribute = PIO_DEFAULT;

		pins_active[PWR_CLK].type = PIO_INPUT;
		pins_active[PWR_CLK].attribute = PIO_DEFAULT;
		pins_active[PWR_SW_3V3].type = PIO_INPUT;
		pins_active[PWR_SW_3V3].attribute = PIO_DEFAULT;

		pins_active[PWR_SDO].type = PIO_INPUT;
		pins_active[PWR_SDO].attribute = PIO_DEFAULT;
		pins_active[PWR_SDI].type = PIO_INPUT;
		pins_active[PWR_SDI].attribute = PIO_DEFAULT;
		pins_active[PWR_SCK].type = PIO_INPUT;
		pins_active[PWR_SCK].attribute = PIO_DEFAULT;
		pins_active[PWR_CS].type = PIO_INPUT;
		pins_active[PWR_CS].attribute = PIO_DEFAULT;
		pins_active[CFG_DIAG0].type = PIO_INPUT;
		pins_active[CFG_DIAG0].attribute = PIO_DEFAULT;
		pins_active[CFG_DIAG1].type = PIO_INPUT;
		pins_active[CFG_DIAG1].attribute = PIO_DEFAULT;

		PIO_Configure(pins_active, PIO_LISTSIZE(pins_active));

		PMC->PMC_PCER0 &= ~(1 << ID_USART0);

		// Disable receiver and transmitter
		USART0->US_CR = US_CR_TXDIS;
		USART0->US_CR = US_CR_RXDIS;

		tcm2130_status = TCM2130_STATUS_IDLE;
	}
}

void tcm2130_read_register_by_bit(uint32_t register_bit) {
	tcm2130_current_read_bit = register_bit;
	uint32_t *read_address = NULL;

	uint8_t reg = 0;
	switch(register_bit) {
		case TMC2130_REG_GSTAT_BIT:      reg = TMC2130_REG_GSTAT;      read_address = &tmc2130_reg_gstat.reg;      break;
		case TMC2130_REG_TSTEP_BIT:      reg = TMC2130_REG_TSTEP;      read_address = &tmc2130_reg_tstep.reg;      break;
		case TMC2130_REG_DRV_STATUS_BIT: reg = TMC2130_REG_DRV_STATUS; read_address = &tmc2130_reg_drv_status.reg; break;
		case TMC2130_REG_PWM_SCALE_BIT:  reg = TMC2130_REG_PWM_SCALE;  read_address = &tmc2130_reg_pwm_scale.reg;  break;
	}

	uint32_t value = tcm2130_read_register(reg, false);
	if(tcm2130_status == TCM2130_STATUS_READ_DONE) {
		if(read_address != NULL) {
			*read_address = value;
		}
	}
}

void tcm2130_write_register_by_bit(uint32_t register_bit) {
	switch(register_bit) {
		case TMC2130_REG_GCONF_BIT:      tcm2130_write_register(TMC2130_REG_GCONF,      tmc2130_reg_gconf.reg,      false); break;
		case TMC2130_REG_IHOLD_IRUN_BIT: tcm2130_write_register(TMC2130_REG_IHOLD_IRUN, tmc2130_reg_ihold_run.reg,  false); break;
		case TMC2130_REG_TPOWERDOWN_BIT: tcm2130_write_register(TMC2130_REG_TPOWERDOWN, tmc2130_reg_tpowerdown.reg, false); break;
		case TMC2130_REG_TPWMTHRS_BIT:   tcm2130_write_register(TMC2130_REG_TPWMTHRS,   tmc2130_reg_tpwmthrs.reg,   false); break;
		case TMC2130_REG_TCOOLTHRS_BIT:  tcm2130_write_register(TMC2130_REG_TCOOLTHRS,  tmc2130_reg_tcoolthrs.reg,  false); break;
		case TMC2130_REG_THIGH_BIT:      tcm2130_write_register(TMC2130_REG_THIGH,      tmc2130_reg_thigh.reg,      false); break;
		case TMC2130_REG_XDIRECT_BIT:    tcm2130_write_register(TMC2130_REG_XDIRECT,    tmc2130_reg_xdirect.reg,    false); break;
		case TMC2130_REG_VDCMIN_BIT:     tcm2130_write_register(TMC2130_REG_VDCMIN,     tmc2130_reg_vdcmin.reg,     false); break;
		case TMC2130_REG_MSLUT0_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT0,     tmc2130_reg_mslut[0].reg,   false); break;
		case TMC2130_REG_MSLUT1_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT1,     tmc2130_reg_mslut[1].reg,   false); break;
		case TMC2130_REG_MSLUT2_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT2,     tmc2130_reg_mslut[2].reg,   false); break;
		case TMC2130_REG_MSLUT3_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT3,     tmc2130_reg_mslut[3].reg,   false); break;
		case TMC2130_REG_MSLUT4_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT4,     tmc2130_reg_mslut[4].reg,   false); break;
		case TMC2130_REG_MSLUT5_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT5,     tmc2130_reg_mslut[5].reg,   false); break;
		case TMC2130_REG_MSLUT6_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT6,     tmc2130_reg_mslut[6].reg,   false); break;
		case TMC2130_REG_MSLUT7_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT7,     tmc2130_reg_mslut[7].reg,   false); break;
		case TMC2130_REG_MSLUTSEL_BIT:   tcm2130_write_register(TMC2130_REG_MSLUTSEL,   tmc2130_reg_mslutsel.reg,   false); break;
		case TMC2130_REG_MSLUTSTART_BIT: tcm2130_write_register(TMC2130_REG_MSLUTSTART, tmc2130_reg_mslutstart.reg, false); break;
		case TMC2130_REG_CHOPCONF_BIT:   tcm2130_write_register(TMC2130_REG_CHOPCONF,   tmc2130_reg_chopconf.reg,   false); break;
		case TMC2130_REG_COOLCONF_BIT:   tcm2130_write_register(TMC2130_REG_COOLCONF,   tmc2130_reg_coolconf.reg,   false); break;
		case TMC2130_REG_DCCTRL_BIT:     tcm2130_write_register(TMC2130_REG_DCCTRL,     tmc2130_reg_dcctrl.reg,     false); break;
		case TMC2130_REG_PWMCONF_BIT:    tcm2130_write_register(TMC2130_REG_PWMCONF,    tmc2130_reg_pwmconf.reg,    false); break;
		case TMC2130_REG_ENCM_CTRL_BIT:  tcm2130_write_register(TMC2130_REG_ENCM_CTRL,  tmc2130_reg_encm_ctrl.reg,  false); break;
	}
}

void tcm2130_handle_register_read_and_write(void) {
	if(!tcm2130_is_active) {
		return;
	}

	switch(tcm2130_status) {
		case TCM2130_STATUS_WRITE: {
			if(USART0->US_CSR & US_CSR_OVRE) {
				USART0->US_CR = US_CR_RSTSTA;
				tcm2130_status = TCM2130_STATUS_IDLE;
				tcm2130_deselect();
				return;
			}
	    	if((USART0->US_CSR & US_CSR_TXBUFE)  &&
	    	   (USART0->US_CSR & US_CSR_ENDTX)  &&
	    	   (USART0->US_CSR & US_CSR_RXBUFF) &&
	    	   (USART0->US_CSR & US_CSR_ENDRX)) {

	    		// Write is done, we can go back to idle
	    		tcm2130_status = TCM2130_STATUS_IDLE;
	    		tcm2130_deselect();
	    		return;
	    	}

	    	break;
		}

		case TCM2130_STATUS_READ_WRITE_REG: {
			if(USART0->US_CSR & US_CSR_OVRE) {
				USART0->US_CR = US_CR_RSTSTA;
				tcm2130_status = TCM2130_STATUS_IDLE;
				tcm2130_deselect();
				return;
			}
	    	if((USART0->US_CSR & US_CSR_TXBUFE)  &&
	    	   (USART0->US_CSR & US_CSR_ENDTX)  &&
	    	   (USART0->US_CSR & US_CSR_RXBUFF) &&
	    	   (USART0->US_CSR & US_CSR_ENDRX)) {

	    		tcm2130_status = TCM2130_STATUS_READ_WRITE_REG_NEXT;
				tcm2130_deselect();
	    		return;
	    	}
			break;
		}

		case TCM2130_STATUS_READ_WRITE_REG_NEXT: {
			tcm2130_read_register_by_bit(tcm2130_current_read_bit);
			return;
		}

		case TCM2130_STATUS_READ: {
			if(USART0->US_CSR & US_CSR_OVRE) {
				USART0->US_CR = US_CR_RSTSTA;
				tcm2130_status = TCM2130_STATUS_IDLE;
				tcm2130_deselect();
				return;
			}
	    	if((USART0->US_CSR & US_CSR_TXBUFE)  &&
	    	   (USART0->US_CSR & US_CSR_ENDTX)  &&
	    	   (USART0->US_CSR & US_CSR_RXBUFF) &&
	    	   (USART0->US_CSR & US_CSR_ENDRX)) {

	    		tcm2130_read_register_by_bit(tcm2130_current_read_bit);
				tcm2130_status = TCM2130_STATUS_IDLE;
				tcm2130_deselect();
	    		return;
	    	}
			break;
		}

		case TCM2130_STATUS_IDLE: {
			if(tcm2130_register_to_write_mask != 0) {
				for(uint8_t i = 0; i < TMC2130_NUM_REGS_TO_WRITE; i++) {
					const uint32_t bit = 1 << i;
					if(tcm2130_register_to_write_mask & bit) {
						tcm2130_write_register_by_bit(bit);
						tcm2130_register_to_write_mask &= ~bit;
						return;
					}
				}
			}
			if(tcm2130_register_to_read_mask != 0) {
				for(uint8_t i = 0; i < TMC2130_NUM_REGS_TO_READ; i++) {
					const uint32_t bit = 1 << i;
					if(tcm2130_register_to_read_mask & bit) {
						tcm2130_read_register_by_bit(bit);
						tcm2130_register_to_read_mask &= ~bit;
						return;
					}
				}
			}

			break;
		}

		default: {
			logw("Unreachable tcm2130 status: %d\n\r", tcm2130_status);
			break;
		}
	}

}

void tcm2130_update_read_registers(void) {
	if(tcm2130_is_active) {
		tcm2130_register_to_read_mask = TMC2130_REG_DRV_STATUS_BIT | TMC2130_REG_PWM_SCALE_BIT;
	}
}

void tcm2130_print_current_state(void) {
	const uint8_t regs[] = {
		0x00, 0x01, 0x04, 0x10, 0x11, 0x12, 0x13, 0x14,
		0x15, 0x2D, 0x33, 0x60, 0x61, 0x62, 0x63, 0x64,
		0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C,
		0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73
	};

	printf("TCM2130 state:\n\r");
	for(uint8_t i = 0; i < sizeof(regs); i++) {
		uint32_t value = tcm2130_read_register(regs[i], true);
		printf(" * Reg %x: %lx\n\r", regs[i], value);
	}
	printf("\n\r");

}
