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
		.ihold = 16,
		.irun = 8,
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
		.intpol = 0, // step_configuration
		.dedge = 1, // Always 1.
		.diss2g = 0 // misc_configuration
	}
};


uint32_t tcm2130_register_to_write_mask = 0;

void tcm2130_select(void) {
	__disable_irq();
	PIO_Clear(&pins_active[PWR_CS]);
	SLEEP_NS(250);
}

void tcm2130_deselect(void) {
	PIO_Set(&pins_active[PWR_CS]);
	__enable_irq();
	SLEEP_NS(250);
}

uint8_t tcm2130_spi_transceive_byte(const uint8_t value) {
	// Wait for transfer buffer to be empty
	while((USART0->US_CSR & US_CSR_TXEMPTY) == 0);
	USART0->US_THR = value;

	// Wait until receive buffer has new data
	while((USART0->US_CSR & US_CSR_RXRDY) == 0);
	return USART0->US_RHR;
}

uint8_t tcm2130_write_buffer_tx[5];
uint8_t tcm2130_read_buffer_tx[5*2];
uint8_t tcm2130_read_buffer_rx[5*2];

void tcm2130_write_register(const uint8_t address, const uint32_t value) {
//return;
/*
	tcm2130_write_buffer_tx[0] = address | TCM2130_WRITE;
	tcm2130_write_buffer_tx[1] = 0xFF & (value >>24);
	tcm2130_write_buffer_tx[2] = 0xFF & (value >>16);
	tcm2130_write_buffer_tx[3] = 0xFF & (value >>8);
	tcm2130_write_buffer_tx[4] = 0xFF & value;
    USART0->US_TPR = (uint32_t)tcm2130_write_buffer_tx;
    USART0->US_TCR = 5;
    USART0->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTDIS;


	while((USART0->US_TCR != 0) || (USART0->US_RCR != 0)) {
		__NOP();
		printf("wait for write to finish\n\r");
	}*/


	tcm2130_select();
	tcm2130_spi_transceive_byte(address | TCM2130_WRITE);
	tcm2130_spi_transceive_byte(0xFF & (value >> 24));
	tcm2130_spi_transceive_byte(0xFF & (value >> 16));
	tcm2130_spi_transceive_byte(0xFF & (value >> 8));
	tcm2130_spi_transceive_byte(0xFF & value);
	tcm2130_deselect();
/*
	tcm2130_select();
	uint8_t res1 = tcm2130_spi_transceive_byte(0);
	uint8_t res2 = tcm2130_spi_transceive_byte(0);
	uint8_t res3 = tcm2130_spi_transceive_byte(0);
	uint8_t res4 = tcm2130_spi_transceive_byte(0);
	uint8_t res5 = tcm2130_spi_transceive_byte(0);
	tcm2130_deselect();
	printf("res: %d %d %d %d %d\n\r", res1, res2, res3, res4, res5);*/
}

uint32_t tcm2130_read_register(const uint8_t address) {
	tcm2130_select();
	tcm2130_spi_transceive_byte(address | TCM2130_READ);
	tcm2130_spi_transceive_byte(0);
	tcm2130_spi_transceive_byte(0);
	tcm2130_spi_transceive_byte(0);
	tcm2130_spi_transceive_byte(0);
	tcm2130_deselect();

	tcm2130_select();
	tcm2130_spi_transceive_byte(0); // drop status
	uint32_t value = tcm2130_spi_transceive_byte(0) << 24;
	value |= tcm2130_spi_transceive_byte(0) << 16;
	value |= tcm2130_spi_transceive_byte(0) << 8;
	value |= tcm2130_spi_transceive_byte(0);
	tcm2130_deselect();

	return value;
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

	/*
    NVIC_DisableIRQ(USART0_IRQn);
    NVIC_ClearPendingIRQ(USART0_IRQn);
    NVIC_SetPriority(USART0_IRQn, PRIORITY_USART_DMA);
    NVIC_EnableIRQ(USART0_IRQn);*/
/*
	Pin pins_config[] = {PINS_CONFIG};
	pins_config[PWR_SDO].type = PIO_PERIPH_A;				// MOSI
	pins_config[PWR_SDI].type = PIO_PERIPH_A;				// MISO
	pins_config[PWR_SCK].type = PIO_PERIPH_B;				// Clock
	pins_config[PWR_CS].type  = PIO_PERIPH_A;				// Chip Select
	PIO_Configure(pins_config, PIO_LISTSIZE(pins_config));
*/
	//tcm2130_print_current_state();

/*	tcm2130_write_register(TMC2130_REG_GCONF, 0x00000001UL);
	tcm2130_write_register(TMC2130_REG_IHOLD_IRUN, 0x00001010UL);
	tcm2130_write_register(TMC2130_REG_CHOPCONF, 0x00008008UL);*/

	printf("%lx %lx %lx\n\r", tmc2130_reg_gconf.reg, tmc2130_reg_ihold_run.reg, tmc2130_reg_chopconf.reg);

	tcm2130_write_register(TMC2130_REG_IHOLD_IRUN, tmc2130_reg_ihold_run.reg);
	tcm2130_write_register(TMC2130_REG_TPOWERDOWN, tmc2130_reg_tpowerdown.reg);
	tcm2130_write_register(TMC2130_REG_TPWMTHRS, tmc2130_reg_tpwmthrs.reg);
	tcm2130_write_register(TMC2130_REG_TCOOLTHRS, tmc2130_reg_tcoolthrs.reg);
	tcm2130_write_register(TMC2130_REG_THIGH, tmc2130_reg_thigh.reg);
	tcm2130_write_register(TMC2130_REG_COOLCONF, tmc2130_reg_coolconf.reg);
	tcm2130_write_register(TMC2130_REG_PWMCONF, tmc2130_reg_pwmconf.reg);
	tcm2130_write_register(TMC2130_REG_ENCM_CTRL, tmc2130_reg_encm_ctrl.reg);
	tcm2130_write_register(TMC2130_REG_GCONF, tmc2130_reg_gconf.reg);
	tcm2130_write_register(TMC2130_REG_CHOPCONF, tmc2130_reg_chopconf.reg);

	tcm2130_print_current_state();
}

void tcm2130_set_active(const bool active) {
	static bool last_active = false;

	if(active == last_active) {
		return;
	}
	last_active = active;

	printf("set_active: %d\n\r", active);

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


		/*// Configure TC2 to be external clock
	    PMC->PMC_PCER0 = 1 << ID_TC2;
		tc_channel_init(&STEPPER_CLK_CHANNEL,
						TC_CMR_WAVE |
						TC_CMR_TCCLKS_TIMER_CLOCK1 |
						TC_CMR_EEVT_XC0 |
						TC_CMR_BCPB_CLEAR |
						TC_CMR_BCPC_SET |
						TC_CMR_WAVSEL_UP_RC);

		tc_channel_stop(&STEPPER_CLK_CHANNEL);
		TC0->TC_WPMR &=  ~(1 << 0);	// clear write protection

		STEPPER_CLK_CHANNEL.TC_RB = 1;
		STEPPER_CLK_CHANNEL.TC_RC = 3;
		STEPPER_CLK_CHANNEL.TC_IER = TC_IER_CPCS;*/

		// Configure PWMH3 to be external clock
		// TODO: We probably have to enable this before we do PIO_Set(&pin_3v3)!
#if 0
		// Unten enable nicht vergessen einzukommentieren!
		PMC->PMC_PCER0 = 1 << ID_PWM;
		PWMC_ConfigureChannel(PWM, 3, PWM_CMR_CPRE_MCK, /*PWM_CMR_CALG*/ 0, PWM_CMR_CPOL);
		PWMC_SetPeriod(PWM, 3, 5);
		PWMC_SetDutyCycle(PWM, 3, 2);

		PWM->PWM_IER1 = PWM_IER1_CHID3;
#endif



		pins_active[PWR_ENABLE].type = PIO_OUTPUT_1;
		pins_active[PWR_ENABLE].attribute = PIO_DEFAULT;
		pins_active[PWR_STEP].type = PIO_OUTPUT_1;
		pins_active[PWR_STEP].attribute = PIO_DEFAULT;
		pins_active[PWR_DIRECTION].type = PIO_OUTPUT_0;
		pins_active[PWR_DIRECTION].attribute = PIO_DEFAULT;
		pins_active[PWR_VREF].type = PIO_INPUT;
		pins_active[PWR_VREF].attribute = PIO_DEFAULT;

		pins_active[PWR_CLK].type = PIO_OUTPUT_0;//PIO_PERIPH_C;
		pins_active[PWR_CLK].attribute = PIO_DEFAULT;
		pins_active[PWR_SW_3V3].type = PIO_OUTPUT_1;
		pins_active[PWR_SW_3V3].attribute = PIO_DEFAULT;

		pins_active[PWR_SDO].type = PIO_PERIPH_A;				// MOSI
		pins_active[PWR_SDI].type = PIO_PERIPH_A;				// MISO
		pins_active[PWR_SCK].type = PIO_PERIPH_B;				// Clock
		pins_active[PWR_CS].type  = PIO_OUTPUT_1; //PIO_PERIPH_A;				// Chip Select
		pins_active[CFG_DIAG0].type = PIO_INPUT;
		pins_active[CFG_DIAG0].attribute = PIO_PULLUP;
		pins_active[CFG_DIAG1].type = PIO_INPUT;
		pins_active[CFG_DIAG1].attribute = PIO_PULLUP;

		PIO_Configure(pins_active, PIO_LISTSIZE(pins_active));


		stepper_set_output_current(VREF_DEFAULT_CURRENT);

#if 0
		printf("before enable\n\r");
		PWMC_EnableChannel(PWM, 3);
		while(!(PWM->PWM_SR & (1 << 3)));
		printf("after enable\n\r");
#endif
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
	}
}

void tcm2130_write_register_by_bit(uint32_t register_bit) {
	switch(register_bit) {
		case TMC2130_REG_GCONF_BIT:      tcm2130_write_register(TMC2130_REG_GCONF,      tmc2130_reg_gconf.reg);      break;
		case TMC2130_REG_IHOLD_IRUN_BIT: tcm2130_write_register(TMC2130_REG_IHOLD_IRUN, tmc2130_reg_ihold_run.reg);  break;
		case TMC2130_REG_TPOWERDOWN_BIT: tcm2130_write_register(TMC2130_REG_TPOWERDOWN, tmc2130_reg_tpowerdown.reg); break;
		case TMC2130_REG_TPWMTHRS_BIT:   tcm2130_write_register(TMC2130_REG_TPWMTHRS,   tmc2130_reg_tpwmthrs.reg);   break;
		case TMC2130_REG_TCOOLTHRS_BIT:  tcm2130_write_register(TMC2130_REG_TCOOLTHRS,  tmc2130_reg_tcoolthrs.reg);  break;
		case TMC2130_REG_THIGH_BIT:      tcm2130_write_register(TMC2130_REG_THIGH,      tmc2130_reg_thigh.reg);      break;
		case TMC2130_REG_XDIRECT_BIT:    tcm2130_write_register(TMC2130_REG_XDIRECT,    tmc2130_reg_xdirect.reg);    break;
		case TMC2130_REG_VDCMIN_BIT:     tcm2130_write_register(TMC2130_REG_VDCMIN,     tmc2130_reg_vdcmin.reg);     break;
		case TMC2130_REG_MSLUT0_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT0,     tmc2130_reg_mslut[0].reg);   break;
		case TMC2130_REG_MSLUT1_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT1,     tmc2130_reg_mslut[1].reg);   break;
		case TMC2130_REG_MSLUT2_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT2,     tmc2130_reg_mslut[2].reg);   break;
		case TMC2130_REG_MSLUT3_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT3,     tmc2130_reg_mslut[3].reg);   break;
		case TMC2130_REG_MSLUT4_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT4,     tmc2130_reg_mslut[4].reg);   break;
		case TMC2130_REG_MSLUT5_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT5,     tmc2130_reg_mslut[5].reg);   break;
		case TMC2130_REG_MSLUT6_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT6,     tmc2130_reg_mslut[6].reg);   break;
		case TMC2130_REG_MSLUT7_BIT:     tcm2130_write_register(TMC2130_REG_MSLUT7,     tmc2130_reg_mslut[7].reg);   break;
		case TMC2130_REG_MSLUTSEL_BIT:   tcm2130_write_register(TMC2130_REG_MSLUTSEL,   tmc2130_reg_mslutsel.reg);   break;
		case TMC2130_REG_MSLUTSTART_BIT: tcm2130_write_register(TMC2130_REG_MSLUTSTART, tmc2130_reg_mslutstart.reg); break;
		case TMC2130_REG_CHOPCONF_BIT:   tcm2130_write_register(TMC2130_REG_CHOPCONF,   tmc2130_reg_chopconf.reg);   break;
		case TMC2130_REG_COOLCONF_BIT:   tcm2130_write_register(TMC2130_REG_COOLCONF,   tmc2130_reg_coolconf.reg);   break;
		case TMC2130_REG_DCCTRL_BIT:     tcm2130_write_register(TMC2130_REG_DCCTRL,     tmc2130_reg_dcctrl.reg);     break;
		case TMC2130_REG_PWMCONF_BIT:    tcm2130_write_register(TMC2130_REG_PWMCONF,    tmc2130_reg_pwmconf.reg);    break;
		case TMC2130_REG_ENCM_CTRL_BIT:  tcm2130_write_register(TMC2130_REG_ENCM_CTRL,  tmc2130_reg_encm_ctrl.reg);  break;
	}
}

void tcm2130_handle_register_write(void) {
	if(tcm2130_register_to_write_mask == 0) {
		return;
	}

	for(uint8_t i = 0; i < TMC2130_NUM_REGS_TO_WRITE; i++) {
		const uint32_t bit = 1 << i;
		if(tcm2130_register_to_write_mask & bit) {
			tcm2130_write_register_by_bit(bit);
			tcm2130_register_to_write_mask &= ~bit;
			return;
		}
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
		uint32_t value = tcm2130_read_register(regs[i]);
		printf(" * Reg %x: %lx\n\r", regs[i], value);
	}
	printf("\n\r");

}
