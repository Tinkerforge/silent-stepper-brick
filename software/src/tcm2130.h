/* silent-stepper-brick
 * Copyright (C) 2015 Olaf Lüke <olaf@tinkerforge.com>
 *
 * tcm2130.h: TCM2130 configuration
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

#ifndef TCM2130_H
#define TCM2130_H

#include <stdint.h>
#include <stdbool.h>

#define TCP2130_CLOCK_FREQUENCY ((uint32_t)12800000) // 12.8MHz supplied by external clock (PWM with period 5, duty cycle 2 @mck)

// ****************** TCM2130 REGISTERS *****************
// R is read-only / W is write-only / R+C is clear upon read

// General Configuration (0x00...0x0F)
#define TMC2130_REG_GCONF       0x00 // RW
#define TMC2130_REG_GSTAT       0x01 // R+C
#define TMC2130_REG_IOIN		0x04 // R

// Velocity dependent driver feature control (0x10...0x1F)
#define TMC2130_REG_IHOLD_IRUN  0x10 //  W
#define TMC2130_REG_TPOWERDOWN  0x11 //  W
#define TMC2130_REG_TSTEP       0x12 // R
#define TMC2130_REG_TPWMTHRS    0x13 //  W
#define TMC2130_REG_TCOOLTHRS   0x14 //  W
#define TMC2130_REG_THIGH       0x15 //  W

// SPI mode (0x2D)
#define TMC2130_REG_XDIRECT     0x2D // RW

// DC-Step minimum velocity (0x33)
#define TMC2130_REG_VDCMIN      0x33 //  W

// Micro stepping control (0x60...0x6B)
#define TMC2130_REG_MSLUT0      0x60 //  W
#define TMC2130_REG_MSLUT1      0x61 //  W
#define TMC2130_REG_MSLUT2      0x62 //  W
#define TMC2130_REG_MSLUT3      0x63 //  W
#define TMC2130_REG_MSLUT4      0x64 //  W
#define TMC2130_REG_MSLUT5      0x65 //  W
#define TMC2130_REG_MSLUT6      0x66 //  W
#define TMC2130_REG_MSLUT7      0x67 //  W
#define TMC2130_REG_MSLUTSEL    0x68 //  W
#define TMC2130_REG_MSLUTSTART  0x69 //  W
#define TMC2130_REG_MSCNT       0x6A // R
#define TMC2130_REG_MSCURACT    0x6B // R

// Driver (0x6C...0x7F)
#define TMC2130_REG_CHOPCONF    0x6C // RW
#define TMC2130_REG_COOLCONF    0x6D //  W
#define TMC2130_REG_DCCTRL      0x6E //  W
#define TMC2130_REG_DRV_STATUS  0x6F // R
#define TMC2130_REG_PWMCONF     0x70 //  W
#define TMC2130_REG_PWM_SCALE   0x71 // R
#define TMC2130_REG_ENCM_CTRL   0x72 //  W
#define TMC2130_REG_LOST_STEPS  0x73 // R

#define TMC2130_REG_MSLUT_NUM   8
#define TCM2130_READ            0
#define TCM2130_WRITE           0x80

#define TMC2130_REG_GCONF_BIT       (1 << 0)
#define TMC2130_REG_IHOLD_IRUN_BIT  (1 << 1)
#define TMC2130_REG_TPOWERDOWN_BIT  (1 << 2)
#define TMC2130_REG_TPWMTHRS_BIT    (1 << 3)
#define TMC2130_REG_TCOOLTHRS_BIT   (1 << 4)
#define TMC2130_REG_THIGH_BIT       (1 << 5)
#define TMC2130_REG_XDIRECT_BIT     (1 << 6)
#define TMC2130_REG_VDCMIN_BIT      (1 << 7)
#define TMC2130_REG_MSLUT0_BIT      (1 << 8)
#define TMC2130_REG_MSLUT1_BIT      (1 << 9)
#define TMC2130_REG_MSLUT2_BIT      (1 << 10)
#define TMC2130_REG_MSLUT3_BIT      (1 << 11)
#define TMC2130_REG_MSLUT4_BIT      (1 << 12)
#define TMC2130_REG_MSLUT5_BIT      (1 << 13)
#define TMC2130_REG_MSLUT6_BIT      (1 << 14)
#define TMC2130_REG_MSLUT7_BIT      (1 << 15)
#define TMC2130_REG_MSLUTSEL_BIT    (1 << 16)
#define TMC2130_REG_MSLUTSTART_BIT  (1 << 17)
#define TMC2130_REG_CHOPCONF_BIT    (1 << 18)
#define TMC2130_REG_COOLCONF_BIT    (1 << 19)
#define TMC2130_REG_DCCTRL_BIT      (1 << 20)
#define TMC2130_REG_PWMCONF_BIT     (1 << 21)
#define TMC2130_REG_ENCM_CTRL_BIT   (1 << 22)

#define TMC2130_REG_GSTAT_BIT       (1 << 0)
#define TMC2130_REG_TSTEP_BIT       (1 << 1)
#define TMC2130_REG_DRV_STATUS_BIT  (1 << 2)
#define TMC2130_REG_PWM_SCALE_BIT   (1 << 3)

#define TMC2130_NUM_REGS_TO_WRITE   23
#define TMC2130_NUM_REGS_TO_READ    4

typedef union {
  struct {
    uint32_t i_scale_analog:1;
    uint32_t internal_rsense:1;
    uint32_t en_pwm_mode:1;
    uint32_t enc_commutation:1;
    uint32_t shaft:1;
    uint32_t diag0_error:1;
    uint32_t diag0_otpw:1;
    uint32_t diag0_stall:1;
    uint32_t diag1_stall:1;
    uint32_t diag1_index:1;
    uint32_t diag1_diag1_onstate:1;
    uint32_t diag1_steps_skipped:1;
    uint32_t diag0_int_pushpull:1;
    uint32_t diag1_pushpull:1;
    uint32_t small_hysteresis:1;
    uint32_t stop_enable:1;
    uint32_t direct_mode:1;
    uint32_t test_mode:1;
  } bit;
  uint32_t reg;
} TMC2130RegGCONF;

typedef union {
  struct {
    uint32_t reset:1;
    uint32_t drv_err:1;
    uint32_t uv_cp:1;
  } bit;
  uint32_t reg;
} TMC2130RegGSTAT;

typedef union {
  struct {
    uint32_t step:1;
    uint32_t dir:1;
    uint32_t dcen_cfg4:1;
    uint32_t dcin_cfg5:1;
    uint32_t drv_enn_cfg6:1;
    uint32_t dco:1;
    uint32_t always_one:1;
    uint32_t dont_care:1;
    uint32_t :16;
    uint32_t version:8;
  } bit;
  uint32_t reg;
} TMC2130RegIOEN;

typedef union {
  struct {
    uint32_t ihold:5;
    uint32_t :3;
    uint32_t irun:5;
    uint32_t :3;
    uint32_t ihold_delay:4;
  } bit;
  uint32_t reg;
} TMC2130RegIHOLD_IRUN;

typedef union {
  struct {
    uint32_t delay:8;
  } bit;
  uint32_t reg;
} TMC2130RegTPOWERDOWN;

typedef union {
  struct {
    uint32_t time:20;
  } bit;
  uint32_t reg;
} TMC2130RegTSTEP;

typedef union {
  struct {
    uint32_t velocity:20;
  } bit;
  uint32_t reg;
} TMC2130RegTPWMTHRS;

typedef union {
  struct {
    uint32_t velocity:20;
  } bit;
  uint32_t reg;
} TMC2130RegTCOOLTHRS;

typedef union {
  struct {
    uint32_t velocity:20;
  } bit;
  uint32_t reg;
} TMC2130RegTHIGH;

typedef union {
  struct {
    uint32_t coil_a:9;
    uint32_t :7;
    uint32_t coil_b:9;
  } bit;
  uint32_t reg;
} TMC2130RegXDIRECT;

typedef union {
  struct {
    uint32_t velocity:23;
  } bit;
  uint32_t reg;
} TMC2130RegVDCMIN;

typedef union {
  struct {
    uint32_t table_entry:32;
  } bit;
  uint32_t reg;
} TMC2130RegMSLUT;

typedef union {
  struct {
    uint32_t w0:2;
    uint32_t w1:2;
    uint32_t w2:2;
    uint32_t w3:2;
    uint32_t x1:8;
    uint32_t x2:8;
    uint32_t x3:8;
  } bit;
  uint32_t reg;
} TMC2130RegMSLUTSEL;

typedef union {
  struct {
    uint32_t start_sin:8;
    uint32_t :8;
    uint32_t start_sin90:8;
  } bit;
  uint32_t reg;
} TMC2130RegMSLUTSTART;

typedef union {
  struct {
    uint32_t counter:10;
  } bit;
  uint32_t reg;
} TMC2130RegMSCNT;

typedef union {
  struct {
    uint32_t cuar_a:9;
    uint32_t :7;
    uint32_t cuar_b:9;
  } bit;
  uint32_t reg;
} TMC2130RegMSCURACT;

typedef union {
  struct {
    uint32_t toff:4;
    uint32_t hstrt:3;
    uint32_t hend:4;
    uint32_t fd3:1;
    uint32_t disfdcc:1;
    uint32_t rndtf:1;
    uint32_t chm:1;
    uint32_t tbl:2;
    uint32_t vsense:1;
    uint32_t vhighfs:1;
    uint32_t vhighchm:1;
    uint32_t sync:4;
    uint32_t mres:4;
    uint32_t intpol:1;
    uint32_t dedge:1;
    uint32_t diss2g:1;
    uint32_t :1; // Set to 0
  } bit;
  uint32_t reg;
} TMC2130RegCHOPCONF;

typedef union {
  struct {
    uint32_t semin:4;
    uint32_t :1; // Set to 0
    uint32_t seup:2;
    uint32_t :1; // Set to 0
    uint32_t semax:4;
    uint32_t :1; // Set to 0
    uint32_t sedn:2;
    uint32_t seimin:1;
    uint32_t sgt:7;
    uint32_t :1; // Set to 0
    uint32_t sfilt:1;
    uint32_t :1; // Set to 0
  } bit;
  uint32_t reg;
} TMC2130RegCOOLCONF;

typedef union {
  struct {
    uint32_t dc_time:10;
    uint32_t :7;
    uint32_t dc_sg:8;
  } bit;
  uint32_t reg;
} TMC2130RegDCCTRL;

typedef union {
  struct {
    uint32_t sg_result:10;
    uint32_t :5;
    uint32_t fsactive:1;
    uint32_t cs_actual:5;
    uint32_t :3;
    uint32_t stall_guard:1;
    uint32_t ot:1;
    uint32_t otpw:1;
    uint32_t s2ga:1;
    uint32_t s2gb:1;
    uint32_t ola:1;
    uint32_t olb:1;
    uint32_t stst:1;
  } bit;
  uint32_t reg;
} TMC2130RegDRV_STATUS;

typedef union {
  struct {
    uint32_t pwm_ampl:8;
    uint32_t pwm_grad:8;
    uint32_t pwm_freq:2;
    uint32_t pwm_autoscale:1;
    uint32_t pwm_symmetric:1;
    uint32_t freewheel:2;
  } bit;
  uint32_t reg;
} TMC2130RegPWMCONF;

typedef union {
  struct {
    uint32_t amplitude_scalar:8;
  } bit;
  uint32_t reg;
} TMC2130RegPWM_SCALE;

typedef union {
  struct {
    uint32_t inv:1;
    uint32_t maxspeed:1;
  } bit;
  uint32_t reg;
} TMC2130RegENCM_CTRL;

typedef union {
  struct {
    uint32_t steps:20;
  } bit;
  uint32_t reg;
} TMC2130RegLOST_STEPS;

void tcm2130_select(void);
void tcm2130_deselect(void);
uint8_t tcm2130_spi_transceive_byte(const uint8_t value);
void tcm2130_write_register(const uint8_t address, const uint32_t value, bool busy_waiting);
uint32_t tcm2130_read_register(const uint8_t address, const bool busy_waiting);
void tcm2130_set_active(const bool active);
void tcm2130_handle_register_read_and_write(void);
void tcm2130_print_current_state(void);
void tcm2130_update_read_registers(void);

#endif
