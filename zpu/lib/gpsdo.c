/*
 * Copyright 2012 Sylvain Munaut <tnt@246tNt.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* peripheral headers */
#include "u2_init.h"
#include "pic.h"
#include "spi.h"
#include "time64.h"
#include "eeprom.h"

#include "memory_map.h"

/* printf headers */
#include "nonstdio.h"

/* standard headers */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

static int gpsdo_debug = 0;


/* DAC */
/* ---- */

#define DAC_BITS	12

static uint16_t dac_value; /* Current DAC value */

static int
_set_vctcxo_dac(uint16_t v)
{
#ifndef BOOTLOADER
  if (gpsdo_debug) printf("DAC: %d\n", v);
#endif
  dac_value = v;
  return spi_transact(
    SPI_TXRX, SPI_SS_DAC,
    v & ((1 << DAC_BITS) - 1),
    16, SPI_PUSH_RISE | SPI_LATCH_RISE
  );
}

static uint16_t
_get_vctcxo_dac(void)
{
    return dac_value;
}

/* PID */
/* --- */

#define PID_SCALE_SHIFT 7

#define PID_TARGET	52000000	/* 52 MHz */
#define PID_MAX_ERR	10000		/* 10 kHz off max */
#define PID_MAX_DEV	((1 << (DAC_BITS-1)) - 1)
#define PID_MID_VAL	(1 << (DAC_BITS-1))	/* 2048 - the middle value of the DAC */

#define MAX_INT ((1<<31)-1)

#define G_PID_PK 64
#define G_PID_IK 16
#define G_PID_DK 256


typedef struct {
  /* Loop constants */
//  int16_t Pk;
//  int16_t Ik;
//  int16_t Dk;

  int32_t max_error;
  int32_t max_sum_error;

  /* Previous value */
  int32_t val_prev;

  /* State */
  int32_t init_val;
  int32_t err_sum;
} pid_data_t;


static pid_data_t g_pid;

static void
_gpsdo_pid_init(int32_t init_val)
{
  /* Configure loop */
  //g_pid.Pk = 64;
  //g_pid.Ik = 16;
  //g_pid.Dk = 256; /* Seems high but we LPF PID input so d is dampened */

  /* Reset state */
  g_pid.val_prev = PID_TARGET;
  g_pid.init_val = init_val;
  g_pid.err_sum  = 0;
}

static void
_gpsdo_pid_first_step(int32_t val)
{
  g_pid.val_prev = val;
}

static void
_gpsdo_pid_step(int32_t val)
{
  int32_t error;
  int32_t p_term, d_term, i_term;
  int32_t tot;

  /* Compute error */
  error = PID_TARGET - val;

  if (error > PID_MAX_ERR)
    error = PID_MAX_ERR;
  else if (error < -PID_MAX_ERR)
    error = -PID_MAX_ERR;

  /* Compute P term */
  p_term = error * G_PID_PK;

  /* Compute I term */
  g_pid.err_sum += error;
  i_term = g_pid.err_sum * G_PID_IK;

  /* Compute D term */
  d_term = (g_pid.val_prev - val) * G_PID_DK;
  g_pid.val_prev = val;

  /* Final value */
  tot = (p_term + i_term + d_term) >> PID_SCALE_SHIFT;

  if (tot > PID_MAX_DEV)
    tot = PID_MAX_DEV;
  else if (tot < -PID_MAX_DEV)
    tot = -PID_MAX_DEV;

#ifndef BOOTLOADER
  if (gpsdo_debug) printf("GPSDO: DAC = %d %d (P=%d, I=%d, D=%d)>>%d limit +-%d\n",
                          g_pid.init_val, tot, p_term, i_term, d_term, PID_SCALE_SHIFT, PID_MAX_DEV);
#endif
  /* Update DAC */
  _set_vctcxo_dac( g_pid.init_val + tot );
}



/* Driver */
/* ------ */

/* Extra bits of precision for g_val_lpf */
#define VAL_LPF_PRECISION 3
#define VAL_LPF_INIT_VALUE (PID_TARGET<<VAL_LPF_PRECISION)
#define SKIP_ON_FIRST_RUN_VAL 2

/* Value > 0 indicates first run. Value itself is a ocunter how many IRQs
 * to skip before starting PID regulation. Value of 2 is selected, because
 * (1) the first tick will produce incorrect frequency counter because frequency
 * correction occue in the middle of counting and (2) we need one valid reading
 * to initialize g_pid.val_prev used for calculation of the D component. Previously
 * we always initialized g_pid.val_prev to a value of the ideal target value
 * which resulted in excessive oscillations. */
static int8_t g_skip_on_first_run = SKIP_ON_FIRST_RUN_VAL;
static uint32_t g_val_lpf = VAL_LPF_INIT_VALUE;
static uint32_t g_prev_secs = 0;
static uint32_t g_prev_ticks = 0;
static uint32_t g_last_calc_freq = 0; /* Last calculated VCTCXO frequency */

static void
_gpsdo_irq_handler(unsigned irq)
{
  if (gpsdo_regs->csr & GPSDO_CSR_RDY)
  {
    /* Counter value */
    uint32_t val = gpsdo_regs->cnt;
    /* Read the current wall time */
    uint32_t cur_secs, cur_ticks;
    time64_read(&cur_secs, &cur_ticks);

    /* Next request */
    gpsdo_regs->csr = GPSDO_CSR_REQ;
#ifndef BOOTLOADER
    if (gpsdo_debug) printf("GPSDO: Counter = %u @ %u sec %u ticks\n", val, cur_secs, cur_ticks);
#endif
    /* Check validity of value */
    if (abs(val - PID_TARGET) < 100000)
    {
      /* Save calculated frequency */
      g_last_calc_freq = val;
      
      if (g_skip_on_first_run > 0) {
        g_skip_on_first_run--;
        g_val_lpf = val<<VAL_LPF_PRECISION;
#ifndef BOOTLOADER
        printf("GPSDO init: Filtered counter = %u + %u/8\n",
               (g_val_lpf>>VAL_LPF_PRECISION), (g_val_lpf&((1<<VAL_LPF_PRECISION)-1)));
#endif
        _gpsdo_pid_first_step(val);
      } else {
        /* LPF the value */
        /* Integer overlow warning! */
        /* This works for val ~= 52M, but don't try to use it with much larger values - it will overflow */
        g_val_lpf = (g_val_lpf * 7 + (val<<VAL_LPF_PRECISION) + 4) >> 3;
#ifndef BOOTLOADER
        if (gpsdo_debug) printf("GPSDO: Filtered counter = %u + %u/8\n",
                                (g_val_lpf>>VAL_LPF_PRECISION), (g_val_lpf&((1<<VAL_LPF_PRECISION)-1)));
#endif
        /* Update PID */
        _gpsdo_pid_step(g_val_lpf>>VAL_LPF_PRECISION);
      }
    }

    /* Save the current wall time */
    g_prev_secs = cur_secs;
    g_prev_ticks = cur_ticks;
  }
}

void
gpsdo_init(void)
{
  uint16_t tcxo_dac = eeprom_read_tcxo_dac();
  printf("TCXO DAC: %d ", tcxo_dac);
  if (tcxo_dac == 0xFFFF) {
    tcxo_dac = PID_MID_VAL;
#ifndef BOOTLOADER
    printf("(hardcoded init)\n");
#endif
  } else {
#ifndef BOOTLOADER
    printf("(init from EEPROM)\n");
#endif
  }

  /* Reset GPSDO */
  g_skip_on_first_run = SKIP_ON_FIRST_RUN_VAL;

  /* Set last saved freq to an invalid value */
  g_last_calc_freq = 0;
  
  /* Set the DAC to initial value */
  _set_vctcxo_dac( tcxo_dac );

  /* Register IRQ handler */
  pic_register_handler(IRQ_GPSDO, _gpsdo_irq_handler);

  /* Init PID */
  _gpsdo_pid_init(tcxo_dac);

  /* Start request */
  gpsdo_regs->csr = GPSDO_CSR_REQ;

  /* Save the current wall time.
   * We can use it to estimate time to lock */
  time64_read(&g_prev_secs, &g_prev_ticks);
}

void gpsdo_set_debug(int level)
{
  gpsdo_debug = level;
}

void gpsdo_set_dac(uint16_t v)
{
  /* Reset PID */
  _gpsdo_pid_init(v);
  /* Reset GPSDO */
  g_skip_on_first_run = SKIP_ON_FIRST_RUN_VAL;
  /* Set the DAC value */
  _set_vctcxo_dac(v);
}

uint16_t gpsdo_get_dac(void)
{
  return _get_vctcxo_dac();
}

uint32_t gpsdo_get_last_freq(void)
{
  return g_last_calc_freq;
}

uint32_t gpsdo_get_lpf_freq(void)
{
  return g_val_lpf;
}

uint32_t gpsdo_get_last_pps_secs(void)
{
  return g_prev_secs;
}

uint32_t gpsdo_get_last_pps_ticks(void)
{
  return g_prev_ticks;
}
