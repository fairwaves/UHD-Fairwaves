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

#include "memory_map.h"

/* printf headers */
#include "nonstdio.h"

/* standard headers */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* #define GPSDO_DEBUG 1 */


/* DAC */
/* ---- */

#define DAC_BITS	12

uint16_t dac_value; /* Current DAC value */

int
set_vctcxo_dac(uint16_t v)
{
#ifdef GPSDO_DEBUG
  printf("DAC: %d\n", v);
#endif
  dac_value = v;
  return spi_transact(
    SPI_TXRX, SPI_SS_DAC,
    v & ((1 << DAC_BITS) - 1),
    16, SPI_PUSH_RISE | SPI_LATCH_RISE
  );
}

uint16_t
get_vctcxo_dac(void)
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

typedef struct {
  /* Loop constants */
  int16_t Pk;
  int16_t Ik;
  int16_t Dk;

  int32_t max_error;
  int32_t max_sum_error;

  /* Previous value */
  int32_t val_prev;

  /* State */
  int32_t err_sum;
} pid_data_t;


static pid_data_t g_pid;

static void
_gpsdo_pid_init(void)
{
  /* Configure loop */
  g_pid.Pk = 64;
  g_pid.Ik = 16;
  g_pid.Dk = 256; /* Seems high but we LPF PID input so d is dampened */

  /* Reset state */
  g_pid.val_prev = PID_TARGET;
  g_pid.err_sum  = 0;
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
  p_term = error * g_pid.Pk;

  /* Compute I term */
  g_pid.err_sum += error;
  i_term = g_pid.err_sum * g_pid.Ik;

  /* Compute D term */
  d_term = (g_pid.val_prev - val) * g_pid.Dk;
  g_pid.val_prev = val;

  /* Final value */
  tot = (p_term + i_term + d_term) >> PID_SCALE_SHIFT;

  if (tot > PID_MAX_DEV)
    tot = PID_MAX_DEV;
  else if (tot < -PID_MAX_DEV)
    tot = -PID_MAX_DEV;

  /* Update DAC */
  set_vctcxo_dac( PID_MID_VAL + tot );
}



/* Driver */
/* ------ */

static int32_t g_val_lpf = PID_TARGET;

static void
_gpsdo_irq_handler(unsigned irq)
{
  if (gpsdo_regs->csr & GPSDO_CSR_RDY)
  {
    /* Counter value */
    int32_t val = gpsdo_regs->cnt;

#ifdef GPSDO_DEBUG
  printf("GPSDO Count: %d\n", val);
#endif

    /* Next request */
    gpsdo_regs->csr = GPSDO_CSR_REQ;

    /* TODO:: Save the current wall time to be able check
       time passed since the last lock later. This is useful
       e.g. to check whether we still have a GPS lock.*/

    /* Check validity of value */
    if (abs(val - PID_TARGET) < 100000)
    {
      /* LPF the value */
      g_val_lpf = (g_val_lpf * 7 + val + 4) >> 3;

      /* Update PID */
      _gpsdo_pid_step(g_val_lpf);
    }
  }
}

void
gpsdo_init(void)
{
  /* Set the DAC to mid value */
  set_vctcxo_dac( PID_MID_VAL );

  /* Register IRQ handler */
  pic_register_handler(IRQ_GPSDO, _gpsdo_irq_handler);

  /* Init PID */
  _gpsdo_pid_init();

  /* Start request */
  gpsdo_regs->csr = GPSDO_CSR_REQ;
}
