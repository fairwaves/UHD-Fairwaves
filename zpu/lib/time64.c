/*
 * Copyright 2017 Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
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

#include "memory_map.h"

/* printf headers */
//#include "nonstdio.h"

/* standard headers */
#include <stddef.h>
//#include <stdlib.h>
//#include <string.h>
#include <stdbool.h>
#include <limits.h>
#define UINT32_MAX UINT_MAX

void
time64_read(uint32_t *secs, uint32_t *ticks)
{
  uint32_t cur_secs, cur_secs2;

  cur_secs = readback_mux->time64_secs_rb;
  *ticks = readback_mux->time64_ticks_rb;
  cur_secs2 = readback_mux->time64_secs_rb;

  /* Check for seconds wrap */
  if (cur_secs2 != cur_secs) {
    /* Decide which seconds reading is correct.
     * Here we assume that we're reading fast and time between
     * two readings is negligible compared to a 32-bit counter
     * wrap time.
     */
    if (*ticks < UINT32_MAX/2) {
      /* First half of the time - wrap has just happened */
      *secs = cur_secs2;
    } else {
      /* Second half - wrap has not happened yet */
      *secs = cur_secs;
    }
  }
}

int
time64_compare(uint32_t secs1, uint32_t ticks1,
               uint32_t secs2, uint32_t ticks2)
{
  if (secs1 == secs2) {
    if (ticks1 < ticks2) {
      return -1;
    } else if (ticks1 == ticks2) {
      return 0;
    } else {
      return 1;
    }
  } else if (secs1 < secs2) {
    return -1;
  } else {
    return 1;
  }
}

void
time64_add_ticks(uint32_t *secs, uint32_t *ticks,
                 uint32_t ticks2)
{
  if (UINT32_MAX - *ticks > ticks2) {
    *secs += 1;
  }
  *ticks += ticks2;
}

bool
time64_is_elapsed(uint32_t secs1, uint32_t ticks1,
                  uint32_t secs2, uint32_t ticks2,
                  uint32_t ticks_elapsed)
{
  time64_add_ticks(&secs1, &ticks1, ticks_elapsed);
  return time64_compare(secs1, ticks1, secs2, ticks2) >= 0;
}
