/*
 * Copyright 2012 Sylvain Munaut <tnt@246tNt.com>
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

#ifndef INCLUDED_GPSDO_H
#define INCLUDED_GPSDO_H


void gpsdo_init(void);

/* Enable/disable GPSDO debug printing */
void gpsdo_set_debug(int level);

/* Set value of the VCTCXO DAC */
void gpsdo_set_dac(uint16_t v);

/* Get the current VCTCXO DAC value */
uint16_t gpsdo_get_dac(void);

/* Get the last calculated VCTCXO frequency */
uint32_t gpsdo_get_last_freq(void);

/* Get the last alpha/8 filtered VCTCXO frequency (29.3 fixed point) */
uint32_t gpsdo_get_lpf_freq(void);

/* Get time (seconds part) of the last PPS pulse  */
uint32_t gpsdo_get_last_pps_secs(void);

/* Get time (ticks part) of the last PPS pulse  */
uint32_t gpsdo_get_last_pps_ticks(void);

#endif /* INCLUDED_GPSDO_H */
