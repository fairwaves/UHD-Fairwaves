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

#ifndef INCLUDED_EEPROM_H
#define INCLUDED_EEPROM_H

/* Safe booted or not? (read from EEPROM) */
bool find_safe_booted_flag(void);

/* Store safe booted flag to EEPROM */
void set_safe_booted_flag(bool flag);

/* Write to EEPROM */
bool eeprom_write(int i2c_addr, int eeprom_offset, const void *buf, int len);

/* Read from EEPROM */
bool eeprom_read(int i2c_addr, int eeprom_offset, void *buf, int len);

/* Read TCXO DAC calibrtion value from EEPROM */
uint16_t eeprom_read_tcxo_dac();

/* Write TCXO DAC calibrtion value to EEPROM */
bool eeprom_write_tcxo_dac(uint16_t tcxo_dac);

#endif /* INCLUDED_EEPROM_H */
