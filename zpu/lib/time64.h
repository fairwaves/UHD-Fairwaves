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

/* Operations with time64 counters handling wrapping safely */

/* Read counters */
void
time64_read(uint32_t *secs, uint32_t *ticks);

/* Compare two counters.
 * Return -1 if conter 1 is less than counter 2.
 * Return  0 if conter 1 is equal counter 2.
 * Return  1 if conter 1 is larger than counter 2.
 */
int
time64_compare(uint32_t secs1, uint32_t ticks1,
               uint32_t secs2, uint32_t ticks2);

/* Add ticks to a counter */
void
time64_add_ticks(uint32_t *secs, uint32_t *ticks,
                 uint32_t ticks2);

/* Is a given amount of ticks elapsed? */
bool
time64_is_elapsed(uint32_t secs1, uint32_t ticks1,
                  uint32_t secs2, uint32_t ticks2,
                  uint32_t ticks_elapsed);
