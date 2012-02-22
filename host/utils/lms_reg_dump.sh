#!/bin/sh
#
# Copyright 2012 Fairwaves
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http:#www.gnu.org/licenses/>.
#
echo "Dump and compare LMS registers..."
REGS=128
RW='./lms_reg_rw'
while [ $REGS -gt 0 ]
do
    REGS=`expr $REGS - 1`
    LMS1=`$RW --lms 1 --address $REGS`
    LMS1=`$RW --lms 2 --address $REGS`
    echo -n "REG=$REGS\tLMS1=$LMS1\tLMS2=$LMS2\t"
    if [ $LMS1 eq $LMS2]
    then
	echo "OK"
    else
	echo "DIFF"
    fi
done
