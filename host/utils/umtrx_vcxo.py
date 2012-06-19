#!/usr/bin/env python
#
# Copyright 2012 Sylvain Munaut <tnt@246tNt.com>
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
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import socket, argparse, time
import umtrx_ctrl


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'UmTRX VCXO debugging tool.', epilog = "UmTRX is detected via broadcast unless explicit address is specified via --umtrx-addr option. 'None' returned while reading\writing indicates error in the process.")
    parser.add_argument('--version', action='version', version='%(prog)s 1.0')

    basic_opt = parser.add_mutually_exclusive_group()
    basic_opt.add_argument('--detect', dest = 'bcast_addr', default = '192.168.10.255', help='broadcast domain where UmTRX should be discovered (default: 192.168.10.255)')
    basic_opt.add_argument('--umtrx-addr', dest = 'umtrx', const = '192.168.10.2', nargs='?', help = 'UmTRX address (default: 192.168.10.2)')

    parser.add_argument('--dac-value', dest = 'dac', type = int, help = '12 bit value to use as DAC value')

    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(umtrx_ctrl.UDP_TIMEOUT)
    umtrx = args.umtrx if args.umtrx is not None else umtrx_ctrl.detect(sock, args.bcast_addr)

    if umtrx is not None: # UmTRX address established
        if umtrx_ctrl.ping(sock, umtrx): # UmTRX probed
            print('UmTRX detected at %s' % umtrx)
            umtrx_vcxo_dev = umtrx_ctrl.umtrx_vcxo_dac(sock, umtrx)
            if args.dac is not None:
                umtrx_vcxo_dev.set_dac(args.dac)
        else:
            print('UmTRX at %s is not responding.' % umtrx)
    else:
        print('No UmTRX detected over %s' % args.bcast_addr)
