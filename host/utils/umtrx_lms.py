#!/usr/bin/env python2.7
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
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
import struct, socket, argparse
# pylint: disable-msg = C0301, C0103, C0111

UDP_CONTROL_PORT = 49152
UDP_MAX_XFER_BYTES = 1024
UDP_TIMEOUT = 1
UDP_POLL_INTERVAL = 0.10 #in seconds
USRP2_CONTROL_PROTO_VERSION = 11 # must match firmware proto

# see fw_common.h
CONTROL_FMT = '!LLL24x'
CONTROL_IP_FMT = '!LLLL20x'
SPI_FMT = '!LLLLLBBBB12x'
n2xx_revs = {
  0x0a00: ["n200_r3", "n200_r2"],
  0x0a10: ["n200_r4"],
  0x0a01: ["n210_r3", "n210_r2"],
  0x0a11: ["n210_r4"],
  0xfa00: ["umtrx"],
  }

# remember kids: drugs are bad...
USRP2_CTRL_ID_HUH_WHAT = ord(' ')
USRP2_CTRL_ID_WAZZUP_BRO = ord('a')
USRP2_CTRL_ID_WAZZUP_DUDE = ord('A')
UMTRX_CTRL_ID_REQUEST = ord('u')
UMTRX_CTRL_ID_RESPONSE = ord('U')
USRP2_CTRL_ID_TRANSACT_ME_SOME_SPI_BRO = ord('s')
USRP2_CTRL_ID_OMG_TRANSACTED_SPI_DUDE = ord('S')
USRP2_CTRL_ID_DO_AN_I2C_READ_FOR_ME_BRO = ord('i')
USRP2_CTRL_ID_HERES_THE_I2C_DATA_DUDE = ord('I')
USRP2_CTRL_ID_WRITE_THESE_I2C_VALUES_BRO = ord('h')
USRP2_CTRL_ID_COOL_IM_DONE_I2C_WRITE_DUDE = ord('H')
USRP2_CTRL_ID_GET_THIS_REGISTER_FOR_ME_BRO = ord('r')
USRP2_CTRL_ID_OMG_GOT_REGISTER_SO_BAD_DUDE = ord('R')
USRP2_CTRL_ID_HOLLER_AT_ME_BRO = ord('l')
USRP2_CTRL_ID_HOLLER_BACK_DUDE = ord('L')
USRP2_CTRL_ID_PEACE_OUT = ord('~')
SPI_EDGE_RISE = ord('r')
SPI_EDGE_FALL = ord('f')

def unpack_format(_str, fmt):
    return struct.unpack(fmt, _str)

def pack_control_fmt(proto_ver, pktid, seq):
    return struct.pack(CONTROL_FMT, proto_ver, pktid, seq)

def pack_spi_fmt(proto_ver, pktid, seq, dev, data, miso, mosi, bits, read):
    return struct.pack(SPI_FMT, proto_ver, pktid, seq, dev, data, miso, mosi, bits, read)

def recv_item(skt, fmt, chk, ind):
    try:
        pkt = skt.recv(UDP_MAX_XFER_BYTES)
        pkt_list = unpack_format(pkt, fmt)
#        print "Received %d bytes: %x, '%c', %x, %s" % (len(pkt), pkt_list[0], pkt_list[1], pkt_list[2], pkt_list[3])
        if pkt_list[1] != chk:
            return None
        return pkt_list[ind]
    except socket.timeout:
        return None

def spi_rw(skt, addr, lms, command):
    skt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)
    out_pkt = pack_spi_fmt(USRP2_CONTROL_PROTO_VERSION, USRP2_CTRL_ID_TRANSACT_ME_SOME_SPI_BRO, 0, lms, command, SPI_EDGE_RISE, SPI_EDGE_RISE, 16, 1)
    skt.sendto(out_pkt, (addr, UDP_CONTROL_PORT))
    return recv_item(skt, SPI_FMT, USRP2_CTRL_ID_OMG_TRANSACTED_SPI_DUDE, 4)

def read_spi(skt, addr, lms, reg):
    return spi_rw(skt, addr, lms, reg << 8)

def write_spi(skt, addr, lms, reg, data):
    return spi_rw(skt, addr, lms, ((0x80 | reg) << 8) | data)

def ping(skt, addr):
    skt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    out_pkt = pack_control_fmt(USRP2_CONTROL_PROTO_VERSION, USRP2_CTRL_ID_WAZZUP_BRO, 0)
    skt.sendto(out_pkt, (addr, UDP_CONTROL_PORT))
    return recv_item(skt, CONTROL_FMT, USRP2_CTRL_ID_WAZZUP_DUDE, 1)

def dump(skt, addr, lms):
    return [read_spi(skt, addr, lms, x) for x in range(0, 128)]

def detect(skt, bcast_addr):
#    print 'Detecting UmTRX over %s:' % bcast_addr
    skt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)    
    out_pkt = pack_control_fmt(USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST, 0)
#    print " Sending %d bytes: %x, '%c',.." % (len(out_pkt), USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST)
    skt.sendto(out_pkt, (bcast_addr, UDP_CONTROL_PORT))
    response = recv_item(skt, CONTROL_IP_FMT, UMTRX_CTRL_ID_RESPONSE, 3)
    if response:
        return socket.inet_ntoa(struct.pack("<L", socket.ntohl(response)))
    return None

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'UmTRX LMS debugging tool.', epilog = 'UmTRX is detected via broadcast unless explicit address is specified via --umtrx-addr option')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--detect', dest = 'bcast_addr', default = '192.168.10.255', help='broadcast domain where UmTRX should be discovered (default: 192.168.10.255)')
    group.add_argument('--umtrx-addr', dest = 'umtrx', const = '192.168.10.2', nargs='?', help = 'UmTRX address (default: 192.168.10.2)')
    parser.add_argument('--reg', type = lambda s: int(s, 16), choices = xrange(0, 0x80), metavar = '0..0x79', help = 'LMS register number')
    parser.add_argument('--data', type = lambda s: int(s, 16), choices = xrange(0, 0x100), metavar = '0..0xFF', help = 'data to be written into LMS register, hex')
    parser.add_argument('--lms', type = int, choices = range(1, 3), help = 'LMS number: 1 or 2, if no other options are given it will dump all registers for corresponding LMS')
    parser.add_argument('--version', action='version', version='%(prog)s 1.3')
    args = parser.parse_args()
    if args.data:
        if not args.reg: # argparse do not have dependency concept for options
            exit('<data> argument requires <reg> argument.')
        if not args.lms: # gengetopt is so much better
            exit('<data> argument requires <lms> argument.')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(UDP_TIMEOUT)
    umtrx = args.umtrx if args.umtrx else detect(sock, args.bcast_addr) 
    if umtrx: # UmTRX address established
        if ping(sock, umtrx): # UmTRX probed
            if args.data:
                print 'writing to 0x%02X [None indicates error]... 0x%02X' % (args.reg, write_spi(sock, umtrx, args.lms, args.reg, args.data))
            elif args.reg:
                print 'reading from 0x%02X [None indicates error]... 0x%02X' % (args.reg, read_spi(sock, umtrx, args.lms if args.lms else 1, args.reg))        
            elif args.lms:
                lms_regs = dump(sock, umtrx, args.lms)
                print 'LMS %u' % args.lms
                print ''.join(map(lambda a, b: '# 0x%02X: 0x%02X\n' % (a, b), range(0, 128), lms_regs))
            else:
                lms1 = dump(sock, umtrx, 1)
                lms2 = dump(sock, umtrx, 2)
                diff = map(lambda l1, l2: 'OK\n' if l1 == l2 else 'DIFF\n', lms1, lms2)
                print ''.join(map(lambda i, l1, l2, d: '# 0x%02X: LMS1=0x%02X \tLMS2=0x%02X\t%s' % (i, l1, l2, d), range(0, 128), lms1, lms2, diff))               
        else:
            print 'UmTRX at %s is not responding.' % umtrx
    else:
        print 'No UmTRX detected over %s' % args.bcast_addr
