#!/usr/bin/env python
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
import sys, struct, socket
# pylint: disable-msg = C0301, C0111

UDP_CONTROL_PORT = 49152
UDP_MAX_XFER_BYTES = 1024
UDP_TIMEOUT = 3
UDP_POLL_INTERVAL = 0.10 #in seconds
USRP2_CONTROL_PROTO_VERSION = 11 # must match firmware proto
UMTRX_BROADCAST = '192.168.10.255' # default UmTRX network domain

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

def read_spi(addr, lms, reg):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)
    sock.settimeout(0.1) 
    out_pkt = pack_spi_fmt(USRP2_CONTROL_PROTO_VERSION, USRP2_CTRL_ID_TRANSACT_ME_SOME_SPI_BRO, 0, lms, reg << 8, SPI_EDGE_RISE, SPI_EDGE_RISE, 16, 1)
    sock.sendto(out_pkt, (addr, UDP_CONTROL_PORT))
    while(True):
        try:
            pkt = sock.recv(UDP_MAX_XFER_BYTES)
            pkt_list = unpack_format(pkt, SPI_FMT)
            if pkt_list[1] != USRP2_CTRL_ID_OMG_TRANSACTED_SPI_DUDE:
                return None
            print ' '.join(hex(x) for x in pkt_list)
            return pkt_list[4]
        except socket.timeout:
            return None

def detect(bcast_addr):
    print 'Detecting UmTRX over %s:' % bcast_addr
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.1)
    out_pkt = pack_control_fmt(USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST, 0)
    print " Sending %d bytes: %x, '%c',.." % (len(out_pkt), USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST)
    sock.sendto(out_pkt, (bcast_addr, UDP_CONTROL_PORT))
    while(True):
        try:
            pkt = sock.recv(UDP_MAX_XFER_BYTES)
            (proto_ver, pktid, rxseq, ip_addr) = unpack_format(pkt, CONTROL_IP_FMT)
            print "Received %d bytes: %x, '%c', %x, %s" % (len(pkt), proto_ver, pktid, rxseq, socket.inet_ntoa(struct.pack("<L", socket.ntohl(ip_addr))))
            if pktid == UMTRX_CTRL_ID_RESPONSE:
                return True
        except socket.timeout:
            return False

# Specify address via command line for non-default broadcasting.

if __name__ == '__main__':
    target = sys.argv[1] if len(sys.argv) > 1 else UMTRX_BROADCAST
    if detect(target):
        print '0x%X' % read_spi("192.168.10.2", 1, 118)
        print '0x%X' % read_spi("192.168.10.2", 2, 118)

