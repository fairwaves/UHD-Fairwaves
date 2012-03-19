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
# pylint: disable-msg = C0301, C0103, C0111

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

def recv_item(skt, fmt, chk, ind):
    try:
        pkt = skt.recv(UDP_MAX_XFER_BYTES)
        pkt_list = unpack_format(pkt, fmt)
# print "Received %d bytes: %x, '%c', %x, %s" % (len(pkt), pkt_list[0], pkt_list[1], pkt_list[2], pkt_list[3])
        if pkt_list[1] != chk:
            return None
        return pkt_list[ind]
    except socket.timeout:
        return None

def read_spi(skt, addr, lms, reg):
    skt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)
    out_pkt = pack_spi_fmt(USRP2_CONTROL_PROTO_VERSION, USRP2_CTRL_ID_TRANSACT_ME_SOME_SPI_BRO, 0, lms, reg << 8, SPI_EDGE_RISE, SPI_EDGE_RISE, 16, 1)
    skt.sendto(out_pkt, (addr, UDP_CONTROL_PORT))
    return recv_item(skt, SPI_FMT, USRP2_CTRL_ID_OMG_TRANSACTED_SPI_DUDE, 4)

def write_spi(skt, addr, lms, reg, data):
    skt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)
    out_pkt = pack_spi_fmt(USRP2_CONTROL_PROTO_VERSION, USRP2_CTRL_ID_TRANSACT_ME_SOME_SPI_BRO, 0, lms, (0x80 | (reg << 8)) | data, SPI_EDGE_RISE, SPI_EDGE_RISE, 16, 1)
    skt.sendto(out_pkt, (addr, UDP_CONTROL_PORT))

def detect(skt, bcast_addr):    
    skt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)    
    out_pkt = pack_control_fmt(USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST, 0)
    print " Sending %d bytes: %x, '%c',.." % (len(out_pkt), USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST)
    skt.sendto(out_pkt, (bcast_addr, UDP_CONTROL_PORT))
    response = recv_item(skt, CONTROL_IP_FMT, UMTRX_CTRL_ID_RESPONSE, 3)
    if response:
        return socket.inet_ntoa(struct.pack("<L", socket.ntohl(response)))
    return None

# Specify address via command line for non-default broadcasting.

if __name__ == '__main__':
    target = sys.argv[1] if len(sys.argv) > 1 else UMTRX_BROADCAST
    print 'Detecting UmTRX over %s:' % target
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.1)
    umtrx = detect(sock, target)
    if umtrx:
        for i in range(0, 128):
            lms1 = read_spi(sock, umtrx, 1, i)
            lms2 = read_spi(sock, umtrx, 2, i)
            diff = 'OK' if lms1 == lms2 else 'DIFF'
            print '# %.3u: LMS1=0x%X \tLMS2=0x%X\t%s' % (i, lms1, lms2, diff)
