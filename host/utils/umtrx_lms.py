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

VCO_HIGH = 0x02
VCO_NORM = 0x00
VCO_LOW = 0x01

FREQ_LIST = [# min, max, val
    (0.2325e9,   0.285625e9, 0x27),
    (0.285625e9, 0.336875e9, 0x2f),
    (0.336875e9, 0.405e9,    0x37),
    (0.405e9,    0.465e9,    0x3f),
    (0.465e9,    0.57125e9,  0x26),
    (0.57125e9,  0.67375e9,  0x2e),
    (0.67375e9,  0.81e9,     0x36),
    (0.81e9,     0.93e9,     0x3e),
    (0.93e9,     1.1425e9,   0x25),
    (1.1425e9,   1.3475e9,   0x2d),
    (1.3475e9,   1.62e9,     0x35),
    (1.62e9,     1.86e9,     0x3d),
    (1.86e9,     2.285e9,    0x24),
    (2.285e9,    2.695e9,    0x2c),
    (2.695e9,    3.24e9,     0x34),
    (3.24e9,     3.72e9,     0x3c)]

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

def select_freq(freq): # test if given freq within the range and return corresponding value
    l = filter(lambda t: True if t[0] < freq <= t[1] else False, FREQ_LIST)
    return l[0][2] if len(l) else None

def lms_pll_tune(skt, addr, lms, ref_clock, out_freq):
    freqsel = select_freq(out_freq)
    if not freqsel:
        return False
    vco_x = 1 << ((freqsel & 0x7) - 3)
    nint = vco_x * out_freq / ref_clock
    nfrack = (1 << 23) * (vco_x * out_freq - nint * ref_clock) / ref_clock
    print "FREQSEL=%d VCO_X=%d NINT=%d  NFRACK=%d" % (freqsel, vco_x, nint, nfrack)
# Write NINT, NFRAC
    write_spi(skt, addr, lms, 0x10, (nint >> 1) & 0xff) # NINT[8:1]
    write_spi(skt, addr, lms, 0x11, ((nfrack >> 16) & 0x7f) | ((nint & 0x1) << 7)) # NINT[0] NFRACK[22:16]
    write_spi(skt, addr, lms, 0x12, (nfrack >> 8) & 0xff) # NFRACK[15:8]
    write_spi(skt, addr, lms, 0x13, (nfrack) & 0xff) # NFRACK[7:0]
# Write FREQSEL
    write_spi(skt, addr, lms, 0x15, (freqsel << 2) | 0x01) # FREQSEL[5:0] SELOUT[1:0]
# Reset VOVCOREG, OFFDOWN to default
    write_spi(skt, addr, lms, 0x18, 0x40) # VOVCOREG[3:1] OFFDOWN[4:0]
    write_spi(skt, addr, lms, 0x19, 0x94) # VOVCOREG[0] VCOCAP[5:0]
# Poll VOVCO
    start_i = -1
    stop_i = -1
    state = VCO_HIGH
    for i in range(0, 64):
        write_spi(skt, addr, lms, 0x19, 0x80 | i)
        comp = read_spi(skt, addr, lms, 0x1a)
        if not comp:
            return False
        switch = comp >> 6
        if VCO_HIGH == switch:
            break
        elif VCO_LOW == switch:
            if state == VCO_NORM:
                stop_i = i - 1
                state = VCO_LOW
                print "Low"
                break
        elif VCO_NORM == switch:
            if state == VCO_HIGH:
                start_i = i
                state = VCO_NORM
                print "Norm"
                break
        else:
            print "ERROR WHILE TUNING"
            return False
        print "VOVCO[%d]=%x" % (i, comp)

    if start_i == -1 or stop_i == -1:
        print "CAN'T TUNE"
        return False
# Tune to the middle of the found VCOCAP range
    avg_i = (start_i + stop_i) / 2
    print "START=%d STOP=%d SET=%d" % (start_i, stop_i, avg_i)
    write_spi(skt, addr, lms, 0x19, 0x80 | avg_i)
    return True

def lms_init(skt, addr, lms):
# INIT defaults
    write_spi(skt, addr, lms, 0x09, 0xC0)
    write_spi(skt, addr, lms, 0x09, 0x80)
    write_spi(skt, addr, lms, 0x17, 0xE0)
    write_spi(skt, addr, lms, 0x27, 0xE3)
    write_spi(skt, addr, lms, 0x64, 0x32)
    write_spi(skt, addr, lms, 0x70, 0x01)
    write_spi(skt, addr, lms, 0x79, 0x37)
    write_spi(skt, addr, lms, 0x59, 0x09)
    write_spi(skt, addr, lms, 0x47, 0x40)
# TX Enable
    write_spi(skt, addr, lms, 0x05, (1 << 5) | (1 << 4) | (1 << 3) | (1 << 1)) # STXEN
    write_spi(skt, addr, lms, 0x09, 0x81)
    write_spi(skt, addr, lms, 0x44, (0 << 3) | (1 << 1) | 1) # PA off
# RF Settings
    write_spi(skt, addr, lms, 0x41, 0x15) # VGA1GAIN
    write_spi(skt, addr, lms, 0x45, 0x00) # VGA2GAIN, ENVD
    lms_pll_tune(skt, addr, lms, 26e6, 925e6) # Tune PLL
# RF Settings
    write_spi(skt, addr, lms, 0x41, (-4 + 35)) # VGA1GAIN
    write_spi(skt, addr, lms, 0x45, (25 << 3) | 0x0) # VGA2GAIN, ENVD
    write_spi(skt, addr, lms, 0x44, (2 << 3) | (1 << 1) | 1) # PA2 on

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
    parser = argparse.ArgumentParser(description = 'UmTRX LMS debugging tool.', epilog = "UmTRX is detected via broadcast unless explicit address is specified via --umtrx-addr option. 'None' returned while reading\writing indicates error in the process.")
    parser.add_argument('--version', action='version', version='%(prog)s 1.8')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--detect', dest = 'bcast_addr', default = '192.168.10.255', help='broadcast domain where UmTRX should be discovered (default: 192.168.10.255)')
    group.add_argument('--umtrx-addr', dest = 'umtrx', const = '192.168.10.2', nargs='?', help = 'UmTRX address (default: 192.168.10.2)')
    parser.add_argument('--lms', type = int, choices = range(1, 3), help = 'LMS number: 1 or 2, if no other options are given it will dump all registers for corresponding LMS')
    group2 = parser.add_mutually_exclusive_group()
    parser.add_argument('--reg', type = lambda s: int(s, 16), choices = xrange(0, 0x80), metavar = '0..0x79', help = 'LMS register number, hex')
    group2.add_argument('--data', type = lambda s: int(s, 16), choices = xrange(0, 0x100), metavar = '0..0xFF', help = 'data to be written into LMS register, hex')
    group2.add_argument('--lms-init', type = int, choices = range(1, 3), help = 'run init sequence for specified LMS')
    group2.add_argument('--pll-out-freq', type = float, metavar = '[1; 5e9]', help = 'PLL frequency')
    parser.add_argument('--pll-ref-clock', type = float, default = 26e6, help = 'PLL reference clock, 26MHz by default')
    args = parser.parse_args()
    if args.data:
        if not args.reg: # argparse do not have dependency concept for options
            exit('<data> argument requires <reg> argument.')
        if not args.lms: # gengetopt is so much better
            exit('<data> argument requires <lms> argument.')
    if args.pll_out_freq:
        if not args.lms:
            exit('<pll-out-freq> argument requires <lms> argument.')
        if args.reg:
            exit('--reg makes no sense with --pll-out-freq, aborting.')
        if not 1 <= args.pll_out_freq <= 5e9:
            exit('<pll-out-freq> is out of range [1; 5e9]')
    if args.lms_init:
        if args.lms:
            exit('--lms is confusing, please specify LMS number as a parameter to --lms-init')
        if args.reg:
            exit('--reg makes no sense with --lms-init, aborting.')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(UDP_TIMEOUT)
    umtrx = args.umtrx if args.umtrx else detect(sock, args.bcast_addr) 
    if umtrx: # UmTRX address established
        if ping(sock, umtrx): # UmTRX probed
            if args.lms_init:
                lms_init(sock, umtrx, args.lms_init)
            elif args.pll_out_freq:
                lms_pll_tune(sock, umtrx, args.lms_init, int(args.pll_ref_clock), int(args.pll_out_freq))
            elif args.data:
                print 'write 0x%02X to REG 0x%02X' % (write_spi(sock, umtrx, args.lms, args.reg, args.data), args.reg)
            elif args.reg:
                print 'read 0x%02X from REG 0x%02X' % (read_spi(sock, umtrx, args.lms if args.lms else 1, args.reg), args.reg)
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
