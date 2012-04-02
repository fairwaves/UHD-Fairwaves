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
import struct, socket, argparse, time
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
#        print("Received %d bytes: %x, '%c', %x, %s" % (len(pkt), pkt_list[0], pkt_list[1], pkt_list[2], pkt_list[3]))
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

def rmw_spi(skt, addr, lms, reg, action):
    """ Read-Modify-Write for LMS register.
    'action' is a lambda(x) expression """
    reg_save = read_spi(skt, addr, lms, reg)
    reg_val = action(reg_save)
    write_spi(skt, addr, lms, reg, reg_val)
    return reg_save

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
    if freqsel is None:
        print("Error: Output frequency is out of range")
        return False

    vco_x = 1 << ((freqsel & 0x7) - 3)
    nint = vco_x * out_freq / ref_clock
    nfrack = (1 << 23) * (vco_x * out_freq - nint * ref_clock) / ref_clock
    print("FREQSEL=%d VCO_X=%d NINT=%d NFRACK=%d" % (freqsel, vco_x, nint, nfrack))

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
        if comp is None:
            return False
        vcocap = comp >> 6
        print("VOVCO[%d]=%x" % (i, vcocap))
        if VCO_HIGH == vcocap:
            pass
        elif VCO_LOW == vcocap:
            if state == VCO_NORM:
                stop_i = i - 1
                state = VCO_LOW
                print("Low")
        elif VCO_NORM == vcocap:
            if state == VCO_HIGH:
                start_i = i
                state = VCO_NORM
                print("Norm")
        else:
            print("ERROR WHILE TUNING")
            return False

    if start_i == -1 or stop_i == -1:
        print("CAN'T TUNE")
        return False
    # Tune to the middle of the found VCOCAP range
    avg_i = (start_i + stop_i) / 2
    print("START=%d STOP=%d SET=%d" % (start_i, stop_i, avg_i))
    write_spi(skt, addr, lms, 0x19, 0x80 | avg_i)
    return True

def lms_init(skt, addr, lms):
    """ INIT with default values (taken from the LMS EVB software)"""
#    write_spi(skt, addr, lms, 0x09, 0xC0) # questionable step
    write_spi(skt, addr, lms, 0x09, 0x80)
    write_spi(skt, addr, lms, 0x17, 0xE0)
    write_spi(skt, addr, lms, 0x27, 0xE3)
    write_spi(skt, addr, lms, 0x64, 0x32)
    write_spi(skt, addr, lms, 0x70, 0x01)
    write_spi(skt, addr, lms, 0x79, 0x37)
    write_spi(skt, addr, lms, 0x59, 0x09)
    write_spi(skt, addr, lms, 0x47, 0x40)
    # RF Settings
    write_spi(skt, addr, lms, 0x41, 0x15) # VGA1GAIN
    write_spi(skt, addr, lms, 0x45, 0x00) # VGA2GAIN, ENVD

def lms_tx_enable(skt, addr, lms):
    """Enable TX"""
    # Enable STXEN: Soft transmit enable
#    write_spi(skt, addr, lms, 0x05, (1 << 5) | (1 << 4) | (1 << 3) | (1 << 1)) # STXEN
    rmw_spi(skt, addr, lms, 0x05, lambda x: x | (1 << 3))
    # Set Tx DSM SPI clock enabled
#    write_spi(skt, addr, lms, 0x09, 0x81)
    rmw_spi(skt, addr, lms, 0x09, lambda x: x | (1 << 0))

def lms_pa_off(skt, addr, lms):
    write_spi(skt, addr, lms, 0x44, (0 << 3) | (1 << 1) | 1)

def lms_pa_on(skt, addr, lms, pa):
    """ Turn on PA, 'pa' parameter is in [1..2] range"""
    write_spi(skt, addr, lms, 0x44, (pa << 3) | (1 << 1) | 1)

# RF Settings for LO leakage tuning
#    write_spi(skt, addr, lms, 0x41, (-4 + 35)) # VGA1GAIN
#    write_spi(skt, addr, lms, 0x45, (25 << 3) | 0x0) # VGA2GAIN, ENVD
#    write_spi(skt, addr, lms, 0x44, (2 << 3) | (1 << 1) | 1) # PA2 on

def lms_general_dc_calibration(skt, addr, lms, dc_addr, calibration_reg_base):
    """ Programming and Calibration Guide: 4.1 General DC Calibration Procedure """
    try_cnt_limit = 10

    reg_val = read_spi(skt, addr, lms, calibration_reg_base+0x03)
    # DC_ADDR := ADDR
    reg_val = (reg_val & 0xf8) | dc_addr
    write_spi(skt, addr, lms, calibration_reg_base+0x03, reg_val)
    # DC_START_CLBR := 1
    reg_val = reg_val | (1 << 5)
    write_spi(skt, addr, lms, calibration_reg_base+0x03, reg_val)
    # DC_START_CLBR := 0
    reg_val = reg_val ^ (1 << 5)
    write_spi(skt, addr, lms, calibration_reg_base+0x03, reg_val)

    for cnt in range(0, try_cnt_limit):
#        print("cnt=%d" % cnt)

        # Wait for 6.4(1.6) us
        time.sleep(6.4e-6)

        # Read DC_CLBR_DONE
        reg_val  = read_spi(skt, addr, lms, calibration_reg_base+0x01)
        DC_CLBR_DONE = (reg_val >> 1) & 0x1
#        print(" DC_CLBR_DONE=%d" % DC_CLBR_DONE)

        # DC_CLBR_DONE == 1?
        if DC_CLBR_DONE:
            continue

        # Read DC_LOCK
        reg_val  = read_spi(skt, addr, lms, calibration_reg_base+0x01)
        DC_LOCK = (reg_val >> 2) & 0x7
#        print(" DC_LOCK=%d" % DC_LOCK)

        # DC_LOCK != 0 or 7?
        if DC_LOCK != 0 and DC_LOCK != 7:
            # Read DC_REGVAL
            DC_REGVAL = read_spi(skt, addr, lms, calibration_reg_base+0x00)
            print("DC_REGVAL = %d" % DC_REGVAL)
            return DC_REGVAL

    # PANIC: Algorithm does Not Converge!
    print("Error: DC Offset Calibration does not converge!")
    return None

def lms_lpf_tuning_dc_calibration(skt, addr, lms):
    """ Programming and Calibration Guide: 4.2 DC Offset Calibration of LPF Tuning Module """
    # Save TopSPI::CLK_EN[5] Register
    # TopSPI::CLK_EN[5] := 1
    clk_en_save = rmw_spi(skt, addr, lms, 0x09, lambda x: x | (1 << 5))

    # Perform DC Calibration Procedure in TopSPI with ADDR := 0 and get Result
    # DCCAL := TopSPI::DC_REGVAL
    DCCAL = lms_general_dc_calibration(skt, addr, lms, 0, 0x0)
    if DCCAL is None:
        # Restore TopSPI::CLK_EN[5] Register
        write_spi(skt, addr, lms, 0x09, clk_en_save)
        return False

    # RxLPFSPI::DCO_DACCAL := DCCAL
    rmw_spi(skt, addr, lms, 0x35, lambda x: (x & 0xc0) | DCCAL)

    # TxLPFSPI::DCO_DACCAL := DCCAL
    rmw_spi(skt, addr, lms, 0x55, lambda x: (x & 0xc0) | DCCAL)

    # Restore TopSPI::CLK_EN[5] Register
    write_spi(skt, addr, lms, 0x09, clk_en_save)
    return True

def lms_txrx_lpf_dc_calibration(skt, addr, lms, is_tx):
    """ Programming and Calibration Guide: 4.3 TX/RX LPF DC Offset Calibration """
    # Determine base address for control registers
    control_reg_base = 0x30 if is_tx else 0x50

    # Save TopSPI::CLK_EN Register
    # TopSPI::CLK_EN := 1
    clk_en_save = rmw_spi(skt, addr, lms, 0x09, lambda x: x | (1 << 1) if is_tx else x | (1 << 3))

    # Perform DC Calibration Procedure in LPFSPI with ADDR := 0 (For channel I) and get Result
    # Perform DC Calibration Procedure in LPFSPI with ADDR := 1 (For channel Q) and get Result
    if lms_general_dc_calibration(skt, addr, lms, 0, control_reg_base) is None \
       or lms_general_dc_calibration(skt, addr, lms, 1, control_reg_base) is None:
        # Restore TopSPI::CLK_EN Register
        write_spi(skt, addr, lms, 0x09, clk_en_save)
        return False

    # Restore TopSPI::CLK_EN Register
    write_spi(skt, addr, lms, 0x09, clk_en_save)
    return True

def lms_rxvga2_dc_calibration(skt, addr, lms):
    """ Programming and Calibration Guide: 4.4 RXVGA2 DC Offset Calibration """
    # Set base address for control registers
    control_reg_base = 0x60

    # Save TopSPI::CLK_EN Register
    # TopSPI::CLK_EN := 1
    clk_en_save = rmw_spi(skt, addr, lms, 0x09, lambda x: x | (1 << 4))

    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 0 (For DC Reference channel) and get Result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 1 (For VGA2A_I channel) and get Result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 2 (For VGA2A_Q channel) and get Result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 3 (For VGA2B_I channel) and get Result
    # Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 4 (For VGA2B_Q channel) and get Result
    if lms_general_dc_calibration(skt, addr, lms, 0, control_reg_base) is None \
       or lms_general_dc_calibration(skt, addr, lms, 1, control_reg_base) is None \
       or lms_general_dc_calibration(skt, addr, lms, 2, control_reg_base) is None \
       or lms_general_dc_calibration(skt, addr, lms, 3, control_reg_base) is None \
       or lms_general_dc_calibration(skt, addr, lms, 4, control_reg_base) is None:
        # Restore TopSPI::CLK_EN Register
        write_spi(skt, addr, lms, 0x09, clk_en_save)
        return False

    # Restore TopSPI::CLK_EN Register
    write_spi(skt, addr, lms, 0x09, clk_en_save)
    return True

def lms_lpf_bandwidth_tuning(skt, addr, lms, ref_clock, lpf_bandwidth_code):
    """ Programming and Calibration Guide: 4.5 LPF Bandwidth Tuning.
    Note, that this function modifies Tx PLL settings. """
    # Save registers 0x05 and 0x09, because we will modify them during lms_tx_enable()
    reg_save_05 = read_spi(skt, addr, lms, 0x05)
    reg_save_09 = read_spi(skt, addr, lms, 0x09)

    # Enable TxPLL and set toProduce 320MHz
    lms_tx_enable(skt, addr, lms)
    lms_pll_tune(skt, addr, lms, ref_clock, int(320e6))

    # Use 40MHz generatedFrom TxPLL: TopSPI::CLKSEL_LPFCAL := 0
    # Power Up LPF tuning clock generation block: TopSPI::PD_CLKLPFCAL := 0
    reg_save_06 = rmw_spi(skt, addr, lms, 0x06, lambda x: x & ~(1 << 3) & ~(1 << 2))

    # Set TopSPI::BWC_LPFCAL
    t = rmw_spi(skt, addr, lms, 0x07, lambda x: (x & ~0x0f) | lpf_bandwidth_code)
    print("code = %x %x %x" % (lpf_bandwidth_code, t, read_spi(skt, addr, lms, 0x07)))
    # TopSPI::RST_CAL_LPFCAL := 1 (Rst Active)
    rst_lpfcal_save = rmw_spi(skt, addr, lms, 0x06, lambda x: x | 0x01)
    # ...Delay 100ns...
    # TopSPI::RST_CAL_LPFCAL := 0 (Rst Inactive)
    write_spi(skt, addr, lms, 0x06, rst_lpfcal_save & ~0x01)
    # RCCAL := TopSPI::RCCAL_LPFCAL
    RCCAL = read_spi(skt, addr, lms, 0x01) >> 5
    print("RCCAL = %d" % RCCAL)
    # RxLPFSPI::RCCAL_LPF := RCCAL
    rmw_spi(skt, addr, lms, 0x56, lambda x: (x & ~(7 << 4)) | (RCCAL << 4))
    # TxLPFSPI::RCCAL_LPF := RCCAL
    rmw_spi(skt, addr, lms, 0x36, lambda x: (x & ~(7 << 4)) | (RCCAL << 4))

    # Restore registers 0x05, 0x06 and 0x09
    write_spi(skt, addr, lms, 0x06, reg_save_06)
    write_spi(skt, addr, lms, 0x05, reg_save_05)
    write_spi(skt, addr, lms, 0x09, reg_save_09)

def lms_auto_calibration(sock, umtrx, lms, ref_clock, lpf_bandwidth_code):
    lms_lpf_tuning_dc_calibration(sock, umtrx, lms)
    lms_lpf_bandwidth_tuning(sock, umtrx, lms, ref_clock, lpf_bandwidth_code)
    lms_txrx_lpf_dc_calibration(sock, umtrx, lms, True)
    lms_txrx_lpf_dc_calibration(sock, umtrx, lms, False)
    lms_rxvga2_dc_calibration(sock, umtrx, lms)

def detect(skt, bcast_addr):
#    print('Detecting UmTRX over %s:' % bcast_addr)
    skt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)    
    out_pkt = pack_control_fmt(USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST, 0)
#    print(" Sending %d bytes: %x, '%c',.." % (len(out_pkt), USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST))
    skt.sendto(out_pkt, (bcast_addr, UDP_CONTROL_PORT))
    response = recv_item(skt, CONTROL_IP_FMT, UMTRX_CTRL_ID_RESPONSE, 3)
    if response:
        return socket.inet_ntoa(struct.pack("<L", socket.ntohl(response)))
    return None

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'UmTRX LMS debugging tool.', epilog = "UmTRX is detected via broadcast unless explicit address is specified via --umtrx-addr option. 'None' returned while reading\writing indicates error in the process.")
    parser.add_argument('--version', action='version', version='%(prog)s 2.2')
    parser.add_argument('--lms', type = int, choices = list(range(1, 3)), help = 'LMS number, if no other options are given it will dump all registers for corresponding LMS')
    parser.add_argument('--reg', type = lambda s: int(s, 16), choices = range(0, 0x80), metavar = '0..0x79', help = 'LMS register number, hex')
    parser.add_argument('--verify', action = 'store_true', help = 'read back written register value to verify correctness')
    parser.add_argument('--pll-ref-clock', type = float, default = 26e6, help = 'PLL reference clock, 26MHz by default')
    parser.add_argument('--lpf-bandwidth-code', type = lambda s: int(s, 16), choices = range(0, 0x10), metavar = '0..0x0f', help = 'LPF bandwidth code (default: 0x0f)')
    basic_opt = parser.add_mutually_exclusive_group()
    basic_opt.add_argument('--detect', dest = 'bcast_addr', default = '192.168.10.255', help='broadcast domain where UmTRX should be discovered (default: 192.168.10.255)')
    basic_opt.add_argument('--umtrx-addr', dest = 'umtrx', const = '192.168.10.2', nargs='?', help = 'UmTRX address (default: 192.168.10.2)')    
    adv_opt = parser.add_mutually_exclusive_group()
    adv_opt.add_argument('--lms-tx-enable', action = 'store_true', help = 'enable TX for LMS')
    adv_opt.add_argument('--data', type = lambda s: int(s, 16), choices = range(0, 0x100), metavar = '0..0xFF', help = 'data to be written into LMS register, hex')
    adv_opt.add_argument('--lms-init', action = 'store_true', help = 'run init sequence for LMS')
    adv_opt.add_argument('--lms-auto-calibration', action = 'store_true', help = 'LPF Tuning, TX/RX LPF, RXVGA2 DC Offset Calibration and LPF bandwidth tuning')
    adv_opt.add_argument('--lms-lpf-tuning-dc-calibration', action = 'store_true', help = 'DC Offset Calibration of LPF Tuning Module')
    adv_opt.add_argument('--lms-tx-lpf-dc-calibration', action = 'store_true', help = 'TX LPF DC Offset Calibration')
    adv_opt.add_argument('--lms-rx-lpf-dc-calibration', action = 'store_true', help = 'RX LPF DC Offset Calibration')
    adv_opt.add_argument('--lms-rxvga2-dc-calibration', action = 'store_true', help = 'RXVGA2 DC Offset Calibration')
    adv_opt.add_argument('--lms-lpf-bandwidth-tuning', action = 'store_true', help = 'LPF bandwidth tuning')
    adv_opt.add_argument('--lms-pa-on', type = int, choices = range(1, 3), help = 'turn on PA')
    adv_opt.add_argument('--lms-pa-off', action = 'store_true', help = 'turn off PA')
    adv_opt.add_argument('--lms-tx-pll-tune', type = float, metavar = '232.5e6..3720e6', help = 'Tune Tx PLL to the given frequency')
    args = parser.parse_args()
    if not args.lms: # argparse do not have dependency concept for options
        if args.data or args.lms_tx_pll_tune or args.lms_init or args.lms_pa_off or args.lms_pa_on \
           or args.lms_lpf_tuning_dc_calibration or args.lms_tx_lpf_dc_calibration \
           or args.lms_rx_lpf_dc_calibration or args.lms_rxvga2_dc_calibration \
           or args.lms_auto_calibration or args.lpf_bandwidth_tuning \
           or args.lms_tx_enable:
            exit('--lms parameter is required for given options.') # gengetopt is so much better
    if args.data:
        if not args.reg:
            exit('<data> argument requires <reg> argument.')
    if args.lms_tx_pll_tune:
        if not 232.5e6 < args.lms_tx_pll_tune <= 3720e6:
            exit('<lms-tx-pll-tune> is out of range 232.5e6..3720e6')
    if args.lms_init:
        if args.reg:
            exit('--reg makes no sense with --lms-init, aborting.')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(UDP_TIMEOUT)
    umtrx = args.umtrx if args.umtrx else detect(sock, args.bcast_addr) 
    if umtrx: # UmTRX address established
        if ping(sock, umtrx): # UmTRX probed
            if args.lms_init:
                lms_init(sock, umtrx, args.lms)
            elif args.lms_tx_enable:
                lms_tx_enable(sock, umtrx, args.lms)
            elif args.lms_auto_calibration:
                # 0x0f - 0.75MHz
                lpf_bw_code = args.lpf_bandwidth_code if args.lpf_bandwidth_code is not None else 0x0f
                lms_auto_calibration(sock, umtrx, args.lms, int(args.pll_ref_clock), int(lpf_bw_code))
            elif args.lms_lpf_tuning_dc_calibration:
                lms_lpf_tuning_dc_calibration(sock, umtrx, args.lms)
            elif args.lms_tx_lpf_dc_calibration:
                lms_txrx_lpf_dc_calibration(sock, umtrx, args.lms, True)
            elif args.lms_rx_lpf_dc_calibration:
                lms_txrx_lpf_dc_calibration(sock, umtrx, args.lms, False)
            elif args.lms_rxvga2_dc_calibration:
                lms_rxvga2_dc_calibration(sock, umtrx, args.lms)
            elif args.lms_pa_on:
                lms_pa_on(sock, umtrx, args.lms, args.lms_pa_on)
            elif args.lms_pa_off:
                lms_pa_off(sock, umtrx, args.lms)
            elif args.lms_tx_pll_tune:
                lms_pll_tune(sock, umtrx, args.lms, int(args.pll_ref_clock), int(args.lms_tx_pll_tune))
            elif args.lms_lpf_bandwidth_tuning:
                # 0x0f - 0.75MHz
                lpf_bw_code = args.lpf_bandwidth_code if args.lpf_bandwidth_code is not None else 0x0f
                lms_lpf_bandwidth_tuning(sock, umtrx, args.lms, int(args.pll_ref_clock), int(lpf_bw_code))
            elif args.data:
                wrt = write_spi(sock, umtrx, args.lms, args.reg, args.data)
                if args.verify:
                    vrfy = read_spi(sock, umtrx, args.lms if args.lms else 1, args.reg)
                    print('written 0x%02X to REG 0x%02X - %s' % (vrfy, args.reg, 'OK' if vrfy == args.data else 'FAIL'))
                else:
                    print('write returned 0x%02X for REG 0x%02X' % (wrt, args.reg))
            elif args.reg:
                print('read 0x%02X from REG 0x%02X' % (read_spi(sock, umtrx, args.lms if args.lms else 1, args.reg), args.reg))
            elif args.lms:
                lms_regs = dump(sock, umtrx, args.lms)
                print('LMS %u' % args.lms)
                print(''.join(map(lambda a, b: '# 0x%02X: 0x%02X\n' % (a, b), list(range(0, 128)), lms_regs)))
            else:
                lms1 = dump(sock, umtrx, 1)
                lms2 = dump(sock, umtrx, 2)
                diff = list(map(lambda l1, l2: 'OK\n' if l1 == l2 else 'DIFF\n', lms1, lms2))
                print(''.join(map(lambda i, l1, l2, d: '# 0x%02X: LMS1=0x%02X \tLMS2=0x%02X\t%s' % (i, l1, l2, d), list(range(0, 128)), lms1, lms2, diff)))               
        else:
            print('UmTRX at %s is not responding.' % umtrx)
    else:
        print('No UmTRX detected over %s' % args.bcast_addr)
