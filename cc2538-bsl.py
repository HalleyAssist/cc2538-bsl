#!/usr/bin/env python

# Copyright (c) 2014, Jelmer Tiete <jelmer@tiete.be>.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.

# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Implementation based on stm32loader by Ivan A-R <ivan@tuxotronic.org>

# Serial boot loader over UART for CC2538
# Based on the info found in TI's swru333a.pdf (spma029.pdf)
#
# Bootloader only starts if no valid image is found or if boot loader
# backdoor is enabled.
# Make sure you don't lock yourself out!! (enable backdoor in your firmware)
# More info at https://github.com/JelmerT/cc2538-bsl

from __future__ import print_function
from subprocess import Popen, PIPE

import sys, getopt
import glob
import time
import tempfile
import os
import subprocess
import struct
import binascii
import traceback

try:
    import magic
    have_magic = True
except ImportError:
    have_magic = False

try:
    from intelhex import IntelHex
    have_hex_support = True
except ImportError:
    have_hex_support = False

#version
VERSION_STRING = "2.1"

# Verbose level
QUIET = 5

# Check which version of Python is running
PY3 = sys.version_info >= (3,0)

try:
    import serial
except ImportError:
    print('{} requires the Python serial library'.format(sys.argv[0]))
    print('Please install it with one of the following:')
    print('')
    if PY3:
        print('   Ubuntu:  sudo apt-get install python3-serial')
        print('   Mac:     sudo port install py34-serial')
    else:
        print('   Ubuntu:  sudo apt-get install python-serial')
        print('   Mac:     sudo port install py-serial')
    sys.exit(1)


def mdebug(level, message, attr='\n'):
    if QUIET >= level:
        print(message, end=attr, file=sys.stderr)

# Takes chip IDs (obtained via Get ID command) to human-readable names
CHIP_ID_STRS = {0xb964: 'CC2538',
                0xb965: 'CC2538'
                }

RETURN_CMD_STRS =  {0x40: 'Success',
                    0x41: 'Unknown command',
                    0x42: 'Invalid command',
                    0x43: 'Invalid address',
                    0x44: 'Flash fail'
                    }

COMMAND_RET_SUCCESS = 0x40
COMMAND_RET_UNKNOWN_CMD = 0x41
COMMAND_RET_INVALID_CMD = 0x42
COMMAND_RET_INVALID_ADR = 0x43
COMMAND_RET_FLASH_FAIL = 0x44

class CmdException(Exception):
    pass

class FirmwareFile(object):
    HEX_FILE_EXTENSIONS = ('hex', 'ihx', 'ihex')

    def __init__(self, path):
        """
        Read a firmware file and store its data ready for device programming.

        This class will try to guess the file type if python-magic is available.

        If python-magic indicates a plain text file, and if IntelHex is
        available, then the file will be treated as one of Intel HEX format.

        In all other cases, the file will be treated as a raw binary file.

        In both cases, the file's contents are stored in bytes for subsequent
        usage to program a device or to perform a crc check.

        Parameters:
            path -- A str with the path to the firmware file.

        Attributes:
            bytes: A bytearray with firmware contents ready to send to the device
        """
        self._crc32 = None
        firmware_is_hex = False

        if have_magic:
            file_type = bytearray(magic.from_file(path, True))

            #from_file() returns bytes with PY3, str with PY2. This comparison
            #will be True in both cases"""
            if file_type == b'text/plain':
                firmware_is_hex = True
                mdebug(5, "Firmware file: Intel Hex")
            elif file_type == b'application/octet-stream':
                mdebug(5, "Firmware file: Raw Binary")
            else:
                error_str = "Could not determine firmware type. Magic " \
                            "indicates '%s'" % (file_type)
                raise CmdException(error_str)
        else:
            if os.path.splitext(path)[1][1:] in self.HEX_FILE_EXTENSIONS:
                firmware_is_hex = True
                mdebug(5, "Your firmware looks like an Intel Hex file")
            else:
                mdebug(5, "Cannot auto-detect firmware filetype: Assuming .bin")

            mdebug(10, "For more solid firmware type auto-detection, install "
                       "python-magic.")
            mdebug(10, "Please see the readme for more details.")

        if firmware_is_hex:
            if have_hex_support:
                self.bytes = bytearray(IntelHex(path).tobinarray())
                return
            else:
                error_str = "Firmware is Intel Hex, but the IntelHex library " \
                            "could not be imported.\n" \
                            "Install IntelHex in site-packages or program " \
                            "your device with a raw binary (.bin) file.\n" \
                            "Please see the readme for more details."
                raise CmdException(error_str)

        with open(path, 'rb') as f:
            self.bytes = bytearray(f.read())

    def crc32(self, n_bytes=0):
        """
        Return the crc32 checksum of the firmware image

        Return:
            The firmware's CRC32, ready for comparison with the CRC
            returned by the ROM bootloader's COMMAND_CRC32
        """
        if self._crc32 is None:
            if n_bytes <= 0:
                n_bytes = len(self.bytes)
            self._crc32 = binascii.crc32(bytearray(self.bytes)[:n_bytes]) & 0xffffffff

        return self._crc32

class CommandInterface(object):
    ACK_BYTE = 0xCC
    NACK_BYTE = 0x33
    def open(self, aport='/dev/tty.usbserial-000013FAB', abaudrate=500000):
        self.sp = serial.Serial(
            port=aport,
            baudrate=abaudrate,     # baudrate
            bytesize=8,             # number of databits
            parity=serial.PARITY_NONE,
            stopbits=1,
            xonxoff=0,              # enable software flow control
            rtscts=0,               # disable RTS/CTS flow control
            timeout=0.5             # set a timeout value, None for waiting forever
        )

    def invoke_bootloader(self, dtr_active_high=False, inverted=False):
        # Use the DTR and RTS lines to control bootloader and the !RESET pin.
        # This can automatically invoke the bootloader without the user
        # having to toggle any pins.
        #
        # If inverted is False (default):
        # DTR: connected to the bootloader pin
        # RTS: connected to !RESET
        # If inverted is True, pin connections are the other way round

        # Some boards have a co-processor that detects this sequence here and
        # then drives the main chip's BSL enable and !RESET pins. Depending on
        # board design and co-processor behaviour, the !RESET pin may get
        # asserted after we have finished the sequence here. In this case, we
        # need a small delay so as to avoid trying to talk to main chip before
        # it has actually entered its bootloader mode.
        #
        # See contiki-os/contiki#1533
        time.sleep(0.1)

    def close(self):
        self.sp.close()


    def _wait_for_ack(self, info = "", timeout = 1):
        stop = time.time() + timeout
        got = bytearray(2)
        while got[-2] != 00 or got[-1] not in (CommandInterface.ACK_BYTE,
                                               CommandInterface.NACK_BYTE):
            got += self._read(1)
            if time.time() > stop:
                raise CmdException("Timeout waiting for ACK/NACK after '%s'"
                                   % (info,))

        # Our bytearray's length is: 2 initial bytes + 2 bytes for the ACK/NACK
        # plus a possible N-4 additional (buffered) bytes
        mdebug(10, "Got %d additional bytes before ACK/NACK" % (len(got) - 4,))

        # wait for ask
        ask = got[-1]

        if ask == CommandInterface.ACK_BYTE:
            # ACK
            return 1
        elif ask == CommandInterface.NACK_BYTE:
            # NACK
            mdebug(10, "Target replied with a NACK during %s" % info)
            return 0

        # Unknown response
        mdebug(10, "Unrecognised response 0x%x to %s" % (ask, info))
        return 0

    def _encode_addr(self, addr):
        byte3 = (addr >> 0) & 0xFF
        byte2 = (addr >> 8) & 0xFF
        byte1 = (addr >> 16) & 0xFF
        byte0 = (addr >> 24) & 0xFF
        if PY3:
            return bytes([byte0, byte1, byte2, byte3])
        else:
            return (chr(byte0) + chr(byte1) + chr(byte2) + chr(byte3))

    def _decode_addr(self, byte0, byte1, byte2, byte3):
        return ((byte3 << 24) | (byte2 << 16) | (byte1 << 8) | (byte0 << 0))

    def _calc_checks(self, cmd, addr, size):
        return ((sum(bytearray(self._encode_addr(addr)))
                 +sum(bytearray(self._encode_addr(size)))
                 +cmd)
                &0xFF)

    def _write(self, data, is_retry=False):
        if PY3:
            if type(data) == int:
                assert data < 256
                goal = 1
                written = self.sp.write(bytes([data]))
            elif type(data) == bytes or type(data) == bytearray:
                goal = len(data)
                written = self.sp.write(data)
            else:
                raise CmdException("Internal Error. Bad data type: {}".format(type(data)))
        else:
            if type(data) == int:
                assert data < 256
                goal = 1
                written = self.sp.write(chr(data))
            else:
                goal = len(data)
                written = self.sp.write(data)
        if written < goal:
            mdebug(10, "*** Only wrote {} of target {} bytes".format(written, goal))
            if is_retry and written == 0:
                raise CmdException("Failed to write data on the serial bus")
            mdebug(10, "*** Retrying write for remainder")
            if type(data) == int:
                return self._write(data, is_retry=True)
            else:
                return self._write(data[written:], is_retry=True)

    def _read(self, length):
        return bytearray(self.sp.read(length))

    def sendAck(self):
        self._write(0x00)
        self._write(0xCC)
        return

    def sendNAck(self):
        self._write(0x00)
        self._write(0x33)
        return


    def receivePacket(self):
        # stop = time.time() + 5
        # got = None
        # while not got:
        got = self._read(2)
        #     if time.time() > stop:
        #         break

        # if not got:
        #     raise CmdException("No response to %s" % info)

        size = got[0] #rcv size
        chks = got[1] #rcv checksum
        data = bytearray(self._read(size - 2)) # rcv data

        mdebug(10, "*** received %x bytes" % size)
        if chks == sum(data)&0xFF:
            self.sendAck()
            return data
        else:
            self.sendNAck()
            #TODO: retry receiving!
            raise CmdException("Received packet checksum error")
            return 0

    def sendSynch(self):
        cmd = 0x55

        self.sp.flushInput() #flush serial input buffer for first ACK reception

        mdebug(10, "*** sending synch sequence")
        self._write(cmd) # send U
        self._write(cmd) # send U
        return self._wait_for_ack("Synch (0x55 0x55)", 2)

    def checkLastCmd(self):
        stat = self.cmdGetStatus()
        if not (stat):
            raise CmdException("No response from target on status request. (Did you disable the bootloader?)")

        if stat[0] == COMMAND_RET_SUCCESS:
            mdebug(10, "Command Successful")
            return 1
        else:
            stat_str = RETURN_CMD_STRS.get(stat[0], None)
            if stat_str is None:
                mdebug(0, 'Warning: unrecognized status returned 0x%x' % stat[0])
            else:
                mdebug(0, "Target returned: 0x%x, %s" % (stat[0], stat_str))
            return 0


    def cmdPing(self):
        cmd = 0x20
        lng = 3

        self._write(lng) # send size
        self._write(cmd) # send checksum
        self._write(cmd) # send data

        mdebug(10, "*** Ping command (0x20)")
        if self._wait_for_ack("Ping (0x20)"):
            return self.checkLastCmd()

    def cmdReset(self):
        cmd = 0x25
        lng = 3

        self._write(lng) # send size
        self._write(cmd) # send checksum
        self._write(cmd) # send data

        mdebug(10, "*** Reset command (0x25)")
        if self._wait_for_ack("Reset (0x25)"):
            return 1

    def cmdGetChipId(self):
        cmd = 0x28
        lng = 3

        self._write(lng) # send size
        self._write(cmd) # send checksum
        self._write(cmd) # send data

        mdebug(10, "*** GetChipId command (0x28)")
        if self._wait_for_ack("Get ChipID (0x28)"):
            version = self.receivePacket() # 4 byte answ, the 2 LSB hold chip ID
            if self.checkLastCmd():
                assert len(version) == 4, "Unreasonable chip id: %s" % repr(version)
                mdebug(10, "    Version 0x%02X%02X%02X%02X" % tuple(version))
                chip_id = (version[2] << 8) | version[3]
                return chip_id
            else:
                raise CmdException("GetChipID (0x28) failed")

    def cmdGetStatus(self):
        cmd = 0x23
        lng = 3

        self._write(lng) # send size
        self._write(cmd) # send checksum
        self._write(cmd) # send data

        mdebug(10, "*** GetStatus command (0x23)")
        if self._wait_for_ack("Get Status (0x23)"):
            stat = self.receivePacket()
            return stat

    def cmdSetXOsc(self):
        cmd = 0x29
        lng = 3

        self._write(lng) # send size
        self._write(cmd) # send checksum
        self._write(cmd) # send data

        mdebug(10, "*** SetXOsc command (0x29)")
        if self._wait_for_ack("SetXOsc (0x29)"):
            return 1
            # UART speed (needs) to be changed!

    def cmdRun(self, addr):
        cmd=0x22
        lng=7

        self._write(lng) # send length
        self._write(self._calc_checks(cmd,addr,0)) # send checksum
        self._write(cmd) # send cmd
        self._write(self._encode_addr(addr)) # send addr

        mdebug(10, "*** Run command(0x22)")
        return 1

    def cmdEraseMemory(self, addr, size):
        cmd=0x26
        lng=11

        self._write(lng) # send length
        self._write(self._calc_checks(cmd,addr,size)) # send checksum
        self._write(cmd) # send cmd
        self._write(self._encode_addr(addr)) # send addr
        self._write(self._encode_addr(size)) # send size

        mdebug(10, "*** Erase command(0x26)")
        if self._wait_for_ack("Erase memory (0x26)",10):
            return self.checkLastCmd()

    def cmdBankErase(self):
        cmd = 0x2C
        lng = 3

        self._write(lng) # send length
        self._write(cmd) # send checksum
        self._write(cmd) # send cmd

        mdebug(10, "*** Bank Erase command(0x2C)")
        if self._wait_for_ack("Bank Erase (0x2C)",10):
            return self.checkLastCmd()

    def cmdCRC32(self, addr, size):
        cmd=0x27
        lng=11

        self._write(lng) # send length
        self._write(self._calc_checks(cmd,addr,size)) # send checksum
        self._write(cmd) # send cmd
        self._write(self._encode_addr(addr)) # send addr
        self._write(self._encode_addr(size)) # send size

        mdebug(10, "*** CRC32 command(0x27)")
        if self._wait_for_ack("Get CRC32 (0x27)",1):
            crc=self.receivePacket()
            if self.checkLastCmd():
                return self._decode_addr(crc[3],crc[2],crc[1],crc[0])

    def cmdCRC32CC26xx(self, addr, size):
        cmd = 0x27
        lng = 15

        self._write(lng) # send length
        self._write(self._calc_checks(cmd, addr, size)) # send checksum
        self._write(cmd) # send cmd
        self._write(self._encode_addr(addr)) # send addr
        self._write(self._encode_addr(size)) # send size
        self._write(self._encode_addr(0x00000000)) # send number of reads

        mdebug(10, "*** CRC32 command(0x27)")
        if self._wait_for_ack("Get CRC32 (0x27)", 1):
            crc=self.receivePacket()
            if self.checkLastCmd():
                return self._decode_addr(crc[3], crc[2], crc[1], crc[0])

    def cmdDownload(self, addr, size):
        cmd=0x21
        lng=11

        if (size % 4) != 0: # check for invalid data lengths
            raise Exception('Invalid data size: %i. Size must be a multiple of 4.' % size)

        self._write(lng) # send length
        self._write(self._calc_checks(cmd,addr,size)) # send checksum
        self._write(cmd) # send cmd
        self._write(self._encode_addr(addr)) # send addr
        self._write(self._encode_addr(size)) # send size

        mdebug(10, "*** Download command (0x21)")
        if self._wait_for_ack("Download (0x21)",2):
            return self.checkLastCmd()

    def cmdSendData(self, data):
        cmd=0x24
        lng=len(data)+3
        # TODO: check total size of data!! max 252 bytes!

        self._write(lng) # send size
        self._write((sum(bytearray(data))+cmd)&0xFF) # send checksum
        self._write(cmd) # send cmd
        self._write(bytearray(data)) # send data

        mdebug(10, "*** Send Data (0x24)")
        if self._wait_for_ack("Send data (0x24)",10):
            return self.checkLastCmd()

    def cmdMemRead(self, addr): # untested
        cmd=0x2A
        lng=8

        self._write(lng) # send length
        self._write(self._calc_checks(cmd,addr,4)) # send checksum
        self._write(cmd) # send cmd
        self._write(self._encode_addr(addr)) # send addr
        self._write(4) # send width, 4 bytes

        mdebug(10, "*** Mem Read (0x2A)")
        if self._wait_for_ack("Mem Read (0x2A)",1):
            data = self.receivePacket()
            if self.checkLastCmd():
                return data # self._decode_addr(ord(data[3]),ord(data[2]),ord(data[1]),ord(data[0]))

    def cmdMemReadCC26xx(self, addr):
        cmd = 0x2A
        lng = 9

        self._write(lng) # send length
        self._write(self._calc_checks(cmd, addr, 2)) # send checksum
        self._write(cmd) # send cmd
        self._write(self._encode_addr(addr)) # send addr
        self._write(1) # send width, 4 bytes
        self._write(1) # send number of reads

        mdebug(10, "*** Mem Read (0x2A)")
        if self._wait_for_ack("Mem Read (0x2A)", 1):
            data = self.receivePacket()
            if self.checkLastCmd():
                return data

    def cmdMemWrite(self, addr, data, width): # untested
        # TODO: check width for 1 or 4 and data size
        cmd=0x2B
        lng=10

        self._write(lng) # send length
        self._write(self._calc_checks(cmd,addr,0)) # send checksum
        self._write(cmd) # send cmd
        self._write(self._encode_addr(addr)) # send addr
        self._write(bytearray(data)) # send data
        self._write(width) # send width, 4 bytes

        mdebug(10, "*** Mem write (0x2B)")
        if self._wait_for_ack("Mem Write (0x2B)",2):
            return checkLastCmd()


# Complex commands section

    def writeMemory(self, addr, data):
        lng = len(data)
        trsf_size = 248 # amount of data bytes transferred per packet (theory: max 252 + 3)
        empty_packet = bytearray((0xFF,) * trsf_size)

        # Boot loader enable check
        # TODO: implement check for all chip sizes & take into account partial firmware uploads
        if (lng == 524288): #check if file is for 512K model
            if not ((data[524247] & (1 << 4)) >> 4): #check the boot loader enable bit  (only for 512K model)
                if not ( conf['force'] or query_yes_no("The boot loader backdoor is not enabled "\
                    "in the firmware you are about to write to the target. "\
                    "You will NOT be able to reprogram the target using this tool if you continue! "\
                    "Do you want to continue?","no") ):
                    raise Exception('Aborted by user.')

        mdebug(5, "Writing %(lng)d bytes starting at address 0x%(addr)08X" %
               { 'lng': lng, 'addr': addr})

        offs = 0
        addr_set = 0

        while lng > trsf_size: #check if amount of remaining data is less then packet size
            if data[offs:offs+trsf_size] != empty_packet: #skip packets filled with 0xFF
                if addr_set != 1:
                    self.cmdDownload(addr,lng) #set starting address if not set
                    addr_set = 1
                mdebug(5, " Write %(len)d bytes at 0x%(addr)08X" % {'addr': addr, 'len': trsf_size}, '\r')
                sys.stdout.flush()

                self.cmdSendData(data[offs:offs+trsf_size]) # send next data packet
            else:   # skipped packet, address needs to be set
                addr_set = 0

            offs = offs + trsf_size
            addr = addr + trsf_size
            lng = lng - trsf_size

        mdebug(5, "Write %(len)d bytes at 0x%(addr)08X" % {'addr': addr, 'len': lng})
        self.cmdDownload(addr,lng)
        return self.cmdSendData(data[offs:offs+lng]) # send last data packet

class Chip(object):
    def __init__(self, command_interface):
        self.command_interface = command_interface

        # Some defaults. The child can override.
        self.flash_start_addr = 0x00000000
        self.has_cmd_set_xosc = False

    def crc(self, address, size):
        return getattr(self.command_interface, self.crc_cmd)(address, size)

    def disable_bootloader(self):
        if not (conf['force'] or query_yes_no("Disabling the bootloader will prevent you from "\
                            "using this script until you re-enable the bootloader "\
                            "using JTAG. Do you want to continue?", "no")):
            raise Exception('Aborted by user.')

        if PY3:
            pattern = struct.pack('<L', self.bootloader_dis_val)
        else:
            pattern = [ord(b) for b in struct.pack('<L', self.bootloader_dis_val)]

        if cmd.writeMemory(self.bootloader_address, pattern):
            mdebug(5, "    Set bootloader closed done                      ")
        else:
            raise CmdException("Set bootloader closed failed             ")

class CC2538(Chip):
    def __init__(self, command_interface):
        super(CC2538, self).__init__(command_interface)
        self.flash_start_addr = 0x00200000
        self.nv_start_addr = 0x0027c800
        self.nv_end_addr = 0x0027F7FF
        self.has_cmd_set_xosc = True
        self.bootloader_dis_val = 0xefffffff
        self.crc_cmd = "cmdCRC32"

        FLASH_CTRL_DIECFG0 = 0x400D3014
        FLASH_CTRL_DIECFG2 = 0x400D301C
        addr_ieee_address_primary = 0x00280028

        #Read out primary IEEE address, flash and RAM size
        model = self.command_interface.cmdMemRead(FLASH_CTRL_DIECFG0)
        self.size = (model[3] & 0x70) >> 4
        if 0 < self.size <= 4:
            self.size *= 0x20000 # in bytes
        else:
            self.size = 0x10000 # in bytes

        
        self.image_valid_address = self.flash_start_addr + self.size - 40
        self.entry_point_address = self.flash_start_addr + self.size - 36
        #self.cca_start = self.flash_start_addr + self.size
        self.bootloader_address = self.flash_start_addr + self.size - 44

        sram = (((model[2] << 8) | model[3]) & 0x380) >> 7
        sram = (2 - sram) << 3 if sram <= 1 else 32 # in KB

        pg = self.command_interface.cmdMemRead(FLASH_CTRL_DIECFG2)
        pg_major = (pg[2] & 0xF0) >> 4
        if pg_major == 0:
            pg_major = 1
        pg_minor = pg[2] & 0x0F

        ti_oui = bytearray([0x00, 0x12, 0x4B])
        ieee_addr = self.command_interface.cmdMemRead(addr_ieee_address_primary)
        ieee_addr_end = self.command_interface.cmdMemRead(addr_ieee_address_primary + 4)
        if ieee_addr[:3] == ti_oui:
            ieee_addr += ieee_addr_end
        else:
            ieee_addr = ieee_addr_end + ieee_addr

        mdebug(5, "CC2538 PG%d.%d: %dKB Flash, %dKB SRAM, CCFG at 0x%08X"
               % (pg_major, pg_minor, self.size >> 10, sram,
                  self.bootloader_address))
        mdebug(5, "Primary IEEE Address: %s" % (':'.join('%02X' % x for x in ieee_addr)))
    
    def get_n_bytes(self, mode):
        if mode == 'full':
            return self.size
        if mode == 'code_only':
            return self.nv_start_addr - self.flash_start_addr
        if mode == 'nvram':
            return self.nv_end_addr - self.nv_start_addr
        if mode == 'post_nv':
            return (self.flash_start_addr + self.size) - self.nv_end_addr
        elif mode == 'cca':
            return 44
        return -1

    def get_start_address(self, mode):
        if mode == 'full':
            return self.flash_start_addr
        elif mode == 'code_only':
            return self.flash_start_addr
        elif mode == 'nvram':
            return self.nv_start_addr
        elif mode == 'post_nv':
            return self.nv_end_addr
        elif mode == 'cca':
            return self.bootloader_address
        
    def crc(self, address, size):
        print('Checksumming %s bytes (0x%08X - 0x%08X)' % (size, address, address + size))
        return getattr(self.command_interface, self.crc_cmd)(address, size)

    def erase(self, start_addr, n_bytes, to_write):
        if n_bytes < 0:
            return False        
       
        mdebug(5, "Erasing %s bytes (0x%08X - 0x%08X) with 0x%02x" % (n_bytes, start_addr, start_addr + n_bytes, to_write))
        if to_write == 0xFF:
            return self.command_interface.cmdEraseMemory(start_addr, n_bytes)
        else:
            return self.command_interface.writeMemory(start_addr, [to_write] * n_bytes)

    def read_memory(self, addr):
        # CC2538's COMMAND_MEMORY_READ sends each 4-byte number in inverted
        # byte order compared to what's written on the device
        data = self.command_interface.cmdMemRead(addr)
        return bytearray([data[x] for x in range(3, -1, -1)])


def query_yes_no(question, default="yes"):
    valid = {"yes":True,   "y":True,  "ye":True,
            "no":False,     "n":False}
    if default == None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        if PY3:
            choice = input().lower()
        else:
            choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "\
                            "(or 'y' or 'n').\n")

# Convert the entered IEEE address into an integer
def parse_ieee_address (inaddr):
    try:
        return int(inaddr, 16)
    except ValueError:
        # inaddr is not a hex string, look for other formats
        if ':' in inaddr:
            bytes = inaddr.split(':')
        elif '-' in inaddr:
            bytes = inaddr.split('-')
        if len(bytes) != 8:
            raise ValueError("Supplied IEEE address does not contain 8 bytes")
        addr = 0
        for i,b in zip(range(8), bytes):
            try:
                addr += int(b, 16) << (56-(i*8))
            except ValueError:
                raise ValueError("IEEE address contains invalid bytes")
        return addr

def print_version():
    # Get the version using "git describe".
    try:
        p = Popen(['git', 'describe', '--tags', '--match', '[0-9]*'],
                stdout=PIPE, stderr=PIPE)
        p.stderr.close()
        line = p.stdout.readlines()[0]
        version = line.strip()
    except:
    # We're not in a git repo, or git failed, use fixed version string.
        version = VERSION_STRING
    print('%s %s' % (sys.argv[0], version))

def usage():
    print("""Usage: %s [-DhqVfewvr] [-l length] [-p port] [-b baud] [-a addr] [-i addr] [--bootloader-active-high] [--bootloader-invert-lines] [file.bin]
    -h, --help               This help
    -q                       Quiet
    -V                       Verbose
    -f                       Force operation(s) without asking any questions
    -e                       Erase (shorthand for --erase full)
    -w                       Write
    -v                       Verify (CRC32 check)
    -r                       Read
    -l length                Length of read
    -p port                  Serial port (default: first USB-like port in /dev)
    -s section               Section for write/erase (full, code_only)
    -b baud                  Baud speed (default: 500000)
    -a addr                  Target address
    -i, --ieee-address addr  Set the secondary 64 bit IEEE address
    --bootloader-active-high Use active high signals to enter bootloader
    --bootloader-invert-lines Inverts the use of RTS and DTR to enter bootloader
    -D, --disable-bootloader After finishing, disable the bootloader
    --version                Print script version

Examples:
    ./%s -e -w -v example/main.bin
    ./%s -e -w -v --ieee-address 00:12:4b:aa:bb:cc:dd:ee example/main.bin

    """ % (sys.argv[0],sys.argv[0],sys.argv[0]))

if __name__ == "__main__":

    conf = {
            'port': 'auto',
            'baud': 500000,
            'force_speed' : 0,
            'address': None,
            'force': 0,
            'erase': None,
            'write': 0,
            'verify': 0,
            'read': 0,
            'len': 0x80000,
            'section': 'full',
            'fname':'',
            'img_valid': None,
            'entry_point': None,
            'ieee_address': 0,
            'bootloader_active_high': False,
            'bootloader_invert_lines' : False,
            'disable-bootloader': 0
        }

# http://www.python.org/doc/2.5.2/lib/module-getopt.html

    try:
        opts, args = getopt.getopt(sys.argv[1:], "DhqVfEwvrp:b:e:a:l:i:s:m:k:", ['help', 'ieee-address=', 'img-valid=', 'entry-point=', 'disable-bootloader', 'bootloader-active-high', 'bootloader-invert-lines', 'version'])
    except getopt.GetoptError as err:
        # print help information and exit:
        print(str(err)) # will print something like "option -a not recognized"
        usage()
        sys.exit(2)

    for o, a in opts:
        if o == '-V':
            QUIET = 10
        elif o == '-q':
            QUIET = 0
        elif o == '-h' or o == '--help':
            usage()
            sys.exit(0)
        elif o == '-f':
            conf['force'] = 1
        elif o == '-s':
            conf['section'] = str(a).lower()
        elif o == '-e':
            conf['erase'] = int(str(a))
        elif o == '-w':
            conf['write'] = 1
        elif o == '-v':
            conf['verify'] = 1
        elif o == '-r':
            conf['read'] = 1
        elif o == '-p':
            conf['port'] = a
        elif o == '-b':
            conf['baud'] = eval(a)
            conf['force_speed'] = 1
        elif o == '-a':
            conf['address'] = eval(a)
        elif o == '-l':
            conf['len'] = eval(a)
        elif o == '-i' or o == '--ieee-address':
            conf['ieee_address'] = str(a)
        elif o == '-m' or o == '--img-valid':
            conf['img_valid'] = int(str(a))
        elif o == "-k" or o == "--entry-point":
            conf["entry_point"] = int(str(a))
        elif o == '--bootloader-active-high':
            conf['bootloader_active_high'] = True
        elif o == '--bootloader-invert-lines':
            conf['bootloader_invert_lines'] = True
        elif o == '-D' or o == '--disable-bootloader':
            conf['disable-bootloader'] = 1
        elif o == '--version':
            print_version()
            sys.exit(0)
        else:
            assert False, "Unhandled option"

    try:
        # Sanity checks
        if conf['write'] or conf['read'] or conf['verify']:    # check for input/output file
            try:
                args[0]
            except:
                raise Exception('No file path given.')

        if conf['write'] and conf['read']:
            if not ( conf['force'] or query_yes_no("You are reading and writing to the same file. This will overwrite your input file. "\
            "Do you want to continue?","no") ):
                raise Exception('Aborted by user.')
            
        if conf['erase'] is not None and conf['read'] and not conf['write']:
            if not ( conf['force'] or query_yes_no("You are about to erase your target before reading. "\
            "Do you want to continue?","no") ):
                raise Exception('Aborted by user.')

        if conf['read'] and not conf['write'] and conf['verify']:
            raise Exception('Verify after read not implemented.')

        if conf['len'] < 0:
            raise Exception('Length must be positive but %d was provided'
                            % (conf['len'],))

        # Try and find the port automatically
        if conf['port'] == 'auto':
            ports = []

            # Get a list of all USB-like names in /dev
            for name in ['ttyACM', 'tty.usbserial', 'ttyUSB', 'tty.usbmodem', 'tty.SLAB_USBtoUART']:
                ports.extend(glob.glob('/dev/%s*' % name))

            ports = sorted(ports)

            if ports:
                # Found something - take it
                conf['port'] = ports[0]
            else:
                raise Exception('No serial port found.')

        cmd = CommandInterface()
        cmd.open(conf['port'], conf['baud'])
        cmd.invoke_bootloader(conf['bootloader_active_high'], conf['bootloader_invert_lines'])
        mdebug(5, "Opening port %(port)s, baud %(baud)d" % {'port':conf['port'],
                                                      'baud':conf['baud']})

        firmware = None
        if conf['write'] or conf['verify']:
            mdebug(5, "Reading data from %s" % args[0])
            firmware = FirmwareFile(args[0])

        mdebug(5, "Connecting to target...")

        if not cmd.sendSynch():
            raise CmdException("Can't connect to target. Ensure boot loader is started. (no answer on synch sequence)")

        # if (cmd.cmdPing() != 1):
        #     raise CmdException("Can't connect to target. Ensure boot loader is started. (no answer on ping command)")

        chip_id = cmd.cmdGetChipId()
        chip_id_str = CHIP_ID_STRS.get(chip_id, None)

        if chip_id_str is None:
            mdebug(10, '    Unrecognized chip ID')
        else:
            mdebug(10, "    Target id 0x%x, %s" % (chip_id, chip_id_str))
            device = CC2538(cmd)


        if conf['force_speed'] != 1 and device.has_cmd_set_xosc:
            if cmd.cmdSetXOsc(): #switch to external clock source
                cmd.close()
                conf['baud'] = 1000000
                cmd.open(conf['port'], conf['baud'])
                mdebug(6, "Opening port %(port)s, baud %(baud)d" % {'port':conf['port'], 'baud':conf['baud']})
                mdebug(6, "Reconnecting to target at higher speed...")
                if (cmd.sendSynch() != 1):
                    raise CmdException("Can't connect to target after clock source switch. (Check external crystal)")
            else:
                raise CmdException("Can't switch target to external clock source. (Try forcing speed)")
        
        bytes_to_write = device.get_n_bytes(conf['section'])

        start_addr = device.get_start_address(conf['section'])

        # Choose a good default address unless the user specified -a
        fw_start = 0
        if conf['address'] is None:
            conf['address'] = start_addr
            fw_start = start_addr - device.flash_start_addr
        
        if firmware:
            bytes_to_write = min(bytes_to_write, len(firmware.bytes) - fw_start)


        if conf['erase'] is not None:
            # we only do full erase for now
            if device.erase(start_addr, bytes_to_write, conf['erase']):
                mdebug(5, "    %s erase done" % (conf['erase']))
            else:
                raise CmdException("%s erase failed" % (conf['erase']))

        if conf['write']:
            # TODO: check if boot loader back-door is open, need to read flash size first to get address
            mdebug(6, "Writing %d bytes (firmware has %d)" % (bytes_to_write, len(firmware.bytes)))
            if cmd.writeMemory(conf['address'], firmware.bytes[fw_start:fw_start+bytes_to_write]):
                mdebug(5, "    Write done                                ")
            else:
                raise CmdException("Write failed                       ")

        if conf['verify']:
            mdebug(5,"Verifying by comparing CRC32 calculations.")
            
            # Checksum target up to input firmware length or erased length
            crc_n_bytes = len(firmware.bytes)
            if (conf['erase']):
                crc_n_bytes = device.get_n_bytes(conf['section'])
            crc_local = firmware.crc32(crc_n_bytes)
            crc_target = device.crc(conf['address'], crc_n_bytes)

            if crc_local == crc_target:
                mdebug(5, "    Verified (match: 0x%08x)" % crc_local)
            else:
                cmd.cmdReset()
                raise Exception("NO CRC32 match: Local = 0x%x, Target = 0x%x" % (crc_local,crc_target))

        if conf['img_valid'] is not None:
            if conf['img_valid']:
                img_valid = [0, 0, 0, 0]
            else:
                img_valid = [0xFF, 0xFF, 0xFF, 0xFF]

            cmd.writeMemory(device.image_valid_address, img_valid)

        if conf['entry_point'] is not None:
            entry_point = struct.pack("<I", conf['entry_point'])
            cmd.writeMemory(device.entry_point_address, img_valid)

        if conf['ieee_address'] != 0:
            ieee_addr = parse_ieee_address(conf['ieee_address'])
            if PY3:
                mdebug(5, "Setting IEEE address to %s" % (':'.join(['%02x' % b for b in struct.pack('>Q', ieee_addr)])))
                ieee_addr_bytes = struct.pack('<Q', ieee_addr)
            else:
                mdebug(5, "Setting IEEE address to %s" % (':'.join(['%02x' % ord(b) for b in struct.pack('>Q', ieee_addr)])))
                ieee_addr_bytes = [ord(b) for b in struct.pack('<Q', ieee_addr)]

            if cmd.writeMemory(device.addr_ieee_address_secondary, ieee_addr_bytes):
                mdebug(5, "    Set address done                                ")
            else:
                raise CmdException("Set address failed                       ")

        if conf['read']:
            length = conf['len']

            # Round up to a 4-byte boundary
            length = (length + 3) & ~0x03

            mdebug(5, "Reading %s bytes starting at address 0x%x" % (length, conf['address']))
            with open(args[0], 'wb') as f:
                for i in range(0, length >> 2):
                    rdata = device.read_memory(conf['address'] + (i * 4)) #reading 4 bytes at a time
                    mdebug(5, " 0x%x: 0x%02x%02x%02x%02x" % (conf['address'] + (i * 4), rdata[0], rdata[1], rdata[2], rdata[3]), '\r')
                    f.write(rdata)
                f.close()
            mdebug(5, "    Read done                                ")

        if conf['disable-bootloader']:
            device.disable_bootloader()

        cmd.cmdReset()

    except Exception as err:
        traceback.print_exc()
        exit('ERROR: %s' % str(err))

