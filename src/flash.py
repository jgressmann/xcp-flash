#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2020 Christian Wressnegger <c.wressnegger@tu-bs.de>
# Copyright (C) 2020 Jean Gressmann <jean@0x42.de>


from enum import Enum
import argparse
import logging
import platform
import sys
import time

from can import Message, Notifier, BufferedReader, CanError
import bincopy

# spec = http://read.pudn.com/downloads192/doc/comm/903802/XCP%20-Part%202-%20Protocol%20Layer%20Specification%20-1.0.pdf

version_info = (0, 1, 0)
version = '.'.join(str(c) for c in version_info)

logger = logging.getLogger("xcp-flash")
logger.setLevel(logging.INFO)
logger.addHandler(logging.StreamHandler(sys.stderr))

class LinuxErrors(Enum):
    """Linux error codes

    See /usr/include/asm-generic/errno.h
        /usr/include/asm-generic/errno-base.h

    for values.
    """
    ENOBUFS = 105 # No buffer space available

XCP_HOST_IS_LINUX = platform.system() == "Linux"
XCP_HOST_IS_WINDOWS = platform.system() == "Windows"


def xcp_timeout_ms(no: int):
    if no == 7:
        return 200

    if no == 6:
        return 5

    return 25

def xcp_timeout_s(no: int):
    return 1e-3 * xcp_timeout_ms(no)

class XCPCommands(Enum):
    """Some codes for relevant xcp commands"""
    CONNECT = 0xFF
    DISCONNECT = 0xFE
    SYNCH = 0xFC
    GET_COMM_MODE_INFO = 0xFB
    GET_ID = 0xFA
    SET_MTA = 0xF6
    UPLOAD = 0xF5
    PROGRAM_START = 0xD2
    PROGRAM_CLEAR = 0xD1
    PROGRAM = 0xD0
    PROGRAM_RESET = 0xCF
    PROGRAM_NEXT = 0xCA


class XCPResponses(Enum):
    """xcp response codes"""
    SUCCESS = 0xFF
    ERROR = 0xFE


class XCPErrors:
    """xcp error codes and their messages"""
    ERR_CMD_SYNCH = 0x00
    ERR_CMD_BUSY = 0x10
    ERR_DAQ_ACTIVE = 0x11
    ERR_PGM_ACTIVE = 0x12
    ERR_CMD_UNKNOWN = 0x20
    ERR_CMD_SYNTAX = 0x21
    ERR_OUT_OF_RANGE = 0x22
    ERR_WRITE_PROTECTED = 0x23
    ERR_ACCESS_DENIED = 0x24
    ERR_ACCESS_LOCKED = 0x25
    ERR_PAGE_NOT_VALID = 0x26
    ERR_MODE_NOT_VALID = 0x27
    ERR_SEGMENT_NOT_VALID = 0x28
    ERR_SEQUENCE = 0x29
    ERR_DAQ_CONFIG = 0x2A
    ERR_MEMORY_OVERFLOW = 0x30
    ERR_GENERIC = 0x31
    ERR_VERIFY = 0x32

    error_messages = {
        ERR_CMD_SYNCH: "Command processor synchronization.",
        ERR_CMD_BUSY: "Command was not executed.",
        ERR_DAQ_ACTIVE: "Command rejected because DAQ is running.",
        ERR_PGM_ACTIVE: "Command rejected because PGM is running.",
        ERR_CMD_UNKNOWN: "Unknown command or not implemented optional command.",
        ERR_CMD_SYNTAX: "Command syntax invalid.",
        ERR_OUT_OF_RANGE: "Command syntax valid but command parameter(s) out of range.",
        ERR_WRITE_PROTECTED: "The memory location is write protected.",
        ERR_ACCESS_DENIED: "The memory location is not accessible.",
        ERR_ACCESS_LOCKED: "Access denied, Seed & Key are required.",
        ERR_PAGE_NOT_VALID: "Selected page is not available.",
        ERR_MODE_NOT_VALID: "Selected page mode is not available.",
        ERR_SEGMENT_NOT_VALID: "Selected segment is not valid.",
        ERR_SEQUENCE: "Sequence error.",
        ERR_DAQ_CONFIG: "DAQ configuration is not valid.",
        ERR_MEMORY_OVERFLOW: "Memory overflow error",
        ERR_GENERIC: "Generic error.",
        ERR_VERIFY: "The slave internal program verify routine detects an error."
    }


class XCPFlash:
    """A Tool for flashing devices like electronic control units via the xcp-protocol."""

    BYTE_ORDER_LE = 0
    BYTE_ORDER_BE = 1



    def __init__(self, tx_id: int, rx_id: int, connection_mode: int=0, **kwargs):
        """Sets up a CAN bus instance with a filter and a notifier.

        :param tx_id:
            Id for outgoing messages
        :param rx_id:
            Id for incoming messages
        :param connection_mode:
            Connection mode for the xcp-protocol. Only set if a custom mode is needed.
        """

        self._tx_id = tx_id
        self._rx_id = rx_id
        self._ag = 1
        self._ag_override = False
        self._byte_order = XCPFlash.BYTE_ORDER_LE
        self._queue_size = 0
        self._max_block_size = 0
        self._conn_mode = connection_mode
        self._initial_comm_mode = connection_mode
        self._extended_id = False
        self._master_block_mode_supported = False
        self._master_block_mode_supported_override = False
        self._min_separation_time_us = 0
        self._base_delay_ms = max(kwargs.get("base_delay_ms", 0), 0)

        interface = kwargs.get("interface", "")
        channel = kwargs.get("channel", "")
        bus_kwargs = kwargs.get("bus_kwargs", {})

        self._extended_id = kwargs.get("extended_id", False)

        ag = kwargs.get("ag", 0)
        if ag <= 0:
            self._ag_override = False
        else:
            self._ag_override = True
            self._ag = ag

        mbm = kwargs.get("master_block_mode", -1)
        if mbm < 0:
            self._master_block_mode_supported_override = False
        else:
            self._master_block_mode_supported_override = True
            self._master_block_mode_supported = mbm != 0

        from can.interface import Bus
        if None is not interface and "" != interface:
            self._bus = Bus(channel=channel, interface=interface, **bus_kwargs)
        else:
            self._bus = Bus(channel=channel, **bus_kwargs)

        self._bus.set_filters(
            [{"can_id": rx_id, "can_mask": rx_id, "extended": self._extended_id}])


    @staticmethod
    def sys_byte_order():
        return XCPFlash.BYTE_ORDER_LE if sys.byteorder == 'little' else XCPFlash.BYTE_ORDER_BE

    @staticmethod
    def swap16(value: int):
        value &= 0xffff # ensure 16 bit
        i0 = (value >> 8) & 0xff
        i1 = value & 0xff
        return i0 | (i1 << 8)

    @staticmethod
    def swap32(value: int):
        value &= 0xffffffff # ensure 32 bit
        i0 = (value >> 24) & 0xff
        i1 = (value >> 16) & 0xff
        i2 = (value >> 8) & 0xff
        i3 = value & 0xff
        return i0 | (i1 << 8) | (i2 << 16) | (i3 << 24)


    @staticmethod
    def print_progress_bar(iteration, total, prefix='', suffix='', decimals=1, length=50, fill='█'):
        """Print a progress bar to the console.

        :param iteration:
            Actual iteration of the task
        :param total:
            Total iteration for completing the task
        :param prefix:
            Text before the progress bar
        :param suffix:
            Text after the progress bar
        :param decimals:
            Number of decimal places for displaying the percentage
        :param length:
            Line length for the progress bar
        :param fill:
            Char to fill the bar
        """
        percent = ("{0:." + str(decimals) + "f}").format(100 *
                                                         (iteration / float(total)))
        filled_length = int(length * iteration // total)
        bar = fill * filled_length + '-' * (length - filled_length)
        sys.stdout.write('\r{} |{}| {}% {}'.format(
            prefix, bar, percent, suffix))
        if iteration == total:
            print()



    def _drain_bus(self):
        rx_msgs = []

        while True:
            received = self._bus.recv(0)

            if received is None:
                break

            if received.arbitration_id == self._rx_id:
                rx_msgs.append(received)

        return rx_msgs

    def _wait_for_rx(self, timeout_s: float):
        time_left = timeout_s

        while time_left >= 0:
            start = time.perf_counter()
            received = self._bus.recv(time_left)
            stop = time.perf_counter()

            time_left -= stop - start

            if received is not None and received.arbitration_id == self._rx_id:
                return received


        return None

    def _timeout_s(self, no: int):
        return 1e-3 * (self._base_delay_ms + xcp_timeout_ms(no))

    @staticmethod
    def _is_can_device_tx_queue_full(e: CanError) -> bool:
        if XCP_HOST_IS_LINUX:
            return XCPFlash._is_linux_enobufs(e)

        return False

    @staticmethod
    def _is_linux_enobufs(e: CanError) -> bool:
        # Unfortunately, CanError doesn't pass the error code the base
        # so we can't check the errno attribute
        #
        # Failed to transmit: [Errno 105] No buffer space available
        return e.args[0].find(str(LinuxErrors.ENOBUFS.value)) >= 0



    def program(self, data):
        """Program the device

        Program the device with the given firmware

        :param data:
            the firmware as byte-array
        """

        # 0 BYTE Command Code = 0xD0
        # 1 Number of data elements [AG] [1..(MAX_CTO-2)/AG]
        # 2..AG-1 BYTE Used for alignment, only if AG >2
        # AG=1: 2..MAX_CTO-2 AG>1: AG  MAX_CTO-AG ELEMENT   Data elements

        # The MASTER_BLOCK_MODE flag indicates whether the Master Block Mode is available.
        # If  the  master  device  block  mode  is  supported,
        # MAX_BS  indicates  the  maximum  allowed  block  size  as  the  number  of
        # consecutive  command  packets  (DOWNLOAD_NEXT  or  PROGRAM_NEXT) in a block sequence.
        # MIN_ST indicates the required minimum separation time between the packets of a block
        # transfer  from  the  master  device  to  the  slave device in units of 100 microseconds.

        data_elements_per_message = int((self._max_cto - 2) / self._ag)
        bytes_per_message = data_elements_per_message * self._ag
        offset = 0

        data_offset_start = 4 if self._ag == 4 else 2
        msg = Message(arbitration_id=self._tx_id, is_extended_id=self._extended_id, data=bytes(8))

        count = 0

        if self._master_block_mode_supported:
            max_bytes_per_block = self._max_block_size * bytes_per_message
            bytes_left_in_block = 0

            while offset < len(data):
                if bytes_left_in_block:
                    msg.data[0] = XCPCommands.PROGRAM_NEXT.value
                else:
                    bytes_left_in_block = min(max_bytes_per_block, len(data) - offset)
                    msg.data[0] = XCPCommands.PROGRAM.value
                    this_block_bytes = bytes_left_in_block

                msg_bytes = min(bytes_left_in_block, bytes_per_message)
                msg.data[1] = bytes_left_in_block
                bytes_left_in_block -= msg_bytes


                msg.data[data_offset_start:data_offset_start + msg_bytes] = data[offset:offset + msg_bytes]
                offset += msg_bytes

                rx_msgs = self._drain_bus()
                for response in rx_msgs:
                    if response.data[0] == XCPResponses.ERROR.value:
                        if response.data[1] == XCPErrors.ERR_CMD_BUSY:
                            timeout_s = self._timeout_s(7)
                            count = 0
                        else:
                            message = 'fix me'
                            if XCPErrors.ERR_SEQUENCE == response.data[1]:
                                message = 'invalid sequence error'
                            else:
                                message = f'0x{response.data[1]:02x}'

                            raise ConnectionError(f'{"PROGRAM" if msg.data[0] == XCPCommands.PROGRAM.value else "PROGRAM_NEXT"} failed with {message} (0x{response.data[1]:02x})')


                while True:
                    try:
                        self._bus.send(msg)
                        break
                    except CanError as e:
                        if XCPFlash._is_can_device_tx_queue_full(e):
                            time.sleep(self._timeout_s(1))
                        else:
                            raise e

                if 0 == bytes_left_in_block:
                    response = self._wait_for_rx(self._timeout_s(5))
                    if None is response:
                        offset -= this_block_bytes
                        if count >= 3:
                            raise ConnectionError(f'PROGRAM_NEXT failed timeout')
                        else:
                            self._sync_or_die()

                            count += 1
                        continue


                    elif response.data[0] == XCPResponses.ERROR.value:
                        if response.data[1] == XCPErrors.ERR_CMD_BUSY:
                            timeout_s = self._timeout_s(7)
                            continue
                        else:
                            raise ConnectionError(f'PROGRAM failed with error code: 0x{response.data[1]:02x}')


                # wait
                if self._min_separation_time_us > 0:
                    time.sleep(self._min_separation_time_us * 1e-6)

        else:
            raise NotImplementedError('PROGRAM path not implemented')
            msg.data[0] = XCPCommands.PROGRAM.value
            msg.data[1] = data_elements_per_message
            steps = int(len(data) / bytes_per_message)

            for i in range(0, steps):
                msg.data[data_offset_start:data_offset_start + bytes_per_message] = data[offset:offset + bytes_per_message]
                offset += bytes_per_message
                self.send_can_msg(msg)


    def _sync_or_die(self):
        msg = Message(arbitration_id=self._tx_id,
                      is_extended_id=self._extended_id, data=bytes(8))


        msg.data[0] = XCPCommands.SYNCH.value
        self._drain_bus()
        self._bus.send(msg)
        response = self._wait_for_rx(self._timeout_s(7))
        if None is response:
            raise ConnectionError("timeout SYNCH")

        if response.data[0] == XCPResponses.ERROR.value:
            if response.data[1] != XCPErrors.ERR_CMD_SYNCH:
                raise ConnectionAbortedError(response.data[1])





    def program_start(self):
        msg = Message(arbitration_id=self._tx_id,
                      is_extended_id=self._extended_id, data=bytes(8))

        msg.data[0] = XCPCommands.PROGRAM_START.value
        timeout_s = self._timeout_s(1)
        count = 0

        while True:

            self._drain_bus()
            self._bus.send(msg)
            response = self._wait_for_rx(timeout_s)

            if None is response:
                if count >= 3:
                    raise ConnectionError(f'PROGRAM_START failed timeout')
                else:
                    self._sync_or_die()

                    count += 1
                continue

            if response.data[0] == XCPResponses.ERROR.value:
                if response.data[1] == XCPErrors.ERR_CMD_BUSY:
                    timeout_s = self._timeout_s(7)
                    count = 0
                    continue
                else:
                    raise ConnectionError(f'PROGRAM_START failed with error code: 0x{response.data[1]:02x}')

            break

        prog_comm_mode = response.data[2]
        # if prog_comm_mode != self._conn_mode:
        #     if self._initial_comm_mode != self._conn_mode:
        #         # prevent a ring around the rosy situation
        #         raise ConnectionError(f'communication mode already switch once from {self._initial_comm_mode} to {self._conn_mode} with new request for {prog_comm_mode}')

        #     self._conn_mode = prog_comm_mode
        #     # fix me: do this automatically
        #     raise ConnectionError(f'device requires comm mode 0x{prog_comm_mode:02x} for programming')

        self._max_block_size = response.data[4]
        self._max_cto = response.data[3]
        if self._max_cto == 0 or self._max_cto % self._ag:
            raise ConnectionError(f'inconsistent device params: MAX_CTO_PGM={self._max_cto}, ADDRESS_GRANULARITY={self._ag}')

        self._queue_size = response.data[6]
        self._min_separation_time_us = 100 * response.data[5]

        if not self._master_block_mode_supported_override:
            self._master_block_mode_supported = (response.data[2] & 1) == 1



    def set_mta(self, addr: int, addr_ext: int = 0):
        msg = Message(arbitration_id=self._tx_id,
                      is_extended_id=self._extended_id, data=bytes(8))

        msg.data[0] = XCPCommands.SET_MTA.value

        addr = self._swap32(addr)
        msg.data[3] = addr_ext
        msg.data[4] = addr & 0xff
        msg.data[5] = (addr >> 8) & 0xff
        msg.data[6] = (addr >> 16) & 0xff
        msg.data[7] = (addr >> 24) & 0xff

        timeout_s = self._timeout_s(1)
        count = 0

        while True:

            self._drain_bus()
            self._bus.send(msg)
            response = self._wait_for_rx(timeout_s)

            if None is response:
                if count >= 3:
                    raise ConnectionError(f'SET_MTA failed timeout')
                else:
                    self._sync_or_die()

                    count += 1
                continue

            if response.data[0] == XCPResponses.ERROR.value:
                if response.data[1] == XCPErrors.ERR_CMD_BUSY:
                    timeout_s = self._timeout_s(7)
                    count = 0
                    continue
                if response.data[1] == XCPErrors.ERR_PGM_ACTIVE:
                    timeout_s = self._timeout_s(7)
                    continue
                else:
                    raise ConnectionError(f'SET_MTA failed with error code: 0x{response.data[1]:02x}')

            break


    def program_clear(self, range: int):
        msg = Message(arbitration_id=self._tx_id,
                      is_extended_id=self._extended_id, data=bytes(8))

        msg.data[0] = XCPCommands.PROGRAM_CLEAR.value

        range = self._swap32(range)
        msg.data[1] = 0 # absolute address
        msg.data[2] = 0 # reserved
        msg.data[3] = 0 # reserved
        msg.data[4] = range & 0xff
        msg.data[5] = (range >> 8) & 0xff
        msg.data[6] = (range >> 16) & 0xff
        msg.data[7] = (range >> 24) & 0xff

        timeout_s = self._timeout_s(1)
        count = 0

        while True:

            self._drain_bus()
            self._bus.send(msg)
            response = self._wait_for_rx(timeout_s)

            if None is response:
                if count >= 3:
                    raise ConnectionError(f'PROGRAM_CLEAR failed timeout')
                else:
                    self._sync_or_die()

                    count += 1
                continue

            if response.data[0] == XCPResponses.ERROR.value:
                if response.data[0] == XCPErrors.ERR_CMD_BUSY:
                    timeout_s = self._timeout_s(7)
                    count = 0
                    continue
                else:
                    raise ConnectionError(f'PROGRAM_CLEAR failed with error code: 0x{response.data[1]:02x}')

            break

    def program_reset(self):
        msg = Message(arbitration_id=self._tx_id,
                      is_extended_id=self._extended_id, data=bytes(8))
        msg.data[0] = XCPCommands.PROGRAM_RESET.value


        timeout_s = self._timeout_s(5)
        count = 0

        while True:

            self._drain_bus()
            self._bus.send(msg)
            response = self._wait_for_rx(timeout_s)

            if None is response:
                if count >= 3:
                    raise ConnectionError(f'PROGRAM_CLEAR failed timeout')
                else:
                    self._sync_or_die()

                    count += 1
                continue

            if response.data[0] == XCPResponses.ERROR.value:
                if response.data[0] == XCPErrors.ERR_CMD_BUSY:
                    timeout_s = self._timeout_s(7)
                    count = 0
                    continue
                if response.data[0] == XCPErrors.ERR_PGM_ACTIVE:
                    timeout_s = self._timeout_s(7)
                    continue
                if response.data[0] == XCPErrors.ERR_CMD_UNKNOWN:
                    break
                else:
                    raise ConnectionError(f'PROGRAM_CLEAR failed with error code: 0x{response.data[1]:02x}')

            break

    def connect(self):
        """Connect to the device"""

        msg = Message(arbitration_id=self._tx_id,
                      is_extended_id=self._extended_id, data=bytes(8))
        msg.data[0] = XCPCommands.CONNECT.value
        msg.data[1] = self._conn_mode

          # spec, p138
        if self._conn_mode == 0:
            timeout_s = xcp_timeout_s(1) # some bootloader require fast startup
        else:
            timeout_s = self._timeout_s(7)

        while True:
            self._drain_bus()

            try:
                self._bus.send(msg)
                response = self._wait_for_rx(timeout_s)

            except CanError as e:
                if not XCPFlash._is_can_device_tx_queue_full(e):
                    raise e

                response = None
                time.sleep(self._timeout_s(7))

            if None is response:
                continue

            if response.data[0] == XCPResponses.ERROR.value:
                raise ConnectionAbortedError(response.data[1])

            break

        self._pgm = (response.data[1] & 0b00010000) != 0
        self._stm = (response.data[1] & 0b00001000) != 0
        self._daq = (response.data[1] & 0b00000100) != 0
        self._cal_pag = (response.data[1] & 0b00000001) != 0

        if not self._pgm:
            raise ConnectionError(
                "Flash programming not supported by the connected device")


        self._max_data = response.data[4] << 8
        self._max_data += response.data[5]
        self._byte_order = response.data[2] & 1

        if not self._ag_override:
            self._ag = 1 << ((response.data[2] >> 1) & 0x3)

        if self._byte_order == self.sys_byte_order():
            self._swap16 = lambda x: x & 0xffff
            self._swap32 = lambda x: x & 0xffffffff
        else:
            self._swap16 = self.swap16
            self._swap32 = self.swap32




    def disconnect(self, ignore_timeout: bool):
        """Disconnect from the device"""

        msg = Message(arbitration_id=self._tx_id,
                      is_extended_id=self._extended_id, data=bytes(8))
        msg.data[0] = XCPCommands.DISCONNECT.value


        timeout_s = self._timeout_s(1)
        count = 0

        while True:

            self._drain_bus()
            self._bus.send(msg)
            response = self._wait_for_rx(timeout_s)

            if None is response:
                if ignore_timeout:
                    break

                if count >= 3:
                    raise ConnectionError(f'PROGRAM_CLEAR failed timeout')
                else:
                    self._sync_or_die()

                    count += 1
                continue

            if response.data[0] == XCPResponses.ERROR.value:
                if response.data[0] == XCPErrors.ERR_CMD_BUSY:
                    timeout_s = self._timeout_s(7)
                    count = 0
                    continue
                if response.data[0] == XCPErrors.ERR_PGM_ACTIVE:
                    timeout_s = self._timeout_s(7)
                    count = 0
                    continue
                if response.data[0] == XCPErrors.ERR_CMD_UNKNOWN:
                    break
                else:
                    raise ConnectionError(f'PROGRAM_CLEAR failed with error code: 0x{response.data[1]:02x}')

            break


    def __call__(self, start_addr, data):
        """Flash the device

        Do all the necessary steps for flashing, including connecting to the device and clearing the memory.

        :param start_addr:
            Start address for the firmware
        :param data:
            The firmware as byte-array
        """

        try:
            logger.info("connecting...")
            self.connect()
            logger.info("connected")
            logger.info("start of programming")
            self.program_start()
            logger.info(f"set MTA to {start_addr:08x}")
            self.set_mta(start_addr)
            logger.info(f"clear range {len(data):08x}")
            self.program_clear(len(data))
            logger.info(f"program data")
            self.program(data)
            logger.info(f"reset")
            self.program_reset()
            logger.info(f"disconnect")
            self.disconnect(True)
        except ConnectionAbortedError as err:
            if err.args[0] == "Timeout":
                logger.error("\nConnection aborted: Timeout")
            else:
                logging.error("\nConnection aborted: {}".format(
                    logger.error_messages[err.args[0]]))
        except ConnectionError as err:
            logger.error("\nConnection error: {}".format(err))
        except OSError as err:
            logger.error(err)


if __name__ == "__main__":
    # select a sensible default value for the interface/channel
    default_interface = ""
    default_channel = ""
    bus_kwargs = {}

    if XCP_HOST_IS_WINDOWS:
        default_interface = "usb2can"
        default_channel = "ED000200"
    elif XCP_HOST_IS_LINUX:
        default_interface = "socketcan"
        default_channel = "can0"
        bus_kwargs = {
            "fd": True,
            "receive_own_messages": False
        }

    parser = argparse.ArgumentParser()
    parser.add_argument("firmware", type=str, help=".s19 firmware file")
    parser.add_argument("--txid", dest="transmission_id", type=str,
                        required=True, help="Message ID for sending (HEX)")
    parser.add_argument("--rxid", dest="response_id", type=str, required=True,
                        help="Message ID for receiving (HEX)")
    parser.add_argument("--mode", dest="conn_mode", type=str, required=False, default="00",
                        help="Connection mode for xcp-session")
    parser.add_argument("--interface", dest="interface", type=str, required=False, default=default_interface,
                        help="Interface to use (e.g 'usb2can', 'socketcan')")
    parser.add_argument("--channel", dest="channel", type=str, required=False, default=default_channel,
                        help="Channel of the interface (Linux: can0, Windows: ED000200)")
    parser.add_argument("--extended", dest="extended", type=int, required=False, default=-1,
                        help="rx/tx use extended identifiers (CAN 2.0B)")
    parser.add_argument("--ag", dest="ag", type=int, required=False, default=0,
                        help="Override address granularity reported by device (4, 2, 1).")
    parser.add_argument("--mbm", dest="master_block_mode", type=int, required=False, default=-1,
                        help="Override master block mode support (0, 1).")
    parser.add_argument("--bus-kwargs", dest="bus_kwargs", metavar="K=V, ...", type=str, required=False, default="",
                        help="Extra key=value,key=value arguments to python-can Bus.")
    parser.add_argument("--base-delay-ms", dest="base_delay_ms", metavar="MS", type=int, required=False, default=9,
                        help="Base delay to add to all XCP waits.")
    parser.add_argument('--version', action='version', version='%(prog)s {}'.format(version), help='show the version number and exit')
    args = parser.parse_args()

    f = bincopy.BinFile(args.firmware)

    txid = int(args.transmission_id, 16)
    rxid = int(args.response_id, 16)

    ext = None
    if args.extended < 0:
        ext = (txid & ~0x7FF) != 0 or (rxid & ~0x7FF) != 0
    elif args.extended == 0:
        ext = False
    else:
        ext = True

    if len(args.bus_kwargs):
        for pair in args.bus_kwargs.split(","):
            if not len(pair):
                logger.warn(f"empty key/value pair in {args.bus_kwargs}")
                continue

            index = pair.find("=")
            if index == -1:
                logger.warn(f"invalid key value pair '{pair}'")
                continue

            k, v = pair.split('=')
            bus_kwargs[str(k)] = str(v)

    xcp_flash = XCPFlash(
        txid, rxid, int(args.conn_mode, 16),
        interface=args.interface,
        channel=args.channel,
        extended_id=ext,
        ag=args.ag,
        master_block_mode=args.master_block_mode,
        bus_kwargs=bus_kwargs,
        base_delay_ms=args.base_delay_ms)
    try:
        xcp_flash(f.minimum_address, f.as_binary())
    except Exception as e:
        logger.exception(e)

    sys.exit(0)
