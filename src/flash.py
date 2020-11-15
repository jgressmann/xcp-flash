#!/usr/bin/env python3

from enum import Enum
import argparse
import platform
import sys
import time

from can import Message, Notifier, BufferedReader
import bincopy


class XCPCommands(Enum):
    """Some codes for relevant xcp commands"""
    CONNECT = 0xFF
    DISCONNECT = 0xFE
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

    _reader = None
    _bus = None
    _notifier = None
    _tx_id = 0
    _rx_id = 0
    _conn_mode = 0
    _ag = 1
    _ag_override = False
    _byte_order = BYTE_ORDER_LE
    _queue_size = 0
    _max_block_size = 0
    _initial_comm_mode = None
    _extended_id = False
    _master_block_mode_supported = False
    _min_separation_time_us = 0

    def __init__(self, tx_id: int, rx_id: int, connection_mode: int=0, **kwargs):
        """Sets up a CAN bus instance with a filter and a notifier.

        :param tx_id:
            Id for outgoing messages
        :param rx_id:
            Id for incoming messages
        :param connection_mode:
            Connection mode for the xcp-protocol. Only set if a custom mode is needed.
        """

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

        self._reader = BufferedReader()

        from can.interface import Bus
        if None is not interface and "" != interface:
            self._bus = Bus(channel=channel, interface=interface, **bus_kwargs)
        else:
            self._bus = Bus(channel=channel, **bus_kwargs)

        self._bus.set_filters(
            [{"can_id": rx_id, "can_mask": rx_id + 0x100, "extended": self._extended_id}])
        self._notifier = Notifier(self._bus, [self._reader])
        self._tx_id = tx_id
        self._rx_id = rx_id
        self._conn_mode = connection_mode
        self._initial_comm_mode = connection_mode

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
        i2 = (value >> 0) & 0xff
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

    def send_can_msg(self, msg, wait=True, timeout=30):
        """Send a message via can

        Send a message over the can-network and may wait for a given timeout for a response.

        :param msg:
            Message to send
        :param wait:
            True if the program should be waiting for a response, False otherwise
        :param timeout:
            Timeout for waiting for a response
        :return:
            The response if wait is set to True, nothing otherwise
        """

        self._bus.send(msg)
        if wait:
            try:
                return self.wait_for_response(timeout, msg)
            except ConnectionAbortedError as err:
                raise err

    def wait_for_response(self, timeout, msg=None):
        """Waiting for a response

        :param timeout:
            Time to wait
        :param msg:
            The message which was send. Set this only if a retry should be made in case of a timeout.
        :return:
            The response from the device
        :raises: ConnectionAbortedError
            if the response is an error
        """

        tries = 1
        while True:
            if tries == 5:
                raise ConnectionAbortedError("Timeout")
            received = self._reader.get_message(timeout)
            if received is None and msg is not None:
                self.send_can_msg(msg, False)
                tries += 1
                continue
            if received is None:
                continue
            if received.arbitration_id == self._rx_id and received.data[0] == XCPResponses.SUCCESS.value:
                return received
            elif received.arbitration_id == self._rx_id and received.data[0] == XCPResponses.ERROR.value:
                raise ConnectionAbortedError(received.data[1])
            elif msg is not None:
                self.send_can_msg(msg, False)

    def _wait_for_rx(self, timeout_s: float):
        time_left_ns = int(timeout_s * 1e9)

        while time_left_ns >= 0:

            start = time.perf_counter_ns()
            received = self._reader.get_message(time_left_ns * 1e-9)
            stop = time.perf_counter_ns()

            time_left_ns -= stop - start

            if received is not None and received.arbitration_id == self._rx_id:
                return received


        return None

    def execute(self, command, **kwargs):
        """Execute a command

        Builds the can-message to execute the given command and sends it to the device.

        :param command:
            The xcp-command to be executed
        :param kwargs:
            Needed arguments for the command.
                :command SET_MTA:
                    'addr_ext' and 'addr'
                :command PROGRAM_CLEAR:
                    'range'
                :command PROGRAM:
                    'size' and 'data'
        :return: response of the command if waited for
        """

        msg = Message(arbitration_id=self._tx_id,
                      is_extended_id=self._extended_id, data=bytes(8))
        msg.data[0] = command.value
        if command == XCPCommands.CONNECT:
            msg.data[1] = self._conn_mode
            response = None

            while True:
                self._bus.send(msg)
                received = self._reader.get_message(0.1) # 100 [ms]
                if None is received:
                    pass

                elif received.arbitration_id == self._rx_id:
                    if received.data[0] == XCPResponses.SUCCESS.value:
                        response = received
                        break

                    raise ConnectionAbortedError(received.data[1])
                else:
                    pass # some other message


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

            return response

        if command == XCPCommands.GET_COMM_MODE_INFO:
            done = False
            while not done:
                self._bus.send(msg)
                received = self._reader.get_message()
                if None is received:
                    break
                elif received.arbitration_id == self._rx_id:
                    if received.data[0] == XCPResponses.SUCCESS.value:
                        #Position                 Type           Description
                        # 0                       BYTE           Packet ID: 0xFF
                        # 1                       BYTE           Reserved
                        # 2                       BYTE           COMM_MODE_OPTIONAL
                        # 3                       BYTE           Reserved
                        # 4                       BYTE           MAX_BS
                        # 5                       BYTE           MIN_ST
                        # 6                       BYTE           QUEUE_SIZE
                        # 7                       BYTE           XCP Driver Version Number
                        self._master_block_mode_supported = (received.data[2] & 1) == 1
                        self._max_block_size = received.data[4]
                        self._min_separation_time_us = 100 * received.data[5]

                    break
                else:
                    pass # some other message

        if command == XCPCommands.DISCONNECT:
            # return self.send_can_msg(msg)
            self._bus.send(msg)
        if command == XCPCommands.SET_MTA:
            addr = self._swap32(kwargs['addr'])
            msg.data[3] = kwargs.get('addr_ext', 0)
            msg.data[4] = addr & 0xff
            msg.data[5] = (addr >> 8) & 0xff
            msg.data[6] = (addr >> 16) & 0xff
            msg.data[7] = (addr >> 24) & 0xff

            return self.send_can_msg(msg)
        if command == XCPCommands.PROGRAM_START:
            # 0 BYTE Packet ID: 0xFF
            # 1 BYTE Reserved
            # 2 BYTE COMM_MODE_PGM
            # 3 BYTE MAX_CTO_PGM [BYTES] Maximum CTO size for PGM
            # 4 BYTE MAX_BS_PGM
            # 5 BYTE MIN_ST_PGM
            # 6 BYTE QUEUE_SIZE_PGM

            response = self.send_can_msg(msg)

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
            self._master_block_mode_supported = (response.data[2] & 1) == 1
            self._min_separation_time_us = 100 * response.data[5]

            return response
        if command == XCPCommands.PROGRAM_CLEAR:
            # range = int(self._swap32(kwargs['range']) / self._ag)
            range = self._swap32(kwargs['range'])
            msg.data[4] = range & 0xff
            msg.data[5] = (range >> 8) & 0xff
            msg.data[6] = (range >> 16) & 0xff
            msg.data[7] = (range >> 24) & 0xff
            return self.send_can_msg(msg)

    def program(self, data):
        """Program the device

        Program the device with the given firmware

        :param data:
            the firmware as byte-array
        """
        print("flashing new firmware...")

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


        if self._master_block_mode_supported:
            max_bytes_per_block = self._max_block_size * bytes_per_message
            bytes_left_in_block = 0

            while offset < len(data):
                if bytes_left_in_block:
                    msg.data[0] = XCPCommands.PROGRAM_NEXT.value
                else:
                    bytes_left_in_block = min(max_bytes_per_block, len(data) - offset)
                    msg.data[0] = XCPCommands.PROGRAM.value

                msg_bytes = min(bytes_left_in_block, bytes_per_message)
                msg.data[1] = bytes_left_in_block
                bytes_left_in_block -= msg_bytes


                msg.data[data_offset_start:data_offset_start + msg_bytes] = data[offset:offset + msg_bytes]
                offset += msg_bytes

                self._bus.send(msg)

                # wait
                if self._min_separation_time_us > 0:
                    time.sleep(self._min_separation_time_us * 1e-6)

                if 0 == bytes_left_in_block:
                    response = self._wait_for_rx(1)

                    if None is response:
                        raise ConnectionError('timeout')

                    if response.data[0] != XCPResponses.SUCCESS.value:
                        raise ConnectionAbortedError(response.data[1])

        else:
            msg.data[0] = XCPCommands.PROGRAM.value
            msg.data[1] = data_elements_per_message
            steps = int(len(data) / bytes_per_message)

            for i in range(0, steps):
                msg.data[data_offset_start:data_offset_start + bytes_per_message] = data[offset:offset + bytes_per_message]
                offset += bytes_per_message
                self.send_can_msg(msg)

        # PROGRAM_RESET is optional, send it and hope for the best
        msg.data[0] = XCPCommands.PROGRAM_RESET.value
        self._bus.send(msg)


    def clear(self, start_addr, length):
        """Clear the memory of the device

        Erase all contents of a given range in the device memory.

        :param start_addr:
            Start address of the range
        :param length:
            Length of the range
        """
        print("erasing device (this may take several minutes)...")
        self.execute(XCPCommands.PROGRAM_START)
        self.execute(XCPCommands.SET_MTA, addr=start_addr)
        self.execute(XCPCommands.PROGRAM_CLEAR, range=length)
        # self.execute(XCPCommands.SET_MTA, addr=start_addr)

    def connect(self):
        """Connect to the device
        """
        print("connecting...")
        response = self.execute(XCPCommands.CONNECT)
        if not response.data[1] & 0b00010000:
            raise ConnectionError(
                "Flash programming not supported by the connected device")

    def disconnect(self):
        """Disconnect from the device"""
        print("disconnecting..")
        self.execute(XCPCommands.DISCONNECT)

    def __call__(self, start_addr, data):
        """Flash the device

        Do all the necessary steps for flashing, including connecting to the device and clearing the memory.

        :param start_addr:
            Start address for the firmware
        :param data:
            The firmware as byte-array
        """

        try:
            self.connect()
            #self.execute(XCPCommands.GET_COMM_MODE_INFO)
            self.clear(start_addr, len(data))
            self.program(data)
        except ConnectionAbortedError as err:
            if err.args[0] == "Timeout":
                print("\nConnection aborted: Timeout")
            else:
                print("\nConnection aborted: {}".format(
                    XCPErrors.error_messages[err.args[0]]))
        except ConnectionError as err:
            print("\nConnection error: {}".format(err))
        finally:
            try:
                self.disconnect()
            except ConnectionAbortedError as err:
                if err.args[0] == "Timeout":
                    print("\nConnection aborted: Timeout")
                else:
                    print("\nConnection aborted: {}".format(
                        XCPErrors.error_messages[err.args[0]]))


if __name__ == "__main__":
    # select a sensible default value for the interface/channel
    default_interface = ""
    default_channel = ""
    if platform.system() == "Windows":
        default_interface = "usb2can"
        default_channel = "ED000200"
    elif platform.system() == "Linux":
        default_interface = "socketcan"
        default_channel = "can0"

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
    parser.add_argument("--bus-kwargs", dest="bus_kwargs", type=str, required=False, default="",
                        help="Extra key=value,key=value arguments to python-can Bus.")
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

    bus_kwargs = {}
    for pair in args.bus_kwargs.split(","):
        k, v = pair.split('=')
        bus_kwargs[str(k)] = str(v)

    xcp_flash = XCPFlash(
        txid, rxid, int(args.conn_mode, 16),
        interface=args.interface, channel=args.channel, extended_id=ext, ag=args.ag, bus_kwargs=bus_kwargs)
    xcp_flash(f.minimum_address, f.as_binary())

    sys.exit(0)
