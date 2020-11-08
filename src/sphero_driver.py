#!/usr/bin/env python

import sys
import binascii
import operator
import threading
import struct

import bluepy
from util import *
from SpheroDict import *

sys.dont_write_bytecode = True

# Service UUIDs
RobotControlService = "22bb746f2ba075542d6f726568705327"
BLEService = "22bb746f2bb075542d6f726568705327"
AntiDosCharacteristic = "22bb746f2bbd75542d6f726568705327"
TXPowerCharacteristic = "22bb746f2bb275542d6f726568705327"
WakeCharacteristic = "22bb746f2bbf75542d6f726568705327"
ResponseCharacteristic = "22bb746f2ba675542d6f726568705327"
CommandsCharacteristic = "22bb746f2ba175542d6f726568705327"


class DelegateObj(bluepy.btle.DefaultDelegate):
    """
    Delegate object that gets called when there is a notification
    """

    def __init__(self, sphero_obj, lock):
        bluepy.btle.DefaultDelegate.__init__(self)
        self._sphero_obj = sphero_obj
        self._callback_dict = {}
        self._wait_list = {}
        self._data_group_callback = {}
        self._enabled_group = []
        self._buffer_bytes = b''
        self._notification_lock = lock

        # init callback dictionaries
        self._async_callback_dict = dict()
        self._sync_callback_dict = dict()

    def register_callback(self, seq, callback):
        self._callback_dict[seq] = callback

    def register_async_callback(self, group_name, callback):
        """ Add asynchronous callback to enabled list """
        self._data_group_callback[group_name] = callback
        self._enabled_group = list(set(self._enabled_group) | {group_name})

    def handle_callbacks(self, packet):
        # unregister callback
        callback = self._callback_dict.pop(packet[3])
        MRSP = packet[2]
        dlen = (packet[4] - 1)
        data = []
        if dlen > 0:
            data = packet[5:5 + dlen]
        # parse the packet
        callback(MRSP, data)

    def wait_for_resp(self, seq, timeout=None):
        # function waits for a response in the handle notification part
        self._wait_list[seq] = None
        while self._wait_list[seq] is None:
            with self._notification_lock:
                self._sphero_obj._device.waitForNotifications(0.05)
        return self._wait_list.pop(seq)

    def parse_single_pack(self, data):
        """ 
        For received packet determine type of response
        and send data to appropriate function.
        ______________________________________________

        Response can be synchronous and asynchronous. Sync response ...
        This driver suports async response for data streaming and power notifications.
        """
        if self._sphero_obj.is_connected:
            if data[1] == '\xff':  # Sync message
                # get the sequence number and check if a callback is assigned
                if data[3] in self._callback_dict:
                    self.handle_callbacks(data)
                # check if we have it in the wait list
                elif from_bytes(data[3], 'big') in self._wait_list:
                    self._wait_list[from_bytes(data[3], 'big')] = data
                elif len(data) == 6 and data[0] == '\xff':  # response on command
                    print("\033[1mSphero response:{}\033[0m".format(MRSP[data[2]]))
                else:
                    print("\033[1mUnknown response:{}\033[0m".format(data))
            elif data[1] == '\xfe':  # Async message

                data_length = (ord(data[3]) << 8) + ord(data[4])
                data_packet = data[:(5 + data_length)]

                if data[2] == '\x03':  # data stream packet
                    self._data_group_callback['\x03'](self.parse_data_strm(data_packet, data_length))
                elif data[2] == '\x01':  # power info packet
                    self._data_group_callback['\x01'](self.parse_pwr_notify(data_packet, data_length))
                else:
                    print("\033[1mUnknown async response with ID code:{}\033[0m".format(from_bytes(data[2])))
            else:
                pass

    def handleNotification(self, cHandle, data):
        """

        Called whenever a notification has been received from a Peripheral.
        The cHandle parameter is the GATT 'handle' for the characteristic
        which is sending the notification. The data parameter is value
        containing the notification data.

        Data is collected in a buffer. Validation of packet is done by
        checking starting bytes (SOP1 and SOP2) and checksum. If packet is valid
        we call parsing function.

        """
        # merge the data with previous incomplete instance
        self._buffer_bytes = self._buffer_bytes + data

        # check if data is valid
        while len(self._buffer_bytes) > 5:  # we need at least 6 bytes
            # split the data until it's a valid chunk
            i = 0
            chks_pckt = 0
            if ((self._buffer_bytes[0] == '\xff') and ((self._buffer_bytes[1] == '\xff')
                                                       or (self._buffer_bytes[1] == '\xfe'))):
                # assuming we found the start (SOP1 & SOP2) od the packet
                if self._buffer_bytes[i + 1] == '\xff':  # sync packet
                    len_int = ord(self._buffer_bytes[i + 4])
                    chks_pckt = cal_packet_checksum(self._buffer_bytes[i + 2:i + 5 + len_int - 1])
                    if len(self._buffer_bytes) >= (len_int + 5):
                        if chks_pckt == ord(self._buffer_bytes[4 + len_int]):  # checksum is good
                            # extract data and call parsing function
                            data_s_pack = self._buffer_bytes[0:5 + len_int]
                            self._buffer_bytes = self._buffer_bytes[len_int + 5:]
                            self.parse_single_pack(data_s_pack)
                        else:  # checksum is not good
                            self._buffer_bytes = self._buffer_bytes[1:]
                    else:
                        break

                elif self._buffer_bytes[i + 1] == '\xfe':  # async packet
                    len_int = (ord(self._buffer_bytes[i + 3]) << 8) + ord(self._buffer_bytes[i + 4])
                    chks_pckt = cal_packet_checksum(
                        self._buffer_bytes[i + 2:i + 5 + len_int - 1])  # -1 for excluding checksum
                    if len(self._buffer_bytes) >= (len_int + 5):
                        if chks_pckt == ord(self._buffer_bytes[4 + len_int]):  # checksum is good
                            # extract data and call parsing function
                            data_s_pack = self._buffer_bytes[0:4 + len_int]
                            self._buffer_bytes = self._buffer_bytes[len_int + 4:]
                            self.parse_single_pack(data_s_pack)
                        else:  # checksum is not good
                            self._buffer_bytes = self._buffer_bytes[1:]
                    elif len_int > 300:
                        self._buffer_bytes = self._buffer_bytes[1:]
                    else:
                        break
            else:
                self._buffer_bytes = self._buffer_bytes[1:]

    def parse_pwr_notify(self, data, data_length):
        """
        The data payload of the async message is 1h bytes long and
        formatted as follows:
          --------
          |State |
          --------
        The power state byte:
          * 01h = Battery Charging,
          * 02h = Battery OK,
          * 03h = Battery Low,
          * 04h = Battery Critical
        """
        return struct.unpack_from('B', ''.join(data[5:]))[0]

    def parse_data_strm(self, data, data_length):
        """ Unpack streaming data using created mask list """
        output = {}
        for i in range((data_length - 1) / 2):
            unpack = struct.unpack_from('>h', ''.join(data[5 + 2 * i:]))
            output[self._sphero_obj.mask_list[i]] = unpack[0]
        return output


class Sphero(threading.Thread):
    """
    Sphero class ...

    """

    def __init__(self, addr=None):
        threading.Thread.__init__(self)

        # If address is not specified, search for Spheros.
        if addr is None:
            sphero_list = search_for_sphero()
            if len(sphero_list) == 0:
                raise Exception("No Sphero Found in Vicinity")
            addr = sphero_list[0]

        self._addr = addr
        self.is_connected = False
        self.seq = 0
        self._stream_rate = 10
        self.shutdown = False
        # load the mask list
        self.stream_mask1 = None
        self.stream_mask2 = None

        self._notification_lock = threading.RLock()

    def connect(self):
        """
        Connects the sphero with the address given in the constructor.
        Peripheral object is Sphero and Delegate object is user(client).

        The Peripheral will throw a BTLEException if connection to the device fails.
        """
        self._device = bluepy.btle.Peripheral(self._addr, addrType=bluepy.btle.ADDR_TYPE_RANDOM)
        self.is_connected = True

        self._notifier = DelegateObj(self, self._notification_lock)
        self._device.withDelegate(self._notifier)  # set notifier to be notified

        self._devModeOn()

        # get the command service
        cmd_service = self._device.getServiceByUUID(RobotControlService)
        self._cmd_characteristics = {}
        characteristic_list = cmd_service.getCharacteristics()
        for characteristic in characteristic_list:
            uuid_str = binascii.b2a_hex(characteristic.uuid.binVal).decode('utf-8')
            self._cmd_characteristics[uuid_str] = characteristic

        self._listening_flag = True
        self._listening_thread = threading.Thread(target=self._listening_loop)
        self._listening_thread.start()
        return True

    def _devModeOn(self):
        """
        Enables developer mode. This is accomplished via sending a special string
        to the Anti-DoS service, setting TX power to 7, and telling the robot to wake up.

        Sequence of unlock codes:
        1. Write '011i3' to AntiDOS
        2. write '0x07' to TX Power
        3. Write '0x01' to Wakeup

        """
        service = self._device.getServiceByUUID(BLEService)
        characteristic_list = service.getCharacteristics()
        # make it into a dict
        characteristic_dict = {}
        for characteristic in characteristic_list:
            uuid_str = binascii.b2a_hex(characteristic.uuid.binVal).decode('utf-8')
            characteristic_dict[uuid_str] = characteristic
        # Anti Denial of Service characteristic
        characteristic = characteristic_dict[AntiDosCharacteristic]
        characteristic.write("011i3".encode(encoding='UTF-8'))
        # TX Power characteristic exposes a device's current transmit power level 
        characteristic = characteristic_dict[TXPowerCharacteristic]
        characteristic.write("07".encode(encoding='UTF-8'))
        # Initiate wake up
        characteristic = characteristic_dict[WakeCharacteristic]
        characteristic.write("01".encode(encoding='UTF-8'))

    def send(self, cmd, data, resp):
        """ 
        Packets are sent from Client -> Sphero in the following byte format:
         -------------------------------------------------------
         | SOP1 | SOP2 | DID | CID | SEQ | DLEN | <data> | CHK |
         -------------------------------------------------------
        """
        packed_data = self.pack_cmd(cmd, data)
        checksum = ~ sum(packed_data) % 256
        if resp:
            output = REQ['WITH_RESPONSE'] + packed_data + [checksum]
        else:
            output = REQ['WITHOUT_RESPONSE'] + packed_data + [checksum]

        with self._notification_lock:
            self._cmd_characteristics[CommandsCharacteristic].write(bytes(output))
        return self.seq

    def _listening_loop(self):
        while self._listening_flag:
            with self._notification_lock:
                self._device.waitForNotifications(0.001)

    """ Functions for packing data """

    def pack_cmd(self, req, cmd):
        """ Pack CID, SEQ, DLEN and DATA for sending commands to Sphero """
        self.inc_seq()
        return req + [self.seq] + [len(cmd) + 1] + cmd

    def inc_seq(self):
        """ Auto-increments seq counter for command and callback queues """
        self.seq = self.seq + 1
        if self.seq > 0xff:
            self.seq = 0

    def clamp(self, n, minn, maxn):
        """ Function for ensuring data is in range """
        return max(min(maxn, n), minn)

    def create_mask_list(self, mask1, mask2):
        """ Create mask list for data streaming """

        # sort items in STRM_MASK1 by values
        sorted_STRM1 = sorted(STRM_MASK1.items(), key=operator.itemgetter(1), reverse=True)
        # create a list containing only the keys specified by user
        self.mask_list1 = [key for key, value in sorted_STRM1 if value & mask1]
        # sort items in STRM_MASK2 by values
        sorted_STRM2 = sorted(STRM_MASK2.items(), key=operator.itemgetter(1), reverse=True)
        # create a list containing only the keys specified by user
        self.mask_list2 = [key for key, value in sorted_STRM2 if value & mask2]

        self.mask_list = self.mask_list1 + self.mask_list2

    """ CORE functionality """

    def ping(self, resp=True):
        """The Ping command verifies the Sphero is awake and receiving commands."""
        return self.send(REQ['CMD_PING'], [], resp)

    def sleep(self, resp=True):
        """ The Sleep command puts Sphero to sleep immediately. """
        return self.send(REQ['CMD_SLEEP'], [0x00, 0x00, 0x00], resp)

    def set_power_notify(self, enable, response):
        """
        This enables Sphero to asynchronously notify the Client
        periodically with the power state or immediately when the power
        manager detects a state change. Timed notifications arrive every 10
        seconds until they're explicitly disabled or Sphero is unpaired. The
        flag is as you would expect, 00h to disable and 01h to enable.

        :param enable:   00h to disable and 01h to enable power notifications.
        :param response: request response back from Sphero.
        """
        self.send(REQ['CMD_SET_PWR_NOTIFY'], [enable], response)

    def get_power_state(self, response):
        """
        This returns the current power state and some additional
        parameters to the Client.

        :param response: request response back from Sphero.
        """
        self.send(REQ['CMD_GET_PWR_STATE'], [], response)

    def get_device_name(self, resp):
        """ Return Bluetooth advertising name and address for Sphero """
        seq_num = self.send(REQ['CMD_GET_BT_NAME'], [], resp)
        response = self._notifier.wait_for_resp(seq_num)
        name_data = {"name": response[5:12].decode('utf-8'),
                     "bta": response[21:33].decode('utf-8'),
                     "color": response[33:36].decode("utf-8")}
        return name_data

    """ Sphero functionality """

    def roll(self, speed, heading, state, resp=False):
        """
        This commands Sphero to roll along the provided vector. Both a
        speed and a heading are required; the latter is considered
        relative to the last calibrated direction. A state Boolean is also
        provided (on or off). The client convention for heading follows the 360
        degrees on a circle, relative to the ball: 0 is straight ahead, 90
        is to the right, 180 is back and 270 is to the left. The valid
        range is 0..359.

        :param speed:    0-255 value representing 0-max speed of the sphero.
        :param heading:  heading in degrees from 0 to 359.
        :param state:    00h for off (braking) and 01h for on (driving).
        :param resp: request response back from Sphero.
        """
        self.send(REQ['CMD_ROLL'], [self.clamp(speed, 0, 255), (heading >> 8), (heading & 0xff), state], resp=resp)

    def set_heading(self, heading, resp=False):
        """
        This allows the client to adjust the orientation of Sphero by
        commanding a new reference heading in degrees, which ranges from 0
        to 359. You will see the ball respond immediately to this command
        if stabilization is enabled.

        :param heading:  heading in degrees from 0 to 359 (motion will be\
                         shortest angular distance to heading command)
        :param resp: request response back from Sphero.
        """
        self.send(REQ['CMD_SET_HEADING'], [(heading >> 8), (heading & 0xff)], resp=resp)

    def set_rgb_led(self, red, green, blue, save, resp=False):
        """
        This allows you to set the RGB LED color. The composite value is
        stored as the "application LED color" and immediately driven to
        the LED (if not overridden by a macro or orbBasic operation). If
        FLAG is true, the value is also saved as the "user LED color"
        which persists across power cycles and is rendered in the gap
        between an application connecting and sending this command.

        :param red:   red color value.
        :param green: green color value.
        :param blue:  blue color value.
        :param save:  01h for save (color is saved as "user LED color").
        :param resp: request response back from Sphero.
        """
        self.send(REQ['CMD_SET_RGB_LED'],
                  [self.clamp(red, 0, 255), self.clamp(green, 0, 255), self.clamp(blue, 0, 255), save], resp=resp)

    def set_back_led(self, brightness, resp=False):
        """  """
        self.send(REQ['CMD_SET_BACK_LED'], [self.clamp(brightness, 0, 255)], resp=resp)

    def set_stabilization(self, bool_flag, resp=False):
        """
        This turns on or off the internal stabilization of Sphero, in
        which the IMU is used to match the ball's orientation to its
        various set points. The flag value is as you would expect, 00h for
        off and 01h for on.

        :param bool_flag:   00h for off and 01h for on (on by default).
        :param resp: Request response back from Sphero.
        """
        data = [0x01 if bool_flag else 0x00]
        self.send(REQ['CMD_SET_STABILIZ'], data, resp=resp)

    def set_raw_motor_values(self, lmode, lpower, rmode, rpower, resp=False):
        """
        This allows you to take over one or both of the motor output
        values, instead of having the stabilization system control
        them. Each motor (left and right) requires a mode (see below) and
        a power value from 0- 255. This command will disable stabilization
        if both modes aren't "ignore" so you'll need to re-enable it via
        CID 02h once you're done.

        :param lmode:  0x00 - off,
                       0x01 - forward,
                       0x02 - reverse,
                       0x03 -brake,
                       0x04 - ignored.
        :param lpower: 0-255 scalar value (units?).
        :param rmode:  Same as lmode.
        :param rpower: Same as lpower.
        :param resp: Request response back from Sphero.
        """
        data = [lmode, int(lpower), rmode, int(rpower)]

        self.send(REQ['CMD_SET_RAW_MOTORS'], data, resp=resp)

    def set_rotation_rate(self, rate, resp):
        """
        This allows you to control the rotation rate that Sphero will use
        to meet new heading commands. The commanded value is in units of
        0.784 degrees/sec. So, setting a value of c8h will set the
        rotation rate to 157 degrees/sec. A value of 255 jumps to the
        maximum and a value of 1 is the minimum.

        :param rate:     rotation rate in units of 0.784degrees/sec (setting 
                         this value will not cause the device to move only 
                         set the rate it will move in other funcation calls).
        :param resp: request response back from Sphero.
        """
        self.send(REQ['CMD_SET_ROTATION_RATE'], [self.clamp(rate, 0, 255)], resp)

    def set_locator(self, flags, x, y, yawTare, resp):
        """  
        Configures Sphero's streaming location data service.
        
        The following options must be provided:

        :param flags:   bit 0 determines whether calibrate commands auto-correct the
                        yaw tare value. When false, positive Y axis coincides with heading 0.
                        Other bits are reserved.
        :param x:       the current x coordinate of Sphero on the ground plane in
                        centimeters
        :param y:       the current y coordinate of Sphero on the ground plane in
                        centimeters
        :param yawTare: controls how the x,y-plane is aligned with Sphero's heading
                        coordinate system. When zero, yaw = 0 corresponds to facing
                        down the y-axis in the positive direction. 
                        Possible values are 0-359 inclusive.
        :param resp:    request response back from Sphero.
        """

        self.send(REQ['CMD_LOCATOR'], [(flags & 0x01), ((x >> 8) & 0xff), (x & 0xff),
                                       ((y >> 8) & 0xff), (y & 0xff), ((yawTare >> 8) & 0xff), (yawTare & 0xff)], resp)

    def read_locator(self, resp):
        """ 
        The Read Locator command gets Sphero's current position (X,Y) and component
        velocities. The position is a signed value in centimeters and the component
        velocities are signed cm/sec.
        """
        seq_num = self.send(REQ['CMD_READ_LOCATOR'], [], resp)
        response = self._notifier.wait_for_resp(seq_num)
        mask_locator = [key for key in LOC_MASK if (1)]
        output = {}
        for i in range(4):
            unpack = struct.unpack_from('>h', ''.join(response[5 + 2 * i:]))
            output[mask_locator[i]] = unpack[0]
        return output

    def set_filtered_data_strm(self, sample_div, sample_frames, pcnt, response):
        """
        Helper function to add Imu and Odometry data to the data strm
        mask, so that the user doesn't have to set the data strm manually.

        :param sample_div:    divisor of the maximum sensor sampling rate.
        :param sample_frames: number of sample frames emitted per packet.
        :param pcnt:          packet count (set to 0 for unlimited streaming).
        :param response:      request response back from Sphero.
        """
        mask1 = 0
        mask2 = 0
        for key, value in STRM_MASK1.items():
            if 'FILTERED' in key:
                if ('GYRO' in key) or ('ACCEL' in key):
                    mask1 = mask1 | value
        for value in STRM_MASK2.values():
            mask2 = mask2 | value
        self.set_data_strm(sample_div, sample_frames, mask1, pcnt, mask2, response)

    def set_data_strm(self, sample_div, sample_frames, sample_mask1, pcnt, sample_mask2, response):
        """
        Currently the control system runs at 400Hz and because it's pretty
        unlikely you will want to see data at that rate, N allows you to
        divide that down. sample_div = 2 yields data samples at 200Hz,
        sample_div = 10, 40Hz, etc. Every data sample consists of a
        "frame" made up of the individual sensor values as defined by the
        sample_mask. The sample_frames value defines how many frames to
        collect in memory before the packet is emitted. In this sense, it
        controls the latency of the data you receive. Increasing
        sample_div and the number of bits set in sample_mask drive the
        required throughput. You should experiment with different values
        of sample_div, sample_frames and sample_mask to see what works
        best for you.

        :param sample_div:    divisor of the maximum sensor sampling rate.
        :param sample_frames: number of sample frames emitted per packet.
        :param sample_mask1:  bitwise selector of data sources to stream.
        :param sample_mask2:  ?
        :param pcnt:          packet count (set to 0 for unlimited streaming).
        :param response:      request response back from Sphero.
        """
        self.send(REQ['CMD_SET_DATA_STRM'],
                  [(sample_div >> 8), (sample_div & 0xff), (sample_frames >> 8), (sample_frames & 0xff),
                   ((sample_mask1 >> 24) & 0xff), ((sample_mask1 >> 16) & 0xff), ((sample_mask1 >> 8) & 0xff),
                   (sample_mask1 & 0xff),
                   pcnt, ((sample_mask2 >> 24) & 0xff), ((sample_mask2 >> 16) & 0xff), ((sample_mask2 >> 8) & 0xff),
                   (sample_mask2 & 0xff)], response)

        self.create_mask_list(sample_mask1, sample_mask2)
        self.stream_mask1 = sample_mask1
        self.stream_mask2 = sample_mask2

    # callbacks

    def add_async_callback(self, callback_type, callback):
        self._notifier.register_async_callback(callback_type, callback)

    def disconnect(self):
        self.is_connected = False
        self._device.disconnect()
        return self.is_connected
