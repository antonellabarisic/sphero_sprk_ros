#! /usr/bin/env python

import rospy
import sys
import math
import sphero_driver

sys.tracebacklimit = 0  # no traceback
sys.dont_write_bytecode = True

from SpheroDict import *
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import ColorRGBA, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class SpheroNode(object):
    """
    ROS node for Sphero SPRK+

    This node allows you to establish connection and communication with Sphero SPRK+.
    It implements sending commands and receiving responses such as velocity,
    speed, color or battery state.
    """

    def __init__(self, default_update_rate=50.0):
        """
        Initialize internal sampling frequency, parameters and subscribers and publishers.

        """
        # Set desired update rate.
        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)
        # Get Sphero name and address.
        self.is_connected = False
        self._namespace = rospy.get_namespace()
        self._address = rospy.get_param("~address")
        self.robot = sphero_driver.Sphero(self._address)  # A new instance of Sphero

        self._init_pubsub()  # Create publishers and subscribers.
        self._init_params()  # Initialize parameters.

    def _init_pubsub(self):
        """Create publishers and subscribers."""

        # Subscribers
        self.heading_sub = rospy.Subscriber('set_heading', Float32, self.set_heading, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel, queue_size=1)
        self.color_sub = rospy.Subscriber('set_color', ColorRGBA, self.set_color, queue_size=1)
        self.stabilization_sub = rospy.Subscriber('disable_stabilization', Bool,
                                                  self.set_stabilization, queue_size=1)
        self.angular_velocity_sub = rospy.Subscriber('set_angular_velocity', Float32,
                                                     self.set_angular_velocity, queue_size=1)
        self.manual_calibration_sub = rospy.Subscriber('manual_calibration', Bool,
                                                       self.manual_calibration, queue_size=1)

        # Publishers
        self.diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)

    def _init_params(self):
        """Initialize parameters for the starting color and velocity."""
        self.cmd_heading = 0
        self.last_cmd_heading = 0
        self.cmd_speed = 0
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_diagnostics_time = rospy.Time.now()
        self.power_state_msg = "No Battery Info"
        self.power_state = 0
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 5.0))
        self.diag_update_rate = rospy.Duration(rospy.get_param('~diag_update_rate', 5.0))

        self.connect_color_red = rospy.get_param('~connect_red', 0)
        self.connect_color_blue = rospy.get_param('~connect_blue', 0)
        self.connect_color_green = rospy.get_param('~connect_green', 255)

        self._data_stream = rospy.get_param("~data_stream", "None")
        self.imu = Imu()
        self.imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]

    # Program
    def start(self):
        """Connect with Sphero, setup power notification and request data stream."""
        try:
            self.is_connected = self.robot.connect()
            rospy.loginfo("Connected to Sphero %s with address: %s", self._namespace, self._address)
            # Set initial color of Sphero indicating connection.
            self.robot.set_rgb_led(self.connect_color_red, self.connect_color_green,
                                   self.connect_color_blue, 1, False)
        except Exception as e:
            rospy.logerr("Failed to connect to Sphero.")
            print("\033[1m\033[31mPlease send this error message to marko.krizmancic@fer.hr:\n\n"
                  "While trying to connect to Sphero, script exited with this error: \n"
                  "Type: {}\n"
                  "Message: {}\n\033[0m".format(sys.exc_info()[0].__name__, e))
            sys.exit(1)

        if self._data_stream == 'All':
            # Setup all data streaming.
            self.robot.set_filtered_data_strm(self.sampling_divisor, 1, 0, False)
            self.robot.add_async_callback(chr(0x03), self.parse_data_strm)
            rospy.loginfo("Data streaming of Imu data and Odometry is set")
        elif self._data_stream == 'Locator':
            # Setup real-time location data stream.
            self.robot.set_locator(1, 0, 0, 0, True)
            rospy.loginfo("Locator data streaming is set")

        # Setup power notification.
        self.robot.set_power_notify(True, True)
        self.robot.add_async_callback(chr(0x01), self.parse_power_notify)

    def spin(self):
        """Loop"""
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self._data_stream == 'Locator':
                self.get_locator_data()

            # If there is no command for velocity longer than 5 seconds, stop the Sphero.
            if (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
                if self.cmd_heading != 0 or self.cmd_speed != 0:
                    self.cmd_heading = 0
                    self.cmd_speed = 0
                    self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)
            # Update diagnostic info every 5 seconds.
            if (now - self.last_diagnostics_time) > self.diag_update_rate:
                self.last_diagnostics_time = now
                self.get_diagnostics(now)
            r.sleep()

    def stop(self):
        """Stop the Sphero before quiting."""
        self.robot.shutdown = True
        rospy.sleep(10.0)
        self.robot.sleep()
        self.is_connected = self.robot.disconnect()

    def cmd_vel(self, msg):
        """Send command for setting the velocity. Expected message type:Twist."""
        if self.is_connected:
            self.last_cmd_vel_time = rospy.Time.now()
            if (msg.linear.x == 0 and msg.linear.y == 0):
                self.cmd_heading = self.last_cmd_heading
            else:
                self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x, msg.linear.y)) * 180 / math.pi
                self.last_cmd_heading = self.cmd_heading
            self.cmd_speed = math.sqrt(math.pow(msg.linear.x, 2) + math.pow(msg.linear.y, 2))
            self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)

    def set_angular_velocity(self, msg):
        """Send command for setting the angular velocity."""
        if self.is_connected:
            rate = int((msg.data * 180 / math.pi) / 0.784)
            self.robot.set_rotation_rate(rate, False)

    def set_color(self, msg):
        """Send command for setting the color. Expected message type:ColorRGBA."""
        if self.is_connected:
            self.robot.set_rgb_led(int(msg.r * 255), int(msg.g * 255), int(msg.b * 255), 0, False)

    def set_stabilization(self, msg):
        """Send command for activating stabilization."""
        if self.is_connected:
            if not msg.data:
                self.robot.set_stabilization(1, False)  # enable stabilization
            else:
                self.robot.set_stabilization(0, False)  # disable stabilization

    def set_heading(self, msg):
        """Send command for setting the heading."""
        if self.is_connected:
            heading_deg = int(self.normalize_angle_positive(math.radians(msg.data)) * 180.0 / math.pi)
            self.robot.set_heading(heading_deg, False)

    def manual_calibration(self, msg):
        """Function for manual heading calibration."""
        if msg.data:
            self.robot.set_stabilization(0, False)  # disable stabilization
            self.robot.set_back_led(255, False)  # turn on tail light
            rospy.logdebug(" Manual calibration mode ON")
        elif not msg.data:
            self.robot.set_heading(0, False)  # set new heading
            self.robot.set_stabilization(1, False)  # re-enable stabilization
            self.robot.set_back_led(0, False)  # turn off tail light
            rospy.logdebug(" Manual calibration mode OFF")

    def normalize_angle_positive(self, angle):
        """Return positive angle in radians. [0,2pi]"""
        return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi)

    def parse_power_notify(self, data):
        """Unpack power data."""
        if self.is_connected:
            self.power_state = data
            self.power_state_msg = battery_state[data]

    def parse_data_strm(self, data):
        """
        Unpack streaming data from accelometer, gyroscope and odometer.
        """
        if self.is_connected:
            now = rospy.Time.now()
            imu = Imu(header=rospy.Header(frame_id="imu_link"))
            imu.header.stamp = now
            imu.orientation.x = data["QUATERNION_Q0"]
            imu.orientation.y = data["QUATERNION_Q1"]
            imu.orientation.z = data["QUATERNION_Q2"]
            imu.orientation.w = data["QUATERNION_Q3"]
            imu.linear_acceleration.x = data["ACCEL_X_FILTERED"] / 4096.0 * 9.8
            imu.linear_acceleration.y = data["ACCEL_Y_FILTERED"] / 4096.0 * 9.8
            imu.linear_acceleration.z = data["ACCEL_Z_FILTERED"] / 4096.0 * 9.8
            imu.angular_velocity.x = data["GYRO_X_FILTERED"] * 10 * math.pi / 180
            imu.angular_velocity.y = data["GYRO_Y_FILTERED"] * 10 * math.pi / 180
            imu.angular_velocity.z = data["GYRO_Z_FILTERED"] * 10 * math.pi / 180

            imu.orientation_covariance = IMU_ORIENTATION_COVARIANCE
            imu.angular_velocity_covariance = IMU_ANG_VEL_COVARIANCE
            imu.linear_acceleration_covariance = IMU_LIN_ACC_COVARIANCE

            self.imu_pub.publish(imu)

            odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
            odom.header.stamp = now
            odom.pose.pose = Pose(
                Point(data["ODOM_X"], data["ODOM_Y"], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
            odom.twist.twist = Twist(
                Vector3(data["VELOCITY_X"], data["VELOCITY_Y"], 0), Vector3(0, 0, 0))

            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE
            self.odom_pub.publish(odom)

    def get_locator_data(self):
        """
        Publish Sphero's current position (X,Y) and component
        velocities from streaming location data service.
        """
        data = self.robot.read_locator(True)
        print(data)
        now = rospy.Time.now()
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
        odom.header.stamp = now
        odom.pose.pose = Pose(
            Point(data["ODOM_X"], data["ODOM_Y"], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        odom.twist.twist = Twist(
            Vector3(data["VELOCITY_X"], data["VELOCITY_Y"], 0), Vector3(0, 0, 0))

        odom.pose.covariance = ODOM_POSE_COVARIANCE
        odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_pub.publish(odom)

    def get_diagnostics(self, time):
        """Publish information about battery in DiagnosticArray msg type."""
        self.robot.get_power_state(False)
        diag = DiagnosticArray()
        diag.header.stamp = time

        stat = DiagnosticStatus(
            name="Battery Status", level=DiagnosticStatus.OK, message=self.power_state_msg)
        if self.power_state == 3:
            stat.level = DiagnosticStatus.WARN
            rospy.logwarn("Battery low")
        if self.power_state == 4:
            stat.level = DiagnosticStatus.ERROR
            rospy.logerr("Battery Critical")
        diag.status.append(stat)

        self.diag_pub.publish(diag)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('sphero', log_level=rospy.DEBUG, disable_signals=True)

    try:
        s = SpheroNode()
        s.start()
        s.spin()
    except KeyboardInterrupt:
        s.stop()
