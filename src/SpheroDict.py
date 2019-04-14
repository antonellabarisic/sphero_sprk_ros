""" Dictionary for Sphero driver """

#These are the message response code that can be return by Sphero.
MRSP = { 
'\x00': "Command succeeded",
'\x01': "General, non non-specific error",
'\x02': "Received checksum failure",
'\x03': "Received command fragment",
'\x04': "Unknown command ID",
'\x05': "Command currently unsupported",
'\x06': "Bad message format",
'\x07': "Parameter value(s) invalid",
'\x08': "Failed to execute command",
'\x09': "Unknown device ID",
'\x31': "Voltage too low for refash operation",
'\x32': "Illegal page number provided",
'\x33': "Page did not reprogram correctly",
'\x34': "Main application corrupt",
'\x35': "Msg state machine timed out",  }

#ID codes for asynchronous packets
IDCODE = dict(
  PWR_NOTIFY = chr(0x01),               #Power notifications
  LEVEL1_DIAG = chr(0x02),              #Level 1 Diagnostic response
  DATA_STRM = chr(0x03),                #Sensor data streaming
  CONFIG_BLOCK = chr(0x04),             #Config block contents
  SLEEP = chr(0x05),                    #Pre-sleep warning (10 sec)
  MACRO_MARKERS =chr(0x06),             #Macro markers
  COLLISION = chr(0x07))                #Collision detected

# SOP1 and SOP2 for async and sync
RECV = dict(
  ASYNC = [chr(0xff), chr(0xfe)],
  SYNC = [chr(0xff), chr(0xff)])

#DID and CID for core(DID=00h) and Sphero(DID=02h) commands
REQ = dict(
  WITH_RESPONSE =[0xff, 0xff],
  WITHOUT_RESPONSE =[0xff, 0xfe],
  CMD_PING = [0x00, 0x01],
  CMD_VERSION = [0x00, 0x02],
  CMD_SET_BT_NAME = [0x00, 0x10],
  CMD_GET_BT_NAME = [0x00, 0x11],
  CMD_SET_AUTO_RECONNECT = [0x00, 0x12],
  CMD_GET_AUTO_RECONNECT = [0x00, 0x13],
  CMD_GET_PWR_STATE = [0x00, 0x20],
  CMD_SET_PWR_NOTIFY = [0x00, 0x21],
  CMD_SLEEP = [0x00, 0x22],
  CMD_GOTO_BL = [0x00, 0x30],
  CMD_RUN_L1_DIAGS = [0x00, 0x40],
  CMD_RUN_L2_DIAGS = [0x00, 0x41],
  CMD_CLEAR_COUNTERS = [0x00, 0x42],
  CMD_ASSIGN_COUNTER = [0x00, 0x50],
  CMD_POLL_TIMES = [0x00, 0x51],

  CMD_SET_HEADING = [0x02, 0x01],
  CMD_SET_STABILIZ = [0x02, 0x02],
  CMD_SET_ROTATION_RATE = [0x02, 0x03],
  CMD_SET_APP_CONFIG_BLK = [0x02, 0x04],
  CMD_GET_APP_CONFIG_BLK = [0x02, 0x05],
  CMD_SET_DATA_STRM = [0x02, 0x11],
  CMD_CFG_COL_DET = [0x02, 0x12],
  CMD_LOCATOR= [0x02, 0x13], 
  CMD_READ_LOCATOR= [0x02, 0x15],

  CMD_SET_RGB_LED = [0x02, 0x20],
  CMD_SET_BACK_LED = [0x02, 0x21],
  CMD_GET_RGB_LED = [0x02, 0x22],
  CMD_ROLL = [0x02, 0x30],
  CMD_BOOST = [0x02, 0x31],
  CMD_SET_RAW_MOTORS = [0x02, 0x33],
  CMD_SET_MOTION_TO = [0x02, 0x34],
  CMD_GET_CONFIG_BLK = [0x02, 0x40],
  CMD_SET_DEVICE_MODE = [0x02, 0x42],
  CMD_SET_CFG_BLOCK = [0x02, 0x43],
  CMD_GET_DEVICE_MODE = [0x02, 0x44],
  CMD_RUN_MACRO = [0x02, 0x50],
  CMD_SAVE_TEMP_MACRO = [0x02, 0x51],
  CMD_SAVE_MACRO = [0x02, 0x52],
  CMD_DEL_MACRO = [0x02, 0x53],
  CMD_INIT_MACRO_EXECUTIVE = [0x02, 0x54],
  CMD_ABORT_MACRO = [0x02, 0x55],
  CMD_GET_MACRO_STATUS = [0x02, 0x56],
  CMD_SET_MACRO_STATUS = [0x02, 0x57])

# Masks for data streaming
STRM_MASK1 = dict(
  GYRO_H_FILTERED    = 0x00000001,
  GYRO_M_FILTERED    = 0x00000002,
  GYRO_L_FILTERED    = 0x00000004,
  LEFT_EMF_FILTERED  = 0x00000020,
  RIGHT_EMF_FILTERED = 0x00000040,
  MAG_Z_FILTERED     = 0x00000080,
  MAG_Y_FILTERED     = 0x00000100,
  MAG_X_FILTERED     = 0x00000200,
  GYRO_Z_FILTERED    = 0x00000400,
  GYRO_Y_FILTERED    = 0x00000800,
  GYRO_X_FILTERED    = 0x00001000,
  ACCEL_Z_FILTERED   = 0x00002000,
  ACCEL_Y_FILTERED   = 0x00004000,
  ACCEL_X_FILTERED   = 0x00008000,
  IMU_YAW_FILTERED   = 0x00010000,
  IMU_ROLL_FILTERED  = 0x00020000,
  IMU_PITCH_FILTERED = 0x00040000,
  LEFT_EMF_RAW       = 0x00200000,
  RIGHT_EMF_RAW      = 0x00400000,
  MAG_Z_RAW          = 0x00800000,
  MAG_Y_RAW          = 0x01000000,
  MAG_X_RAW          = 0x02000000,
  GYRO_Z_RAW         = 0x04000000,
  GYRO_Y_RAW         = 0x08000000,
  GYRO_X_RAW         = 0x10000000,
  ACCEL_Z_RAW        = 0x20000000,
  ACCEL_Y_RAW        = 0x40000000,
  ACCEL_X_RAW        = 0x80000000)

STRM_MASK2 = dict(
  QUATERNION_Q0      = 0x80000000,
  QUATERNION_Q1      = 0x40000000,
  QUATERNION_Q2      = 0x20000000,
  QUATERNION_Q3      = 0x10000000,
  ODOM_X             = 0x08000000,
  ODOM_Y             = 0x04000000,
  ACCELONE           = 0x02000000,
  VELOCITY_X         = 0x01000000,
  VELOCITY_Y         = 0x00800000)

#Mask for Locator data 
LOC_MASK = dict(
  VELOCITY_X         = 0x01000000,
  VELOCITY_Y         = 0x00800000,
  ODOM_X             = 0x08000000,
  ODOM_Y             = 0x04000000
  )

battery_state = {1: "Battery Charging",
                 2: "Battery OK",
                 3: "Battery Low",
                 4: "Battery Critical"}

ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]

IMU_ORIENTATION_COVARIANCE = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
IMU_ANG_VEL_COVARIANCE = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
IMU_LIN_ACC_COVARIANCE = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]