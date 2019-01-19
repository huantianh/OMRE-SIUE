# Imports
from i2c import I2C

from constants import *


# Code
class LSM6DS33(I2C):
    """ Class to set up and access LSM6DS33 accelerometer and gyroscope.
    """

    ##
    ## Class variables and constants
    ##

    
    # Register addresses
    #  ([+] = used in the code, [-] = not used or useful, [ ] = TBD)
    LSM_FUNC_CFG_ACCESS   = 0x01  # [-] Configuration of embedded
                                  #     functions, e.g. pedometer

    LSM_FIFO_CTRL1        = 0x06  # [-] FIFO threshold setting
    LSM_FIFO_CTRL2        = 0x07  # [-] FIFO control register
    LSM_FIFO_CTRL3        = 0x08  # [-] Gyro/Acceleromter-specific FIFO settings
    LSM_FIFO_CTRL4        = 0x09  # [-] FIFO data storage control
    LSM_FIFO_CTRL5        = 0x0A  # [-] FIFO ODR/Mode selection

    LSM_ORIENT_CFG_G      = 0x0B  # [ ] Gyroscope sign/orientation

    LSM_INT1_CTRL         = 0x0D  # [-] INT1 pad control - unavailable for AltIMU
    LSM_INT2_CTRL         = 0x0E  # [-] INT2 pad control - unavailable for AltIMU
    LSM_WHO_AM_I          = 0x0F  # [-] Returns 0x69 (read only)
    LSM_CTRL1_XL          = 0x10  # [+] Acceleration sensor control
    LSM_CTRL2_G           = 0x11  # [+] Angular rate sensor (gyroscope) control
    LSM_CTRL3_C           = 0x12  # [+] Device/communication settings
    LSM_CTRL4_C           = 0x13  # [ ] Bandwith/sensor/communication settings
    LSM_CTRL5_C           = 0x14  # [ ] Rounding/self-test control
    LSM_CTRL6_C           = 0x15  # [ ] Gyroscope settings
    LSM_CTRL7_G           = 0x16  # [ ] Gyroscope settings
    LSM_CTRL8_XL          = 0x17  # [ ] Acceleration sensor settings
    LSM_CTRL9_XL          = 0x18  # [ ] Acceleration sensor axis control
    LSM_CTRL10_C          = 0x19  # [ ] Gyroscope axis control / misc. settings

    LSM_WAKE_UP_SRC       = 0x1B  # [-] Wake up interrupt source register
    LSM_TAP_SRC           = 0x1C  # [-] Tap source register
    LSM_D6D_SRC           = 0x1D  # [-] Orientation sensing for Android devices

    LSM_STATUS_REG        = 0x1E  # [ ] Status register. Shows if new data
                                  #     is available from one or more of the
                                  #     sensors

    LSM_OUT_TEMP_L        = 0x20  # [+] Temperature output, low byte
    LSM_OUT_TEMP_H        = 0x21  # [+] Temperature output, high byte
    LSM_OUTX_L_G          = 0x22  # [+] Gyroscope X output, low byte
    LSM_OUTX_H_G          = 0x23  # [+] Gyroscope X output, high byte
    LSM_OUTY_L_G          = 0x24  # [+] Gyroscope Y output, low byte
    LSM_OUTY_H_G          = 0x25  # [+] Gyroscope Y output, high byte
    LSM_OUTZ_L_G          = 0x26  # [+] Gyroscope Z output, low byte
    LSM_OUTZ_H_G          = 0x27  # [+] Gyroscope Z output, high byte
    LSM_OUTX_L_XL         = 0x28  # [+] Accelerometer X output, low byte
    LSM_OUTX_H_XL         = 0x29  # [+] Accelerometer X output, high byte
    LSM_OUTY_L_XL         = 0x2A  # [+] Accelerometer Y output, low byte
    LSM_OUTY_H_XL         = 0x2B  # [+] Accelerometer Y output, high byte
    LSM_OUTZ_L_XL         = 0x2C  # [+] Accelerometer Z output, low byte
    LSM_OUTZ_H_XL         = 0x2D  # [+] Accelerometer Z output, high byte

    LSM_FIFO_STATUS1      = 0x3A  # [-] Number of unread words in FIFO
    LSM_FIFO_STATUS2      = 0x3B  # [-] FIFO status control register
    LSM_FIFO_STATUS3      = 0x3C  # [-] FIFO status control register
    LSM_FIFO_STATUS4      = 0x3D  # [-] FIFO status control register
    LSM_FIFO_DATA_OUT_L   = 0x3E  # [-] FIFO data output, low byte
    LSM_FIFO_DATA_OUT_H   = 0x3F  # [-] FIFO data output, high byte

    LSM_TIMESTAMP0_REG    = 0x40  # [-] Time stamp first byte data output
    LSM_TIMESTAMP1_REG    = 0x41  # [-] Time stamp second byte data output
    LSM_TIMESTAMP2_REG    = 0x42  # [-] Time stamp third byte data output

    LSM_STEP_TIMESTAMP_L  = 0x49  # [-] Time stamp of last step (for pedometer)
    LSM_STEP_TIMESTAMP_H  = 0x4A  # [-] Time stamp of last step, high byte
    LSM_STEP_COUNTER_L    = 0x4B  # [-] Step counter output, low byte
    LSM_STEP_COUNTER_H    = 0x4C  # [-] Step counter output, high byte

    LSM_FUNC_SRC          = 0x53  # [-] Interrupt source register for
                              #     embedded functions

    LSM_TAP_CFG           = 0x58  # [-] Configuration of embedded functions
    LSM_TAP_THS_6D        = 0x59  # [-] Orientation and tap threshold
    LSM_INT_DUR2          = 0x5A  # [-] Tap recognition settings
    LSM_WAKE_UP_THS       = 0x5B  # [-] Wake up threshold settings
    LSM_WAKE_UP_DUR       = 0x5C  # [-] Wake up function settings
    LSM_FREE_FALL         = 0x5D  # [-] Free fall duration settings
    LSM_MD1_CFG           = 0x5E  # [-] Function routing for INT1
    LSM_MD2_CFG           = 0x5F  # [-] Function routing for INT2

    # Output registers used by the accelerometer
    lsmAccRegisters = [
        LSM_OUTX_L_XL,       # low byte of X value
        LSM_OUTX_H_XL,       # high byte of X value
        LSM_OUTY_L_XL,       # low byte of Y value
        LSM_OUTY_H_XL,       # high byte of Y value
        LSM_OUTZ_L_XL,       # low byte of Z value
        LSM_OUTZ_H_XL,       # high byte of Z value
    ]

    # Output registers used by the gyroscope
    lsmGyroRegisters = [
        LSM_OUTX_L_G,       # low byte of X value
        LSM_OUTX_H_G,       # high byte of X value
        LSM_OUTY_L_G,       # low byte of Y value
        LSM_OUTY_H_G,       # high byte of Y value
        LSM_OUTZ_L_G,       # low byte of Z value
        LSM_OUTZ_H_G,       # high byte of Z value
    ]

    # Output registers used by the temperature sensor
    lsmTempRegisters = [
        LSM_OUT_TEMP_L,     # low byte of temperature value
        LSM_OUT_TEMP_H,     # high byte of temperature value
    ]

    ##
    ## Class methods
    ##

    ## Private methods
    def __init__(self, busId = 1):
        """ Set up I2C connection and initialize some flags and values.
        """
        super(LSM6DS33, self).__init__(busId)
        self.accEnabled = False
        self.gyroEnabled = False
        self.lsmTempEnabled = False
        self.gyroCalibrated = False
        self.accCalibrated = False
        

    def __del__(self):
        """ Clean up routines. """
        try:
            # Power down accelerometer
            self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL1_XL, 0x00)
            # Power down gyroscope
            self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL2_G, 0x00)
            super(LSM6DS33, self).__del__()
        except:
            pass


    ## Public methods
    def enableLSM(self, accelerometer = True, gyroscope = True,
                  temperature = True):
        """ Enable and set up the given sensors in the IMU device. """
        # Disable accelerometer and gyroscope first
        self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL1_XL, 0x00)
        self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL2_G, 0x00)
        self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL3_C, 0x00)
        
        # Initialize flags
        self.accEnabled = False
        self.gyroEnabled = False
        self.lsmTempEnabled = False
        self.accCalibrated = False
        self.gyroCalibrated = False
        
        # Disable FIFO
        self._writeRegister(LSM6DS33_ADDR, self.LSM_FIFO_CTRL5, 0x00)

        if accelerometer:
            # Accelerometer reading in millimeters per second^2
            # 1.66 kHz / +/- 4g
            # 50 Hz Anti-aliasing filter bandwidth
            # 0101 1011b
            self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL1_XL, 0x5B)
            self.accEnabled = True

        if gyroscope:
            # Gyro reading in millidegrees per second
            # 208 Hz high performance / 1000 dps
            # 0101 100b
            self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL2_G, 0x58)
            # Enable High pass filter at 2.07 Hz cutoff frequency
            # 1100 0100b  
            self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL7_G, 0xC8)
            self.gyroEnabled = True

        if temperature:
            # Temperature sensor on LSM6DS33 is "always on"
            self.lsmTempEnabled = True


    def getAccelerometerRaw(self):
        """ Return a 3-dimensional vector (list) of raw accelerometer data. """
        # Check if accelerometer has been enabled
        if not self.accEnabled:
            raise(Exception('Accelerometer has to be enabled first'))

        # Read sensor data
        return self._getSensorRawLoHi3(LSM6DS33_ADDR, self.lsmAccRegisters)


    def getGyroscopeRaw(self):
        """ Return a 3-dimensional vector (list) of raw gyroscope data. """
        # Check if gyroscope has been enabled
        if not self.gyroEnabled:
            raise(Exception('Gyroscope has to be enabled first'))

        # Read sensor data
        return self._getSensorRawLoHi3(LSM6DS33_ADDR, self.lsmGyroRegisters)

    def getLSMTemperatureRaw(self):
        """ Return the raw temperature value. """
        # Check if device has been set up
        if not self.lsmTempEnabled:
            raise(Exception('Temperature sensor has to be enabled first'))

        # Read sensor data
        return self._getSensorRawLoHi1(LSM6DS33_ADDR, self.lsmAccRegisters)


    def getIMURaw(self):
        """ Return a 6-element list of the raw output values of both IMU
            sensors, accelerometer and gyroscope.
        """
        return self.getAccelerometerRaw() + self.getGyroscopeRaw()


    def getAllRaw(self):
        """ Return a 7-element list of the raw output of all three
            sensors, accelerometer, gyroscope, temperature.
        """
        return self.getAccelerometerRaw() \
                + self.getGyroscopeRaw() \
                + [self.getLSMTemperatureRaw()]
    
    def getLSMTemperatureCelsius(self, rounded = True):
        """ Return the temperature sensor reading in C as a floating
            point number rounded to one decimal place.
        """
        # According to the datasheet, the raw temperature value is 0
        # @ 25 degrees Celsius and the resolution of the sensor is 16
        # steps per degree Celsius.
        # Thus, the following statement should return the temperature in
        # degrees Celsius.
        if rounded:
            return round(25.0 + self.getLSMTemperatureRaw() / 16.0, 1)
        return 25.0 + self.getLSMTemperatureRaw()

    
