import numpy, time
from constants import *
from lis3mdl import LIS3MDL
from lsm6ds33 import LSM6DS33

class minIMU(LIS3MDL, LSM6DS33):
    ''' Class to control Pololu's minIMU-9v5. '''

    ##
    ## Class variables and constants
    ##

    GYR_GAIN = 0.035
    ACC_GAIN = 0.122 / 1000.0 * 981.0
    ALPHA = 2.0 / 6.0
    
    # Private methods
    def __init__(self, busId = 1):
        ''' Initialize some flags and values. '''
        super(minIMU, self).__init__()
        
    def __del__(self):
        ''' Cleanup routine. '''
        super(minIMU, self).__del__()


    ## Public Methods
    def enable(self, accelerometer = True, gyroscope = True, magnetometer = True, temperature = True):
        ''' Enable the devices. '''
        # Initialize imu biases
        self.gyrXMean = 0
        self.gyrYMean = 0
        self.gyrZMean = 0
        self.accXMean = 0
        self.accYMean = 0
        self.accZMean = 0

        # Initialize gyroscope and accelerometer filter stuff
        self.gyrX = 0
        self.gyrY = 0
        self.gyrZ = 0
        self.accX = 0
        self.accY = 0
        self.accZ = 0
        
        # Enable LSM6DS33 if accelerometer and/or gyroscope are requested
        if accelerometer or gyroscope:
            self.enableLSM(accelerometer = accelerometer, gyroscope = gyroscope, temperature = temperature)

            if accelerometer:
                # "Calibrate" tracked accelerometer values
                self.calibrateAccelerometer()
            
            if gyroscope:
                # "Calibrate" tracked gyroscope angles
                self.calibrateGyroscope()

        # Enable LIS3MDL if magnetometer is requested
        if magnetometer:
            self.enableLIS(magnetometer = magnetometer, temperature = temperature)

    def calibrateGyroscope(self):
        ''' Find the gyroscope biases. '''
        print("Calibrating the gyrscope, please do not move sensor.... ")
        xbuff = []
        ybuff = []
        zbuff = []
        for t in range(1024):
            omegas = self.getGyroscopeRaw()
            xbuff.append(omegas[0])
            ybuff.append(omegas[1])
            zbuff.append(omegas[2])

        self.gyrXMean = numpy.mean(xbuff)
        self.gyrYMean = numpy.mean(ybuff)
        self.gyrZMean = numpy.mean(zbuff)
        
    def getGyroCal(self):
        """ Returns a vector (list) of the calibrated gyroscope values in degrees/s. """
        rawGyr = self.getGyroscopeRaw()

        xGyr = (rawGyr[0] - self.gyrXMean) * self.GYR_GAIN
        yGyr = (rawGyr[1] - self.gyrYMean) * self.GYR_GAIN
        zGyr = (rawGyr[2] - self.gyrZMean) * self.GYR_GAIN
        
        return  [xGyr, yGyr, zGyr]

    def getGyroFil(self):
        """ Returns a vector (list) of gyroscope measurements run through a exponential moving average filter. """
        gyros = self.getGyroCal()
        self.gyrX = ((gyros[0] - self.gyrX) * self.ALPHA) + self.gyrX
        self.gyrY = ((gyros[1] - self.gyrY) * self.ALPHA) + self.gyrY
        self.gyrZ = ((gyros[2] - self.gyrZ) * self.ALPHA) + self.gyrZ
        return [self.gyrX, self.gyrY, self.gyrZ]
    
    def calibrateAccelerometer(self):
        ''' Find the accelerometer biases. '''
        print("Calibrating the accelerometer, please do not move sensor....")
        xbuff = []
        ybuff = []
        zbuff = []
        for t in range(1024):
            accels = self.getAccelerometerRaw()
            xbuff.append(accels[0])
            ybuff.append(accels[1])
            zbuff.append(accels[2])

        self.accXMean = numpy.mean(xbuff)
        self.accYMean = numpy.mean(ybuff)
        self.accZMean = numpy.mean(zbuff)
        
    def getAccelCal(self):
        """ Returns a vector (list) of the calibrated accelerometer values in m/s^2. """
        rawAccel = self.getAccelerometerRaw()

        xAcc = (rawAccel[0] - self.accXMean) * self.ACC_GAIN
        yAcc = (rawAccel[1] - self.accYMean) * self.ACC_GAIN
        zAcc = (rawAccel[2] - self.accZMean) * self.ACC_GAIN
        
        return [xAcc, yAcc, zAcc]

    def getAccelFil(self):
        """ Returns a vector (list) of accelerometer measurements run through a exponential moving average filter. """
        accels = self.getAccelCal()
        self.accX = ((accels[0] - self.accX) * self.ALPHA) + self.accX
        self.accY = ((accels[1] - self.accY) * self.ALPHA) + self.accY
        self.accZ = ((accels[2] - self.accZ) * self.ALPHA) + self.accZ
        return [self.accX, self.accY, self.accZ]

    def getIMUCal(self):
        """ Returns a vector (list) of IMU measurememnts. """
        return self.getAccelCal() + self.getGyroCal()
    
    def getIMUFil(self):
        """  Returns a vector (list) of IMU measurements. """
        return self.getAccelFil() + self.getGyroFil()
