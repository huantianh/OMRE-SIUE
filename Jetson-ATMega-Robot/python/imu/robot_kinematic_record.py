import serial, time os
from minIMU import minIMU

# Constants
SAVE_FOLDER = "robot_data/" # Specifiy folder to save text data files in
COMMAND_TO_VELOCITY = 0.23529 # Converts pwm signal to velocity in cm/s (255 pwm signal = 60 cm/s)

# Declare initial motor velocities
v_f = 0
v_r = 60
v_l = 60
V = [v_f,v_r,v_l]

# Create folder for storing data if it doesn't already exist
# If folder already exists, it will replace any data files that are already there
current_directory = os.getcwd()
save_directory = os.path.join(current_directory, save_folder)
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# Storage arrays (lists) for graphing and storing to text files
times_arduino = []
encoders_left = []
encoders_right = []
encoders_front = []
velocities_left = []
velocities_right = []
velocities_front = []

times_pi = []
omegas = []

# IMU setup
imu = minIMU()
imu.enable() # Enables and calibrates the imu, make sure it is stationary

# Serial setup, this lets you plug the arduino into any USB port on the raspi
i = 0
portname = '/dev/ttyACM'
while True:
    try:
        ser = serial.Serial(portname + str(i), 115200, timeout=1)
        ser.flushInput()
        ser.flushOutput()
        break
    except:
        i += 1
# Arduino should restart upon sucessful serial connection being made

# Wait for ready signal from arduino
while True:
    if ser.inWaiting() > 0:
        response = ser.readline()
        break
    
# Get starting time
start_time = time.time()
    
# Send initial motor velocities to arduino as pwm signals
# velocities are sent as a pair (direction, speed)
# direction of 0 => motors truns CW, direction of 1 => motors turns CCW
# Check initial velocity directions and print to serial
for velocity in V: 
    if velocity < 0: # Move counter-clockwise
        ser.write(chr(1))
        ser.write(chr(int(abs(velocity / COMMAND_TO_VELOCITY))))
    else: # Move clockwise
        ser.write(chr(0))
        ser.write(chr(int(velocity / COMMAND_TO_VELOCITY)))

# Data collection
try:
    while True:
        # Wait for data from arduino to become available in serial
        while True: 
            if ser.inWaiting() >= 7:
                break
            
        # Get data from arduino  
        arduino_time = ser.readline()
        enc_F = ser.realine()
        enc_R = ser.readline()
        enc_L = ser.readline()
        v_F = ser.readline()
        v_R = ser.readline()
        v_L = ser.readline()
        
        # Get IMU data run through a moving average filter
        pi_time = time.time() - start_time
        [ax, ay, az, wx, wy, wz] = imu.getIMUFil()
        
        # Store measurement data to arrays
        times_arduino.append(arduino_time)
        encoders_front.append(enc_F)
        encoders_right.append(enc_R)
        encoders_left.append(enc_L)
        velocities_front.append(v_F)
        velocities_right.append(v_R)
        velocities_left.append(v_L)
        times_pi.append(pi_time)
        omegas.append(wz)
        
        # Controls stuff here
        # Once wheel velocities are decided, store them to the list V
        # V = [v_f,v_r,v_l]
        
        # Send motor velocities to arduino as pwm signals
        # velocities are sent as a pair (direction, speed)
        # direction of 0 => motors truns CW, direction of 1 => motors turns CCW
        # Check initial velocity directions and print to serial
        for velocity in V: 
            if velocity < 0: # Move counter-clockwise
                ser.write(chr(1))
                ser.write(chr(int(abs(velocity / COMMAND_TO_VELOCITY))))
            else: # Move clockwise
                ser.write(chr(0))
                ser.write(chr(int(velocity / COMMAND_TO_VELOCITY)))

        # Wait 100 ms
        time.sleep(0.01)

## Ctrl + c to stop robot
except KeyboardInterrupt:
        # Close serial connection
            ser.close()

## Write Data to text files
tafile = open(SAVE_FOLDER + "times_arduino.txt","w")
elfile = open(SAVE_FOLDER + "encoders_left.txt","w")
erfile = open(SAVE_FOLDER + "encoders_right.txt","w")
effile = open(SAVE_FOLDER + "encoders_front.txt","w")
vlfile = open(SAVE_FOLDER + "velocities_left.txt","w")
vrfile = open(SAVE_FOLDER + "velocities_right.txt","w")
vffile = open(SAVE_FOLDER + "velocities_front.txt","w")
tpfile = open(SAVE_FOLDER + "times_pi.txt","w")
wzfile = open(SAVE_FOLDER + "omegas.txt","w")

tafile.writelines(["%s\n" % item for item in times_arduino])
elfile.writelines(["%s\n" % item for item in encoders_left])
erfile.writelines(["%s\n" % item for item in encoders_right])
effile.writelines(["%s\n" % item for item in encoders_front])
vlfile.writelines(["%s\n" % item for item in velocities_left])
vrfile.writelines(["%s\n" % item for item in velocities_right])
vffile.writelines(["%s\n" % item for item in velociites_front])
tpfile.writelines(["%s\n" % item for item in times_pi])
wzfile.writelines(["%s\n" % item for item in omegas])

tafile.close()
elfile.close()
erfile.close()
effile.close()
vlfile.close()
vrfile.close()
vffile.close()
tpfile.close()
wzfile.close()