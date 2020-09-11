 # import the necessary packages
from __future__ import print_function
import imutils
import cv2
 
 # load the Tetris block image, convert it to grayscale, and threshold the image
print("OpenCV Version: {}".format(cv2.__version__))
image = cv2.imread("tetris_blocks.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
thresh = cv2.threshold(gray, 225, 255, cv2.THRESH_BINARY_INV)[1]
 
 # check to see if we are using OpenCV 2.X or OpenCV 4
if imutils.is_cv2() or imutils.is_cv4():
	(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	 cv2.CHAIN_APPROX_SIMPLE)
 
#~ # check to see if we are using OpenCV 3
elif imutils.is_cv3():
	(_, cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	 cv2.CHAIN_APPROX_SIMPLE)
 
# draw the contours on the image
	cv2.drawContours(image, cnts, -1, (240, 0, 159), 3)
	cv2.imshow("Image", image)
	cv2.waitKey(0)

# ~ def open_cam_rtsp(uri, width, height, latency):
    # ~ gst_str = ("rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! "
               # ~ "nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
               # ~ "videoconvert ! appsink").format(uri, latency, width, height)
    # ~ return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

# ~ def open_cam_usb(dev, width, height):
    # ~ # We want to set width and height here, otherwise we could just do:
    # ~ #     return cv2.VideoCapture(dev)
    # ~ gst_str = ("v4l2src device=/dev/video{} ! "
               # ~ "video/x-raw, width=(int){}, height=(int){}, format=(string)RGB ! "
               # ~ "videoconvert ! appsink").format(dev, width, height)
    # ~ return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

# ~ def open_cam_onboard(width, height):
    # ~ # On versions of L4T previous to L4T 28.1, flip-method=2
    # ~ # Use Jetson onboard camera
    # ~ gst_str = ("nvcamerasrc ! "
               # ~ "video/x-raw(memory:NVMM), width=(int)2592, height=(int)1458, format=(string)I420, framerate=(fraction)30/1 ! "
               # ~ "nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
               # ~ "videoconvert ! appsink").format(width, height)
    # ~ return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
