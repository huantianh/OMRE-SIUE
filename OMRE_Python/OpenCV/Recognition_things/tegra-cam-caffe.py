# --------------------------------------------------------
# Camera Caffe sample code for Tegra X2/X1
#
# This program captures and displays video from IP CAM,
# USB webcam, or the Tegra onboard camera, and do real-time
# image classification (inference) with Caffe. Refer to the
# following blog post for how to set up and run the code:
#
#   https://jkjung-avt.github.io/tx2-camera-caffe/
#
# Written by JK Jung <jkjung13@gmail.com>
# --------------------------------------------------------

import os
import sys
import argparse
import cv2
import numpy as np

CAFFE_ROOT = "/home/nvidia/caffe/"
sys.path.insert(0, CAFFE_ROOT + "python")
import caffe
from caffe.proto import caffe_pb2

DEFAULT_PROTOTXT = CAFFE_ROOT + "models/bvlc_reference_caffenet/deploy.prototxt"
DEFAULT_MODEL    = CAFFE_ROOT + "models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel"
DEFAULT_LABELS   = CAFFE_ROOT + "data/ilsvrc12/synset_words.txt"
DEFAULT_MEAN     = CAFFE_ROOT + "data/ilsvrc12/imagenet_mean.binaryproto"

windowName = "CameraCaffeDemo"
helpText = "'Esc' to Quit, 'H' to Toggle Help, 'F' to Toggle Fullscreen"

def parse_args():
    """
    Parse input arguments
    """
    parser = argparse.ArgumentParser(description="Capture and display live camera video, and do real-time image classification with Caffe on Jetson TX2/TX1")
    parser.add_argument("--rtsp", dest="use_rtsp",
                        help="use IP CAM (remember to also set --uri)",
                        action="store_true")
    parser.add_argument("--uri", dest="rtsp_uri",
                        help="RTSP URI string, e.g. rtsp://192.168.1.64:554",
                        default=None, type=str)
    parser.add_argument("--latency", dest="rtsp_latency",
                        help="latency in ms for RTSP [200]",
                        default=200, type=int)
    parser.add_argument("--usb", dest="use_usb",
                        help="use USB webcam (remember to also set --vid)",
                        action="store_true")
    parser.add_argument("--vid", dest="video_dev",
                        help="video device # of USB webcam (/dev/video?) [1]",
                        default=1, type=int)
    parser.add_argument("--width", dest="image_width",
                        help="image width [640]",
                        default=640, type=int)
    parser.add_argument("--height", dest="image_height",
                        help="image height [480]",
                        default=480, type=int)
    parser.add_argument("--cpu", dest="cpu_mode",
                        help="use CPU mode for Caffe (GPU mode is used by default)",
                        action="store_true")
    parser.add_argument("--crop", dest="crop_center",
                        help="crop the square at center of image for Caffe inferencing [False]",
                        action="store_true")
    parser.add_argument("--prototxt", dest="caffe_prototxt",
                        help="[{}]".format(DEFAULT_PROTOTXT),
                        default=DEFAULT_PROTOTXT, type=str)
    parser.add_argument("--model", dest="caffe_model",
                        help="[{}]".format(DEFAULT_MODEL),
                        default=DEFAULT_MODEL, type=str)
    parser.add_argument("--labels", dest="caffe_labels",
                        help="[{}]".format(DEFAULT_LABELS),
                        default=DEFAULT_LABELS, type=str)
    parser.add_argument("--mean", dest="caffe_mean",
                        help="[{}]".format(DEFAULT_MEAN),
                        default=DEFAULT_MEAN, type=str)
    parser.add_argument("--output", dest="caffe_output",
                        help='name of Caffe output blob [prob]',
                        default="prob", type=str)
    args = parser.parse_args()
    return args

def get_caffe_mean(filename):
    mean_blob = caffe_pb2.BlobProto()
    with open(filename, "rb") as f:
        mean_blob.ParseFromString(f.read())
    mean_array = np.asarray(mean_blob.data, dtype=np.float32).reshape(
                 (mean_blob.channels, mean_blob.height, mean_blob.width))
    return mean_array.mean(1).mean(1)

def open_cam_rtsp(uri, width, height, latency):
    gst_str = ("rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! "
               "nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
               "videoconvert ! appsink").format(uri, latency, width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

def open_cam_usb(dev, width, height):
    # We want to set width and height here, otherwise we could just do:
    #     return cv2.VideoCapture(dev)
    gst_str = ("v4l2src device=/dev/video{} ! "
               "video/x-raw, width=(int){}, height=(int){} ! "
               "videoconvert ! appsink").format(dev, width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

def open_cam_onboard(width, height):
    # On versions of L4T prior to 28.1, add 'flip-method=2' into gst_str
    # Use Jetson onboard camera
    gst_str = ("nvcamerasrc ! "
               "video/x-raw(memory:NVMM), width=(int)2592, height=(int)1458, format=(string)I420, framerate=(fraction)30/1 ! "
               "nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
               "videoconvert ! appsink").format(width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

def open_window(windowName, width, height):
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowName, width, height)
    cv2.moveWindow(windowName, 0, 0)
    cv2.setWindowTitle(windowName, "Camera Caffe Classification Demo for Jetson TX2/TX1")

def show_top_preds(img, top_probs, top_labels):
    x = 10
    y = 40
    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(top_probs)):
        pred = "{:.4f} {:20s}".format(top_probs[i], top_labels[i])
        #cv2.putText(img, pred,
        #            (x+1,y), font, 1.0, (32,32,32), 4, cv2.LINE_AA)
        cv2.putText(img, pred,
                    (x,y), font, 1.0, (0,0,240), 1, cv2.LINE_AA)
        y += 20

def read_cam_and_classify(windowName, cap, net, transformer, labels, caffe_output, crop):
    showHelp = True
    showFullScreen = False
    font = cv2.FONT_HERSHEY_PLAIN
    while True:
        if cv2.getWindowProperty(windowName, 0) < 0: # Check to see if the user closed the window
            # This will fail if the user closed the window; Nasties get printed to the console
            break;
        ret_val, img = cap.read()

        if crop:
            height, width, channels = img.shape
            if height < width:
                img_crop = img[:, ((width-height)//2):((width+height)//2), :]
            else:
                img_crop = img[((height-width)//2):((height+width)//2), :, :]
        else:
            img_crop = img;
        # inferencing the image
        net.blobs["data"].data[...] = transformer.preprocess("data", img_crop)
        output = net.forward()
        output_prob = output[caffe_output][0]  # output["prob"][0]
        top_inds = output_prob.argsort()[::-1][:3]
        top_probs = output_prob[top_inds]
        top_labels = labels[top_inds]
        show_top_preds(img, top_probs, top_labels)

        if showHelp == True:
            cv2.putText(img, helpText, (11,20), font, 1.0, (32,32,32), 4, cv2.LINE_AA)
            cv2.putText(img, helpText, (10,20), font, 1.0, (240,240,240), 1, cv2.LINE_AA)
        cv2.imshow(windowName, img)
        key = cv2.waitKey(10)
        if key == 27: # ESC key: quit program
            break
        elif key == ord('H') or key == ord('h'): # toggle help message
            showHelp = not showHelp
        elif key == ord('F') or key == ord('f'): # toggle fullscreen
            showFullScreen = not showFullScreen
            if showFullScreen == True: 
                cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            else:
                cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL) 

if __name__ == "__main__":
    args = parse_args()
    print("Called with args:")
    print(args)
    print("OpenCV version: {}".format(cv2.__version__))

    if not os.path.isfile(args.caffe_prototxt):
        sys.exit("File not found: {}".format(args.caffe_prototxt))
    if not os.path.isfile(args.caffe_model):
        sys.exit("File not found: {}".format(args.caffe_model))
    if not os.path.isfile(args.caffe_labels):
        sys.exit("File not found: {}".format(args.caffe_labels))
    if not os.path.isfile(args.caffe_mean):
        sys.exit("File not found: {}".format(args.caffe_mean))

    # initialize Caffe
    if args.cpu_mode:
        print("Running Caffe in CPU mode")
        caffe.set_mode_cpu()
    else:
        print("Running Caffe in GPU mode")
        caffe.set_device(0)
        caffe.set_mode_gpu()
    net = caffe.Net(args.caffe_prototxt, args.caffe_model, caffe.TEST)
    mu = get_caffe_mean(args.caffe_mean)
    print("Mean-subtracted values:", zip('BGR', mu))
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
    transformer.set_transpose("data", (2,0,1))
    transformer.set_mean("data", mu)
    # no need to to swap color channels since captured images are already BGR
    labels = np.loadtxt(args.caffe_labels, str, delimiter='\t')

    # initialize camera
    if args.use_rtsp:
        cap = open_cam_rtsp(args.rtsp_uri, args.image_width, args.image_height, args.rtsp_latency)
    elif args.use_usb:
        cap = open_cam_usb(args.video_dev, args.image_width, args.image_height)
    else: # by default, use the Jetson onboard camera
        cap = open_cam_onboard(args.image_width, args.image_height)

    if not cap.isOpened():
        sys.exit("Failed to open camera!")

    # start capturing live video and do inference
    open_window(windowName, args.image_width, args.image_height)
    read_cam_and_classify(windowName, cap, net, transformer, labels,
                          args.caffe_output, args.crop_center)

    cap.release()
    cv2.destroyAllWindows()