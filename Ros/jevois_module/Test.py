import serial

import signal
import sys
import time

import CameraPublisher
import DiceCounter
import DemoArUco
import FaceTracking
import QRcode
import rospy
from geometry_msgs.msg import Point

stop = False


pub_xyv = rospy.Publisher('/jevois/attention/xyv', Point, queue_size=10)
rospy.init_node('jevois_talker', anonymous=True)

def mean(l):
    out = sum(l) / float(len(l))
    return out

def publish(msg):
    if not rospy.is_shutdown():
        vals = msg
        if len(vals) == 11:
            t, v, n, x0, y0, x1, y1, x2, y2, x3, y3 = vals
            v = int(v[1:])
            points = map(int, vals[3:])
            x = mean(points[0::2])
            y = mean(points[1::2])
            pub_xyv.publish(x, y, v)

def signal_handler(signal, frame):
    print('^C')
    global stop
    stop = True
    mode.close()
    sys.exit()


def on_error(err):
    global stop
    stop = True
    print str(err)


if __name__ == "__main__":
    serdev = '/dev/ttyACM0'
    signal.signal(signal.SIGINT, signal_handler)
    always_refresh = CameraPublisher.ALWAYS_REFRESH
    refresh_on_new = CameraPublisher.REFRESH_ON_NEW
    try:
        freq_callback = 0.05
        baudrate = 115200
        
        # mode_class = DiceCounter.DiceCounter
        # mode_class = QRcode.QRcode
        # mode_class = FaceTracking.FaceTracking
        mode_class = DemoArUco.DemoArUco
        
        mode = mode_class(serdev, callback=publish,
                          refresh_callback=refresh_on_new,
                          freq_callback=freq_callback,
                          baudrate=baudrate)
        mode.set_on_error(on_error)
        mode.start()
        t_end = time.time() + 10
        # while time.time() < t_end:
        while not stop:
            time.sleep(0.5)
            # print "get_value : " + str(mode.get_value())
    except serial.serialutil.SerialException as e:
        print str(e)
