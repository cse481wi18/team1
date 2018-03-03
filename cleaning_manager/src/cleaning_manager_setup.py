#! /usr/bin/env python
import rospy
from std_msgs.msg import Int64
import time


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class CleaningManager(object):
    def __init__(self, segmenter):
        self._requests = []
        self._cleaning = False
        self._segmenter = segmenter
    
    def callback(self, data):
        rospy.loginfo('Received data: %d', data.data)
        self._requests.append(data.data)
        if not self._cleaning:
            self._cleaning = True
            print "starting to clean"
        else:
            print "Already cleaning"

    def start(self):
        rospy.Subscriber('qr_code_interface', Int64, callback=self.callback)

def main():
    rospy.init_node('cleaning_manager')
    wait_for_time()

    segmenter = segmenter(table_pub, marker_pub, object_pub, coord_pub)

    manager = CleaningManager(segmenter)
    manager.start()

    rospy.spin()


if __name__=='__main__':
    main()