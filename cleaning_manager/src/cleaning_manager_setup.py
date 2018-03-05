#! /usr/bin/env python
import rospy

from std_msgs.msg import Int64
import navigator import Navigator
import time


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class CleaningManager(object):
    def __init__(self, segmenter, navigator):
        self._requests = []
        self._cleaning = False
        self._segmenter = segmenter
        self._navigator = navigator
    

    def qr_callback(self, data):
        rospy.loginfo('Received data: %d', data.data)
        self._requests.append(data.data)
        if not self._cleaning:
            self._cleaning = True
            print "starting to clean"
            self._start_cleaning_sequence()

        else:
            print "Already cleaning"


    def start(self):
        rospy.Subscriber('qr_code_interface', Int64, callback=self.qr_callback)
    

    def start_cleaning_sequence(self):
        result = self._navigator.goto(self._requests.pop(0))
        if result == 1:
            print "Navigating"
                # raise torso
                # START SWEEP SEQUENCE
        else: 
            print "Failed navigation"
                # RETRY? 

        if len(self._requests) > 0:
            start_cleaning_sequence()
        else:
            print "No more requests"
            # return to station



def main():
    rospy.init_node('cleaning_manager')
    wait_for_time()

    navigator = Navigator(15) # 15 second timeout for each pose

    manager = CleaningManager(segmenter)
    manager.start()

    rospy.spin()


if __name__=='__main__':
    main()