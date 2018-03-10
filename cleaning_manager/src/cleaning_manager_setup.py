#! /usr/bin/env python
import rospy

from std_msgs.msg import Int64
from navigator import Navigator
import time


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class CleaningManager(object):
    def __init__(self, navigator):
        self._requests = [1]
        self._cleaning = False
        self._navigator = navigator
        self._table = None
        
        self._bucket_release_sequence = None
        self._attachment_grab_sequence = None
        self._cleaning_sequence_1 = None
        self._cleaning_sequence_2 = None 
        self._attachment_release_sequence = None 
        self._bucket_grab = None
        self._load_files()

        self._program_manager = ProgramManager()
    

    def qr_callback(self, data):
        rospy.loginfo('Received data: %d', data.data)
        self._requests.append(data.data)

        
        # rospy.loginfo('Received data: %d', data)
        # self._requests.append(data)
        if not self._cleaning:
            self._cleaning = True
            print "starting to clean"
            self.start_cleaning_sequence()
        else:
            print "Already cleaning"
        return 'OK'


    def start(self):
        print "Starting robot"
        rospy.Subscriber('qr_code_interface', Int64, callback=self.qr_callback)
        # self.qr_callback(1)
    

    def start_cleaning_sequence(self):
        table = str(self._requests.pop(0))
        print "starting table " + table

        self._program_manager.run_program("torso_nav.p")
        result = self._navigator.goto(table)

        if result == 1:
            print "Navigating"
                self._program_manager.run_program("torso_bucket.p")
                self._program_manager.run_program("bucket_release.p")
                self._program_manager.run_program("attachment_grab.p")
                self._program_manager.run_program("cleaning_sequence_1.p")
                self._program_manager.run_program("cleaning_sequence_2.p")
                self._program_manager.run_program("attachment_release.p")
                self._program_manager.run_program("bucket_grab.p")
        else: 
            print "Failed navigation"
                # RETRY? 


        if len(self._requests) > 0:
            self.start_cleaning_sequence()
        else:
            self._cleaning = False
            print "No more requests"
            # return to station


       

def main():
    rospy.init_node('cleaning_manager')
    wait_for_time()

    navigator = Navigator(15) # 15 second timeout for each pose

    manager = CleaningManager(navigator)
    manager.start()

    rospy.spin()


if __name__=='__main__':
    main()