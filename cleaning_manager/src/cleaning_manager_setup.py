#! /usr/bin/env python
import rospy

from std_msgs.msg import Int64
from navigator import Navigator
import time
from map_annotator import MapAnnotator
from perception import ProgramManager
import fetch_api

PERTURBANCE_DISTANCE = 0.07
FORWARD_DISTANCE = 0.33
DEMO = False

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class CleaningManager(object):
    def __init__(self, map_annotator):
        self._requests = []
        self._cleaning = False
        self._map_annotator = map_annotator
        self._table = None
        self._home = "home"
        
        self._bucket_release_sequence = None
        self._attachment_grab_sequence = None
        self._cleaning_sequence_1 = None
        self._cleaning_sequence_2 = None 
        self._attachment_release_sequence = None 
        self._bucket_grab = None

        self._program_manager = ProgramManager()

        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self._torso = fetch_api.Torso()
        self._base = fetch_api.Base()

    def qr_callback(self, data):
        if DEMO:
            rospy.loginfo('Received data: %d', data.data)
            self._requests.append(data.data)
        else:
            rospy.loginfo('Received data: %d', data)
            self._requests.append(data)

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
        if not DEMO:
            #  force call back
            self.qr_callback(2)
    
    # assumes starts at home
    def start_cleaning_sequence(self):
        table = str(self._requests.pop(0))
        print "starting table " + table

        # arm travel position
        self._program_manager.run_program("arm_travel")

        # go to table
        result = 0
        for i in range(3):
            result = self._map_annotator.goto('table')
            if result:
                break
            else: 
                self._base.go_forward(PERTURBANCE_DISTANCE)

        if not result:
            print "failed to go to table"
        
        print "Starting cleaning sequence"
        # set torso 
        self._torso.set_height(0.32)

        print "Opening gripper"
        # open gripper
        self._gripper.open()

        # grab attachment 
        print "Grabbing attachment"
        self._program_manager.run_program("attachment_grab_v3")

        # intermediary pose after grab
        self._program_manager.run_program("intermediary_after_grab")

        # move to forward position
        self._base.go_forward(FORWARD_DISTANCE)
        rospy.sleep(1)

        print "Sweeping sequence"

        # broom sequence
        self._program_manager.run_program("broom_base_link_1")
        self._program_manager.run_program("broom_base_link_2")

        print "Swiffering sequence"
        # swiffer sequence 
        self._program_manager.run_program("swiffer_base_link_1")
        self._program_manager.run_program("swiffer_base_link_2")

        print "Releasing attachment"
        # intermediary before attachment release 
        self._program_manager.run_program("intermediary_before_release")

        # grab attachment 
        self._program_manager.run_program("attachment_release_v2")

        print "Moving arm to travel position"
        # arm travel position
        self._program_manager.run_program("arm_travel")

        print "Finished and going home"
        # go home
        self._map_annotator.goto('home')



        # if len(self._requests) > 0:
        #     self.start_cleaning_sequence()
        # else:
        #     self._cleaning = False
        #     print "No more requests"
        #     result = self._map_annotator.goto('home')
        #     # return to station


       

def main():
    rospy.init_node('cleaning_manager')
    wait_for_time()

    map_annotator = None 
    if DEMO:
        map_annotator = MapAnnotator("/home/team1/map_poses/demo_poses.p")
    else: 
        map_annotator = MapAnnotator("/home/team1/map_poses/sim_poses.p")
    map_annotator.pickle_load()

    manager = CleaningManager(map_annotator)
    manager.start()

    rospy.spin()


if __name__=='__main__':
    main()