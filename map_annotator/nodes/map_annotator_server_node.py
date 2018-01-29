#!/usr/bin/env python


import fetch_api
import rospy
from map_annotator import MapAnnotator
from map_annotator.msg import PoseNames, UserAction
import atexit

QUEUE_SIZE = 10

def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass   

class MapAnnotatorServer(object):


    def __init__(self):
        self._pose_pub = rospy.Publisher('/map_annotator/pose_names', PoseNames, queue_size = QUEUE_SIZE, latch = True)
        self._map_annotator = MapAnnotator()
        self._map_annotator.pickle_load()


    # request is of type UserAction message
    # callback for handling messages from /map_annotator/user_actions topic
    def handle_request(self, request):
        print "handling request"
        if request.command == request.CREATE:
            self._map_annotator.save(request.name)
            self._republish_poses()
        elif request.command == request.DELETE:
            self._map_annotator.delete(request.name)
            self._republish_poses()
        elif request.command == request.GOTO:
            self._map_annotator.goto(request.name)
        elif request.command == request.RENAME:
            self._map_annotator.rename(request.name, request.updated_name)
        else:
            return -1
    
    # when pose list changes, republish to the /map_annotator/pose_names latched topic
    def _republish_poses(self):
        msg = PoseNames()
        msg.poses = self._map_annotator.list_poses()
        self._pose_pub.publish(msg)

    # persist pose list
    def server_exit(self):
        self._map_annotator.pickle_dump()
    
    

def main():
    
    rospy.init_node('map_annotator_server')
    wait_for_time()
    server = MapAnnotatorServer()
    atexit.register(server.server_exit)
    # server subscribes to user_actions topic and executes requests as they come in
    rospy.Subscriber("/map_annotator/user_actions", UserAction, server.handle_request)
    rospy.spin()



if __name__ == "__main__":
    main()