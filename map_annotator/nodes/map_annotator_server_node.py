#!/usr/bin/env python


import fetch_api
import rospy
from map_annotator import MapAnnotator
from map_annotator.msg import PoseNames, UserAction
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

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
        self._republish_poses()


    # request is of type UserAction message
    # callback for handling messages from /map_annotator/user_actions topic
    def handle_request(self, request):
        print "handling request"
        print request.command
        if request.command == "create":
            self._map_annotator.save(request.name)
            self._republish_poses()
        elif request.command == "delete":
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

    
def handle_viz_input(input):
    print "call back"
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')
    

def main():
    
    rospy.init_node('map_annotator_server')
    wait_for_time()
    server = MapAnnotatorServer()
    interactiveServer = InteractiveMarkerServer("interactive_marker_proxy_basic_controls")

    # creates abd inits an interactiveMarker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "my_marker"
    int_marker.description = "Simple Click Control"
    int_marker.pose.position.x = 1
    int_marker.pose.orientation.w = 1

    # creates a teal cube Marker for the interactiveMarker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    #creates an interactioveMarkerControl and adds the marker to it and adds the control to the InteractiveMarker
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    interactiveServer.insert(int_marker, handle_viz_input)
    interactiveServer.applyChanges()

    # server subscribes to user_actions topic and executes requests as they come in
    rospy.Subscriber("/map_annotator/user_actions", UserAction, server.handle_request)
    rospy.on_shutdown(server.server_exit)
    rospy.spin()



if __name__ == "__main__":
    main()