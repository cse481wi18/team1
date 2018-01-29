#!/usr/bin/env python


import fetch_api
import rospy
from map_annotator import MapAnnotator
from map_annotator.msg import PoseNames, UserAction
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import copy
import itertools
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
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

        # interactive marker server
        self._interactiveServer = InteractiveMarkerServer("map_annotator/map_poses")
       
        for name, pose in self._map_annotator.get_all_poses().iteritems():
            self._createMarker(pose, name)

    # request is of type UserAction message
    # callback for handling messages from /map_annotator/user_actions topic
    def handle_request(self, request):
        print "handling request"
        print request.command
        if request.command == request.CREATE:
            self._map_annotator.save(request.name)
            self._republish_poses()
            pose = self._map_annotator.get_pose_from_name(request.name)
            self._createMarker(pose, request.name)
        elif request.command == request.DELETE:
            val = self._map_annotator.delete(request.name)
            self._republish_poses()
            if val == 0: 
                self._deleteMarker(request.name)
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

    def _createMarker(self, pose, name):
            # creates and inits an interactiveMarker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.pose = pose
        int_marker.pose.position.z = 0.2
        int_marker.description = name

        # creates an arrow marker 
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        # text
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = name
        text_marker.pose.position.z = text_marker.pose.position.z + 0.4
        text_marker.scale=Vector3(0.3, 0.3, 0.3)
        text_marker.color = ColorRGBA(0.0, 0.0, 0.0, 0.8)

        text_control = InteractiveMarkerControl()
        text_control.always_visible = True
        text_control.markers.append(text_marker)

        int_marker.controls.append(copy.deepcopy(text_control))


        # rotation control
        rotation_control = InteractiveMarkerControl()
        rotation_control.orientation.w = 1
        rotation_control.orientation.x = 0
        rotation_control.orientation.y = 1
        rotation_control.orientation.z = 0
        rotation_control.name = "rotate"
        rotation_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotation_control.always_visible = True

        int_marker.controls.append(copy.deepcopy(rotation_control))

        # move control
        move_control = InteractiveMarkerControl()
        move_control.orientation.w = 1
        move_control.orientation.x = 0
        move_control.orientation.y = 1
        move_control.orientation.z = 0
        move_control.name = "move"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.always_visible = True

        # only append arrow to move control and not rotation control
        move_control.markers.append(copy.deepcopy(arrow_marker))
        int_marker.controls.append(copy.deepcopy(move_control))

        self._interactiveServer.insert(int_marker, self._processFeedback)
        self._interactiveServer.setCallback(int_marker.name, self._markerCallback, InteractiveMarkerFeedback.POSE_UPDATE)
        self._interactiveServer.applyChanges()
    
    def _deleteMarker(self, name):
        self._interactiveServer.erase(name)
        self._interactiveServer.applyChanges()

    def _markerCallback(self, feedback):
        print "call back"
        pose = feedback.pose
        val = self._map_annotator.change_pose(feedback.marker_name, pose)
        if val == 0:
            self._republish_poses()

    def _processFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo("Pose changed")
        self._interactiveServer.applyChanges()

def main():
    rospy.init_node('map_annotator_server')
    wait_for_time()
    server = MapAnnotatorServer()

    # server subscribes to user_actions topic and executes requests as they come in
    rospy.Subscriber("/map_annotator/user_actions", UserAction, server.handle_request)
    rospy.on_shutdown(server.server_exit)
    rospy.spin()



if __name__ == "__main__":
    main()