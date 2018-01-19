#! /usr/bin/env python                                                                                 
                                                                                                       
import fetch_api                                                                                       
from joint_state_reader import JointStateReader
import rospy                  
                              
                       
def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass                                                                                           
                                                                                                       
                                                                                                       
def main():                              
    print "starting main"                                                              
    rospy.init_node('joint_reader_demo')                                                               
    wait_for_time()                                                                                    
    print "done waiting"
    argv = rospy.myargv()  
    print "getting argv"                                                                            
    reader = JointStateReader()
    print "made reader"
    rospy.sleep(0.5)
    print "getting names"
    names = fetch_api.ArmJoints.names()
    print "getting arm values"
    arm_vals = reader.get_joints(names)
    print "pre loop"
    for k, v in zip(names, arm_vals):
        print "looping"
        print '{}\t{}'.format(k, v)
    print "post loop"                  
                      
if __name__ == '__main__':
    main()