from flask import Flask 
import rospy
from std_msgs.msg import Int64
app = Flask(__name__)

@app.route('/call_robot/<int:table>')
def test(table):
    #return 'Sending robot to table %d'% table
    pub = rospy.Publisher('qr_code_interface', Int64, queue_size=10)
    rospy.init_node('flask_web_server')
    pub.publish(Int64(table))
    rospy.loginfo('published table number: ' + str(table))
    return 'OK'
               
