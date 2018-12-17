import cPickle
import rospy
from tf import TransformListener, TransformBroadcaster, transformations
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

count, gmsg = 0, dict()
def scan(msg):
    global count, gmsg
    # gmsg = msg
    print msg.ranges
    gmsg['ranges'] = list(msg.ranges)
    gmsg['intensities'] = list(msg.intensities)
    count = count + 1


rospy.init_node('landmark_localization')
scan = rospy.Subscriber('scan', LaserScan, scan, queue_size=10)
listener = TransformListener()
broadcaster = TransformBroadcaster()
stop_amcl_tf = rospy.ServiceProxy('stop_amcl_tf', Empty)

r = rospy.Rate(100)
while not rospy.is_shutdown():
    if count > 20:
        myFile = file('dimg2laser.data', 'a')
        cPickle.dump(gmsg, myFile)
        myFile.close()
        break
    r.sleep()
