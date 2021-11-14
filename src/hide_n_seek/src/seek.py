from threading import Thread
import math
import rospy
import sys
import os
from geometry_msgs.msg import Vector3Stamped

sys_paths = ['../']
for p in sys_paths:
  p = os.path.abspath(p)
  if p not in sys.path:
    sys.path.append(p)

from nodes.motion_node import Motion
from nodes.mapper_node import Mapper
from nodes.navigation_node import Navigation
# from nodes.angle_pub_node import AnglePublisher

rospy.init_node('seek')

rospy.sleep(5)

# * HUSARION
# DEFAULT_SCAN_TOPIC = 'scan'
# LASER_FRAME = 'laser'
# SCAN_ANGLE_OFFSET = math.pi

# * SIMULATOR
DEFAULT_SCAN_TOPIC = 'base_scan'
LASER_FRAME = 'base_laser_link'
SCAN_ANGLE_OFFSET = 0

motion = Motion(linear_vel=0.1, angular_vel=math.pi / 8)
mapper = Mapper(scan_topic=DEFAULT_SCAN_TOPIC,
  laser_frame=LASER_FRAME,
  scan_angle_offset=SCAN_ANGLE_OFFSET)
navigation = Navigation()
# angle_pub = AnglePublisher()

def shutdown():
  motion.stop()
  mapper.save_map()

rospy.on_shutdown(shutdown)

Thread(target=mapper.spin).start()
Thread(target=motion.run).start()
Thread(target=navigation.spin).start()
# Thread(target=angle_pub.pub).start()

# ! DEBUG

rospy.sleep(30)

# angle_pub = rospy.Publisher('object_angle', Vector3Stamped, queue_size=1)

msg = Vector3Stamped()
msg.header.frame_id = 'odom'
msg.header.stamp = rospy.Time(5)
msg.vector.x = 0.1

