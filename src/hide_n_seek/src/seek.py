from threading import Thread
import math
import rospy
import sys

sys_paths = ['../']
for p in sys_paths:
  p = os.path.abspath(p)
  if p not in sys.path:
      sys.path.append(p)

from nodes.motion_node import Motion
from nodes.mapper_node import Mapper
from nodes.navigation_node import Navigation

rospy.init_node('seek')

rospy.sleep(5)

motion = Motion(linear_vel=0.1, angular_vel=math.pi / 8)
mapper = Mapper()
navigation = Navigation()

def shutdown():
  motion.stop()
  mapper.save_map()

rospy.on_shutdown(shutdown)

Thread(target=mapper.spin).start()
Thread(target=motion.run).start()
Thread(target=navigation.spin).start()
