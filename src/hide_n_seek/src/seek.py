from threading import Thread
import rospy

from motion import Motion
from mapper import Mapper

rospy.init_node('seek')

rospy.sleep(5)

motion = Motion()
mapper = Mapper()

def shutdown():
  motion.stop()
  mapper.save_map()

rospy.on_shutdown(shutdown)

Thread(target=mapper.spin).start()
Thread(target=motion.run).start()
