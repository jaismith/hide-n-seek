#!/usr/bin/env python

# Author: Jai Smith
# Date: Nov 11, 2021

# * imports
import math
import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from motion.msg import MotionStatus
from enum import Enum

# * constants
# topics
CMD_VEL_TOPIC = 'cmd_vel'
ODOM_TOPIC = 'odom'
STATUS_TOPIC = 'motion_status'

# frames
ODOM_FRAME = 'odom'
BASE_LINK_FRAME = 'base_link'

# runtime
FREQ = 100

# motion
LINEAR_VEL = 0.2 # m/s
ANGULAR_VEL = math.pi / 4 # rad/s

# * fsm
class fsm(Enum):
    """
    Finite State Machine for the robot.
    """
    CALC = 1 # calculate movement
    ROTATE = 2 # rotate to face the goal
    FORWARD = 3 # move forward
    WAIT = 6 # wait for next command

# * node
class Motion:
  """
  Motion node, moves the robot to provided coordinates in the map frame,
  based on odometry data.
  """

  def __init__(self, linear_vel=LINEAR_VEL, angular_vel=ANGULAR_VEL):
    """ Constructor """

    self._linear_vel = linear_vel
    self._angular_vel = angular_vel

    self._listener = tf.TransformListener()
    self._rate = rospy.Rate(FREQ)

    self._cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
    self._status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=10)

    self._state = fsm.WAIT
    self._queue = []

  def get_position(self):
    """ Get the current position of the robot, based on odometry """

    translation, rotation = None, None
    err_count = 0

    while translation is None or rotation is None:
      try:
        rospy.loginfo('Getting base link transform...')
        translation, rotation = self._listener.lookupTransform(
          ODOM_FRAME, BASE_LINK_FRAME, rospy.Time(0)
        )
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
        err_count += 1

        if err_count > 50:
          rospy.logerr('Error looking up transform: {}'.format(str(err)))
          raise err
        
        self._rate.sleep()
        continue

    x, y, _ = translation
    _, _, w = euler_from_quaternion(rotation)

    return x, y, w

  def move(self, linear_vel, angular_vel):
    """ Send a Twist command """

    twist_msg = Twist()

    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel

    self._cmd_pub.publish(twist_msg)

  def stop(self):
    """ Stop the robot """

    self.move(0, 0)

  def move_to(self, x, y, w):
    """ Move the robot to the given coordinates """

    rospy.loginfo('Received move command: {} {} {}'.format(x, y, w))
    self._queue.append((x, y, w))

  @staticmethod
  def optimize_theta(theta):
    """ Optimize theta to be between -pi and pi """

    while theta > math.pi:
      theta -= 2 * math.pi

    while theta < -math.pi:
      theta += 2 * math.pi

    return theta

  def run(self):
    """ Run the node """

    pending = []
    duration = None
    direction = None
    start_time = None

    # loop while alive
    while not rospy.is_shutdown():
      if self._state == fsm.CALC:
        # get goal
        x, y, w = self._queue.pop(0)

        # get pose
        x_cur, y_cur, w_cur = self.get_position()

        rospy.loginfo('Calculating move\n\tto {} {} {}\n\tfrom {} {} {}'
          .format(x, y, w, x_cur, y_cur, w_cur))

        # calculate offset
        dx = x - x_cur
        dy = y - y_cur

        rospy.loginfo('Calculated offset: {} {}'.format(dx, dy))

        yaw = math.atan2(dy, dx)
        theta = Motion.optimize_theta(yaw - w_cur)

        rospy.loginfo('Calculated angle: {}'.format(theta))

        dist = math.sqrt((dx ** 2) + (dy ** 2))
        theta2 = Motion.optimize_theta(w - yaw)

        # queue pending actions
        pending.extend([
          (
            fsm.ROTATE,
            rospy.Duration.from_sec(abs(theta) / self._angular_vel),
            1 if theta > 0 else -1
          ),
          (
            fsm.FORWARD,
            rospy.Duration.from_sec(dist / self._linear_vel),
            1 # always move forward (our sensor suite is biased in this direction)
          ),
          (
            fsm.ROTATE,
            rospy.Duration.from_sec(abs(theta2) / self._angular_vel),
            1 if theta2 > 0 else -1
          ),
        ])

        # move to wait state
        self._state = fsm.WAIT

      if self._state == fsm.ROTATE:
        rospy.loginfo('Rotating')

        # rotate
        self.move(0, self._angular_vel * direction)

        # if duration elapsed, move to wait state
        if rospy.get_rostime() - start_time > duration:
          self.stop()
          self._state = fsm.WAIT

      if self._state == fsm.FORWARD:
        rospy.loginfo('Moving forward')

        # move forward
        self.move(self._linear_vel * direction, 0)

        # if duration elapsed, move to wait state
        if rospy.get_rostime() - start_time > duration:
          self.stop()
          self._state = fsm.WAIT

      if self._state == fsm.WAIT:
        rospy.loginfo('Checking for pending actions...')

        # if pending action, execute
        if len(pending) > 0:
          [self._state, duration, direction] = pending.pop(0)
          start_time = rospy.get_rostime()
          rospy.loginfo('Executing pending action: {} {} {}'
            .format(self._state, duration, direction))

        # if command in queue, move to calc state
        elif len(self._queue) > 0:
          self._state = fsm.CALC

        # if no pending actions, stop
        if self._state == fsm.WAIT:
          self.stop()

      self._rate.sleep()

if __name__ == '__main__':
  # initialize node
  rospy.init_node('motion_node')

  rospy.sleep(5)

  # create motion node
  motion = Motion()

  # stop on shut down
  rospy.on_shutdown(motion.stop)

  # queue commands to move in a square
  motion.move_to(1, 0, math.pi / 2)
  motion.move_to(1, 1, math.pi)
  motion.move_to(0, 1, (math.pi * 3) / 2)
  motion.move_to(0, 0, 0)

  # run node
  motion.run()
