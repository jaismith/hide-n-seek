#!/usr/bin/env python

# Author: Jai Smith
# Date: Nov 11, 2021

# * imports
import math
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Path
from hide_n_seek.msg import MotionStatus
from enum import Enum

# * constants
# topics
CMD_VEL_TOPIC = 'cmd_vel'
STATUS_TOPIC = 'motion_status'
PATH_TOPIC = 'navigation_path'

# frames
FIXED_FRAME = 'odom'
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
    CALC = 'CALC' # calculate movement
    ROTATE = 'ROTATE' # rotate to face the goal
    FORWARD = 'FORWARD' # move forward
    WAIT = 'WAIT' # wait for next command

# * node
class Motion:
  """
  Motion node, moves the robot to provided coordinates in the map frame,
  based on odometry data.
  """

  def __init__(self, linear_vel=LINEAR_VEL, angular_vel=ANGULAR_VEL,
      cmd_vel_topic=CMD_VEL_TOPIC, status_topic=STATUS_TOPIC,
      path_topic=PATH_TOPIC, lookup_should_timeout=True, freq=FREQ):
    """ Constructor """

    self._linear_vel = linear_vel
    self._angular_vel = angular_vel

    self._listener = tf.TransformListener()
    self._rate = rospy.Rate(freq)

    self._cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    self._status_pub = rospy.Publisher(status_topic, MotionStatus,
      queue_size=10)

    self._path_sub = rospy.Subscriber(path_topic, Path, self._path_callback)

    self._lookup_should_timeout = lookup_should_timeout
    self._state = fsm.WAIT
    self._queue = []
    self._goal = Pose()

  def _path_callback(self, msg):
    """ Callback for the path topic, this overrides the current queue """

    rospy.loginfo('Received new path of length {}'.format(len(msg.poses)))
    self._queue = list(pose_stamped.pose for pose_stamped in msg.poses)

  def get_position(self):
    """ Get the current position of the robot, based on odometry """

    translation, rotation = None, None
    err_count = 0

    while translation is None or rotation is None:
      try:
        rospy.loginfo('Getting base link transform...')
        translation, rotation = self._listener.lookupTransform(
          FIXED_FRAME, BASE_LINK_FRAME, rospy.Time(0)
        )
      except (tf.LookupException, tf.ConnectivityException, 
          tf.ExtrapolationException) as err:
        err_count += 1

        if err_count > 50 and self.lookup_should_timeout:
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

  def move_to(self, pose):
    """ Add a waypoint to the queue """

    rospy.loginfo('Received new waypoint:\n{}'.format(str(pose)))
    self._queue.append(pose)

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

    seq = 0
    pending = []
    duration = None
    direction = None
    start_time = None

    # loop while alive
    while not rospy.is_shutdown():
      if self._state == fsm.CALC:
        # get goal
        self._goal = self._queue.pop(0)
        x, y = self._goal.position.x, self._goal.position.y
        _, _, w = euler_from_quaternion([self._goal.orientation.x,
          self._goal.orientation.y, self._goal.orientation.z,
          self._goal.orientation.w])

        # get robot pose
        x_cur, y_cur, w_cur = self.get_position()

        rospy.loginfo('Calculating move\n\tto {} {} {}\n\tfrom {} {} {}'
          .format(x, y, w, x_cur, y_cur, w_cur))

        # calculate offsets
        dx = x - x_cur
        dy = y - y_cur

        yaw = math.atan2(dy, dx)
        theta = Motion.optimize_theta(yaw - w_cur)

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
            1 # always move forward
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
        # rotate
        self.move(0, self._angular_vel * direction)

        # if duration elapsed, move to wait state
        if rospy.get_rostime() - start_time > duration:
          self.stop()
          self._state = fsm.WAIT

      if self._state == fsm.FORWARD:
        # move forward
        self.move(self._linear_vel * direction, 0)

        # if duration elapsed, move to wait state
        if rospy.get_rostime() - start_time > duration:
          self.stop()
          self._state = fsm.WAIT

      if self._state == fsm.WAIT:
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

      # publish status
      motion_status_msg = MotionStatus()
      motion_status_msg.header.seq = seq
      motion_status_msg.header.stamp = rospy.Time.now()
      motion_status_msg.header.frame_id = FIXED_FRAME
      motion_status_msg.goal = self._goal
      motion_status_msg.state = self._state.value
      self._status_pub.publish(motion_status_msg)

      seq += 1
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
  def pose_from_xyw(x, y, w):
    pose_stamped = PoseStamped()
    pose_stamped.header.seq = 0
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = FIXED_FRAME
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    orientation = quaternion_from_euler(0, 0, w)
    pose_stamped.pose.orientation.x = orientation[0]
    pose_stamped.pose.orientation.y = orientation[1]
    pose_stamped.pose.orientation.z = orientation[2]
    pose_stamped.pose.orientation.w = orientation[3]
    return pose_stamped

  path = Path()
  path.header.seq = 0
  path.header.stamp = rospy.Time.now()
  path.header.frame_id = FIXED_FRAME
  path.poses.append(pose_from_xyw(1, 0, math.pi / 2))
  path.poses.append(pose_from_xyw(1, 1, math.pi))
  path.poses.append(pose_from_xyw(0, 1, (math.pi * 3) / 2))
  path.poses.append(pose_from_xyw(0, 0, 0))

  # publish path
  path_pub = rospy.Publisher('navigation_path', Path, queue_size=10)
  while len(motion._queue) == 0:
    path_pub.publish(path)
    rospy.sleep(0.1)

  # run node
  motion.run()
