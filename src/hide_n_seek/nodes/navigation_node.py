#!/usr/bin/env python

# Author: Sihao Huang
# Date: 11/11/2021
# The navigation node for hide-n-seek.

# Import of python modules.
import math
import numpy as np

# import of relevant libraries.
import rospy # module for ROS APIs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, translation_matrix
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped # message type for cmd_vel
from nav_msgs.msg import OccupancyGrid, Path
from hide_n_seek.msg import MotionStatus
from visualization_msgs.msg import Marker, MarkerArray

RELEASE = True

# Constants.
# Topic names
# assuming both maps have same params, and target object will be marked with a special value on seen map
MAP_TOPIC = 'map_combined' if RELEASE else 'map' # name of topic for calculated occupancy grid
MOTION_TOPIC = 'motion_status'   # current goal point published by movement node
NAVIGATION_TOPIC = 'navigation_path' # topic the calculated path gets published to
GOAL_TOPIC = 'goal' # topic
FRAME_ID = 'odom'   # the static reference frame
DEFAULT_MARKER_TOPIC = 'visualization_marker'
DEFAULT_MARKER_ARRAY_TOPIC = 'visualization_marker_array'
# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Threshold of minimum clearance distance from the obstacles
MIN_THRESHOLD_DISTANCE = 0.2 # m, threshold distance, should be smaller than range_max
OBSTACLE_THRESHOLD_PROBABILITY = 0.2    # cells on grid with probability greater than this will be treated as obstacles
TARGET_VALUE = -50          # special value used to mark the target on the seen map

pose_seq = 0
marker_id = 0

class Navigation():
    def __init__(self, min_threshold_distance = MIN_THRESHOLD_DISTANCE):
        """Constructor."""
        # # Setting up publishers/subscribers.
        # Setting up the publishers.
        self._navigation_pub = rospy.Publisher(NAVIGATION_TOPIC, Path, queue_size=1)
        self._marker_pub = rospy.Publisher(DEFAULT_MARKER_TOPIC, Marker, queue_size=5)
        self._marker_array_pub = rospy.Publisher(DEFAULT_MARKER_ARRAY_TOPIC, MarkerArray, queue_size=100)
        # Setting up subscribers.
        self._map_sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self._map_callback, queue_size=1)
        self._motion_sub = rospy.Subscriber(MOTION_TOPIC, MotionStatus, self._motion_callback, queue_size=1)
        # self._goal_sub = rospy.Subscriber(GOAL_TOPIC, PoseStamped, self._goal_callback, queue_size=1)

        # Parameters.
        self.min_threshold_distance = min_threshold_distance
        self._map_width = None
        self._map_height = None
        self._map_resolution = None
        self._seen = None
        self._pos = None    # current position of the robot
        self._yaw = None
        self._goal = None   # a target yaw
        self._path_seq = 0

    def _motion_callback(self, msg):
        # all points are in /odom, no need to transform
        self._pos = (msg.goal.position.x, msg.goal.position.y)
        quat = msg.goal.orientation
        self._yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[-1]

    # def _goal_callback(self, msg):
    #     if self._pos is None:
    #         return
    #
    #     quat = msg.goal.position.orientation
    #     self._goal = self._yaw + euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[-1]
    #     #
    #     if self._goal > math.pi:







    def _map_callback(self, msg):
        self._map_width = msg.info.width
        self._map_height = msg.info.height
        self._map_resolution = msg.info.resolution
        # util funcs
        self.offset = int(MIN_THRESHOLD_DISTANCE / self._map_resolution)  # the number of cells away from a wall
        self.get_coords = lambda id: (id % self._map_width, id / self._map_width)
        self.get_id = lambda i, j: i + j * self._map_width
        self.in_bound = lambda i, j: 0 <= i < self._map_width and 0 <= j < self._map_height
        self.odom_to_map = lambda pt: (pt[0] - msg.info.origin.position.x, pt[1] - msg.info.origin.position.y)
        # self.map_coords_to_odom = lambda pt: (pt[0] * self._map_resolution + msg.info.origin.position.x, pt[1] * self._map_resolution + msg.info.origin.position.y)
        self.map_coords_to_odom = lambda pt: (round(pt[0] * self._map_resolution + msg.info.origin.position.x, 2), round(pt[1] * self._map_resolution + msg.info.origin.position.y, 2))
        self._seen = msg.data[len(msg.data) / 2:]
        # print "Seen map received"

        grid = msg.data[:len(msg.data) / 2] if RELEASE else msg.data

        self._pos = (0, 0)
        self._yaw = 0

        # wait until having heard from both map and movement
        if RELEASE:
            if self._pos is None or self._seen is None: return []
        # # determine next target point from seen map
            target = self.get_target(grid, self._seen, self.to_grid_id(self._pos))
        else:
            target = (1, 1)

        if target is None:
            print "Naviagtion: Can't find a target"
            return

        path = self.bfs(grid, self.to_grid_id(self._pos), self.to_grid_id(target))

        # print path

        # format message using generated path
        path_msg = Path()
        path_msg.header.frame_id = FRAME_ID
        path_msg.header.seq = self._path_seq
        path_msg.header.stamp = rospy.Time.now()
        self._path_seq += 1
        # path_msg.poses.append(to_pose(path[0], self._yaw))

        for r in range(1, len(path)):
            path_msg.poses.append(to_pose(path[r], get_yaw(path[r - 1][0], path[r - 1][1], path[r][0], path[r][1])))

        poses = [ extract_pose(pose) for pose in path_msg.poses]
        print poses

        # if len(poses) == 0:
        #     side = 5
        #
        #     for r in range(-side / 2, side / 2 + 1):
        #         s = ''
        #         nx, ny = target
        #         for c in range(-side / 2, side / 2 + 1):
        #             s += "{}, {}, {}".format(self.map_coords_to_odom((nx + r, ny + c)), access(nx + r, ny + c),
        #                                      access_seen(nx + r, ny + c)) + '\t'
        #         print s

        self.mark(target)
        self.mark_poses(path_msg.poses)

        print "Navigation: Path published."
        # publish the path to navigation
        self._navigation_pub.publish(path_msg)

    def get_target(self, map, seen, curr_idx):
        """Return a target point (in odom) from the seen map, assuming unseen cells are marked as -1."""
        map = self.expand(map)
        seen = list(seen)
        access = lambda i, j: map[self.get_id(i, j)]
        access_seen = lambda i, j: seen[self.get_id(i, j)]
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        if self._goal is not None:
            pass

        # search for target if it's been found, otherwise the closest unseen cell
        search_for = -1
        # go towards the target
        q = [self.get_coords(curr_idx)]
        visited = set()
        visited.add(curr_idx)

        while len(q):
            new_q = []
            for coords in q:
                for dir in directions:
                    nx = coords[0] + dir[0]
                    ny = coords[1] + dir[1]
                    nidx = self.get_id(nx, ny)

                    if not self.in_bound(nx, ny):
                        continue

                    # target found
                    if access_seen(nx, ny) == search_for:
                        # print "Exploring Target: {}".format((nx, ny))
                        # print "{}, {}".format(access(nx, ny), access_seen(nx, ny))
                        # print "Target: {}".format((nx, ny))
                        return self.map_coords_to_odom((nx, ny))
                    # only expand the free cells
                    if 0 <= access(nx, ny) < OBSTACLE_THRESHOLD_PROBABILITY and nidx not in visited:
                        visited.add(nidx)
                        new_q.append((nx, ny))
            q = new_q

        return None

    def expand(self, map):
        res = list(map)
        walls = list(filter(lambda idx: res[idx] >= OBSTACLE_THRESHOLD_PROBABILITY, range(len(res))))
        q = [self.get_coords(idx) for idx in walls]
        offset = self.offset
        while len(q) and offset > 0:
            new_q = []
            for coords in q:
                for dir in directions:
                    nx = coords[0] + dir[0]
                    ny = coords[1] + dir[1]
                    # only rewrite new coord that is in bound and free,
                    if self.in_bound(nx, ny) and access(nx, ny) == 0:
                        res[self.get_id(nx, ny)] = 100
                        new_q.append((nx, ny))
            q = new_q
            offset -= 1
        return res

    def bfs(self, map, curr_idx, tar_idx):
        """Return a path from current point to target point."""
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        # Change free cells within minimum distance from walls to walls.
        res = self.expand(map)
        access = lambda i, j: res[self.get_id(i, j)]

        # find a path from curr -> tar
        q = [self.get_coords(curr_idx)]
        parent = {curr_idx: -1}
        finished = False

        # trivial: [] -> already here
        if curr_idx == tar_idx:
            return [curr_idx]

        print "From {} to {}".format(self.map_coords_to_odom(self.get_coords(curr_idx)), self.map_coords_to_odom(self.get_coords(tar_idx)))

        while len(q) and not finished:
            new_q = []
            for coords in q:
                if finished: break
                idx = self.get_id(coords[0], coords[1])
                for dir in directions:
                    nx = coords[0] + dir[0]
                    ny = coords[1] + dir[1]
                    nidx = self.get_id(nx, ny)
                    if not self.in_bound(nx, ny):
                        continue
                    # target found
                    if nidx == tar_idx:
                        parent[nidx] = idx
                        finished = True
                        break
                    # new coord is in bound, free and not visited
                    if 0 <= access(nx, ny) < OBSTACLE_THRESHOLD_PROBABILITY and nidx not in parent:
                        # set parent to the previous node
                        parent[nidx] = idx
                        # add to new queue
                        new_q.append((nx, ny))
            q = new_q

        # build path from the parent map
        path = []
        curr = tar_idx
        while curr in parent:
            path.append(curr)
            curr = parent.get(curr)
        # format the points
        path = [self.map_coords_to_odom(self.get_coords(idx)) for idx in path]
        path.reverse()

        # [] -> unreachable, otherwise a path from curr to tar
        return path

    def mark_poses(self, poses):
        marker_array = MarkerArray()

        for pose in poses:
            marker_array.markers.append(to_marker(pose))

        self._marker_array_pub.publish(marker_array)

    def mark(self, target):
        marker_msg = Marker()
        # header
        marker_msg.header.frame_id = FRAME_ID
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.id = 0
        marker_msg.ns = "nav"
        # marker appearance
        marker_msg.type = Marker.SPHERE
        marker_msg.scale.x = 0.05
        marker_msg.scale.y = 0.05
        marker_msg.scale.z = 0.5
        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = target[0]
        marker_msg.pose.position.y = target[1]
        marker_msg.pose.position.z = 0.5
        marker_msg.pose.orientation.w = 1.0

        self._marker_pub.publish(marker_msg)

    def to_grid_id(self, pt):
        """Translate a pair of coords in odom to an id in an 1D array."""
        x, y = self.odom_to_map(pt)

        # bound to coords
        x = max(min(x, (self._map_width - 1) * self._map_resolution), 0)
        y = max(min(y, (self._map_height - 1) * self._map_resolution), 0)
        # snap to grid points
        x = int(round(x / self._map_resolution))
        y = int(round(y / self._map_resolution))
        return x + y * self._map_width


    def spin(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():

            rate.sleep()


def get_yaw(x, y, nx, ny):
    dx = nx - x
    dy = ny - y
    return math.copysign(math.acos(dx / math.sqrt(dx ** 2 + dy ** 2)), dy)

def to_marker(pose):
    """Mark a point in MAP reference frame with a certain type marker."""
    global marker_id
    marker_msg = Marker()

    # header
    marker_msg.header.frame_id = FRAME_ID
    marker_msg.header.stamp = rospy.Time.now()

    marker_msg.id = marker_id
    marker_id += 1
    marker_msg.ns = "nav"

    # marker appearance
    marker_msg.type = Marker.ARROW
    marker_msg.scale.x = 1
    marker_msg.scale.y = 0.2
    marker_msg.scale.z = 0.2

    marker_msg.color.r = 1.0
    marker_msg.color.g = 0.0
    marker_msg.color.b = 0.0
    marker_msg.color.a = 1.0

    marker_msg.action = Marker.ADD
    marker_msg.pose = pose.pose

    return marker_msg

def to_pose(pt, yaw):
    global pose_seq

    pose = PoseStamped()
    pose.header.frame_id = FRAME_ID
    pose.header.stamp = rospy.Time.now()
    pose.header.seq = pose_seq

    pose.pose.position.x = pt[0]
    pose.pose.position.y = pt[1]
    pose.pose.position.z = 0

    quaternion = quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    pose_seq += 1
    return pose

def extract_pose(pose):
    quat = pose.pose.orientation
    return pose.pose.position.x, pose.pose.position.y, euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[-1]/math.pi * 180

if __name__ == "__main__":
    """Main function."""
    # 1st. initialization of node.
    rospy.init_node("navigation")

    # wait for the registration.
    rospy.sleep(2)

    # Initialize a nav node.
    navigation = Navigation()

    # No-op on shutdown

    try:
        navigation.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


