#!/usr/bin/env python

# Author: Sihao Huang
# Date: 11/11/2021
# The navigation node for hide-n-seek.

# Import of python modules.
import math

# import of relevant libraries.
import rospy # module for ROS APIs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, translation_matrix
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped, PoseStamped, Vector3Stamped # message type for cmd_vel
from nav_msgs.msg import OccupancyGrid, Path
from hide_n_seek.msg import MotionStatus
from visualization_msgs.msg import Marker, MarkerArray

# Constants.
# Topic names
# assuming both maps have same params, and target object will be marked with a special value on seen map
MAP_TOPIC = 'map_combined'
MOTION_TOPIC = 'motion_status'   # current goal point published by movement node
NAVIGATION_TOPIC = 'navigation_path' # topic the calculated path gets published to
GOAL_TOPIC = 'object_angle'
GOAL_POSE_TOPIC = 'goal_pose'
FRAME_ID = 'odom'   # the static reference frame
DEFAULT_MARKER_TOPIC = 'visualization_marker'
# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

FRONTIER_NAVIGATION = True

# Threshold of minimum clearance distance from the obstacles
MIN_THRESHOLD_DISTANCE = 0.2 # m, threshold distance, should be smaller than range_max
OBSTACLE_THRESHOLD_PROBABILITY = 0.2    # cells on grid with probability greater than this will be treated as obstacles
TARGET_VALUE = -50          # special value used to mark the target on the seen map

pose_seq = 0
marker_id = 0

class Navigation():
    def __init__(self, min_threshold_distance = MIN_THRESHOLD_DISTANCE, frontier_navigation = FRONTIER_NAVIGATION):
        """Constructor."""
        # # Setting up publishers/subscribers.
        # Setting up the publishers.
        self._navigation_pub = rospy.Publisher(NAVIGATION_TOPIC, Path, queue_size=1)
        self._marker_pub = rospy.Publisher(DEFAULT_MARKER_TOPIC, Marker, queue_size=5)
        # Setting up subscribers.
        self._map_sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self._map_callback, queue_size=1)
        self._motion_sub = rospy.Subscriber(MOTION_TOPIC, MotionStatus, self._motion_callback, queue_size=1)
        self._goal_sub = rospy.Subscriber(GOAL_TOPIC, Vector3Stamped, self._goal_callback, queue_size=1)


        # Parameters.
        self.frontier_navigation = frontier_navigation
        self.min_threshold_distance = min_threshold_distance
        self._map_width = None
        self._map_height = None
        self._map_resolution = None
        self._seen = None
        self._pos = None    # current position of the robot
        self._yaw = None
        self._goal_yaw = None   # a target yaw
        self._goal = None
        self._path_seq = 0
        self._map_metadata = None
        self._marker_array = MarkerArray()
        self._directions = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        self._state = None

    def _motion_callback(self, msg):
        """Save the state and movement goal information received from the motion node for later planning. """
        # all points are in /odom, no need to transform
        self._pos = (msg.goal.position.x, msg.goal.position.y)
        quat = msg.goal.orientation
        self._state = msg.state
        self._yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[-1]

    def _goal_callback(self, msg):
        """Save the goal information received from the object detection node."""
        if self._pos is None:
            return

        print "Navigation: goal {}".format(msg.x / math.pi * 180)

        # format the final yaw
        goal_yaw = self._yaw + msg.x
        if goal_yaw  > math.pi:
            goal_yaw  = goal_yaw  - 2 * math.pi
        elif self._goal < -math.pi:
            goal_yaw  = goal_yaw  + 2 * math.pi

        self._goal_yaw = goal_yaw

    def _map_callback(self, msg):
        """Handler for messages received from the mapper node. """

        # save the metadata every time as the map can expand
        self._map_metadata = msg.info
        self._map_width = msg.info.width
        self._map_height = msg.info.height
        self._map_resolution = msg.info.resolution
        # # util funcs
        self.offset = int(MIN_THRESHOLD_DISTANCE / self._map_resolution)  # the number of cells away from a wall
        self.get_coords = lambda id: (id % self._map_width, id / self._map_width)
        self.get_id = lambda i, j: i + j * self._map_width
        self.in_bound = lambda i, j: 0 <= i < self._map_width and 0 <= j < self._map_height
        # transformations
        self.odom_to_map = lambda pt: (pt[0] - msg.info.origin.position.x, pt[1] - msg.info.origin.position.y)
        self.map_coords_to_odom = lambda pt: (round(pt[0] * self._map_resolution + msg.info.origin.position.x, 2), round(pt[1] * self._map_resolution + msg.info.origin.position.y, 2))

        # only calculate new paths when the robot is sitting idle
        if self._state == 'WAIT':
            # the map and seen map are stacked together - extract both
            self._seen = msg.data[len(msg.data) / 2:]
            grid = msg.data[:len(msg.data) / 2]
            grid = [-1 if prob < 0 else 1 if prob > OBSTACLE_THRESHOLD_PROBABILITY else 0 for prob in grid]

            # mark the obstacles based on the probability threshold
            grid = [0 if 0 <= prob < OBSTACLE_THRESHOLD_PROBABILITY else 1 for prob in grid]

            # wait until having heard from both map and movement
            if self._pos is None or self._seen is None: return []

            # preprocess the map to prevent paths from running too close to obstacles
            grid = self.expand(grid)

            # determine next target point from seen map
            target = self.get_target(grid, self._seen, self.to_grid_id(self._pos))

            print "Got target: {}".format(target)

            if target is None:
                print "Naviagtion: Can't find a target"
                return
            # bfs a path from current postion to the movement target
            path = self.bfs(grid, self.to_grid_id(self._pos), self.to_grid_id(target))

            # format message using generated path
            path_msg = Path()
            path_msg.header.frame_id = FRAME_ID
            path_msg.header.seq = self._path_seq
            path_msg.header.stamp = rospy.Time.now()
            self._path_seq += 1

            for r in range(1, len(path)):
                path_msg.poses.append(to_pose(path[r], get_yaw(path[r - 1][0], path[r - 1][1], path[r][0], path[r][1])))

            poses = [ extract_pose(pose) for pose in path_msg.poses]

            if len(poses) == 1:
                print(path_msg.poses[0])

            print poses

            # marker out the movement target in rviz for easier debugging
            self.mark(target)

            print "Navigation: Path published."
            # publish the path to navigation
            self._navigation_pub.publish(path_msg)

    def get_target(self, m, seen, curr_idx):
        """Return a target point (in odom) from the seen map, assuming unseen cells are marked as -1."""

        # goal is found, drive straight towards it
        if self._goal_yaw is not None:
            dx = self._map_resolution / 2
            dy = dx * math.tan(self._goal_yaw)
            x, y = list(self._pos)
            while m[self.to_grid_id((x, y))] == 0:
                x += dx
                y += dy

            goal = (x - dx, y - dy)
            print 'Goal found: {}'.format(goal)
            self.mark(goal)
            return goal

        print 'Getting target'
        seen = list(seen)
        access = lambda i, j: m[self.get_id(i, j)]
        access_seen = lambda i, j: seen[self.get_id(i, j)]

        if self.frontier_navigation:
            # gather all the reachable nodes
            q = [self.get_coords(curr_idx)]
            reachable = set()
            reachable.add(curr_idx)
            while len(q):
                new_q = []
                for coords in q:
                    for dir in self._directions:
                        nx = coords[0] + dir[0]
                        ny = coords[1] + dir[1]
                        nidx = self.get_id(nx, ny)
                        # only rewrite new coord that is in bound and free,
                        if self.in_bound(nx, ny) and nidx not in reachable and access(nx, ny) == 0:
                            reachable.add(nidx)
                            new_q.append((nx, ny))
                q = new_q

            frontiers = []
            # # find frontiers
            q = [self.get_coords(curr_idx)]
            visited = set()
            visited.add(curr_idx)

            while len(q):
                new_q = []
                for coords in q:
                    for dir in self._directions:
                        nx = coords[0] + dir[0]
                        ny = coords[1] + dir[1]
                        nidx = self.get_id(nx, ny)

                        if not self.in_bound(nx, ny):
                            continue

                        # found an unseen node, explore
                        if access_seen(nx, ny) == -1:
                            visited.add(nidx)
                            frontier = self.explore_frontier(nx, ny, visited, reachable, seen)
                            # print "frontier len: {}".format(len(frontier))
                            frontiers.append(frontier)

                        # otherwise only expand free cells
                        elif access(nx, ny) == 0 and nidx not in visited:
                            visited.add(nidx)
                            new_q.append((nx, ny))
                q = new_q

            # selected the biggest frontier
            if len(frontiers) == 0:
                print "Unable to find a frontier"
                return None

            length = 0
            fr = []
            for frontier in frontiers:
                if len(frontier) > length:
                    length = len(frontier)
                    fr = frontier

            print 'Max frontier: {}'.format(len(fr))
            # return the midpoint in the selected frontier (not quite)
            return self.map_coords_to_odom(fr[len(fr) / 2])
        else:
            # search for target if it's been found, otherwise the closest unseen cell
            search_for = -1
            # go towards the target
            q = [self.get_coords(curr_idx)]
            visited = set()
            visited.add(curr_idx)

            while len(q):
                new_q = []
                for coords in q:
                    for dir in self._directions:
                        nx = coords[0] + dir[0]
                        ny = coords[1] + dir[1]
                        nidx = self.get_id(nx, ny)

                        if not self.in_bound(nx, ny):
                            continue

                        # target found
                        if access_seen(nx, ny) == search_for:
                            return self.map_coords_to_odom((nx, ny))
                        # only expand the free cells
                        if access(nx, ny) == 0 and nidx not in visited:
                            visited.add(nidx)
                            new_q.append((nx, ny))
                q = new_q

            return None

    def explore_frontier(self, x, y, visited, reachable, seen):
        """A bfs from the specified grid to outline the frontier."""
        access_seen = lambda i, j: seen[self.get_id(i, j)]
        frontier = [(x, y)]
        q = [(x, y)]

        while len(q) > 0:
            new_q = []
            for x, y in q:
                for dx in (-1, 2):
                    for dy in (-1, 2):
                        nx = x + dx
                        ny = y + dy
                        nidx = self.get_id(nx, ny)
                        # skip current, seen points, and visited points
                        if (dx == 0 and dy == 0) or not self.in_bound(nx, ny) or nidx in visited or access_seen(nx,ny) != -1:
                            continue

                        # filter out nodes farther away from the actual frontier or unreachable
                        is_neighboring_seen = False
                        is_reachable = nidx in reachable
                        for dir in self._directions:
                            nnx = nx + dir[0]
                            nny = ny + dir[1]
                            if self.in_bound(nnx, nny):
                                if self.get_id(nnx, nny) in reachable:
                                    is_reachable = True
                                if access_seen(nnx, nny) != -1:
                                    is_neighboring_seen = True

                        # if is_reachable:
                        if is_reachable and is_neighboring_seen:
                            visited.add(nidx)
                            frontier.append((nx, ny))
                            new_q.append((nx, ny))
            q = new_q
        return frontier

    def expand(self, m):
        """Essentially a bfs that 'blur' the map using the preset threshold. """
        res = list(m)
        access = lambda i, j: res[self.get_id(i, j)]
        walls = list(filter(lambda idx: res[idx] >= OBSTACLE_THRESHOLD_PROBABILITY, range(len(res))))
        q = [self.get_coords(idx) for idx in walls]
        offset = self.offset
        while len(q) and offset > 0:
            new_q = []
            for coords in q:
                for dir in self._directions:
                    nx = coords[0] + dir[0]
                    ny = coords[1] + dir[1]
                    # only rewrite new coord that is in bound and free,
                    if self.in_bound(nx, ny) and access(nx, ny) == 0:
                        res[self.get_id(nx, ny)] = 1
                        new_q.append((nx, ny))
            q = new_q
            offset -= 1
        return res

    def bfs(self, m, curr_idx, tar_idx):
        """Return a path from current point to target point."""
        access = lambda i, j: m[self.get_id(i, j)]

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
                for dir in self._directions:
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
                    if access(nx, ny) == 0 and nidx not in parent:
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

    def to_occupancy_grid(self, m):
        """Format a 1D array of map data into a occupancy grid message."""
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = FRAME_ID
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.info = self._map_metadata
        grid_msg.data = m

        return grid_msg

    def mark(self, target):
        """Mark the given point using a marker in rviz."""

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
    """Wrap a pose in a marker message."""
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
    """Wrap a pair point and yaw in a PoseStamped message."""
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


