#!/usr/bin/env python

# Author: Sihao Huang
# Date: 11/11/2021
# The navigation node for hide-n-seek.

# Import of python modules.
import math
import numpy as np

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import PolygonStamped, Point32 # message type for cmd_vel
from nav_msgs.msg import OccupancyGrid



# Constants.
# Topic names
# assuming both maps have same params, and target object will be marked with a special value on seen map
MAP_TOPIC = 'map' # name of topic for calculated occupancy grid
SEEN_MAP_TOPIC = 'seen'  # name of topic for the seen map
NEXT_POINT_TOPIC = 'next'   # current goal point published by movement node
NAVIGATION_TOPIC = 'navigation' # topic the calculated path gets published to
FRAME_ID = 'odom'   # the static reference frame
# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Threshold of minimum clearance distance from the obstacles
MIN_THRESHOLD_DISTANCE = 0.2 # m, threshold distance, should be smaller than range_max
OBSTACLE_THRESHOLD_PROBABILITY = 0.2    # cells on grid with probability greater than this will be treated as obstacles
TARGET_VALUE = -50          # special value used to mark the target on the seen map

class Navigation():
    def __init__(self, min_threshold_distance = MIN_THRESHOLD_DISTANCE):
        """Constructor."""

        # # Setting up publishers/subscribers.
        # Setting up the publishers.
        self._navigation_pub = rospy.Publisher(NAVIGATION_TOPIC, PolygonStamped, queue_size=1)
        # Setting up subscribers.
        self._map_sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self._map_callback, queue_size=1)
        self._seen_map_sub = rospy.Subscriber(SEEN_MAP_TOPIC, OccupancyGrid, self._seen_map_callback, queue_size=1)
        self._next_point_sub = rospy.Subscriber(NEXT_POINT_TOPIC, Point32, self._next_point_callback, queue_size=1)

        # Parameters.
        self.min_threshold_distance = min_threshold_distance
        self._map_width = None
        self._map_height = None
        self._map_resolution = None
        self._seen = None
        self._pos = None    # current position of the robot

    def _next_point_callback(self, msg):
        # all points are in /odom, no need to transform
        self._pos = (msg.x, msg.y)

    def _seen_map_callback(self, msg):
        self._seen = msg.data

    def _map_callback(self, msg):
        if self._map_width is None:
            print "Navigation: map metadata set."
            self._map_width = msg.info.width
            self._map_height = msg.info.height
            self._map_resolution = msg.info.resolution
            # util funcs
            self.offset = int(MIN_THRESHOLD_DISTANCE / self._map_resolution)  # the number of cells away from a wall
            self.get_coords = lambda id: (id % self._map_width, id / self._map_width)
            self.get_id = lambda i, j: i + j * self._map_width
            self.in_bound = lambda i, j: 0 <= i < self._map_width and 0 <= j < self._map_height

        # wait until having heard from both map and movement
        if self._pos is None or self._seen is None: return []

        # # determine next target point from seen map
        target = get_target(msg.data, self._seen, self.to_grid_id(self._pos))

        if target is None:
            print "Naviagtion: Can't find a target"
            return

        path = self.bfs(msg.data, self.to_grid_id(self._pos), self.to_grid_id(target))

        # print path

        # format message using generated path
        nav = PolygonStamped()
        nav.header.stamp = rospy.Time.now()
        nav.header.frame_id = FRAME_ID
        nav.polygon = [Point32(pt[0], pt[1], 0.0) for pt in path]
        print "Navigation: Path published."
        # publish the path to navigation
        self._navigation_pub.publish(nav)

    def get_target(self, map, seen, curr_idx):
        """Return a target point from the seen map, assuming unseen cells are marked as -1."""
        map = list(map)
        seen = list(seen)
        access = lambda i, j: map[self.get_id(i, j)]
        access_seen = lambda i, j: seen[self.get_id(i, j)]
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        # search for target if it's been found, otherwise the closest unseen cell
        search_for = TARGET_VALUE if TARGET_VALUE in map else -1
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
                    print "Target: {}".format((nx, ny))
                    return (nx, ny)
                # only expand the free cells
                if 0 <= access(nx, ny) < OBSTACLE_THRESHOLD_PROBABILITY and nidx not in visited:
                    visited.add(nidx)
                    new_q.append((nx, ny))
            q = new_q

        return None

    def bfs(self, map, curr_idx, tar_idx):
        """Return a path from current point to target point."""
        res = list(map)
        access = lambda i, j: res[self.get_id(i, j)]
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        # Change free cells within minimum distance from walls to walls.
        walls = list(filter(lambda idx: res[idx] >= OBSTACLE_THRESHOLD_PROBABILITY, range(len(map))))
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
        # find a path from curr -> tar
        q = [self.get_coords(curr_idx)]
        parent = {curr_idx: -1}
        finished = False

        # trivial: [] -> already here
        if curr_idx == tar_idx:
            return [curr_idx]

        print "From {} to {}".format(self.get_coords(curr_idx), self.get_coords(tar_idx))

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
        path = [self.get_coords(idx) for idx in path]
        path.reverse()

        # [] -> unreachable, otherwise a path from curr to tar
        return path


    def to_grid_id(self, pt):
        """Translate a pair of coords to an id in an 1D array."""
        x, y = pt

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


