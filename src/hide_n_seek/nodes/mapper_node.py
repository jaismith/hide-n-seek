#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Zack Nathan
# Date: Nov 13 2021

### imports

from math import sin, cos, pi, log, exp
from multiprocessing import Lock
import rospy
import tf
import numpy as np
import csv
import time
import os
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

### constants

DEFAULT_SCAN_TOPIC = 'scan'  # for the simulation, it's 'base_scan'
DEFAULT_MAP_TOPIC = 'map'
DEFAULT_MAP_SEEN_TOPIC = 'map_seen'
DEFAULT_MAP_COMBINED_TOPIC = 'map_combined'
FREQUENCY = 1  # Hz
MAP_SIZE = 256  # cells, width and height for a square map
GRID_RESOLUTION = 0.1  # cell size, m
LASER_FRAME = 'laser'  # for the simulation, it's 'base_laser_link'
CAMERA_FOV = pi / 3  # horizontal field of view of the camera, 60 degrees for the ROSBot
SCAN_ANGLE_OFFSET = pi  # pi for the ROSBot which has laser scan angles offset by pi from the simulations


# static function, calculate a probability value for the occupancy grid message given log odds
def occupancy_grid_probability(log_odds):

    # -1 for unknown cells
    if log_odds == 0.0:
        return -1

    # probability on a scale from 0 to 100
    return int(round(100 * (1.0 - 1.0 / (1 + exp(log_odds)))))


class Mapper:
    def __init__(self, frequency=FREQUENCY, map_size=MAP_SIZE, grid_resolution=GRID_RESOLUTION,
                 laser_frame=LASER_FRAME, camera_fov=CAMERA_FOV, scan_angle_offset=SCAN_ANGLE_OFFSET):

        # setting up the subscriber to receive laser scan messages
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        # setting up the publishers to send the occupancy and seen grids
        self._map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        self._map_seen_pub = rospy.Publisher(DEFAULT_MAP_SEEN_TOPIC, OccupancyGrid, queue_size=1)
        self._map_combined_pub = rospy.Publisher(DEFAULT_MAP_COMBINED_TOPIC, OccupancyGrid, queue_size=1)

        # setting up the transformation listener
        self.listener = tf.TransformListener()

        # parameters
        self.frequency = frequency
        self.rate = rospy.Rate(frequency)
        self.chunk_size = map_size / 2
        self.map_width = map_size
        self.map_height = map_size
        self.grid_resolution = grid_resolution
        self.laser_frame = laser_frame
        self.camera_fov = camera_fov
        self.scan_angle_offset = scan_angle_offset

        # initialize empty occupancy grid maps
        # Bayesian grid stores log-odds probability of occupancy for each cell
        self.bayesian_map = np.full((self.map_height, self.map_width), 0.0)
        # seen grid stores whether each grid has been seen by the camera
        self.seen_map = np.full((self.map_height, self.map_width), -1)

        # row and column indices of the cell (0, 0) in the map, in the center at first
        self.map_origin = [self.map_height / 2, self.map_width / 2]
        self.map_seq = 0

        self.laser_scan = None

        # mutex lock to prevent race conditions for occupancy grid updates
        self.mutex = Lock()

        # create the data directory if it doesn't exist
        try:
            os.mkdir('data')
        except OSError:
            pass

        rospy.sleep(2)


    # update the homogenous transformation matrix between the odom and base_link reference frames
    def get_transformations(self, lookup_time):
        for i in range(10):
            try:
                (trans, rot) = self.listener.lookupTransform('odom', 'base_link', lookup_time)
                t = tf.transformations.translation_matrix(trans)  # transform translation vector trans to 4x4 matrix t
                R = tf.transformations.quaternion_matrix(rot)  # transform quaternion rot to 4x4 matrix R
                odom_T_bl = t.dot(R)  # matrix multiplication to create the homogeneous transformation matrix

                (trans, rot) = self.listener.lookupTransform('base_link', self.laser_frame, lookup_time)
                t = tf.transformations.translation_matrix(trans)  # transform translation vector trans to 4x4 matrix t
                R = tf.transformations.quaternion_matrix(rot)  # transform quaternion rot to 4x4 matrix R
                bl_T_bll = t.dot(R)  # matrix multiplication to create the homogeneous transformation matrix

                return odom_T_bl, bl_T_bll

            # sometimes, the given time is later than the latest available transformation
            # raising an ExtrapolationException, so wait for the transformations to update
            except tf.ExtrapolationException:
                rospy.sleep(0.01)
        raise tf.ExtrapolationException


    # save the map as a csv file
    def save_map(self):
        filename = 'data/map_%i.csv' % int(time.time())
        with open(filename, 'w') as csv_file:
            map_writer = csv.writer(csv_file)

            # headers are the x coordinates of each column in the odom reference frame
            map_writer.writerow(['x=' + str((column - self.map_origin[1]) * self.grid_resolution)
                                 for column in range(self.map_width)])

            # write a row in the csv file for each row in the map
            for row in reversed(range(self.map_height)):
                map_writer.writerow([occupancy_grid_probability(log_odds) for log_odds in self.bayesian_map[row]])


    # instantiate an occupancy grid message with metadata
    def occupancy_grid_message(self):
        msg = OccupancyGrid()

        # header
        msg.header.frame_id = 'odom'
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.map_seq

        # metadata
        msg.info.resolution = self.grid_resolution
        msg.info.height = self.map_height
        msg.info.width = self.map_width
        msg.info.origin.position.x = -(self.map_origin[1]) * self.grid_resolution
        msg.info.origin.position.y = -(self.map_origin[0]) * self.grid_resolution
        msg.info.origin.orientation.w = 1
        return msg


    # publish the map and seen map messages
    def publish_maps(self):
        map_msg = self.occupancy_grid_message()
        seen_map_msg = self.occupancy_grid_message()
        combined_map_msg = self.occupancy_grid_message()
        self.map_seq += 1

        # populate maps and publish messages
        map_msg.data = [occupancy_grid_probability(log_odds) for log_odds in np.nditer(self.bayesian_map)]
        seen_map_msg.data = list(self.seen_map.flatten())
        combined_map_msg.data = map_msg.data + seen_map_msg.data
        self._map_pub.publish(map_msg)
        self._map_seen_pub.publish(seen_map_msg)
        self._map_combined_pub.publish(combined_map_msg)


    # extend the maps in the given direction, note that (0, 0) is at the top left so
    # up is -y, down is +y, left is -x, right is +x
    def extend_maps(self, direction):

        # vertical extension
        if direction == "up" or direction == "down":
            self.map_height += self.chunk_size
            extension = np.full((self.chunk_size, self.map_width), -1)

            # push origin down if adding rows above it
            if direction == "up":
                self.map_origin[0] += self.chunk_size
                self.bayesian_map = np.concatenate((extension + 1.0, self.bayesian_map), axis=0)
                self.seen_map = np.concatenate((extension, self.seen_map), axis=0)

            elif direction == "down":
                self.bayesian_map = np.concatenate((self.bayesian_map, extension + 1.0), axis=0)
                self.seen_map = np.concatenate((self.seen_map, extension), axis=0)

        # horizontal extension
        elif direction == "left" or direction == "right":
            self.map_width += self.chunk_size
            extension = np.full((self.map_height, self.chunk_size), -1)

            # push origin right if adding columns to the left
            if direction == "left":
                self.map_origin[1] += self.chunk_size
                self.bayesian_map = np.concatenate((extension + 1.0, self.bayesian_map), axis=1)
                self.seen_map = np.concatenate((extension, self.seen_map), axis=1)

            elif direction == "right":
                self.bayesian_map = np.concatenate((self.bayesian_map, extension + 1.0), axis=1)
                self.seen_map = np.concatenate((self.seen_map, extension), axis=1)


    # given the origin and target points (x, y) in odom, return the set of grid cells (row, column)
    # that intersect the ray/line between the origin and target points
    def raytracing(self, origin, target):

        # get cell coordinates for origin and target points
        x_origin = int(round(origin[0] / self.grid_resolution, 5))
        y_origin = int(round(origin[1] / self.grid_resolution, 5))
        x_target = int(round(target[0] / self.grid_resolution, 5))
        y_target = int(round(target[1] / self.grid_resolution, 5))

        # edge cases where the entire ray is inside one row or column
        if x_origin == x_target:

            # add target at the end because range stops short, can't just add one because of negatives
            return [(y, x_origin) for y in range(y_origin, y_target)] + [(y_target, x_origin)]
        if y_origin == y_target:
            return [(y_origin, x) for x in range(x_origin, x_target)] + [(y_origin, x_target)]

        # start from the left point so that x is always increasing
        if origin[0] < target[0]:
            slope = (target[1] - origin[1]) / (target[0] - origin[0])
            x = origin[0]
            y = origin[1]
            x_limit = target[0]
            y_limit = target[1]
        else:
            slope = (origin[1] - target[1]) / (origin[0] - target[0])
            x = target[0]
            y = target[1]
            x_limit = origin[0]
            y_limit = origin[1]

        row_step = 1 if slope > 0 else -1  # to help with creating ranges of rows
        cells = set()

        # move from left to right, adding all cells intersecting the line for each column
        while x < x_limit:
            column = int(round(x / self.grid_resolution, 5))
            x = (column + 1) * self.grid_resolution

            # calculate y-intercept of the line and the next column boundary
            y_next = slope * (x - origin[0]) + origin[1]
            y_next = min(y_next, y_limit) if slope > 0 else max(y_next, y_limit)  # bound by y at the end of the line

            # use y and y_next to get the range of rows for the current column
            cells.update((row, column) for row in range(int(round(y / self.grid_resolution, 5)),
                                                        int(round(y_next / self.grid_resolution, 5)) + row_step,
                                                        row_step))
            y = y_next

        return cells


    def update_maps(self):
        # mutex lock to prevent multiple calls from modifying the map or laser scan simultaneously
        with self.mutex:

            # wait if laser scan data does not exist yet
            if self.laser_scan is None:
                return

            # get the relevant homogenous transformation matrices at the time of the laser scan
            duration = rospy.rostime.Duration(self.laser_scan.time_increment * len(self.laser_scan.ranges) / 2)
            stamp = rospy.Time(self.laser_scan.header.stamp.secs, self.laser_scan.header.stamp.nsecs)
            odom_T_bl, bl_T_bll = self.get_transformations(stamp - duration / 2)
            origin = odom_T_bl.dot(bl_T_bll.dot(np.array([0, 0, 0, 1]).T))[:2]

            # generate the sets of all cells, occupied cells, and seen cells by iterating
            # over the range measurements using raytracing to determine which cells intersect
            all_cells = set()
            occupied = set()
            seen = set()
            angle = self.laser_scan.angle_min - self.laser_scan.angle_increment + self.scan_angle_offset
            for measurement in self.laser_scan.ranges:
                angle += self.laser_scan.angle_increment

                # discard any measurements outside of the bounds
                if measurement < self.laser_scan.range_min or measurement > self.laser_scan.range_max:
                    continue

                # transform the endpoint of the measurement into the odom reference frame for raytracing
                x_bll = measurement * cos(angle)
                y_bll = measurement * sin(angle)
                target = odom_T_bl.dot(bl_T_bll.dot(np.array([x_bll, y_bll, 0, 1]).T))[:2]

                # use raytracing to add the set of cells along the ray of the laser measurement
                cells = self.raytracing(origin, target)
                all_cells.update(cells)
                if abs(angle) < self.camera_fov / 2.0:
                    seen.update(cells)

                # add target cell to the set of occupied cells if within range
                if measurement < self.laser_scan.range_max:
                    index = tuple(int(round(target[i] / self.grid_resolution, 5)) for i in (1, 0))
                    occupied.add(index)
                    all_cells.add(index)

            # update the occupancy grid maps according to the sets of cells (all_cells, occupied, seen)
            # sets of cells contain row/column indices where (0, 0) is the odom origin rather than map[0, 0]]

            # get the x and y bounds of the set of cells
            y_range = [0, 0]
            x_range = [0, 0]
            for cell in all_cells:
                if cell[0] < y_range[0]:
                    y_range[0] = cell[0]
                elif cell[0] > y_range[1]:
                    y_range[1] = cell[0]
                if cell[1] < x_range[0]:
                    x_range[0] = cell[1]
                elif cell[1] > x_range[1]:
                    x_range[1] = cell[1]

            # extend the grids if the x and y bounds are outside of the map limits
            while y_range[0] < -self.map_origin[0]:
                self.extend_maps("up")
            while y_range[1] >= self.map_height - self.map_origin[0]:
                self.extend_maps("down")
            while x_range[0] < -self.map_origin[1]:
                self.extend_maps("left")
            while x_range[1] >= self.map_width - self.map_origin[1]:
                self.extend_maps("right")

            # update each cell in the occupancy grid maps
            for cell in all_cells:
                cell_index = (self.map_origin[0] + cell[0], self.map_origin[1] + cell[1])

                # mark seen cells in the seen map
                if cell in seen:
                    self.seen_map[cell_index] = 100

                # update Bayesian filter cells using log odds probabilities of occupancy
                # probability of a cell being occupied based on a single "occupied" observation is 2/3
                # probability of a cell being free based on a single "free" observation is 2/3
                log_odds = log(2.0) if cell in occupied else log(0.5)
                self.bayesian_map[cell_index] += log_odds

        self.publish_maps()


    # save the laser scan data to be used later
    def _laser_callback(self, msg):
        # mutex lock to avoid updating the laser scan in the middle of a map processing step
        with self.mutex:
            self.laser_scan = msg


    def spin(self):
        # loop until shutdown
        i = 0
        while not rospy.is_shutdown():

            # update the occupancy grid using the most recent laser scan data
            self.update_maps()

            # write occupancy grid data to file every 10 cycles
            if i % 10 == 0:
                self.save_map()

            self.rate.sleep()
            i += 1


def main():
    # initialize node
    rospy.init_node("mapper")

    # wait for registration
    rospy.sleep(2)

    # initialize robot motion class
    mapper = Mapper()

    # stop before shutdown
    rospy.on_shutdown(mapper.save_map)

    # start moving the robot
    try:
        mapper.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    main()
