#!/usr/bin/env python3
import rospy
import math
import copy
import numpy as np
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from slam.msg import incorporated_sensor_data
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

map_topic = rospy.Publisher("/map_test", OccupancyGrid, queue_size=1)
print("start")
rospy.init_node("GridMap", anonymous=True)
first_map_time = rospy.Time()
p_occupied = 0.7
p_free = 0.3
p_lo = 0.5
robot_frame = rospy.get_param("~robot_frame", "base_link")
map_frame = rospy.get_param("~map_frame", "map")
print("start2")

map_width = 1000
map_height = 1000
map_resolution = 0.1
map_x = (-map_height / 2) * map_resolution
map_y = (-map_height / 2) * map_resolution
prev_x = -99999999
prev_y = -99999999
occupancy_msg = OccupancyGrid()
# occupancy_msg.header.frame_id = map_frame
occupancy_msg.info.resolution = map_resolution
occupancy_msg.info.width = int(map_width)
occupancy_msg.info.height = int(map_height)
occupancy_msg.info.origin.position.x = map_x
occupancy_msg.info.origin.position.y = map_y

print("after start")


class Prob_Map:
    def __init__(
        self, map_center_x, map_center_y, map_width, map_height, map_resolution
    ):
        self.map_center_x = map_center_x
        self.map_center_y = map_center_y
        self.map_size_x = map_width
        self.map_size_y = map_height
        self.map_resolution = map_resolution
        self.grid_8 = 0
        self.p_occupied = np.log(0.7 / (1 - 0.7))
        self.p_free = np.log(0.3 / (1 - 0.3))
        self.p_lo = np.log(0.5 / (1 - 0.5))
        self.isD = False
        map_rows = int(map_height)
        map_cols = int(map_width)
        self.gridmap = self.p_lo * np.ones((map_rows, map_cols))
        self.laser_reading = rospy.Subscriber(
            "/sensors_topic", incorporated_sensor_data, self.sensor_data, queue_size=1
        )
        print("initialize")

    def is_inside(self, i, j):
        # print('INSIDE is_inside function:\n', i, map_width, j, map_height, i<map_width and j<map_height and i>=0 and j>=0)
        return i < map_width and j < map_height and i >= 0 and j >= 0

    def bresenham(self, i0, j0, i1, j1, d, debug=False):

        dx = np.absolute(j1 - j0)
        dy = -1 * np.absolute(i1 - i0)
        sx = -1
        if j0 < j1:
            sx = 1
        sy = -1
        if i0 < i1:
            sy = 1
        jp, ip = j0, i0
        err = dx + dy  # error value e_xy
        while True:
            if (
                (jp == j1 and ip == i1)
                or (np.sqrt((jp - j0) ** 2 + (ip - i0) ** 2) >= d)
                or not self.is_inside(ip, jp)
            ):
                return ip, jp, False
            elif self.gridmap[int(ip), int(jp)] == 100:
                return ip, jp, True

            if self.is_inside(ip, jp):
                # miss:
                self.gridmap[int(ip), int(jp)] += self.p_free - self.p_lo

            e2 = 2 * err
            if e2 >= dy:  # e_xy+e_x > 0
                err += dy
                jp += sx
            if e2 <= dx:  # e_xy+e_y < 0
                err += dx
                ip += sy

        # while True:
        #     if (jp == y1 and ip == x1) or (np.sqrt((jp-y0)**2+(ip-x0)**2) >= cells) or not (ip<prob_map.gridmap.shape[0] and jp<prob_map.gridmap.shape[1] and ip>=0 and jp>=0):
        #         return ip, jp, False
        #     elif prob_map.gridmap[int(ip),int(jp)]==100:
        #         return ip, jp, True

        #     if ip<prob_map.gridmap.shape[0] and jp<prob_map.gridmap.shape[1] and ip>=0 and jp>=0:
        #         prob_map.gridmap[int(ip),int(jp)] += prob_map.p_free - prob_map.p_lo

        #     e2 = 2*err
        #     if e2 >= dy:
        #         err += dy
        #         jp += sx
        #     if e2 <= dx:
        #         err += dx
        #         ip += sy

    # def to_file():
    #     f = open("prob.txt", "w")
    #     for j in range(map_height):
    #         for i in range(map_height):

    #         f.write(prob_map.gridmap[i][j])
    #         f.write("\n")
    #     f.close()
    # def from_file():
    #     f = open("prob.txt", "r")
    #     Lines = f.readlines()
    #     counter=0
    #     for i in Lines:
    #         prob_map.gridmap[counter]=
    #         f.write(i)
    #         f.write("\n")
    #     f.close()

    def quart_to_yaw(self, qx, qy, qz, qw):
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return yaw

    # flag=0
    def sensor_data(self, msg):
        # global flag
        print("sensor")
        # print("global 0 ", flag)
        # if not flag:
        #     prob_map=Prob_Map(map_x,map_y,map_width,map_height,map_resolution)
        #     flag=1
        # prob_map=Prob_Map(map_x,map_y,map_width,map_height,map_resolution)
        # print("global ", flag)
        readings = msg.ranges
        # print("reading",readings)
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y
        orientation_odom = msg.pose.pose.orientation
        yaw = self.quart_to_yaw(
            orientation_odom.x,
            orientation_odom.y,
            orientation_odom.z,
            orientation_odom.w,
        )
        i = 0

        # rospy.sleep(20.)
        for r in readings:
            print(i, " ", len(readings))
            if r != -1:
                if r > msg.range_max:
                    r = msg.range_max
                # r =msg.range_max if r>msg.range_max
                if r < msg.range_min:
                    r = msg.range_min
                # r =msg.range_max if r>msg.range_max
                angle = -yaw + i * msg.angle_increment + math.pi / 2

                x_map0 = int(y_odom / map_resolution + map_height / 2)
                y_map0 = int(x_odom / map_resolution + map_width / 2)
                dist_x = -y_odom + r * math.cos(angle)
                dist_y = -x_odom + r * math.sin(angle)
                x_map1 = int(dist_y / map_resolution + map_height / 2)
                y_map1 = int(-dist_x / map_resolution + map_width / 2)
                cell_no = int(r / map_resolution)  # no of cells ray is passing through

                endx, endy, flag = self.bresenham(
                    x_map0, y_map0, x_map1, y_map1, cell_no
                )
                # print(cells)
                # print(x_map1," ", y_map1)
                if self.is_inside(endx, endy):
                    self.gridmap[int(x_map1), int(y_map1)] += (
                        self.p_occupied - self.p_lo
                    )

                i += 1
        print("Dude did you even publish?")
        # Publish map
        # flat_grid=prob_map.gridmap.flatten()
        # gridmap_p = 1 - (1/(1+np.exp(flat_grid)))
        # gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
        flat_grid = self.gridmap.flatten()
        gridmap_p = 1 - (1 / (1 + np.exp(flat_grid)))
        gridmap_int8 = (gridmap_p * 100).astype(dtype=np.int8)
        print(gridmap_int8)
        occupancy_msg.data = copy.copy(gridmap_int8)

        occupancy_msg.header.frame_id = "robot_map"
        print(occupancy_msg.data)
        map_topic.publish(occupancy_msg)

        print("Yes bro")
        print("start3")

        # print("out of bresenham")
        # if int(ip)<prob_map.gridmap.shape[0] and int(jp)<prob_map.gridmap.shape[1] and ip>=0 and jp>=0:
        #     prob_map.gridmap[int(ip),int(jp)] += prob_map.p_occupied - prob_map.p_lo


def main():
    prob_map = Prob_Map(map_x, map_y, map_width, map_height, map_resolution)
    rospy.spin()


# tf_obj = tf.TransformListener()
# while not rospy.is_shutdown():
#     print("hello")
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
