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

def normalize_angle(angle):
    """Normalize the angle between -pi and pi"""

    while angle > math.pi:
        angle = angle - 2. * math.pi

    while angle < - math.pi:
        angle = angle + 2. * math.pi

    return angle

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
        self.mu = np.zeros((360, 3))
        self.sigma = np.zeros((360, 3, 3))
        self.weight = (1/360) * np.ones(360)
        self.best_weight_index = -1
        self.best_weight = math.inf
        map_rows = int(map_height)
        map_cols = int(map_width)
        self.gridmap = self.p_lo * np.ones((map_rows, map_cols))
        self.laser_reading = rospy.Subscriber("/sensors_topic", incorporated_sensor_data, self.sensor_data, queue_size=1)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_time_stamp = 0
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

    def slam_prediction(self, index, v, w, t):
        x = self.mu[index][0]
        y = self.mu[index][1]
        theta = self.mu[index][2]
        x_new = x + ((-v / w) * math.sin(theta) + (v / w) * math.sin(theta + w * t))
        y_new = y + ((v / w) * math.cos(theta) - (v / w) * math.cos(theta + w * t))
        theta_new = theta + w * t

        self.mu[index] = [x_new, y_new, theta_new]

        # partial derivative?? for V matrix
        V = np.matrix([
            [
                (-math.sin(theta) + math.sin(t * w + theta)) / w,
                (
                    v
                    * (
                        t * w * math.cos(theta + t * w)
                        + math.sin(theta)
                        - math.sin(t * w + theta)
                    )
                    / w
                ),
            ],
            [
                (math.cos(theta) + math.cos(t * w + theta)) / w,
                (
                    v
                    * (
                        t * w * math.sin(theta + t * w)
                        - math.cos(theta)
                        + math.cos(t * w + theta)
                    )
                    / w
                ),
            ],
            [0, t],
        ])
        M = 0.1 * np.identity(2)
        G = np.matrix([
            [1, 0, (v * math.cos(theta + w * t) - v * math.cos(theta)) / w],
            [0, 1, (v * (math.sin(theta + w * t) - v * math.sin(theta))) / w],
            [0, 0, 1],
        ])
        self.sigma[index] = np.matmul(np.matmul(G, self.sigma[index]), np.transpose(G)) + np.matmul(
            np.matmul(V, M), np.transpose(V)
        )

    def slam_correction(self, index, reading, reading_angle):
        Q_t = 0.1 * np.identity(3)
        reading_angle = normalize_angle(reading_angle)
        new_mu_x = self.mu[index][0] + reading * math.cos(reading_angle + self.mu[index][2])
        new_mu_y = self.mu[index][1] + reading * math.sin(reading_angle + self.mu[index][2])
        S = np.matrix([[new_mu_x - self.mu[index][0]], [new_mu_y - self.mu[index][0]]])
        q = np.matmul(S.T, S)
        z_t = np.matrix([[math.sqrt(q)], [math.atan2(S[1], S[0]) - self.mu[index][2]]])
        # TODO: Add H.
        # H Weird 2x5 matrix
        H = np.ones(2, 3)
        S_t = np.matmul(np.matmul(H, self.sigma[index]), H.T) + Q_t
        K_t = np.matmul(self.sigma[index], H.T, np.linalg.inv(S_t))
        # TODO: Fix z hat/expected.
        z_expected = np.atleast_2d([[reading], [normalize_angle(reading_angle)]])
        z_error = z_t - z_expected
        self.mu[index] += np.matmul(K_t, z_error)
        self.weight[index] = (np.linalg.norm(2 * np.pi * S_t) ** 0.5) * np.exp(-0.5 * np.matmul(np.matmul(np.atleast_2d(z_error).T, np.linalg.inv(S_t)), z_error))
        if self.weight < self.best_weight:
            self.best_weight = self.weight
            self.best_weight_index = index

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
        
        # x_odom = msg.pose.pose.position.x
        # y_odom = msg.pose.pose.position.y
        # orientation_odom = msg.pose.pose.orientation
        # yaw = self.quart_to_yaw(
        #     orientation_odom.x,
        #     orientation_odom.y,
        #     orientation_odom.z,
        #     orientation_odom.w,
        # )
        yaw = normalize_angle(self.theta)
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
                # angle = -yaw + i * msg.angle_increment + math.pi / 2
                
                # v = msg.twist.twist.linear
                # w = msg.twist.twist.angular
                # t = msg.header.stamp
                # self.slam_prediction(i, v, w, t)
                # self.slam_correction(i, r, i * msg.angle_increment)
                
                # x_map0 = int(y_odom / map_resolution + map_height / 2)
                # y_map0 = int(x_odom / map_resolution + map_width / 2)
                # dist_x = -y_odom + r * math.cos(angle)
                # dist_y = -x_odom + r * math.sin(angle)
                # x_map1 = int(dist_y / map_resolution + map_height / 2)
                # y_map1 = int(-dist_x / map_resolution + map_width / 2)
                # cell_no = int(r / map_resolution)  # no of cells ray is passing through
                
                
                v = msg.twist.twist.linear
                w = msg.twist.twist.angular
                t = msg.header.stamp - self.last_time_stamp
                self.last_time_stamp = msg.header.stamp
                # TODO: Fix passing proper x, y, and t and in correction need to pass proper angle.
                self.slam_prediction(i, v, w, t)
                self.slam_correction(i, r, i * msg.angle_increment)
                x = self.mu[self.best_weight_index][0]
                y = self.mu[self.best_weight_index][1]
                angle = -self.mu[self.best_weight_index][2] + i * msg.angle_increment + math.pi / 2
                x_map0 = int(y / map_resolution + map_height / 2)
                y_map0 = int(x / map_resolution + map_width / 2)
                dist_x = -y + r * math.cos(angle)
                dist_y = -x + r * math.sin(angle)
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
