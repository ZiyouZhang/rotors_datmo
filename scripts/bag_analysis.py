"""
/**
 * @file bag_analysis.py
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief ROS bag analysis script.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
"""

#!/usr/bin/env python

import rosbag
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv
import glob
import os
from tf.transformations import euler_from_quaternion
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import numpy as np

if (len(sys.argv) > 1):
    filename = "../bag/" + sys.argv[1] + ".bag"
else:
    list_of_files = glob.glob('../bag/*')
    latest_file = max(list_of_files, key=os.path.getctime)
    filename = latest_file


def get_rpy(msg):

    roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                              msg.pose.pose.orientation.y,
                                              msg.pose.pose.orientation.z,
                                              msg.pose.pose.orientation.w])
    return roll, pitch, yaw


def diff_percent(a, b):
    if b != 0:
        return (a-b)/b * 100
    else:
        return (b-a)/a * 100


class topic_data:

    def __init__(self, topic_name, filename):
        self.topic_name = topic_name
        self.data = {
            "time": [],
            "r_x": [],
            "r_y": [],
            "r_z": [],
            "q_x": [],
            "q_y": [],
            "q_z": [],
            "q_w": [],
            "roll": [],
            "pitch": [],
            "yaw": [],
            "v_x": [],
            "v_y": [],
            "v_z": [],
            "omega_x": [],
            "omega_y": [],
            "omega_z": []
        }
        self.get_data_from_bag(filename)

    def get_data_from_bag(self, filename):
        with rosbag.Bag(filename) as bag:
            for topic, msg, t in bag.read_messages(topics=[self.topic_name]):
                self.data["time"].append(
                    msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
                self.data["r_x"].append(msg.pose.pose.position.x)
                self.data["r_y"].append(msg.pose.pose.position.y)
                self.data["r_z"].append(msg.pose.pose.position.z)
                self.data["q_x"].append(msg.pose.pose.orientation.x)
                self.data["q_y"].append(msg.pose.pose.orientation.y)
                self.data["q_z"].append(msg.pose.pose.orientation.z)
                self.data["q_w"].append(msg.pose.pose.orientation.w)
                self.data["roll"].append(get_rpy(msg)[0])
                self.data["pitch"].append(get_rpy(msg)[1])
                self.data["yaw"].append(get_rpy(msg)[2])
                self.data["v_x"].append(msg.twist.twist.linear.x)
                self.data["v_y"].append(msg.twist.twist.linear.y)
                self.data["v_z"].append(msg.twist.twist.linear.z)
                self.data["omega_x"].append(msg.twist.twist.angular.x)
                self.data["omega_y"].append(msg.twist.twist.angular.y)
                self.data["omega_z"].append(msg.twist.twist.angular.z)

    def calculate_velocities(self):
        self.data["v_x"] = []
        self.data["v_y"] = []
        self.data["v_z"] = []
        self.data["omega_x"] = []
        self.data["omega_y"] = []
        self.data["omega_z"] = []

        self.data["v_x"].append(0)
        self.data["v_y"].append(0)
        self.data["v_z"].append(0)
        self.data["omega_x"].append(0)
        self.data["omega_y"].append(0)
        self.data["omega_z"].append(0)

        for i in range(1, len(self.data["time"])):
            self.data["v_x"].append((self.data["r_x"][i] - self.data["r_x"][i-1]) / (self.data["time"][i] - self.data["time"][i-1]))
            self.data["v_y"].append((self.data["r_y"][i] - self.data["r_y"][i-1]) / (self.data["time"][i] - self.data["time"][i-1]))
            self.data["v_z"].append((self.data["r_z"][i] - self.data["r_z"][i-1]) / (self.data["time"][i] - self.data["time"][i-1]))
            # p = R.from_quat([self.data["q_x"][i]-1, self.data["q_y"][i-1], self.data["q_z"][i-1], self.data["q_w"][i-1]])
            # q = R.from_quat([self.data["q_x"][i], self.data["q_y"][i], self.data["q_z"][i], self.data["q_w"][i]])
            # r = np.matmul(p.as_dcm().transpose(), q.as_dcm())
            # r = p.as_dcm()-q.as_dcm()
            # print(r)

            self.data["omega_x"].append((self.data["roll"][i] - self.data["roll"][i-1]) / (self.data["time"][i] - self.data["time"][i-1]))
            self.data["omega_y"].append((self.data["pitch"][i] - self.data["pitch"][i-1]) / (self.data["time"][i] - self.data["time"][i-1]))
            self.data["omega_z"].append((self.data["yaw"][i] - self.data["yaw"][i-1]) / (self.data["time"][i] - self.data["time"][i-1]))

if __name__ == "__main__":

    truth = topic_data('/tag_box_pose_ground_truth', filename)
    detected = topic_data('/detected_object_state', filename)
    predicted = topic_data('/predicted_object_state', filename)
    ekf = topic_data('/updated_object_state', filename)

    detected.calculate_velocities()

    start_time = 3.12
    simulation_time = 0.79

    truth.data['time'] = [x - start_time for x in truth.data['time']]
    detected.data['time'] = [x - start_time for x in detected.data['time']]
    predicted.data['time'] = [x - start_time for x in predicted.data['time']]
    ekf.data['time'] = [x - start_time for x in ekf.data['time']]

    plot_trajectory = 0
    plot_error = 0
    plot_all = 0
    plot_group = 1
    plot_seperate = 0
    save_csv = 0
    plot_latency = 0

    def compare_plot(attr, include_detected):
        plt.plot(truth.data['time'], truth.data[attr],  'r.-', label='truth')
        if include_detected:
            plt.plot(detected.data['time'],
                    detected.data[attr], 'g.', label='detected')
        plt.plot(predicted.data['time'],
                predicted.data[attr], 'y.', label='predicted')
        plt.plot(ekf.data['time'], ekf.data[attr], 'b.', label='ekf')
        plt.title(attr)
        plt.legend()
        plt.xlim(2, 5)
        plt.show()
        plt.savefig(attr, dpi=400)
        plt.clf()

    def generate_plot(ax, attr, is_detection_estimation, y1=None, y2=None, title=None):

        ax.plot(truth.data['time'], truth.data[attr],  'r.-', label='ground truth', ms=3, lw=1)
        if is_detection_estimation:
            ax.plot(detected.data['time'], detected.data[attr], 'g.-', label='detected estimation')
        else:
            ax.plot(detected.data['time'], detected.data[attr], 'g.-', label='detected state')
        ax.plot(predicted.data['time'],  predicted.data[attr], 'y.-', label='predicted state')
        ax.plot(ekf.data['time'], ekf.data[attr], 'b.-', label='EKF estimation')
        ax.set_title(title if title else attr)
        ax.legend()
        ax.set_xlim(0, simulation_time)
        ax.set_ylim(y1, y2)
        ax.set_xlabel("time [s]")
        if attr == "r_x" or attr == "r_y" or attr == "r_z":
            ax.set_ylabel("position [m]")
        if attr == "roll" or attr == "pitch" or attr == "yaw":
            ax.set_ylabel("rotation [rad]")
        if attr == "v_x" or attr == "v_y" or attr == "v_z":
            ax.set_ylabel("linear velocity [m/s]")
        if attr == "omega_x" or attr == "omega_y" or attr == "omega_z":
            ax.set_ylabel("angular velocity [rad/s]")

    def error_plot(ax, attr):
        f = interp1d(truth.data['time'], truth.data[attr])
        ax.plot(ekf.data['time'], f(ekf.data['time']))
        ax.plot(ekf.data['time'], ekf.data[attr])
        err_det = []
        err_ekf = []
        for i in range(len(ekf.data['time'])):
            err_det.append(diff_percent(
                detected.data[attr][i], f(detected.data['time'][i])))
            err_ekf.append(diff_percent(ekf.data[attr][i], f(ekf.data['time'][i])))

        ax.plot(detected.data['time'], err_det, 'g.-', label='detected')
        ax.plot(ekf.data['time'], err_ekf, 'b.-', label='ekf')

        ax.set_title(attr)
        ax.legend()
        # ax.set_xlim()

    if plot_trajectory:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(truth.data["r_x"][300:390], truth.data["r_y"][300:390], truth.data["r_z"]
                [300:390], 'r.-', ms=6, lw=2, label='gournd truth trajectory')
        # ax.plot(predicted.data["r_x"][:-5], predicted.data["r_y"][:-5], predicted.data["r_z"][:-5],'y.-', ms=12, lw= 2,label='predicted trajectory')
        ax.plot(detected.data["r_x"][1:-5], detected.data["r_y"][1:-5], detected.data["r_z"][1:-5],'g.-', ms=12, lw= 2,label='detected trajectory')
        ax.plot(ekf.data["r_x"][:-5], ekf.data["r_y"][:-5], ekf.data["r_z"][:-5], 'b.-', ms=12, lw=2, label='ekf trajectory')
        ax.legend()
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_xlim(ekf.data['r_x'][-1], ekf.data['r_x'][0])
        ax.set_ylim()
        ax.set_zlim()
        plt.figure(dpi=400)
        plt.show()

    if plot_error:
        plt.clf()
        fig, axs = plt.subplots(4, 3)
        error_plot(axs[0][0], 'r_x')
        error_plot(axs[0][1], 'r_y')
        error_plot(axs[0][2], 'r_z')
        error_plot(axs[1][0], 'roll')
        error_plot(axs[1][1], 'pitch')
        error_plot(axs[1][2], 'yaw')
        error_plot(axs[2][0], 'v_x')
        error_plot(axs[2][1], 'v_y')
        error_plot(axs[2][2], 'v_z')
        error_plot(axs[3][0], 'omega_x')
        error_plot(axs[3][1], 'omega_y')
        error_plot(axs[3][2], 'omega_z')
        plt.show()

    if plot_all:
        plt.clf()
        fig, axs = plt.subplots(4, 3)
        generate_plot(axs[0][0], 'r_x', False)
        generate_plot(axs[0][1], 'r_y', False)
        generate_plot(axs[0][2], 'r_z', False)
        generate_plot(axs[1][0], 'roll', False)
        generate_plot(axs[1][1], 'pitch', False)
        generate_plot(axs[1][2], 'yaw', False)
        generate_plot(axs[2][0], 'v_x', True)
        generate_plot(axs[2][1], 'v_y', True)
        generate_plot(axs[2][2], 'v_z', True)
        generate_plot(axs[3][0], 'omega_x', True)
        generate_plot(axs[3][1], 'omega_y', True)
        generate_plot(axs[3][2], 'omega_z', True)
        plt.show()

    if plot_group:
        # fig, axs = plt.subplots(1, 3, dpi=200, figsize=(14,4))
        # generate_plot(axs[0], 'r_x', False, title=r'$r_x$', y1=1.5, y2=5.5)
        # generate_plot(axs[1], 'r_y', False, title=r'$r_y$', y1=-2, y2=2)
        # generate_plot(axs[2], 'r_z', False, title=r'$r_z$', y1=-1, y2=3)
        # plt.savefig('/home/ziyou/Desktop/report/figures/s2_r_plot.png', dpi=400)
        # plt.show()
        # plt.clf()

        # fig, axs = plt.subplots(1, 3, dpi=200, figsize=(14,4))
        # generate_plot(axs[0], 'roll', False, title=r'$roll$', y1=-0.5, y2=1)
        # generate_plot(axs[1], 'pitch', False, title=r'$pitch$', y1=-0.4, y2=1.1)
        # generate_plot(axs[2], 'yaw', False, title=r'$yaw$', y1=-0.75, y2=0.75)
        # plt.savefig('/home/ziyou/Desktop/report/figures/s2_q_plot.png', dpi=400)
        # plt.show()
        # plt.clf()

        # fig, axs = plt.subplots(1, 3, dpi=200, figsize=(14,4))
        # generate_plot(axs[0], 'v_x', True, title=r'$v_x$', y1=-6, y2=4)
        # generate_plot(axs[1], 'v_y', True, title=r'$v_y$', y1=-4.5, y2=5.5)
        # generate_plot(axs[2], 'v_z', True, title=r'$v_z$', y1=-5, y2=5)
        # plt.savefig('/home/ziyou/Desktop/report/figures/s2_v_plot.png', dpi=400)
        # plt.show()
        # plt.clf()

        fig, axs = plt.subplots(1, 3, dpi=200, figsize=(14,4))
        generate_plot(axs[0], 'omega_x', True, title=r'$\omega_x$', y1=-15, y2=15)
        generate_plot(axs[1], 'omega_y', True, title=r'$\omega_y$', y1=-15, y2=15)
        generate_plot(axs[2], 'omega_z', True, title=r'$\omega_z$', y1=-15, y2=15)
        plt.savefig('/home/ziyou/Desktop/report/figures/s2_omega_plot.png', dpi=400)
        plt.show()
        plt.clf()

    if plot_seperate:
        compare_plot('r_x', True)
        compare_plot('r_y', True)
        compare_plot('r_z', True)
        compare_plot('q_x', True)
        compare_plot('q_y', True)
        compare_plot('q_z', True)
        compare_plot('q_w', True)
        compare_plot('roll', True)
        compare_plot('pitch', True)
        compare_plot('yaw', True)
        compare_plot('v_x', False)
        compare_plot('v_y', False)
        compare_plot('v_z', False)
        compare_plot('omega_x', False)
        compare_plot('omega_y', False)
        compare_plot('omega_z', False)

    if save_csv:
        # rows = [truth.time,
        #         truth.r_z,
        #         truth.roll,
        #         truth.v_z,
        #         truth.omega_x,
        #         predicted.time,
        #         predicted.r_z,
        #         predicted.roll,
        #         predicted.v_z,
        #         predicted.omega_x,
        #         detected.time,
        #         detected.r_z,
        #         detected.roll,
        #         detected.v_z,
        #         detected.omega_x,
        #         ekf.time,
        #         ekf.r_z,
        #         ekf.roll,
        #         ekf.v_z,
        #         ekf.omega_x]
        rows = []

        with open('analysis.csv', 'wb') as f:
            writer = csv.writer(f, delimiter=',', quoting=csv.QUOTE_ALL)
            for row in rows:
                writer.writerow(row)

    if plot_latency:
        latency = [20, 20, 20, 10, 20, 20, 20, 10, 20, 20, 30, 20, 20, 20, 20, 10, 10]
        seq = range(1, len(latency)+1)
        plt.clf()
        plt.bar(seq, latency)
        plt.xlabel("Image processing sequence")
        plt.ylabel("detected latency [ms]")
        plt.xticks(np.arange(1, len(latency)+1, step=1))
        plt.show()
        print(sum(latency) / len(latency))
