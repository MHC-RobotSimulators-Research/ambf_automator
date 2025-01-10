"""Robot Control for Dataset Generation

This script contains the classes and functions to move the PSM to a designated location and subscribe to the camera's
depth data and state as well as the toolpitchlink's state for capturing force data. Modified from
surgical_robotics_challenge example interface_via_method_api.py This script does require the surgical_robotics_challenge
(https://github.com/surgical-robotics-ai/surgical_robotics_challenge) and the path needs to be added below.

@Author: Natalie Chalfant
@Contact: chalf22n@mtholyoke.edu
@Date: January 10, 2025
"""
# Add the surgical_robotics_challenge to the systems python path
import sys
sys.path.insert(0, "/home/nataliechalfant/surgical_robotics_challenge/scripts")

import numpy as np
import rospy
import time

from PyKDL import Frame, Rotation, Vector
from random import randint
from sensor_msgs.msg import PointCloud2

from ambf_msgs.msg import CameraState
from ambf_msgs.msg import RigidBodyState
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.scene import Scene
from surgical_robotics_challenge.kinematics.psmKinematics import ToolType
from surgical_robotics_challenge.simulation_manager import SimulationManager


class SubManager:
    """
    Contains the methods to subsribe to and retrieve data from a rostopic
    """
    def __init__(self, topic, msg_type):
        """
        Initializes the SubManager class
        :param topic: ROS topic to subscribe to as a string
        :param msg_type: Message type to subscribe to
        """
        self.topic_sub = rospy.Subscriber(topic, msg_type, self.msg_cb)
        self.msg = msg_type()

    def msg_cb(self, msg):
        """
        Callback method for messages received from the rostopic
        :param msg: ROS message
        """
        self.msg = msg

    def get_msg(self):
        """
        Returns the message received from the rostopic
        :return: message received from the rostopic
        """
        return self.msg


def gather_data():
    """
    Initializes subscribers and connects to AMBF to move PSM and gather camera, depth, and force data.
    :return: List containing the initial point cloud, final point cloud, camera state and toolpitchlink state
    """
    # Create an instance of the client
    simulation_manager = SimulationManager('ambf_automator')
    time.sleep(1)
    world_handle = simulation_manager.get_world_handle()

    # Get a handle to PSM1
    psm1 = PSM(simulation_manager, 'psm1')
    # Get a handle  to PSM2 (I think we only need one for now)
    # psm2 = PSM(simulation_manager, 'psm2')
    # Get a handle to ECM
    # ecm = ECM(simulation_manager, 'camera1')

    # Add ROS subs
    point_cloud_sub = SubManager('/ambf/env/cameras/camera1/DepthData', PointCloud2)
    camera_state_sub = SubManager('/ambf/env/cameras/camera1/State', CameraState)
    pitch_link_state_sub = SubManager('ambf/env/psm1/toolpitchlink/State', RigidBodyState)

    time.sleep(2)

    ####
    # Your control / ML / RL Code will go somewhere in this script
    ####

    # Capture pre-deformation point cloud
    initial_pc = point_cloud_sub.get_msg()

    # The PSMs can be controlled either in joint space or cartesian space. For the
    # latter, the `servo_cp` command sets the end-effector pose w.r.t its Base frame.

    # T_e_b for ruth corner with gravity
    # T_e_b_1 = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(0.0, -0.03, -0.17))
    # T_e_b_2 = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(0.05, -0.085, -0.195))
    # T_e_b_3 = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(0.06, -0.007, -0.2))
    # T_e_b_4 = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(-0.01, -0.06, -0.135))
    # T_e_b = [T_e_b_1, T_e_b_2, T_e_b_3, T_e_b_4]

    # T_e_b for ruth corner without gravity
    T_e_b_1 = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(0.0, -0.03, -0.16))
    T_e_b_2 = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(0.05, -0.085, -0.175))
    T_e_b_3 = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(0.06, -0.007, -0.19))
    T_e_b_4 = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(-0.01, -0.045, -0.12))
    T_e_b = [T_e_b_1, T_e_b_2, T_e_b_3, T_e_b_4]

    # print("Setting the end-effector frame of PSM1 w.r.t Base", T_e_b)
    psm1.move_cp(T_e_b[randint(0,3)], 1)
    # psm1.move_cp(T_e_b_4, 1)
    psm1.set_jaw_angle(0.2)
    time.sleep(2)

    # Capture deformed point cloud and force info
    final_pc = point_cloud_sub.get_msg()
    cam_state = camera_state_sub.get_msg()
    link_state = pitch_link_state_sub.get_msg()


    # T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/4.), Vector(0.01, -0.01, -0.13))
    # print("Setting the end-effector frame of PSM2 w.r.t Base", T_e_b)
    # psm2.servo_cp(T_e_b)
    # psm2.set_jaw_angle(0.5)
    # time.sleep(1.0)

    # Clean up after collecting data. Note: surgical_robotics_challenge needs to be modified to add this method
    simulation_manager.clean_up()

    return [initial_pc, final_pc, cam_state, link_state]


if __name__ == "__main__":
    # Run gather data for testing purposes
    gather_data()
