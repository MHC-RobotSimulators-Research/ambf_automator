# Nothing works unless I dont this, IDK why... :(
import sys

from ambf_msgs.msg import CameraState
from ambf_msgs.msg import RigidBodyState

sys.path.insert(0, "/home/nataliechalfant/surgical_robotics_challenge/scripts")

# Modified from surgical_robotics_challenge example interface_via_method_api.py

# Import the relevant classes

from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.scene import Scene
import rospy
from sensor_msgs.msg import PointCloud2

from surgical_robotics_challenge.kinematics.psmKinematics import ToolType

from PyKDL import Frame, Rotation, Vector
import numpy as np

# Import AMBF Client
from surgical_robotics_challenge.simulation_manager import SimulationManager
import time

class SubManager:
    def __init__(self, topic, msg_type):
        self.topic_sub = rospy.Subscriber(topic, msg_type, self.msg_cb)
        self.msg = msg_type()

    def msg_cb(self, msg):
        self.msg = msg

    def get_msg(self):
        return self.msg


def gather_data():
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

    # Reset and wait for AMBF to stabilize
    print("Resetting the world")
    world_handle.reset()
    time.sleep(2)

    ####
    # Your control / ML / RL Code will go somewhere in this script
    ####

    # Capture pre-deformation point cloud
    initial_pc = point_cloud_sub.get_msg()

    # The PSMs can be controlled either in joint space or cartesian space. For the
    # latter, the `servo_cp` command sets the end-effector pose w.r.t its Base frame.
    T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(0.0, -0.03, -0.15))
    print("Setting the end-effector frame of PSM1 w.r.t Base", T_e_b)
    psm1.move_cp(T_e_b, 1)
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

    simulation_manager.clean_up()

    return [initial_pc, final_pc, cam_state, link_state]
