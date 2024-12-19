# Nothing works unless I dont this, IDK why... :(
import sys
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

class PointCloudSub:
    def __init__(self, image_topic):
        self.image_sub = rospy.Subscriber(image_topic, PointCloud2, self.point_cloud_cb)
        self.point_cloud_msg = PointCloud2()

    def point_cloud_cb(self, point_cloud_msg):
        self.point_cloud_msg = point_cloud_msg

    def get_point_cloud(self):
        return self.point_cloud_msg


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

    # Add you camera stream subs
    point_cloud_sub = PointCloudSub('/ambf/env/cameras/camera1/DepthData')
    # cameraR_sub = ImageSub('/ambf/env/cameras/cameraR/ImageData')

    print("Resetting the world")
    world_handle.reset()
    time.sleep(3)

    ####
    # Your control / ML / RL Code will go somewhere in this script
    ####

    # Capture pre-deformation point cloud
    initial_pc = point_cloud_sub.get_point_cloud()

    # The PSMs can be controlled either in joint space or cartesian space. For the
    # latter, the `servo_cp` command sets the end-effector pose w.r.t its Base frame.
    T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/2.0), Vector(0.0, -0.03, -0.15))
    print("Setting the end-effector frame of PSM1 w.r.t Base", T_e_b)
    psm1.move_cp(T_e_b, 1)
    psm1.set_jaw_angle(0.2)
    time.sleep(1)

    # Capture deformed point cloud
    final_pc = point_cloud_sub.get_point_cloud()


    # T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/4.), Vector(0.01, -0.01, -0.13))
    # print("Setting the end-effector frame of PSM2 w.r.t Base", T_e_b)
    # psm2.servo_cp(T_e_b)
    # psm2.set_jaw_angle(0.5)
    # time.sleep(1.0)

    simulation_manager.clean_up()

    return [initial_pc, final_pc]
