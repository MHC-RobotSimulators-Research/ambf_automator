#!/usr/bin/env python
"""AMBF Simulation Manager for Dataset Generation

This script runs an AMBF instance for the specified ADFs, gathers data, and writes data using the parameters specified
in config.py

@Author: Natalie Chalfant
@Contact: chalf22n@mtholyoke.edu
@Date: January 10, 2025
"""
import config
import ctypes
import numpy as np
import open3d as o3d
import os
import pandas
import PyKDL
import sensor_msgs.point_cloud2 as pc2
import signal
import struct
import subprocess
import time
import tf_conversions.posemath as pm
import threading
from os import listdir
from os.path import isfile, join
from pynput.mouse import Button, Controller

from robot_control import gather_data

# Global used to signal exit from round of simulation
STOP = False


def click_ambf():
    """
    Activates "AMBF Simulator Window 2", moves mouse to the center of the screen, and clicks once.
    Used to ensure AMBF Simulator is working as expected before collecting data.
    """
    # Ensure corrected AMBF window is in focus
    os.system('xdotool search --name "AMBF Simulator Window 2" | xargs xdotool windowactivate')

    # Move mouse to center of screen and click once
    mouse = Controller()
    mouse.position = (1280, 800)
    mouse.click(Button.left)


def launch_ambf(launch_file, body):
    """
    Launches an instance of AMBF and keeps running until STOP is set to true.
    :param launch_file: path to launch file
    :param body: path to body file which will be added
    """
    global STOP # used to signal killing AMBF when ran in separate thread
    cmd = ('/home/nataliechalfant/ambf/bin/lin-x86_64/ambf_simulator --launch_file '
           + launch_file + ' -l 2,3,4,5' + ' -a ' + body)
    print(cmd)
    ambf_term = subprocess.Popen([cmd], stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

    time.sleep(2) # wait for AMBF to initialize

    click_ambf() # helps avoid instances where AMBF does not fully initialize before running data collection

    # Kill AMBF when STOP is set True
    while True:
        if STOP:
            os.killpg(os.getpgid(ambf_term.pid), signal.SIGINT)
            break
        time.sleep(1)


def get_files(path):
    """
    Returns a list of paths for all files in a directory.
    :param path: Directory containing files to be processed
    :return: Sorted list of paths for all files in path
    """
    files = [f for f in listdir(path) if isfile(join(path, f))]
    for i in range(len(files)):
        files[i] = path + files[i]
    return sorted(files)


# Some lightly modified code from stack exchange to process point clouds into files 
# https://robotics.stackexchange.com/questions/108066/converting-pointcloud2-to-structured-numpy-array-in-ros2
def write_point_cloud(point_cloud, path):
    """
    Writes the passed PointCloud2 object to a ".ply" file using open3d.
    :param point_cloud: PointCloud object to be written
    :param path: Directory + name of file to be written as a string (ex: "/home/user/pointcloud.ply")
    """
    # self.lock.acquire()
    gen = pc2.read_points(point_cloud, skip_nans=True)
    int_data = list(gen)
    xyz = np.empty((len(int_data), 3))
    rgb = np.empty((len(int_data), 3))

    idx = 0
    for x in int_data:
        test = x[3]
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f', test)
        i = struct.unpack('>l', s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)
        # prints r,g,b values in the 0-255 range
        # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz[idx] = x[:3]
        rgb[idx] = [r, g, b]

    out_pcd = o3d.geometry.PointCloud()
    out_pcd.points = o3d.utility.Vector3dVector(xyz)
    out_pcd.colors = o3d.utility.Vector3dVector(rgb)
    o3d.io.write_point_cloud(path, out_pcd)


def write_force(camera_state, link_state, path):
    """
    Orients the links force vector w.r.t the camera and writes the force vector and its location to CSV
    :param camera_state: CameraState message object
    :param link_state: RigidBodyState message object
    :param path: Directory + name of file to be written as a string (ex: "/home/user/force_data.csv")
    """
    # Extract frames and vectors from messages
    link_pos = pm.fromMsg(link_state.pose)
    cam_pos = pm.fromMsg(camera_state.pose)
    fl_msg = link_state.wrench.force
    force_link = PyKDL.Vector(fl_msg.x, fl_msg.y, fl_msg.z)

    # Reorient force vector to camera
    force_cam = cam_pos.M * link_pos.Inverse().M * force_link

    # Save to CSV with Pandas
    df = pandas.DataFrame([link_pos.p.x(), link_pos.p.y(), link_pos.p.z(), force_cam.x(), force_cam.y(), force_cam.z()])
    df.to_csv(path)


def run_data_collection(paths, start = 0, end = None):
    """
    Runs an AMBF instance for the specified range of ADFs in path, gathers data, and writes data.
    :param paths: List of paths to ADF files to be processed
    :param start: Index of the first ADF file being processed
    :param end: Index of the last ADF file being processed
    """
    global STOP # used to signal killing AMBF thread

    # When no end is specified process all ADFs in paths
    count = start
    if end is None:
        end = len(paths)

    while count < end:
        STOP = False # allows AMBF thread to run
        print(paths[count])
        p_ambf = threading.Thread(target=launch_ambf, args=(config.base_env, paths[count])) # starts AMBF thread
        p_ambf.start()
        time.sleep(1)

        # can do stuff here
        try: # try is used to catch error when PSM doesn't load, wish I knew why...
            data = gather_data()
        except RuntimeError or TypeError: # if error occurs don't collect data and try again
            STOP = True
            p_ambf.join()
            time.sleep(1)
            continue

        # cleanup time
        STOP = True # signals killing AMBF thread
        p_ambf.join()
        time.sleep(1)

        # Build file names
        ipc_name = config.data_ipc_location + config.test_name + '_ipc_' + "{:05d}".format(count + 1) + '.ply'
        fpc_name = config.data_fpc_location + config.test_name + '_fpc_' + "{:05d}".format(count + 1) + '.ply'
        force_name = config.data_force_location + config.test_name + '_force_' + "{:05d}".format(count + 1) + '.csv'

        # Save ourt data
        write_point_cloud(data[0], ipc_name)
        write_point_cloud(data[1], fpc_name)
        write_force(data[2], data[3], force_name)

        # and onto the next one
        count += 1


def find_missing(paths):
    """
    Scans existing dataset for missing datapoints and returns indices of ADFs for missing datapoints.
    :param paths: List of paths to ADF files to be processed
    :return: List of indices of missing ADFs
    """
    # Load dataset file names
    ipc = get_files(config.data_ipc_location)
    fpc = get_files(config.data_fpc_location)
    force = get_files(config.data_force_location)

    # Initialize pointers for each dataset component
    p_ipc = 0
    p_fpc = 0
    p_force = 0

    # Get length path for each dataset component, used to extract ID at the end of the filename
    ipc_len = len(ipc[0])
    fpc_len = len(fpc[0])
    force_len = len(force[0])

    missing = []

    # Check for datapoints for each ADF in paths
    for i in range(len(paths)):
        missing_i = False

        # Check ipc
        if int(ipc[p_ipc][ipc_len - 9 : ipc_len - 4]) == i + 1:
            p_ipc += 1
        else:
            missing_i = True

        # Check fpc
        if int(fpc[p_fpc][fpc_len - 9: fpc_len - 4]) == i + 1:
            p_fpc += 1
        else:
            missing_i = True

        # Check force
        if int(force[p_force][force_len - 9: force_len - 4]) == i + 1:
            p_force += 1
        else:
            missing_i = True

        # If any datapoint is missing add index to missing list
        if missing_i:
            missing.append(i)

    return missing


def fix_missing(paths, missing):
    """
    Runs run_data_collection for all indices in missing.
    :param paths: List of paths to ADF files to be processed
    :param missing: List of indices of ADFs of missing datapoints
    """
    for idx in missing:
        run_data_collection(paths, idx, idx + 1)

if __name__ == '__main__':

    # Load ADFs and run AMBF
    paths = get_files(config.data_adf_location)

    # Runs data collection for all ADFs in paths
    run_data_collection(paths)

    # Find missing datapoints
    # missing = find_missing(paths)
    # print(missing)

    # Re-run data collection for all missing datapoints
    # fix_missing(paths, missing)

    # Check is there are still missing datapoints
    # missing = find_missing(paths)
    # print(missing)