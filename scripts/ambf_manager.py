import os
import subprocess
import threading
import signal
import time
import numpy as np
import open3d as o3d
import struct
import ctypes
import sensor_msgs.point_cloud2 as pc2
from os import listdir
from os.path import isfile, join
from robot_control import gather_data
import config


# Used to signal exit from round of simulation
STOP = False


def launch_ambf(launch_file, body):
    global STOP
    cmd = '/home/nataliechalfant/ambf/bin/lin-x86_64/ambf_simulator --launch_file ' + launch_file + ' -l 2,3,4,5' + ' -a ' + body
    print(cmd)
    ambf_term = subprocess.Popen([cmd], stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

    while True:
        if STOP:
            os.killpg(os.getpgid(ambf_term.pid), signal.SIGINT)
            break
        time.sleep(1)


def get_files(path):
    files = [f for f in listdir(path) if isfile(join(path, f))]
    for i in range(len(files)):
        files[i] = path + files[i]
    return sorted(files)

# Some lightly modified code from stack exchange to process point clouds into files 
# https://robotics.stackexchange.com/questions/108066/converting-pointcloud2-to-structured-numpy-array-in-ros2
def write_point_cloud(point_cloud, path):
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


def run_data_collection(paths):
    global STOP
    count = 1

    for path in paths:
        STOP = False
        print(path)
        p_ambf = threading.Thread(target=launch_ambf, args=(config.base_env, path))
        p_ambf.start()
        time.sleep(1)

        # can do stuff here
        data = gather_data()
        print("made it here!")

        # cleanup time
        STOP = True
        p_ambf.join()
        time.sleep(1)

        # Save our data
        ipc_name = config.data_ipc_location + config.test_name + '_ipc_' + "{:05d}".format(count) + '.ply'
        fpc_name = config.data_fpc_location + config.test_name + '_fpc_' + "{:05d}".format(count) + '.ply'

        write_point_cloud(data[0], ipc_name)
        write_point_cloud(data[1], fpc_name)
        count += 1


if __name__ == '__main__':

    # Load ADFs and run AMBF
    # paths = get_files(config.data_adf_location)[0:10]
    paths = get_files(config.data_adf_location)
    print(paths)
    run_data_collection(paths)

