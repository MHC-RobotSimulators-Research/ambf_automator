import os
import subprocess
import threading

import yaml
import numpy as np
# import threading
import multiprocessing as mp
from os import listdir, kill
from os.path import isfile, join
import signal
import time

base_env = '/home/nataliechalfant/ambf_automator/softbody/launch.yaml'
tissue_patch = '/home/nataliechalfant/ambf_automator/softbody/tissue_patch.yaml'

test_location = '/home/nataliechalfant/ambf_automator/softbody/test_adf/'
test_name = 'soft'

klST_steps = 10
kDF_steps = 10
kMT_steps = 10

STOP = False


def launch_ambf(launch_file, body):
    global STOP
    ambf_term = subprocess.Popen(['/home/nataliechalfant/ambf/bin/lin-x86_64/ambf_simulator --launch_file ' + launch_file + ' -a ' + body], stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    while True:
        if STOP:
            os.killpg(os.getpgid(ambf_term.pid), signal.SIGINT)
            break
        time.sleep(0.1)

def load_adf(filename):
    with open(filename, 'r') as f:
        return yaml.safe_load(f)


def write_adf(data, filename):
    with open(filename, 'w') as f:
        yaml.dump(data, f)


def generate_soft_params():
    params = np.zeros(((klST_steps + 1) * (kDF_steps + 1) * (kMT_steps + 1), 3), dtype=float)
    count = 0

    for i in range(klST_steps + 1):
        for j in range(kDF_steps + 1):
            for k in range(kMT_steps + 1):
                params[count, 0] = i / klST_steps
                params[count, 1] = j / kDF_steps
                params[count, 2] = k / kMT_steps
                count += 1

    return params


def generate_adfs(body, params, path):
    count = 1

    # update model directories
    body['high resolution path'] = '.' + body['high resolution path']
    body['low resolution path'] = '.' + body['low resolution path']


    # apply generated parameters
    for i in range(len(params)):
        body[body['soft bodies'][0]]['config']['klST'] = float(params[i, 0])
        body[body['soft bodies'][0]]['config']['kDF'] = float(params[i, 1])
        body[body['soft bodies'][0]]['config']['kMT'] = float(params[i, 2])

        f_name = path + test_name + '_' + "{:05d}".format(count) + '.yaml'
        write_adf(body, f_name)
        count += 1


def get_files(path):
    files = [f for f in listdir(path) if isfile(join(path, f))]
    for i in range(len(files)):
        files[i] = path + files[i]
    return sorted(files)


def run_data_collection(paths):
    global STOP
    for path in paths:
        STOP = False
        print(path)
        p_ambf = threading.Thread(target=launch_ambf, args=(base_env, path))
        p_ambf.start()

        # can do stuff here
        time.sleep(10)

        STOP = True
        p_ambf.join()


if __name__ == '__main__':
    # launch_ambf(base_env, tissue_patch)
    # print(generate_soft_params())

    # Build all ADFs for the dataset
    # body = load_adf(tissue_patch)
    # params = generate_soft_params()
    # generate_adfs(body, params, test_location)

    paths = get_files(test_location)[0:10]
    # print(paths)
    run_data_collection(paths)
