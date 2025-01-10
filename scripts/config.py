"""Configuration for Dataset Generation

This script contains the relevant paths and parameters for generating incremented soft body datasets

@Author: Natalie Chalfant
@Contact: chalf22n@mtholyoke.edu
@Date: January 10, 2025
"""
# Base launch file for AMBF
base_env = '/home/nataliechalfant/ambf_automator/launch.yaml'

# Base ADF for the body which will have its soft body parameters incremented
target_body = '/home/nataliechalfant/ambf_automator/ruth_corner1/ruth_corner1.yaml'

# Base dataset directory
data_location = '/home/nataliechalfant/ambf_automator/ruth_corner1/'

# Directory where generated ADFs will be saved
data_adf_location = data_location + 'data_adf/'

# Directory where initial point clouds will be saved
data_ipc_location = data_location + 'data_ipc/'

# Directory where final point clouds will be saved
data_fpc_location = data_location + 'data_fpc/'

# Directory where force data will be saved
data_force_location = data_location + 'data_force/'

# Dataset name
test_name = 'ruth_corner1'

# kLST parameters
kLST_min = 0.1
kLST_max = 0.9
kLST_range = kLST_max - kLST_min
kLST_steps = 10

# kDF parameters
kDF_min = 0.1
kDF_max = 1
kDF_range = kDF_max - kDF_min
kDF_steps = 10

# kMT parameters
kMT_min = 0.001
kMT_max = 0.01
kMT_range = kMT_max - kMT_min
kMT_steps = 10