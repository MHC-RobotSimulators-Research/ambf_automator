base_env = '/home/nataliechalfant/ambf_automator/launch.yaml'
target_body = '/home/nataliechalfant/ambf_automator/ruth_1000/default_soft.yaml'

data_location = '/home/nataliechalfant/ambf_automator/ruth_1000/'
data_adf_location = data_location + 'data_adf/'
data_ipc_location = data_location + 'data_ipc/'
data_fpc_location = data_location + 'data_fpc/'
data_force_location = data_location + 'data_force/'

test_name = 'ruth_1000_soft'

kLST_min = 0
kLST_max = 0.9
kLST_range = kLST_max - kLST_min
kLST_steps = 10

kDF_min = 0
kDF_max = 1
kDF_range = kDF_max - kDF_min
kDF_steps = 10

kMT_min = 0.0001
kMT_max = 0.01
kMT_range = kMT_max - kMT_min
kMT_steps = 10