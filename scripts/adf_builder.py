import yaml
import numpy as np
import config


def load_adf(filename):
    with open(filename, 'r') as f:
        return yaml.safe_load(f)


def write_adf(data, filename):
    with open(filename, 'w') as f:
        yaml.dump(data, f)


def generate_soft_params():
    params = np.zeros(((config.kLST_steps + 1) * (config.kDF_steps + 1) * (config.kMT_steps + 1), 3), dtype=float)
    count = 0

    for i in range(config.kLST_steps + 1):
        for j in range(config.kDF_steps + 1):
            for k in range(config.kMT_steps + 1):
                params[count, 0] = i / config.kLST_steps * config.kLST_range + config.kLST_min
                params[count, 1] = j / config.kDF_steps * config.kDF_range + config.kDF_min
                params[count, 2] = k / config.kMT_steps * config.kMT_range + config.kMT_min
                count += 1

    return params


def generate_kLST_params():
    params = np.zeros(((config.kLST_steps + 1), 3), dtype=float)
    count = 0

    for i in range(config.kLST_steps + 1):
        params[count, 0] = i / config.kLST_steps * config.kLST_range + config.kLST_min
        params[count, 1] = 0.5 * config.kDF_range + config.kDF_min
        params[count, 2] = 0.5 * config.kMT_range + config.kMT_min
        count += 1

    return params


def generate_kDF_params():
    params = np.zeros(((config.kDF_steps + 1), 3), dtype=float)
    count = 0

    for i in range(config.kLST_steps + 1):
        params[count, 0] = 0.5 * config.kLST_range + config.kLST_min
        params[count, 1] = i / config.kDF_steps * config.kDF_range + config.kDF_min
        params[count, 2] = 0.5 * config.kMT_range + config.kMT_min
        count += 1

    return params


def generate_kMT_params():
    params = np.zeros(((config.kMT_steps + 1), 3), dtype=float)
    count = 0

    for i in range(config.kLST_steps + 1):
        params[count, 0] = 0.5 * config.kLST_range + config.kLST_min
        params[count, 1] = 0.5 * config.kDF_range + config.kDF_min
        params[count, 2] = i / config.kMT_steps * config.kMT_range + config.kMT_min
        count += 1

    return params


def generate_adfs(body, params, path):
    count = 1

    # update model directories
    body['high resolution path'] = '.' + body['high resolution path']
    body['low resolution path'] = '.' + body['low resolution path']


    # apply generated parameters
    for i in range(len(params)):
        body[body['soft bodies'][0]]['config']['kLST'] = float(params[i, 0])
        body[body['soft bodies'][0]]['config']['kDF'] = float(params[i, 1])
        body[body['soft bodies'][0]]['config']['kMT'] = float(params[i, 2])

        f_name = path + config.test_name + '_' + "{:05d}".format(count) + '.yaml'
        write_adf(body, f_name)
        count += 1




if __name__ == '__main__':

    # Build test ADFs
    body = load_adf(config.target_body)
    params = generate_kLST_params()
    # params = generate_kDF_params()
    # params = generate_kMT_params()
    generate_adfs(body, params, config.data_adf_location)

    # # Build all ADFs for the dataset
    # body = load_adf(config.target_body)
    # params = generate_soft_params()
    # generate_adfs(body, params, config.data_adf_location)


