import csv
import yaml

def save_dict(data, path):
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    return

def get_dict_avg(data):
    sum = 0.0
    for key in data:
        sum = sum + float(data[key])
    return sum / float(len(data))

def get_dict_max(data):
    max = 0.0
    for key in data:
        v = float(data[key])
        if v > max: max = v
    return max

def get_E2E_response_time(first_node_path, last_node_path, front_filter_ratio=0.20, end_filter_ratio=0.98):
    instance_info = {}
    start_instance = -1
    E2E_response_time = {}

    # E2E Response Time
    with open(last_node_path) as f:
        reader = csv.reader(f)        
        for i, row in enumerate(reader):            
            if i == 0: continue # Skip first row

            end_time = float(row[3])
            instance_id = int(row[4])
            if i == 1: start_instance = instance_id
            instance_info[instance_id] = {'start_time': -1.0, 'end_time': end_time}

    with open (first_node_path) as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i == 0: continue # Skip first row

            if i < start_instance: continue
            start_time = float(row[2])
            instance_id = int(row[4])

            if instance_id not in instance_info: continue
            instance_info[instance_id]['start_time'] = start_time
    
    for instance_id in instance_info:
        response_time = instance_info[instance_id]['end_time'] - instance_info[instance_id]['start_time']
        E2E_response_time[instance_id] = float(response_time * 1000) # unit: ms

    keys = list(E2E_response_time.keys())
    keys = keys[int(len(keys) * front_filter_ratio):int(len(keys) * end_filter_ratio)]

    remove_target = []
    for k in E2E_response_time:
        if k not in keys: remove_target.append(k)
    
    for k in remove_target: E2E_response_time.pop(k, None)

    return E2E_response_time
