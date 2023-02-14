import csv
import yaml
# import rosbag
import math
import os
import signal
import subprocess
import matplotlib.pyplot as plt

def save_dict(data, path):
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    return

def subsctract_dicts(data1, data2):
    output = {}
    remove_targets = []
    keys = data1.keys()
    for k in keys:
        if k not in data2: 
            remove_targets.append(k)
            continue
        output[k] = data1[k] - data2[k]
    
    for remove_target in remove_targets:
        output.pop(remove_target)

    return output

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

'''
def read_topics_from_bag(rosbag_path, topic_name):
    print(rosbag_path)
    bag = rosbag.Bag(rosbag_path)
    output = []
    for _, msg, _ in bag.read_messages():        
        output.append(msg)
    return output
'''

def get_instance_pair_by_x(center_offset_path, start_x, end_x):
    output_start_instance = -1.0
    output_end_instance = -1.0
    column_idx = {}
    with open(center_offset_path) as f:
        reader = csv.reader(f)
        filtered_instance_list = []
        for i, line in enumerate(reader):
            if i == 0: 
                column_idx = get_column_idx_from_csv(line)
                continue
            instance = float(line[column_idx['instance']])
            x = float(line[column_idx['x']])
            if start_x > end_x:
                if x <= start_x and x >= end_x: filtered_instance_list.append(instance)
            else:
                if x >= start_x and x <= end_x: filtered_instance_list.append(instance)
        if len(filtered_instance_list) == 0:
            output_start_instance = -1
            output_end_instance = -1
        else:
            output_start_instance = min(filtered_instance_list)
            output_end_instance = max(filtered_instance_list)

    # print(output_start_instance, output_end_instance)
    return output_start_instance, output_end_instance    

def get_E2E_response_time(first_node_path, last_node_path, E2E_start_instance, E2E_end_instance, type):
    if type != 'shortest' and type != 'longest':
        print('[ERROR] Invalidate type:', type)
        exit()

    instance_info = {}
    start_instance = -1
    E2E_response_time = {}
    column_idx = {}

    # E2E Response Time
    with open(last_node_path) as f_last:
        reader = csv.reader(f_last)        
        for i, row in enumerate(reader):            
            if i == 0: 
                column_idx = get_column_idx_from_csv(row)
                continue
            end_time = float(row[column_idx['end']])
            instance_id = int(row[column_idx['instance']])
            if type == 'shortest':
                if instance_id in instance_info: continue
            if i == 1: start_instance = instance_id         
            instance_info[instance_id] = {'start_time': -1.0, 'end_time': end_time}

    with open (first_node_path) as f_start:        
        reader = csv.reader(f_start)
        for i, row in enumerate(reader):
            if i == 0: 
                column_idx = get_column_idx_from_csv(row)
                continue       

            start_time = float(row[column_idx['start']])
            instance_id = int(row[column_idx['instance']])
            if instance_id < start_instance: continue
            if instance_id not in instance_info: continue
            if type == 'shortest':
                if instance_info[instance_id]['start_time'] > 0: continue
            instance_info[instance_id]['start_time'] = start_time
    for instance_id in instance_info:
        response_time = instance_info[instance_id]['end_time'] - instance_info[instance_id]['start_time']        
        E2E_response_time[instance_id] = float(response_time * 1000) # unit: ms

    keys = list(E2E_response_time.keys())

    does_start_instance_found = False
    for key in keys:
        if key >= E2E_start_instance and does_start_instance_found == False:
            E2E_start_instance = key
            does_start_instance_found = True
            continue                       
        if key >= E2E_end_instance and E2E_end_instance > 0.0: 
            E2E_end_instance = key
            break        
    remove_target = []
    for k in E2E_response_time:
        if k < E2E_start_instance or k > E2E_end_instance or k not in keys: remove_target.append(k)

    for k in remove_target: E2E_response_time.pop(k, None)
    if len(E2E_response_time) == 0:
        avg_E2E_response_time = 0.0    
        max_E2E_response_time = 0.0
    else:
        avg_E2E_response_time = get_dict_avg(E2E_response_time)
        max_E2E_response_time = get_dict_max(E2E_response_time)

    return E2E_response_time, max_E2E_response_time, avg_E2E_response_time

'''
def start_rosbag_record(topic_names):   
    topic_str = '' 
    for topic in topic_names:
        topic_str = topic_str + ' ' + topic

    subprocess.Popen('source /opt/ros/melodic/setup.bash && rosbag record -O output' + topic_str, shell=True, executable='/bin/bash')
    return
'''

def get_center_offset(center_offset_path):
    column_idx = {}
    center_offset = {}
    with open(center_offset_path) as f:
        reader = csv.reader(f)
        for i, line in enumerate(reader):
            if i == 0: 
                column_idx = get_column_idx_from_csv(line)
                continue
            instance = float(line[column_idx['instance']])
            center_offset[instance] = abs(float(line[column_idx['center_offset']]))

    max_center_offset = get_dict_max(center_offset)
    avg_center_offset = get_dict_avg(center_offset)

    return center_offset, max_center_offset, avg_center_offset

def get_waypoints(center_offset_path):
    waypoints = []
    column_idx = {}
    with open(center_offset_path) as f:
        reader = csv.reader(f)
        for i, line in enumerate(reader):
            if i == 0: 
                column_idx = get_column_idx_from_csv(line)
                continue
            pose_x = float(line[column_idx['x']])
            pose_y = float(line[column_idx['y']])
            waypoints.append([pose_x,pose_y])
    
    return waypoints

def get_center_line(center_line_path):
    column_idx = {}
    center_line = []
    with open(center_line_path) as f:
        reader = csv.reader(f)
        for i, line in enumerate(reader):
            if i == 0: 
                column_idx = get_column_idx_from_csv(line)
                continue
            pose_x = float(line[column_idx['center_x']])
            pose_y = float(line[column_idx['center_y']])
            center_line.append([pose_x,pose_y])
    
    return center_line

def get_experiment_info(experiment_info_path):
    experiment_info = {}
    with open(experiment_info_path, 'r') as f:
        experiment_info = yaml.safe_load(f)
    return experiment_info

def stop_rosbag_record():
    _output = str(os.popen('ps au | grep rosbag').read())
    _output = _output.split('\n')
    for line in _output:    
        if not '/opt/ros/melodic/bin/rosbag' in line: continue
        pid = -1
        for v in line.split(' '):
            try: pid = int(v)
            except: continue        
            break

        if pid != -1: os.kill(pid, signal.SIGINT)

def get_number_of_files(path):
    output = str(os.popen('ls ' + path).read())
    output = output.split('\n')
    return len(output) - 1

def check_matching_is_failed(center_offset_path, start_instance, end_instance):    
    column_idx = {}
    with open(center_offset_path) as f:
        reader = csv.reader(f)
        for i, line in enumerate(reader):
            if i == 0: 
                column_idx = get_column_idx_from_csv(line)
                continue
            instance = int(line[column_idx['instance']])
            ndt_score = float(line[column_idx['ndt_score']])
            if instance < start_instance: continue
            if ndt_score > 2.0: return True
            if instance > end_instance: break
            
    return False

def mouse_event(event):
    print('x: {} and y: {}'.format(event.xdata, event.ydata))
    return

def convert_boolean_list_to_int_list(list_of_booleans):    
    return [int(item) for item in list_of_booleans]

def get_idices_of_one_from_list(input, reverse=False):
    output = []
    for i, v in enumerate(input):
        if reverse: v = 2**v%2
        if v: output.append(i)
    return output

def merge_binary_list_to_idx_list(a, b):
    output = []
    for i,_ in enumerate(a):
        if a[i] == 1 or b[i] == 1: output.append(i)
    return output
    
def get_column_idx_from_csv(line):
    output = {}
    for i, v in enumerate(line):
        output[v] = i
    return output
