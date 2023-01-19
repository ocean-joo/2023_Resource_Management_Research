import yaml
import matplotlib.pyplot as plt
import os
import scripts.autoware_analyzer_lib as aa
import numpy as np
from tqdm import tqdm

configs = {}

def profile_response_time(dir_path, start_instance, end_instance, is_collapsed, is_matching_failed, filter=0.8):
    _profile_response_time(dir_path, start_instance, end_instance, is_collapsed, is_matching_failed, 'shortest', filter)
    _profile_response_time(dir_path, start_instance, end_instance, is_collapsed, is_matching_failed, 'longest', filter)
    return

def _profile_response_time(dir_path, start_instance, end_instance, is_collapsed, is_matching_failed, type, filter=0.8):
    if type == 'shortest': label = 'Shortest'
    elif type == 'longest': label = 'Longest'
    else: 
        print('[ERROR] Invalidate type:', type)
        exit()

    first_node_path = dir_path + '/' + configs['first_node'] + '.csv'
    last_node_path = dir_path + '/' + configs['last_node'] + '.csv'

    E2E_response_time, max_E2E_response_time, avg_E2E_response_time \
                = aa.get_E2E_response_time(first_node_path, last_node_path, start_instance, end_instance, type=type)

    exp_title = dir_path.split('/')[1]
    exp_id = dir_path.split('/')[2]

    shortest_output_dir_path = 'analyzation/' + exp_title + '/' + type + '_E2E_response_time'
    if not os.path.exists(shortest_output_dir_path): os.system('mkdir -p ' + shortest_output_dir_path)

    # Plot graph
    x_data = list(E2E_response_time.keys()) # Instance IDs
    y_data = list(E2E_response_time.values()) # E2E response time(ms)
    if not is_collapsed:
        x_data = x_data[:int(len(x_data) * filter)]
        y_data = y_data[:int(len(y_data) * filter)]

    plot_path = shortest_output_dir_path+'/' + exp_title + '_' + exp_id + '_' + type + '_E2E_plot.png'
    if is_collapsed: reference_x = x_data[-1]
    else: reference_x = x_data[0] + 100    

    plt.plot(x_data, y_data)
    plt.axhline(y = max_E2E_response_time, color = 'r', linestyle = ':', label='Max')
    plt.axhline(y = avg_E2E_response_time, color = 'b', linestyle = ':', label='Avg')    
    plt.axvline(x = reference_x, color='k', label='Avoid')
    plt.legend()
    plt.ylim(0, 1000)
    plt.xlabel('Instance ID')
    plt.ylabel(label + ' E2E Response Time (ms)')
    plt.yticks(np.arange(0,1000,100))
    plt.title('is_collapsed='+str(is_collapsed) + '/ is_matching_failed='+str(is_matching_failed))
    plt.savefig(plot_path)
    plt.close()

    return

def profile_response_time_for_experiment(base_path, is_collapsed_list, is_matching_failed_list, filter=0.8, deadline=450.0, instance_offset=20):    
    exp_title = base_path.split('/')[1]
    instance_offset = int(instance_offset)
    _profile_response_time_for_experiment(base_path, exp_title, is_collapsed_list, is_matching_failed_list, deadline, instance_offset, type='shortest', mode='all', filter=filter)
    _profile_response_time_for_experiment(base_path, exp_title, is_collapsed_list, is_matching_failed_list, deadline, instance_offset, type='shortest', mode='collision', filter=filter)
    _profile_response_time_for_experiment(base_path, exp_title, is_collapsed_list, is_matching_failed_list, deadline, instance_offset, type='shortest', mode='matching_failed', filter=filter)    

    _profile_response_time_for_experiment(base_path, exp_title, is_collapsed_list, is_matching_failed_list, deadline, instance_offset, type='longest', mode='all', filter=filter)
    _profile_response_time_for_experiment(base_path, exp_title, is_collapsed_list, is_matching_failed_list, deadline, instance_offset, type='longest', mode='collision', filter=filter)
    _profile_response_time_for_experiment(base_path, exp_title, is_collapsed_list, is_matching_failed_list, deadline, instance_offset, type='longest', mode='matching_failed', filter=filter)

    return

def _profile_response_time_for_experiment(base_path, exp_title, is_collapsed_list, is_matching_failed_list, deadline, instance_offset, type, mode, filter=0.8):
    if type == 'shortest':
        label = 'Shortest'
    elif type == 'longest':
        label = 'Longest'
    else:
        print('[ERROR] Invalid mode:', label)
    
    available_mode = ['all', 'collision', 'matching_failed']
    if mode not in available_mode:
        print('[ERROR] Invalidate mode:', mode)
        exit()

    n = len(is_collapsed_list)
    collision_cnt = sum(is_collapsed_list)
    deadline_miss_cnt_when_collpased = 0
    deadline_miss_cnt_when_not_collpased = 0    

    target_experiment_idx_list = []
    if mode == 'all': target_experiment_idx_list = range(n)
    elif mode == 'collision': target_experiment_idx_list = aa.get_idices_of_one_from_list(is_collapsed_list)
    elif mode == 'matching_failed': target_experiment_idx_list = aa.get_idices_of_one_from_list(is_matching_failed_list)
    else:
        merged_indices = aa.merge_binary_list(is_collapsed_list, is_matching_failed_list)
        target_experiment_idx_list = aa.get_idices_of_one_from_list(merged_indices, reverse=True)

    for idx in target_experiment_idx_list:
        is_collapsed = is_collapsed_list[idx]        
        response_time_path = base_path + '/' + str(idx) + '/response_time'
        first_node_path = response_time_path + '/' + configs['first_node'] + '.csv'
        last_node_path = response_time_path + '/' + configs['last_node'] + '.csv'
        E2E_response_time, _, _ \
            = aa.get_E2E_response_time(first_node_path, last_node_path, start_instance, end_instance, type)

        x_data = list(E2E_response_time.keys()) # Instance IDs
        y_data = list(E2E_response_time.values()) # E2E response time(ms)
        if not is_collapsed:
            x_data = x_data[:int(len(x_data) * filter)]
            y_data = y_data[:int(len(y_data) * filter)]

        # Validate miss deadline during avoidance
        if is_collapsed:
            reference_x = x_data[-1]
            for k in range(reference_x - instance_offset, reference_x + 1):
                if not k in E2E_response_time.keys(): continue
                if E2E_response_time[k] >= deadline:
                    if is_collapsed_list[idx] == 1: deadline_miss_cnt_when_collpased = deadline_miss_cnt_when_collpased + 1
                    else: deadline_miss_cnt_when_not_collpased = deadline_miss_cnt_when_not_collpased + 1
                    break            
        else:
            reference_x = x_data[0] + 100
            for k in range(reference_x + 100 - instance_offset, reference_x + 100 + instance_offset):
                if not k in E2E_response_time.keys(): continue
                if E2E_response_time[k] >= deadline:
                    if is_collapsed_list[idx] == 1: deadline_miss_cnt_when_collpased = deadline_miss_cnt_when_collpased + 1
                    else: deadline_miss_cnt_when_not_collpased = deadline_miss_cnt_when_not_collpased + 1
                    break

        color = 'b'
        if is_collapsed_list[idx] == 1: color = 'r' 

        plt.plot(x_data, y_data, color, linewidth=1.0)

    plot_path = 'analyzation/' + exp_title + '/' + exp_title + '_' + mode + '_' + type + '_E2E_response_time.png'

    plt.legend()    
    plt.xlabel('Instance ID')
    plt.ylabel(label + ' E2E Response Time (ms)')
    plt.ylim(0, 1000)
    plt.yticks(np.arange(0,1000,100))
    plt.title('Iteration: ' + str(n) \
            + ' / Collision ratio: ' + str(sum(is_collapsed_list)/len(is_collapsed_list)) \
            + ' / Matching failure ratio: '+ str(sum(is_matching_failed_list)/len(is_matching_failed_list)))
    plt.savefig(plot_path)
    plt.close()

    deadline_miss_info_path = 'analyzation/' + exp_title + '/' + exp_title + '_deadline_miss_info(' + mode + ').yaml'
    deadline_miss_info = {}
    deadline_miss_info['deadline_ms'] = deadline
    if collision_cnt != 0:
        deadline_miss_info['deadline_miss_ratio_when_collpased']  = deadline_miss_cnt_when_collpased/collision_cnt
        deadline_miss_info['deadline_miss_ratio_when_not_collpased'] =  (deadline_miss_cnt_when_not_collpased/(len(is_collapsed_list) - collision_cnt))
    else:
        deadline_miss_info['deadline_miss_ratio_when_collpased'] = 0.0
        deadline_miss_info['deadline_miss_ratio_when_not_collpased'] = 0.0
    with open(deadline_miss_info_path, 'w') as f: yaml.dump(deadline_miss_info, f, default_flow_style=False)

    return

def profile_center_offset(dir_path, center_offset, max_center_offset, avg_center_offset, is_collapsed):
    exp_title = dir_path.split('/')[1]
    exp_id = dir_path.split('/')[2]
    output_dir_path = 'analyzation/' + exp_title + '/center_offset'
    if not os.path.exists(output_dir_path): os.system('mkdir -p ' + output_dir_path)

    # Plot graph
    x_data = list(center_offset.keys()) # Instance IDs
    y_data = list(center_offset.values()) # Center offset(m)
    plot_path = output_dir_path+'/' + exp_title + '_' + exp_id + '_center_offset.png'

    plt.plot(x_data, y_data)
    plt.axhline(y = max_center_offset, color = 'r', linestyle = ':', label='Max')
    plt.axhline(y = avg_center_offset, color = 'b', linestyle = ':', label='Avg')    
    plt.legend()    
    plt.xlabel('Instance ID')
    plt.ylabel('Center offset(m)')
    plt.title('is_collapsed='+str(is_collapsed))
    plt.savefig(plot_path)
    plt.close()

def profile_waypoints(dir_path, is_collapsed, is_matching_failed):
    exp_title = dir_path.split('/')[1]
    exp_id = dir_path.split('/')[2]
    output_dir_path = 'analyzation/' + exp_title + '/trajectories'
    if not os.path.exists(output_dir_path): os.system('mkdir -p ' + output_dir_path)

    # Centerline
    center_line_path = dir_path + '/center_line.csv'
    center_line = aa.get_center_line(center_line_path)
    center_line_x = []
    center_line_y = []
    
    for center_line_point in center_line:
        center_line_x.append(float(center_line_point[0]))
        center_line_y.append(float(center_line_point[1]))  

    plt.plot(center_line_x, center_line_y, 'k', label='Center line')

    # Waypoints
    exp_title = base_path.split('/')[1]

    center_offset_path = dir_path + '/center_offset.csv'
    waypoints = aa.get_waypoints(center_offset_path)
    waypoints_x = []
    waypoints_y = []
    
    for waypoint in waypoints:
        waypoints_x.append(float(waypoint[0]))
        waypoints_y.append(float(waypoint[1]))

    color = 'b'
    if is_collapsed: color = 'r'
    
    plt.plot(waypoints_x, waypoints_y, color, linewidth=1.0)

    # Objects
    npc1_x = [6, 6, -1, -1, 6]
    npc1_y = [51, 48, 48, 51, 51]
    npc2_x = [6, 6, -1, -1, 6]
    npc2_y = [55, 52, 52, 55, 55]
    plt.plot(npc1_x, npc1_y, 'k')
    plt.plot(npc2_x, npc2_y, 'k')

    # Plot
    plot_path = output_dir_path + '/' + exp_title + '_' + exp_id + '_waypoints.png'
            
    # plt.xlim(-70, 40)
    # plt.ylim(20,75)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('is_collapsed='+str(is_collapsed) + '/ is_matching_failed='+str(is_matching_failed))
    plt.legend()
    plt.savefig(plot_path)

    plt.close()

def profile_waypoints_for_experiment(base_path, is_collapsed_list, is_matching_failed_list):
    _profile_waypoints_for_experiment(base_path, is_collapsed_list, is_matching_failed_list, mode='all')
    _profile_waypoints_for_experiment(base_path, is_collapsed_list, is_matching_failed_list, mode='collision')
    _profile_waypoints_for_experiment(base_path, is_collapsed_list, is_matching_failed_list, mode='matching_failed')

def _profile_waypoints_for_experiment(base_path, is_collapsed_list, is_matching_failed_list, mode='all'):
    available_mode = ['all', 'collision', 'matching_failed']
    if mode not in available_mode:
        print('[ERROR] Invalidate mode:', mode)
        exit()
    
    n = len(is_collapsed_list)

    # Centerline
    center_line_path = base_path + '/0/center_line.csv'
    center_line = aa.get_center_line(center_line_path)
    center_line_x = []
    center_line_y = []
    
    for center_line_point in center_line:
        center_line_x.append(float(center_line_point[0]))
        center_line_y.append(float(center_line_point[1]))  

    plt.plot(center_line_x, center_line_y, 'k', label='Center line')
    
    target_experiment_idx_list = []
    if mode == 'all': target_experiment_idx_list = range(n)
    elif mode == 'collision':
        target_experiment_idx_list = aa.get_idices_of_one_from_list(is_collapsed_list)
    elif mode == 'matching_failed': target_experiment_idx_list = aa.get_idices_of_one_from_list(is_matching_failed_list)
    else:
        merged_indices = aa.merge_binary_list(is_collapsed_list, is_matching_failed_list)
        target_experiment_idx_list = aa.get_idices_of_one_from_list(merged_indices, reverse=True)

    exp_title = base_path.split('/')[1]

    # Waypoints
    for idx in target_experiment_idx_list:        
        exp_id = str(idx)
        label = exp_title + '_' + exp_id

        center_offset_path = base_path + '/' + str(idx) + '/center_offset.csv'
        waypoints = aa.get_waypoints(center_offset_path)
        waypoints_x = []
        waypoints_y = []
        
        for waypoint in waypoints:
            waypoints_x.append(float(waypoint[0]))
            waypoints_y.append(float(waypoint[1]))

        color = 'b'
        if is_collapsed_list[idx] == 1: color = 'r' 

        plt.plot(waypoints_x, waypoints_y, color, linewidth=1.0)

    # Objects
    npc1_x = [6, 6, -1, -1, 6]
    npc1_y = [51, 48, 48, 51, 51]
    npc2_x = [6, 6, -1, -1, 6]
    npc2_y = [55, 52, 52, 55, 55]
    plt.plot(npc1_x, npc1_y, 'k')
    plt.plot(npc2_x, npc2_y, 'k')

    # Plot
    plot_path = 'analyzation/' + exp_title + '/' + exp_title + '_' + mode + '_waypoints.png'        
            
    plt.xlim(-70, 40)
    plt.ylim(20,75)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Iteration: ' + str(n) \
            + ' / Collision ratio: ' + str(sum(is_collapsed_list)/len(is_collapsed_list)) \
            + ' / Matching failure ratio: '+ str(sum(is_matching_failed_list)/len(is_matching_failed_list)))
    plt.savefig(plot_path)

    plt.close()

def profile_analyzation_info(base_path, is_collapsed_list, is_matching_failed_list):
    exp_title = base_path.split('/')[1]

    analyzation_info = {}
    collision_index_list = aa.get_idices_of_one_from_list(is_collapsed_list)
    matching_failure_index_list = aa.get_idices_of_one_from_list(is_matching_failed_list)
    
    analyzation_info['collision_index'] = collision_index_list
    analyzation_info['collision_ratio'] = len(collision_index_list)/len(is_collapsed_list)
    analyzation_info['matching_failure_index'] = matching_failure_index_list
    analyzation_info['matching_failure_ratio'] = len(matching_failure_index_list)/len(is_matching_failed_list)

    analyzation_info_path = 'analyzation/' + exp_title + '/analyzation_info.yaml'
    with open(analyzation_info_path, 'w') as f: yaml.dump(analyzation_info, f, default_flow_style=False)

    return

if __name__ == '__main__':    
    with open('yaml/autoware_analyzer.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)
    
    for experiment_title in configs['experiment_title']:
        base_path = 'results/' + experiment_title
        n = aa.get_number_of_files(base_path)
        is_collapsed_list = []
        is_matching_failed_list = []
        pbar = tqdm(range(n))
        pbar.set_description(experiment_title)
        for idx in pbar:
            # Collision
            experiment_info_path = base_path + '/' + str(idx) + '/experiment_info.yaml'
            experiment_info = aa.get_experiment_info(experiment_info_path)
            is_collapsed = experiment_info['is_collaped']
            is_collapsed_list.append(is_collapsed)

            # Center offset
            center_offset_path = base_path + '/' + str(idx) + '/center_offset.csv'
            center_offset, max_center_offset, avg_center_offset \
                = aa.get_center_offset(center_offset_path)
            start_instance = int(list(center_offset.keys())[0])
            end_instance = int(list(center_offset.keys())[-1])
            profile_center_offset(center_offset_path, center_offset, max_center_offset, avg_center_offset, is_collapsed)

            # Check matching is failed
            is_matching_failed = aa.check_matching_is_failed(center_offset_path, start_instance, end_instance)
            is_matching_failed_list.append(is_matching_failed)

            # E2E Response Time
            response_time_path = base_path + '/' + str(idx) + '/response_time'            
            profile_response_time(response_time_path, start_instance, end_instance, is_collapsed, is_matching_failed)

            # Trajectories
            dir_path = base_path + '/' + str(idx)
            profile_waypoints(dir_path, is_collapsed_list[idx], is_matching_failed)

        # Profile for whole experiment
        profile_analyzation_info(base_path, is_collapsed_list, is_matching_failed_list)
        is_collapsed_list = aa.convert_boolean_list_to_int_list(is_collapsed_list)        
        is_matching_failed = aa.convert_boolean_list_to_int_list(is_matching_failed_list)        
        profile_response_time_for_experiment(base_path, is_collapsed_list, is_matching_failed)
        profile_waypoints_for_experiment(base_path, is_collapsed_list, is_matching_failed)