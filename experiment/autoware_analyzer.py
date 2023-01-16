import yaml
import matplotlib.pyplot as plt
import os
import scripts.autoware_analyzer_lib as aa
import numpy as np

configs = {}

def profile_response_time(dir_path, E2E_response_time, max_E2E_response_time, avg_E2E_response_time, is_collapsed, filter=0.8):
    exp_title = dir_path.split('/')[1]
    exp_id = dir_path.split('/')[2]
    output_dir_path = 'analyzation/' + exp_title + '/E2E_response_time'
    if not os.path.exists(output_dir_path): os.system('mkdir -p ' + output_dir_path)

    # Plot graph
    x_data = list(E2E_response_time.keys()) # Instance IDs
    y_data = list(E2E_response_time.values()) # E2E response time(ms)
    if not is_collapsed:
        x_data = x_data[:int(len(x_data) * filter)]
        y_data = y_data[:int(len(y_data) * filter)]

    plot_path = output_dir_path+'/' + exp_title + '_' + exp_id + '_E2E_plot.png'
    
    plt.plot(x_data, y_data)
    plt.axhline(y = max_E2E_response_time, color = 'r', linestyle = ':', label='Max')
    plt.axhline(y = avg_E2E_response_time, color = 'b', linestyle = ':', label='Avg')    
    plt.axvline(x = x_data[0] + 110, color='k', label='Avoid')
    plt.legend()
    plt.ylim(0, 1000)
    plt.xlabel('Instance ID')
    plt.ylabel('E2E Response Time (ms)')
    plt.yticks(np.arange(0,1000,100))
    plt.title('is_collapsed='+str(is_collapsed))
    plt.savefig(plot_path)
    plt.close()

    return

def profile_response_time_for_experiment(base_path, is_collapsed_list, filter=0.8, deadline=500.0):
    n = len(is_collapsed_list)
    exp_title = base_path.split('/')[1]

    collision_cnt = sum(is_collapsed_list)
    deadline_miss_cnt_when_collpased = 0
    deadline_miss_cnt_when_not_collpased = 0

    for idx in range(n):
        response_time_path = base_path + '/' + str(idx) + '/response_time'
        first_node_path = response_time_path + '/' + configs['first_node'] + '.csv'
        last_node_path = response_time_path + '/' + configs['last_node'] + '.csv'
        E2E_response_time, _, _ \
            = aa.get_E2E_response_time(first_node_path, last_node_path, start_instance, end_instance)

        x_data = list(E2E_response_time.keys()) # Instance IDs
        y_data = list(E2E_response_time.values()) # E2E response time(ms)
        if not is_collapsed:
            x_data = x_data[:int(len(x_data) * filter)]
            y_data = y_data[:int(len(y_data) * filter)]

        # Validate miss deadline during avoidance
        for k in range(x_data[0]+110 - 10, x_data[0]+110 + 10):
            if not k in E2E_response_time.keys(): continue
            if E2E_response_time[k] >= deadline:
                if is_collapsed_list[idx] == 1: deadline_miss_cnt_when_collpased = deadline_miss_cnt_when_collpased + 1
                else: deadline_miss_cnt_when_not_collpased = deadline_miss_cnt_when_not_collpased + 1
                break

        color = 'b'
        if is_collapsed_list[idx] == 1: color = 'r' 

        plt.plot(x_data, y_data, color, linewidth=1.0)

    plot_path = 'analyzation/' + exp_title + '/' + exp_title +'_E2E_response_time.png'

    plt.legend()    
    plt.xlabel('Instance ID')
    plt.ylabel('E2E Response Time (ms)')
    plt.ylim(0, 1000)
    plt.yticks(np.arange(0,1000,100))
    plt.title('is_collapsed='+str(is_collapsed))
    plt.savefig(plot_path)
    plt.close()

    deadline_miss_info_path = 'analyzation/' + exp_title + '/' + exp_title +'_deadline_miss_info.yaml'
    if collision_cnt != 0:
        deadline_miss_info = {'deadline_miss_ratio_when_collpased':deadline_miss_cnt_when_collpased/collision_cnt, \
                            'deadline_miss_ratio_when_not_collpased': (deadline_miss_cnt_when_not_collpased/(len(is_collapsed_list) - collision_cnt))}
    else:
        deadline_miss_info = {'deadline_miss_ratio_when_collpased':0.0, \
                            'deadline_miss_ratio_when_not_collpased': (0.0)}
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

def profile_waypoints(dir_path, is_collapsed):
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
    plt.title('is_collapsed='+str(is_collapsed))
    plt.legend()
    plt.savefig(plot_path)

    plt.close()

def profile_waypoints_for_experiment(base_path, is_collapsed_list):
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

    # Waypoints
    for idx in range(n):
        exp_title = base_path.split('/')[1]
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
    plot_path = 'analyzation/' + exp_title + '/' + exp_title +'_waypoints.png'
    title = 'Iteration: ' + str(n) + ' / Collision ratio: ' + str(sum(is_collapsed_list)/len(is_collapsed_list)) \
            
    plt.xlim(-70, 40)
    plt.ylim(20,75)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title(title)
    plt.savefig(plot_path)

    plt.close()

if __name__ == '__main__':    
    with open('yaml/autoware_analyzer.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)
    
    for base_path in configs['experiment_title']:
        base_path = 'results/' + base_path
        n = aa.get_number_of_files(base_path)
        is_collapsed_list = []

        for idx in range(n):
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

            # E2E Response Time
            response_time_path = base_path + '/' + str(idx) + '/response_time'
            first_node_path = response_time_path + '/' + configs['first_node'] + '.csv'
            last_node_path = response_time_path + '/' + configs['last_node'] + '.csv'
            E2E_response_time, max_E2E_response_time, avg_E2E_response_time \
                = aa.get_E2E_response_time(first_node_path, last_node_path, start_instance, end_instance)
            profile_response_time(response_time_path, E2E_response_time, max_E2E_response_time, avg_E2E_response_time, is_collapsed)

            # Trajectories
            dir_path = base_path + '/' + str(idx)
            profile_waypoints(dir_path, is_collapsed_list[idx])

        # Waypoints
        is_collapsed_list = aa.convert_boolean_list_to_int_list(is_collapsed_list)        
        profile_response_time_for_experiment(base_path, is_collapsed_list)
        profile_waypoints_for_experiment(base_path, is_collapsed_list)