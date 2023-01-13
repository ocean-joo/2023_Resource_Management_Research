import yaml
import matplotlib.pyplot as plt
import os
import scripts.autoware_analyzer_lib as aa

configs = {}

def profile_response_time(dir_path, E2E_response_time, max_E2E_response_time, avg_E2E_response_time):
    exp_title = dir_path.split('/')[1]
    exp_id = dir_path.split('/')[2]
    output_dir_path = 'analyzation/' + exp_title
    if not os.path.exists(output_dir_path): os.system('mkdir -p ' + output_dir_path)

    # Plot graph
    x_data = list(E2E_response_time.keys()) # Instance IDs
    y_data = list(E2E_response_time.values()) # E2E response time(ms)
    plot_path = output_dir_path+'/' + exp_title + '_' + exp_id + '_E2E_plot.png'
    
    plt.plot(x_data, y_data)
    plt.axhline(y = max_E2E_response_time, color = 'r', linestyle = ':', label='Max')
    plt.axhline(y = avg_E2E_response_time, color = 'b', linestyle = ':', label='Avg')    
    plt.legend()
    plt.xlabel('Instance ID')
    plt.ylabel('E2E Response Time (ms)')
    plt.savefig(plot_path)
    plt.close()

    return

def profile_center_offset(dir_path, center_offset, max_center_offset, avg_center_offset):
    exp_title = dir_path.split('/')[1]
    exp_id = dir_path.split('/')[2]
    output_dir_path = 'analyzation/' + exp_title
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
    plt.savefig(plot_path)
    plt.close()

def profile_waypoints_for_experiment(base_path, n):
    # Centerline
    center_line_path = base_path + '/0/center_line.csv'
    center_line = aa.get_center_line(center_line_path)
    center_line_x = []
    center_line_y = []
    
    for center_line_point in center_line:
        center_line_x.append(float(center_line_point[0]))
        center_line_y.append(float(center_line_point[1]))  

    plt.plot(center_line_x, center_line_y, label='Center line')

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

        plt.plot(waypoints_x, waypoints_y, linewidth=1.0, label=label)

    # Objects
    npc1_x = [6, 6, -1, -1, 6]
    npc1_y = [51, 48, 48, 51, 51]
    npc2_x = [6, 6, -1, -1, 6]
    npc2_y = [55, 52, 52, 55, 55]
    plt.plot(npc1_x, npc1_y, 'k', label='Obstacle')
    plt.plot(npc2_x, npc2_y, 'k')


    # Plot
    plot_path = 'analyzation/' + exp_title + '_' +'_waypoints.png'
    plt.xlim(-70, 40)
    plt.ylim(20,75)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')

    plt.legend()
    plt.savefig(plot_path)

    plt.close()

if __name__ == '__main__':    
    with open('yaml/autoware_analyzer.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)
    
    for base_path in configs['experiment_title']:
        base_path = 'results/' + base_path
        n = len(next(os.walk(base_path))[1])
        for idx in range(n):
            # Center offset
            center_offset_path = base_path + '/' + str(idx) + '/center_offset.csv'
            center_offset, max_center_offset, avg_center_offset \
                = aa.get_center_offset(center_offset_path)
            start_instance = int(list(center_offset.keys())[0])
            end_instance = int(list(center_offset.keys())[-1])
            profile_center_offset(center_offset_path, center_offset, max_center_offset, avg_center_offset)

            # E2E Response Time
            response_time_path = base_path + '/' + str(idx) + '/response_time'
            first_node_path = response_time_path + '/' + configs['first_node'] + '.csv'
            last_node_path = response_time_path + '/' + configs['last_node'] + '.csv'
            E2E_response_time, max_E2E_response_time, avg_E2E_response_time \
                = aa.get_E2E_response_time(first_node_path, last_node_path, start_instance, end_instance)
            profile_response_time(response_time_path, E2E_response_time, max_E2E_response_time, avg_E2E_response_time)            

            
        profile_waypoints_for_experiment(base_path, n)    