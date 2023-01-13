import yaml
import matplotlib.pyplot as plt
import os
import autoware_anlyzer_lib as aa

configs = {}

def profile_response_time(dir_path):
    exp_title = dir_path.split('/')[1]
    exp_id = dir_path.split('/')[2]
    output_dir_path = 'analyzation/' + exp_title
    if not os.path.exists(output_dir_path): os.system('mkdir -p ' + output_dir_path)

    # avg, worst, plot
    first_node_path = dir_path + '/' + configs['first_node'] + '.csv'
    last_node_path = dir_path + '/' + configs['last_node'] + '.csv'

    # Unit: ms
    E2E_response_time = aa.get_E2E_response_time(first_node_path, last_node_path)
    E2E_response_time['avg'] = avg = aa.get_dict_avg(E2E_response_time)
    E2E_response_time['max'] = max = aa.get_dict_max(E2E_response_time)
    E2E_path = output_dir_path+'/' + exp_title + '_' + exp_id + '_E2E.yaml'
    aa.save_dict(E2E_response_time, E2E_path)
    
    # Plot graph
    E2E_response_time.pop('avg', None)
    E2E_response_time.pop('max', None)
    x_data = list(E2E_response_time.keys()) # Instance IDs
    y_data = list(E2E_response_time.values()) # E2E response time(ms)
    plot_path = output_dir_path+'/' + exp_title + '_' + exp_id + '_E2E_plot.png'
    
    plt.plot(x_data, y_data)
    plt.axhline(y = max, color = 'r', linestyle = ':', label='Max')
    plt.axhline(y = avg, color = 'b', linestyle = ':', label='Avg')    
    plt.legend()
    plt.xlabel('Instance ID')
    plt.ylabel('E2E Response Time (ms)')
    plt.savefig(plot_path)
    plt.close()

    return

if __name__ == '__main__':    
    with open('yaml/autoware_analyzer.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)
    
    for base_path in configs['result_dirs']:
        for idx in range(10):
            path = base_path + '/' + str(idx) + '/response_time'
            profile_response_time(path)
    