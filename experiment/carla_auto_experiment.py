import os
import subprocess
import threading
import time
import rospy
import yaml
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped
from carla_msgs.msg import CarlaCollisionEvent
import scripts.slack_library as slack_library
import signal
from multiprocessing import Process
from tqdm import tqdm

is_roscore_started = threading.Event()
is_carla_started = threading.Event()
is_autorunner_started = threading.Event()
is_collapsed = threading.Event()

configs = {}
node_info = {}
target_environment = 'null'
slack_webhook = 'null'

def get_ps_info_with_grep(grep_str):
    ps_info_list = []
    _output = str(os.popen('ps ax | grep ' + grep_str).read()).split('\n')
    for line in _output:        
        ps_info = line.split(' ')
        ps_info = [v for v in ps_info if v != '']
        ps_info_list.append(ps_info)
    return ps_info_list

def kill_processes_by_ps_info(ps_info, prefix=''):    
    if len(ps_info) == 0: return
    pid = ps_info[0]
    
    os.system(prefix + ' '+ 'kill -9 ' + pid)
    return

def kill_processes_by_ps_info_list(ps_info_list, prefix=''):
    for ps_info in ps_info_list:
        kill_processes_by_ps_info(ps_info)
    return

def check_roscore_starts():
    _output = str(os.popen('rosnode list').read()).split('\n')
    if '/rosout' in _output: return True

    return False

def roscore():
    os.system(configs[target_environment]['roscore_cmd'])

def carla_simulator():
    os.system(configs['common']['carla_simulator_cmd'])

def carla_autoware():
    os.system(configs['common']['carla_autoware_cmd'])

def autorunner():
    print(configs[target_environment]['carla_lkas_autorunner_cmd'])
    if configs['autorunner_mode'] == 'LKAS': os.system(configs[target_environment]['carla_lkas_autorunner_cmd'])
    elif configs['autorunner_mode'] == 'FULL': os.system(configs[target_environment]['carla_full_autorunner_cmd'])
    else: print('Invalidate mode:', configs['autorunner_mode'])
    return

def save_result(iter, experiment_info):
    # Response time
    output_path = 'results/'+configs['experiment_title']+'/'+str(iter)
    os.system('mkdir '+output_path)
    if configs['target_environment'] == 'desktop':
        os.system('cp -r '+configs[target_environment]['response_time_path']+ ' ' + output_path)
    elif configs['target_environment'] == 'exynos':
        os.system('scp -r root@192.168.0.8:'+configs[target_environment]['response_time_path']+ ' ' + output_path)
    else:
        print('[Error] Invalid target environment:',configs['target_envirnment'])

    # Center line
    os.system('mv ./center_line.csv ' + output_path)
    
    # Center offset
    os.system('mv ./center_offset.csv ' + output_path)
    
    # Experiment info
    experiment_info_path = output_path + '/experiment_info.yaml'
    with open(experiment_info_path, 'w') as f: yaml.dump(experiment_info, f, default_flow_style=False)

    return

def kill_carla_nodes():
    for node in node_info['carla_nodes']:
        _output = str(os.popen('rosnode list').read()).split('\n')        
        for line in _output:
            if node in line: os.system('rosnode kill '+line)
    while True:
        time.sleep(1)
        _output = str(os.popen('rosnode list').read())
        if 'points_relay' not in _output: break
    return


def kill_autorunner():
    for node in node_info['autoware_nodes']:
        _output = str(os.popen('rosnode list').read()).split('\n')        
        for line in _output:
            if node in line: os.system('rosnode kill '+line)
    
    while True:
        time.sleep(1)
        _output = str(os.popen('rosnode list').read())
        if 'ndt_matching' not in _output: break
    return

def experiment_manager():    
    for i in range(configs['max_iteration']):
        while not check_roscore_starts():
            print('# Wait roscore')
            time.sleep(0.5)
        is_roscore_started.set()

        print('- Manager: Start Carla Simulator')
        # Start carla simulator
        carla_simulator_process = Process(target=carla_simulator)
        carla_simulator_process.start()

        # Get the pid of carla simulator
        carla_simulator_pid_list = []
        while len(carla_simulator_pid_list) != 3:
            carla_simulator_pid_list.clear()
            ps_info_list = get_ps_info_with_grep('carla')        
            for ps_info in ps_info_list:
                if len(ps_info) == 0: continue
                if 'RenderOffScreen' not in ps_info[-1]: continue
                pid = ps_info[0]
                carla_simulator_pid_list.append(pid)
            print('# Carla simulator roscore')
            time.sleep(0.5)

        print('- Manager: Start Carla Autoware')
        # Start carla autoware
        carla_autoware_process = Process(target=carla_autoware)
        carla_autoware_process.start()

        while not is_carla_started.is_set():
            print('# Wait carla autoware')
            time.sleep(0.5)

        print('- Manager: Start Autorunner')
        experiment_info = {}

        # Start autorunner
        autorunner_process = Process(target=autorunner)
        autorunner_process.start()

        while not is_autorunner_started.is_set():
            time.sleep(0.5)

        start_writing_position_info()

        pbar = tqdm(range(configs['duration']))
        for _ in pbar:
            pbar.set_description('Experiment ' + str(i+1) + '/' + str(configs['max_iteration']))
            time.sleep(1)
            if is_collapsed.is_set(): break
        stop_writing_position_info()  

        # Kill nodes
        kill_carla_nodes()
        kill_autorunner()
        # Kil carla simulator
        carla_ps_info_list = get_ps_info_with_grep('carla')
        for line in carla_ps_info_list:
            if 'carla_auto_experiment.py' in line: continue
            kill_processes_by_ps_info(line)   
        
        autorunner_process.join()
        carla_autoware_process.join()
        carla_simulator_process.join()
        experiment_info['is_collaped'] = is_collapsed.is_set()

        save_result(i, experiment_info)
        
        is_carla_started.clear()
        is_autorunner_started.clear()
        is_collapsed.clear()   
        is_roscore_started.clear()
            
    message = '- Manager: Experiment is finished: '+configs['experiment_title']
    payload = {"text": message}
    slack_library.send_slack_message(payload, slack_webhook)
    
    os.system('rosnode kill auto_experiment')
    
    return

def start_writing_position_info():
    subprocess.Popen('python3 scripts/write_position_info.py', shell=True, executable='/bin/bash')
    return

def stop_writing_position_info():
    _output = str(os.popen('ps ax | grep write_position_info').read())
    _output = _output.split('\n')
    for line in _output:    
        if not 'write_position_info.py' in line: continue
        pid = -1
        for v in line.split(' '):
            try: pid = int(v)
            except: continue
            break
        if pid != -1: os.kill(pid, signal.SIGINT)
    return

def twist_cmd_cb(msg):
    is_autorunner_started.set()
    return

def points_cb(msg):
    is_carla_started.set()
    return

def carla_collision_event_cb(msg):
    is_collapsed.set()
    return

if __name__ == '__main__':
    slack_webhook = slack_library.get_slack_webhook()

    with open('yaml/carla_auto_experiment_configs.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)
    
    with open('yaml/node_info.yaml') as f:
        node_info = yaml.load(f, Loader=yaml.FullLoader)
    
    does_dir_exist = os.path.exists('results/'+configs['experiment_title'])
    # If dir exist
    if configs['experiment_title'] == 'test':
        if does_dir_exist:
            os.system('rm -r results/'+configs['experiment_title'])
    else:
        if does_dir_exist:
            print('[Error] Experiment result exists already')
            exit()
    os.mkdir('results/'+configs['experiment_title'])

    target_environment = configs['target_environment']
    if target_environment not in ['desktop', 'exynos']:
        print('[Error] Invalid target environment')
        exit()
    
    manager_thread = threading.Thread(target=experiment_manager)
    manager_thread.start()

    rospy.init_node('auto_experiment')
    rospy.Subscriber('/points_raw_origin', PointCloud2, points_cb)
    rospy.Subscriber('/twist_cmd', TwistStamped, twist_cmd_cb)
    rospy.Subscriber('/carla/ego_vehicle/collision', CarlaCollisionEvent, carla_collision_event_cb)
    rospy.spin()