import os
import subprocess
import threading
import time
import rospy
import yaml
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import NDTStat
import slack_library
import svl_scenario as svl
import signal


is_experiment_running = threading.Event()
is_scenario_started = threading.Event()
is_autorunner_started = threading.Event()
is_autorunner_killed = threading.Event()
is_experiment_finished = threading.Event()
barrier = threading.Barrier(2)

configs = {}
target_environment = 'null'
slack_webhook = 'null'

def autorunner():
    while True:
        if is_experiment_running.is_set(): break
    while True:
        time.sleep(1)
        if is_scenario_started.is_set():
            if configs['autorunner_mode'] == 'LKAS': os.system(configs[target_environment]['cubetown_lkas_autorunner_cmd'])
            elif configs['autorunner_mode'] == 'FULL': os.system(configs[target_environment]['cubetown_full_autorunner_cmd'])
            else:
                print('Invalidate mode:', configs['autorunner_mode'])
        else:
            continue
        if not is_experiment_running.is_set(): break
        barrier.wait()

    print('- Turn off Autorunner thread')

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
    experiment_info_path = output_path + '/experiemnt_info.yaml'
    with open(experiment_info_path, 'w') as f: yaml.dump(experiment_info, f, default_flow_style=False)

    return

def kill_autorunner():
    p = subprocess.Popen(configs[target_environment]['termination_cmd'], shell=True, stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    p.wait()
    
    while True:
        time.sleep(1)
        _output = str(os.popen('rosnode list').read())
        if 'ndt_matching' not in _output: break
    return

def imu_cb(msg):
    if is_experiment_running.is_set():
        is_scenario_started.set()
    else:
        is_scenario_started.clear()
    return

def twist_cmd_cb(msg):
    if is_experiment_running.is_set():
        is_autorunner_started.set()
    else:
        is_autorunner_started.clear()
    return

def experiment_manager(main_thread_pid):
    svl_scenario = svl.svl_scenario(configs['svl_cfg_path'])

    # Threads
    autorunner_thread = threading.Thread(target=autorunner)
    autorunner_thread.start()
    
    # Check rosbridge is started
    while True:
        _output = str(os.popen('rosnode list').read())
        if 'rosbridge_websocket' in _output: break

    for i in range(configs['max_iteration']):
        experiment_info = {}
        is_experiment_running.set()

        # Initialize SVL scenario
        svl_scenario.init()
        while not is_autorunner_started.is_set():
            svl_scenario.run(timeout=5, is_init=True)
        
        # Start Experiment
        start_writing_position_info()
        is_collapsed, _ = svl_scenario.run(timeout=configs['duration'], label='Iteration: ' + str(i+1)+'/'+str(configs['max_iteration']))
        stop_writing_position_info()
        experiment_info['is_collaped'] = is_collapsed

        if i+1 == int(configs['max_iteration']): is_experiment_running.clear()

        # Terminate
        kill_autorunner()
        is_autorunner_started.clear()
        is_scenario_started.clear()
        save_result(i, experiment_info)       

        if not is_experiment_running.is_set():
            message = 'Experiment is finished: '+configs['experiment_title']
            payload = {"text": message}
            slack_library.send_slack_message(payload, slack_webhook)
            break
        
        barrier.wait()
        barrier.reset()                    

    return os.kill(main_thread_pid, signal.SIGQUIT)

def start_writing_position_info():
    subprocess.Popen('python3 scripts/write_position_info.py', shell=True, executable='/bin/bash')
    return

def stop_writing_position_info():
    _output = str(os.popen('ps au | grep center_offset').read())
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

if __name__ == '__main__':
    main_thread_pid = os.getpid()

    slack_webhook = slack_library.get_slack_webhook()

    with open('yaml/auto_experiment_configs.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)

    if os.path.exists('results/'+configs['experiment_title']):
        print('[Error] Experiment result exists already')
        exit()
    os.mkdir('results/'+configs['experiment_title'])

    target_environment = configs['target_environment']
    if target_environment not in ['desktop', 'exynos']:
        print('[Error] Invalid target environment')
        exit()

    manager_thread = threading.Thread(target=experiment_manager, args=(main_thread_pid, ))    
    manager_thread.start()

    rospy.init_node('svl_validation', anonymous=True)
    rospy.Subscriber('imu_raw', Imu, imu_cb)
    rospy.Subscriber('twist_cmd', TwistStamped, twist_cmd_cb)
    rospy.spin()
