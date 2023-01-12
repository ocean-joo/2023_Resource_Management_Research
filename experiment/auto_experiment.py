import os
import subprocess
import threading
import time
import rospy
import yaml
from tqdm import tqdm
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
import slack_library

is_experiment_started = threading.Event()
is_scenario_started = threading.Event()
is_autorunner_started = threading.Event()
is_autorunner_killed = threading.Event()
is_experiment_finished = threading.Event()
barrier = threading.Barrier(3)

configs = {}
target_environment = 'null'
slack_webhook = 'null'

def svl_scenario():
    while True:
        time.sleep(1)
        if is_experiment_started.is_set():
            print('- Execute SVL Secnario')
            os.system(configs[target_environment]['svl_scenario_cmd'])
        else:
            continue
        print('- SVL scenario is killed')
        if is_experiment_finished.is_set(): break
        barrier.wait()

    print('- Turn off SVL scenario thread')

    return

def autorunner():
    while True:
        time.sleep(1)
        if is_experiment_started.is_set() and is_scenario_started.is_set():
            if configs['autorunner_mode'] == 'LKAS': os.system(configs[target_environment]['cubetown_lkas_autorunner_cmd'])
            elif configs['autorunner_mode'] == 'FULL': os.system(configs[target_environment]['cubetown_full_autorunner_cmd'])
            else:
                print('Invalidate mode:', configs['autorunner_mode'])
        else:
            continue
        print('- Autorunner is killed')
        if is_experiment_finished.is_set(): break
        barrier.wait()

    print('- Turn off Autorunner thread')

    return

def experiment_manager():
    # Threads
    svl_scenario_thread = threading.Thread(target=svl_scenario)
    autorunner_thread = threading.Thread(target=autorunner)

    # Start rosbridge
    svl_scenario_thread.start()    
    autorunner_thread.start()
    
    for i in range(configs['max_iteration']):  
        is_rosbridge_on = False
        is_experiment_started.set()
        while True:
            _output = str(os.popen('rosnode list').read())
            if 'rosbridge_websocket' in _output: is_rosbridge_on = True
            else: is_rosbridge_on = False

            if is_rosbridge_on and is_scenario_started.is_set(): break
                
        while not is_autorunner_started.is_set():
            time.sleep(1)

        pbar = tqdm(range(configs['duration']))
        for _ in pbar:
            pbar.set_description('Duration: ' + str(i+1)+'/'+str(configs['max_iteration']))
            time.sleep(1)

        if i+1 == int(configs['max_iteration']): is_experiment_finished.set()
        
        # Terminate
        kill_svl_scenario()
        kill_autorunner()
        is_experiment_started.clear()
        is_autorunner_started.clear()
        is_scenario_started.clear()
        save_result(i)        

        if is_experiment_finished.is_set(): break
        
        barrier.wait()
        barrier.reset()                    
    
    message = 'Experiment is finished: '+configs['experiment_title']
    payload = {"text": message}

    slack_library.send_slack_message(payload, slack_webhook)

    kill_autorunner()

    exit()

    return

def save_result(iter):
    # Response time
    output_path = 'results/'+configs['experiment_title']+'/'+str(iter)
    os.system('mkdir '+output_path)
    if configs['target_environment'] == 'desktop':
        os.system('cp -r '+configs['target_environment']['response_time_path']+ ' ' + output_path)
    elif configs['target_environment'] == 'exynos':
        os.system('scp -r root@192.168.0.8:'+configs[target_environment]['response_time_path']+ ' ' + output_path)
    else:
        print('[Error] Invalid target environment:',configs['target_envirnment'])

    return

def kill_autorunner():
    p = subprocess.Popen(configs[target_environment]['termination_cmd'], shell=True, stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    p.wait()
    
    while True:
        time.sleep(1)
        _output = str(os.popen('rosnode list').read())
        if 'ndt_matching' not in _output: break
    return

def kill_svl_scenario():
    output = str(os.popen('ps ax | grep CubetownBase.py').read())
    output = output.split('\n')
    for line in output:
        if configs['svl_scenario_path'] + '/CubetownBase.py' in line:
            pid = line.split()[0]
            os.system('kill -9 '+pid)

    return

def kill_rosbridge():
    print('- Kill rosbridge')

    output = str(os.popen('ps ax | grep rosbridge_websocket.launch').read())
    output = output.split('\n')
    for line in output:
        if '/usr/bin/python /opt/ros/melodic/bin/roslaunch' in line:
            pid = line.split()[0]
            os.system('kill -2 '+pid)
            return True
    return False

def imu_cb(msg):
    if is_experiment_started.is_set():
        is_scenario_started.set()
    else:
        is_scenario_started.clear()
    return

def twist_cmd_cb(msg):
    if is_experiment_started.is_set():
        is_autorunner_started.set()
    else:
        is_autorunner_started.clear()
    return

if __name__ == '__main__':
    slack_webhook = slack_library.get_slack_webhook()

    with open('configs.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)

    if os.path.exists('results/'+configs['experiment_title']):
        print('[Error] Experiment result exists already')
        exit()
    os.mkdir('results/'+configs['experiment_title'])

    target_environment = configs['target_environment']
    if target_environment not in ['desktop', 'exynos']:
        print('[Error] Invalid target environment')
        exit()

    manager_thread = threading.Thread(target=experiment_manager)    
    manager_thread.start()

    rospy.init_node('svl_validation', anonymous=True)
    rospy.Subscriber('imu_raw', Imu, imu_cb)
    rospy.Subscriber('twist_cmd', TwistStamped, twist_cmd_cb)
    rospy.spin()
