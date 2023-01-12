import os
import subprocess
import threading
import time
import rospy
import yaml
from tqdm import tqdm
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

is_experiment_started = threading.Event()
is_scenario_started = threading.Event()
is_autorunner_started = threading.Event()
is_autorunner_killed = threading.Event()
barrier = threading.Barrier(3)

configs = {}

def rosbridge():
    print('- Execute rosbridge')
    os.system(configs['rosbridge_cmd'] + ' >> /dev/null')
    return

def svl_scenario():
    while True:
        time.sleep(1)
        if is_experiment_started.is_set():
            os.system(configs['svl_scenario_cmd'])
        else:
            continue
        barrier.wait()
    
    return

def autorunner():
    while True:
        time.sleep(1)
        if is_experiment_started.is_set() and is_scenario_started.is_set():
            if configs['autorunner_mode'] == 'LKAS': os.system(configs['cubetown_lkas_autorunner_cmd'])
            elif configs['autorunner_mode'] == 'FULL': os.system(configs['cubetown_full_autorunner_cmd'])
            else:
                print('Invalidate mode:', configs['autorunner_mode'])
        else:
            continue
        barrier.wait()

    return

def experiment_manager():
    # Threads
    rosbridge_thread = threading.Thread(target=rosbridge)
    svl_scenario_thread = threading.Thread(target=svl_scenario)
    autorunner_thread = threading.Thread(target=autorunner)

    # Start rosbridge
    rosbridge_thread.start()
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
            pbar.set_description(str(i+1)+'/'+str(configs['max_iteration']))
            time.sleep(1)
                
        # Terminate
        kill_svl_scenario()
        kill_autorunner()
        is_experiment_started.clear()
        is_autorunner_started.clear()
        is_scenario_started.clear()
        save_result(i)
        barrier.wait()
        barrier.reset()
    
    return

def save_result(iter):
    # Response time
    output_path = 'results/'+configs['experiment_title']+'/'+str(iter)
    os.system('mkdir '+output_path)
    os.system('cp -r '+configs['response_time_path']+ ' ' + output_path)

    return

def kill_autorunner():
    print('- Kill autorunner')
    p = subprocess.Popen(configs['termination_cmd'], shell=True, stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    p.wait()
    
    while True:
        time.sleep(1)
        _output = str(os.popen('rosnode list').read())
        if 'twist_gate' not in _output: break
    return

def kill_svl_scenario():
    output = str(os.popen('ps ax | grep CubetownBase.py').read())
    output = output.split('\n')
    for line in output:
        if configs['svl_scenario_path'] + '/CubetownBase.py' in line:
            pid = line.split()[0]
            os.system('kill -2 '+pid)

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
    with open('desktop_configs.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)

    if os.path.exists('results/'+configs['experiment_title']):
        print('Experiment result exists already')
        exit()
    os.mkdir('results/'+configs['experiment_title'])

    manager_thread = threading.Thread(target=experiment_manager)    
    manager_thread.start()

    rospy.init_node('svl_validation', anonymous=True)
    rospy.Subscriber('imu_raw', Imu, imu_cb)
    rospy.Subscriber('twist_cmd', TwistStamped, twist_cmd_cb)
    rospy.spin()
