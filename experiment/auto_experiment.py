import os
import subprocess
import threading
import multiprocessing
import time
import rospy
from tqdm import tqdm
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

MODE = 'LKAS' # LKAS, FULL
ITERATION = 3
TIME = 3


is_experiment_started = threading.Event()
is_scenario_started = threading.Event()
is_autorunner_started = threading.Event()
is_autorunner_killed = threading.Event()
barrier = threading.Barrier(3)

def rosbridge():
    print('- Execute rosbridge')
    os.system('roslaunch /home/hayeonp/rosbridge_websocket.launch >> /dev/null')
    return

def svl_scenario():
    while True:
        time.sleep(1)
        if is_experiment_started.is_set():
            os.system('python3 /home/hayeonp/git/lgsvl-demo/CubetownBase.py')
        else:
            continue
        barrier.wait()
    
    return

def autorunner():
    while True:
        time.sleep(1)
        if is_experiment_started.is_set() and is_scenario_started.is_set():
            if MODE == 'LKAS': os.system('roslaunch rubis_autorunner cubetown_lkas_autorunner.launch')
            elif MODE == 'FULL': os.system('roslaunch rubis_autorunner cubetown_full_autorunner.launch')
            else:
                print('Invalidate MODE:',MODE)
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
    
    for i in range(ITERATION):  
        is_rosbridge_on = False
        is_experiment_started.set()
        while True:
            _output = str(os.popen('rosnode list').read())
            if 'rosbridge_websocket' in _output: is_rosbridge_on = True
            else: is_rosbridge_on = False

            if is_rosbridge_on and is_scenario_started.is_set(): break
                
        while not is_autorunner_started.is_set():
            time.sleep(1)

        pbar = tqdm(range(TIME))
        for _ in pbar:
            pbar.set_description(str(i+1)+'/'+str(ITERATION))
            time.sleep(1)
                
        # Terminate
        kill_svl_scenario()
        kill_autorunner()
        is_experiment_started.clear()
        is_autorunner_started.clear()
        is_scenario_started.clear()
        barrier.wait()
        barrier.reset()
    
    return

def kill_autorunner():
    print('- Kill autorunner')
    p = subprocess.Popen('./terminate.sh', shell=True, stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
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
        if '/home/hayeonp/git/lgsvl-demo/CubetownBase.py' in line:
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
    manager_thread = threading.Thread(target=experiment_manager)    
    manager_thread.start()

    rospy.init_node('svl_validation', anonymous=True)
    rospy.Subscriber('imu_raw', Imu, imu_cb)
    rospy.Subscriber('twist_cmd', TwistStamped, twist_cmd_cb)
    rospy.spin()
