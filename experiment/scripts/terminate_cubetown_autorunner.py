import os

output = str(os.popen('ps ax | grep cubetown').read())
output = output.split('\n')
for line in output:
    if '/opt/ros/melodic/lib/rubis_autorunner/cubetown_' in line:
        pid = line.split(' ')[2]
        os.system('kill -2 '+pid)
