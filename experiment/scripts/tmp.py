import os

output = str(os.popen('ps -ax | grep cube'))
output = output.split('\n')
for line in output:
    for v in line.split(' '):
        try:
            pid = str(int(v))
            os.system('kill -9 '+pid)
        except:
            continue

