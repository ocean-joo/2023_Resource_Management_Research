import subprocess
import os
import signal
import time

subprocess.Popen('python3 scripts/write_center_offset.py', shell=True, executable='/bin/bash')

time.sleep(3)

_output = str(os.popen('ps au | grep center_offset').read())
_output = _output.split('\n')
for line in _output:    
    if not 'write_center_offset.py' in line: continue
    pid = -1
    for v in line.split(' '):
        try: pid = int(v)
        except: continue
        break

    if pid != -1: os.kill(pid, signal.SIGINT)