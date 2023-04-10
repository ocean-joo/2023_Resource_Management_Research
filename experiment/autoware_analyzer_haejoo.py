import yaml
import matplotlib.pyplot as plt

import os
import scripts.autoware_analyzer_lib as aa
import numpy as np
import copy

EXPERIMENT = '5'
FIG_SIZE = (10,6)

roi_range =[0.27, 0.91]

video = {
    '5': [67, 62],
    '7': [31, 51],
}

with open('yaml/autoware_analyzer.yaml') as f:
    configs = yaml.load(f, Loader=yaml.FullLoader)

chain_info = configs['node_chain']
avoidance_x_range = configs['avoidance_x_range']
target_speed = configs['target_speed']