#!/bin/bash

python3 svl_auto_experiment.py

ssh root@192.168.0.8 killall a.out

mv /home/hayeonp/git/2023_Resource_Management_Research/experiment/yaml/svl_auto_experiment_configs.yaml /home/hayeonp/git/2023_Resource_Management_Research/experiment/yaml/svl_auto_experiment_configs1.yaml
mv /home/hayeonp/git/2023_Resource_Management_Research/experiment/yaml/svl_auto_experiment_configs2.yaml /home/hayeonp/git/2023_Resource_Management_Research/experiment/yaml/svl_auto_experiment_configs.yaml

python3 svl_auto_experiment.py
