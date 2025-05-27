# motion_imagination_planner
Chengjin Wang, Yanmin Zhou, Zhipeng Wang, Zheng Yan, Feng Luan, Shuo Jiang, Runjie Shen, Hongrui Sang, Bin He
## Overview
This repo contains the imagination-inspired motion planner (I-MP) framework implementation for paper "A Reliable Robot Motion Planner in Complex Real-world Environments via Action Imagination".
## Content
* Installation
* About Configs and Logs
* Algorithm Tests
* Baseline Comparison
* Troppling Tests
* Stress Tests
* Sensory Ablation Tests
## Installation
Our code has been tested on Ubuntu 18.04 and Ubuntu 20.04 with python 3.10 virtual environment and dependencies.
```
conda create -n i-mp python=3.10
conda activate i-mp
pip install -r requirements.txt
```
## About Configs and Logs
Before evaluation, we first introduce the configuration and logging structure.
Configs: all the specific parameters used for I-MP evaluation are indicated in ./config/planningConfig.json. If you would like to play with other parameters, feel free to copy the existing config file and modify it. You will then just need to change the config file path in the following training steps to point to the new configuration file.

