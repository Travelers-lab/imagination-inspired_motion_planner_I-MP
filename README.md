# motion_imagination_planner
Chengjin Wang, Yanmin Zhou, Zhipeng Wang, Zheng Yan, Feng Luan, Shuo Jiang, Runjie Shen, Hongrui Sang, Bin He
## Overview
This repo contains the imagination-inspired motion planner (I-MP) framework implementation for paper "A Reliable Robot Motion Planner in Complex Real-world Environments via Action Imagination".
## Content
* Installation
* About Configs and Logs
* Usage
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
**Configs:** all the specific parameters used for I-MP evaluation are indicated in `./config/planningConfig.json.` If you would like to play with other parameters, feel free to copy the existing config file and modify it. You will then just need to change the config file path in the following training steps to point to the new configuration file.
**Logs:** the  evaluation results will be saved in the data folder for each experiment. The data folders will be located under ./data folder. The last digit in the logs folder indicates the random seed. Inside the logs folder, the structure and contents are:
```
├── baseline_comparison_I-MP_based.csv
├── baseline_comparison_model_based.csv
├── baseline_comparison_probability_based.csv
├── baseline_comparison_simulation_based.csv
├── sensory_failure_experiments_Force_faulty_inoperable.csv
├── sensory_failure_experiments_Force_faulty_operable.csv
├── sensory_failure_experiments_None_faulty_inoperable.csv
├── sensory_failure_experiments_None_faulty_operable.csv
├── sensory_failure_experiments_Proximity_faulty_inoperable.csv
├── sensory_failure_experiments_Proximity_faulty_operable.csv
└── stress_testing_thread.csv
```
## Usage
**Planner Test:**
To test the I-MP planner in pybullet simulator, run the following command.
```
python planner_eval.py
```
**Baseline Comaprison:**
To conduct the baseline comparison with simulation-based, model-based, probability-based planning methods, you can either create a test scenario first or use the previously randomly generated one..
For the generation of random test scenarios, you can run the following commands.
```
cd ./testCondition
python testScenarioGeneration.py
```
To conduct the baseline comparison, you can run the following command.
```
python baselingComaprison.py
```
**Stress Testing:**
Similarly, you can first generate test scenarios for stress testing by running the following command.
```
cd ./testCondition
python stressTestGeneration.py
```
Next, perform the stress-testing simulation by executing the code below.
```
python I-MP_stress_testing.py
```
**Toplling Impact Testing:**
You can conduct the toppling inpact test by running the following coammand.
```
python I-MP_topllling_test.py
```
## License
This repository is released under the MIT license. See [LICENSE]() for additional details.
## Acknowledgments
We would like to thank the following individuals and organizations for their contributions to this project:
This work received support from the National Natural Science Foundation of China (No. 62088101), the Science and Technology Commission of Shanghai Municipality (No.2021SHZDZX0100, 22ZR1467100), and the Fundamental Research Funds for the Central Universities (No.22120240291).

