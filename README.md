# bayes_id
Bayesian Inference based System Identification

STAT 927 Final Project - Spring 2022

## Dependencies
Install the dependencies in a virtualenv with

```bash
cd bayes_id
virtualenv bayes_env
pip install -r requirements.txt
```

## Directories and Files
1. `traj_collector/` is the ROS 2 package that runs on F1TENTH vehicles to collect hardware data.
2. `data/saved_traj.npz` is the collected data from hardware.
3. `model.py` includes all vehicle model utilities.
4. `gibbs.py` is the Gibbs Sampling script.