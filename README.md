# 🚁 Drone Combat Simulation

A Webots-based simulation project featuring an autonomous drone that engages in combat with enemy drones using PID controllers and planning algorithms.

![Webots R2023b Required](https://img.shields.io/badge/Webots-R2023b-blue)
![Python 3.8.8](https://img.shields.io/badge/Python-3.8.8-green)

## 🎯 Overview

This project simulates drone combat scenarios where a controlled drone must neutralize enemy drones with varying levels of complexity. The system employs:
- PID controllers for precise drone movement
- Advanced planning algorithms for tactical decision-making
- Customizable enemy drone behaviors
- Multiple simulation environments (with/without obstacles)

## 🛠️ Installation

1. Clone this repository:
```bash
git clone https://github.com/wr0om/cogrobs-project.git
cd cogrobs-project
```

2. Install required packages:
```bash
pip install -r requirements.txt
```

3. Ensure Webots R2023b is installed on your system

## 🎮 Running Simulations

### Basic Usage
1. Open Webots and load one of the world files from the `worlds` folder:
   - `base_world.wbt` - Standard environment
   - `with_obstacles.wbt` - Environment with obstacles

2. Click the play button in Webots to start the simulation.
3. When all enemy drones are neutralized, the simulation will stop, results will be saved, and a visualization of the drone's path and eliminations will be displayed and saved as a PNG file in the `controllers/experiments/` folder.

### Customizing Experiments

Navigate to `controllers/my_utils/classes_and_constants.py` to modify simulation parameters:

#### Core Settings
- `SEED` (int): Controls randomization of drone spawns and decisions

#### Enemy Drone Behavior
- `INPLACE` (bool): Static position vs. spherical movement
- `MOVING` (bool): Enable position changes via Poisson process
- `ADVERSARIAL` (bool): Enable evasive maneuvers
- `OBSTACLES` (bool): Add obstacles (iff `with_obstacles.wbt`)

#### Planning Algorithms
- `USE_RANDOM` (bool): Enable random action selection
- `USE_RADIUS` (bool): Use radius-based planning
- `PLANNING_EXPERIMENT` (1-4): Select planning strategy:
  1. Random planner
  2. Optimal planner
  3. Centroid planner (Deprecated)
  4. Radius planner

## 📊 Ablation Studies

To analyze performance with different numbers of enemy drones:

1. In `classes_and_constants.py`, set:
   - `NUMBER_OF_ENEMY_DRONES_STUDY = True`
   - Configure desired `SEED`

2. Modify enemy drone count in Webots:
   - Delete/add drones using the scene tree
   - Ensure proper naming: `EnemyDrone1`, `EnemyDrone2`, etc.

3. Run the simulation

## 📈 Results Analysis

Results are automatically saved in `controllers/experiments/`:
- `all_results.csv`: Standard experiment results
- `all_results_study.csv`: Ablation study results

### Viewing Results
Use `controllers/experiments/give_raw_results.ipynb` to:
- View aggregated results tables
- Generate visualizations
- Analyze ablation study metrics

## 🙏 Acknowledgments

- [Crazyflie Simulation](https://github.com/bitcraze/crazyflie-simulation) - Drone controller implementation
- [Cogrobs Project](https://github.com/Noam-Sasson/cogrobs-project) - Controller optimization assistance

## 📝 License

This project is part of academic research. Feel free to use and modify the code, but please provide appropriate attribution.

## 📧 Contact

For questions or issues, please [open an issue](https://github.com/wr0om/cogrobs-project/issues) on the GitHub repository.