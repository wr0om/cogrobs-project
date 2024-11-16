# For Experiments
SEED = 2 # 1-3 for each experiment

# enemy drone
INPLACE = False
MOVING = True
ADVERSARIAL = True
OBSTACLES = True

# planning
PLANNING_EXPERIMENT = 2 # 1-4 for each experiment (without 3!!!)

if PLANNING_EXPERIMENT == 1:
    USE_RANDOM = True
    USE_CENTROIDS = False
    USE_RADIUS = False
elif PLANNING_EXPERIMENT == 2:
    USE_RANDOM = False
    USE_CENTROIDS = False
    USE_RADIUS = False
elif PLANNING_EXPERIMENT == 3: # DEPRECATED
    USE_RANDOM = False
    USE_CENTROIDS = True
    USE_RADIUS = False
elif PLANNING_EXPERIMENT == 4:
    USE_RANDOM = False
    USE_CENTROIDS = False
    USE_RADIUS = True

# for ablation study
NUMBER_OF_ENEMY_DRONES_STUDY = False

# Channels
CPU_CHANNEL = 0
DRONE_CHANNEL = 1
ENEMY_DRONE_CHANNEL = 2

EPSILON = 0.5
LAMBDA = 30