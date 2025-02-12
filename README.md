# Project 1 Control fo Robotic Systems
link of article : https://ieeexplore.ieee.org/document/10272747
github: https://github.com/Jomaxdrill/POT_UAV_SWARM
## AUTHORS OF IMPLEMENTATION
jcrespo@umd.edu
rhamsaa@umd.edu

# AUTHORS OF THE ARTICLE
Tagir Muslimov, Rustem Munasypov, Evgeny Kozlov

## DEPENDENCIES and PACKAGES
python 3.11.7 or 3.8
(pip installer used)

## LIBRARIES
import math
import numpy as np
from itertools import combinations
import matplotlib.pyplot as plt

## GENERAL SETUP
-The general structure of the folder is the following
POT_UAV_SWARM/
```
├── .gitignore         # Git ignore file
├── curl_free.py       # control algorithm for potential field method using curl-free vector field for fixed obstacles
├── drone_swarm_tsting.ipynb # Testing functions and debugging
├── drone_swarm.py     # Main file of the simulation, here you can set initial positions, number of drones,etc..
├── hybrid.py          #control algorithm as a combination of the standard potential field and the curl-free vector field.
├── LICENSE            # MIT LICENSE
├── plotting.py        # Function using matplotlib to graph different results and information
├── potential_field.py # control algorithm using the standard potential field alogrithm for fixed obstacles
├── README.md          # Project overview and instructions
└── utilities.py       # Dependencies list
```
## RUNNING SCRIPT:
```sh
cd ~\ [Path to the POT_UAV_SWARM]
```

```sh
python3 drone_swarm.py
```

## CHANGE PARAMETERS

At the drone_swarm.py check for the variables at the beginning of the drone_swarm.py file and their commentaries to modify as required.
To modify control constants check the corresponding python file constants (check folder structure) at the beginning of the code.

## CHANGE BETWEEN ALGORITHMS

Use as reference the main variable ALGORITHM_SET at the drone_swarm.py and switch with the variable selection at the if __name__ == "__main__" section


-In case of missing libraries you can use pip to install them:
```sh
pip install NAME_OF_LIBRARY
pip install numpy
pip install matplotlib
```
-When running it will provide a series of succesive plots as you close them as the trajectories, short distances to obstacle, and positions versus time.
![figure7](https://github.com/user-attachments/assets/4f7efcbf-8532-4dd3-8c54-d09b690915f9)


