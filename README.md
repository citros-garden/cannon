# Cannon

This is a ROS implementation of the [cannonball simulation](https://nasa.github.io/trick/tutorial/ATutASimpleSim) provided by NASA Johnson Space Center as part of the tutorial for the 
[Trick Simulation Environment](https://nasa.github.io/trick/).

It determines the trajectory and time of impact of a cannon ball that is fired with an initial speed and initial angle, assuming a constant acceleration of gravity (g), and no aerodynamic forces.

![Cannonball](CannonInit.png "Cannonball")

Two versions of the simulation are provided: an analytic solution and a numeric integration solution.

## Prerequisites

- [Visual Studio code](https://code.visualstudio.com/download)
- [Docker](https://www.docker.com/)
- [Foxglove](https://foxglove.dev/) (optional)

## Installation

        git clone git@github.com:citros-garden/citros_cannon.git
        cd ~/citros_cannon
        code .
and open the repository inside a container using VScode's *reopen in container* option.

## Build 
        colcon build
        source install/local_setup.bash

## Run the analytic solution
        ros2 launch scheduler cannon_analytic.launch.py

## Run the numeric integration solution
        ros2 launch scheduler cannon_numeric.launch.py


Running either of the two simulations will result in the logger output being written to the console.

## Implementation Overview
The project is made out of three ROS nodes - `cannon_analytic`, `cannon_numeric` and `scheduler`. The scheduler node is responsible for driving the simulation by publishing a `scheduler` topic at a given rate (say, 100Hz). The cannon nodes subscribe to this topic, and upon receiving it perform a single calculation step. The rate (`dt`) is a ROS parameter for the scheduler node, which means you may change its value in the `config/params.yaml` file, without the need to recompile. The two cannon nodes also have `params.yaml` files of their own, in which you can set the initial speed and angle, and also the time/integration delta (`dt`).

Additionally, the `scheduler` node subscribes to a `debug` topic, which, together with the provided Foxglove layout, facilitates a play/pause/step/resume functionality. 

## Foxglove
To view a graphical representation of the simulation, you can open [Foxglove](https://foxglove.dev/) and load the `CITROS_Cannon.json` layout file, or create your own layout.

It is recommended to start the simulation in a paused state, and then, once your foxglove layout is ready, resume it via the Play/Pause button. 

To do that, in the `__init__` member function of the `scheduler` node (in `scheduler.py`), change the line

        self.debug_mode = False

to

        self.debug_mode = True
You will need to build (and source) again.

Output example:
![Foxglove screenshot](foxglove_screenshot.png)


## Production Docker
```bash
docker build -t cannon .

# *** when building from MAC M1 chip add FROM --platform=linux/amd64 ***
docker buildx build --platform linux/amd64 -t lulav_wespace_sim .   


citros docker-login
docker tag cannon us-central1-docker.pkg.dev/citros/lulav/cannon
docker push us-central1-docker.pkg.dev/citros/lulav/cannon
```