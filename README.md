# walking-controllers
The **walking-controllers** project is a _suite_ of modules for achieving bipedal locomotion of the humanoid robot iCub.

The suite includes:

* **Walking_module**: this is the main module and it implements all the controller architecture that allows iCub to walk.
* **Joypad_module**: this module allows using the Joypad as reference input for the trajectory generated by the Walking Module
* **WalkingLogger_module**: an module that can be useful to dump data coming from the Walking Module

# Overview
 - [:orange_book: Some theory behind the code](#orange_book-some-theory-behind-the-code)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)
 - [:hammer: Build the suite](#hammer-build-the-suite)
 - [:computer: How to run the simulation](#computer-how-to-run-the-simulation)
 - [:running: How to test on iCub](#running-how-to-test-on-icub)

# :orange_book: Some theory behind the code
This module allows iCub walking using the position control interface.
It implements the following architecture
![controller_architecture](https://user-images.githubusercontent.com/16744101/37352869-757896b4-26de-11e8-97bc-10700add7759.jpg)
where two controller loops can be distinguished:
* an inner ZMP-CoM control loop http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4359259&tag=1;
* an outer DCM control loop:
   * model predictive controller;
   * reactive controller.

Two different inverse kinematics solver are implemented:
* a standard non-linear IK solver;
* a standard QP Jacobian based IK solver.

## Reference paper
A paper describing some of the algorithms implemented in this repository  can be downloaded [here](https://arxiv.org/abs/1809.02167).
If you're going to use the code developed for your work, please quote it within any resulting publication:
```
G. Romualdi, S. Dafarra, Y. Hu, D. Pucci "A Benchmarking of DCM Based
Architectures for Position and Velocity Controlled Walking of Humanoid Robots",
2018
```

The bibtex code for including this citation is provided:
```
@misc{1809.02167,
Author = {Giulio Romualdi and Stefano Dafarra and Yue Hu and Daniele Pucci},
Title = {A Benchmarking of DCM Based Architectures for Position and Velocity Controlled Walking of Humanoid Robots},
Year = {2018},
Eprint = {arXiv:1809.02167},
}
```

# :page_facing_up: Dependencies
* [YARP](http://www.yarp.it/): to handle the comunication with the robot;
* [iDynTree](http://wiki.icub.org/codyco/dox/html/idyntree/html/): to handle the robot kinematics;
* [iCubContrib](https://github.com/robotology/icub-contrib-common): to configure the modules;
* [osqp-eigen](https://github.com/robotology/osqp-eigen): to solve the MPC problem;
* [qpOASES](https://github.com/robotology-dependencies/qpOASES): to solve the IK problem;
* [Unicycle footstep planner](https://github.com/robotology/unicycle-footstep-planner/tree/dcmTrajectoryGenerator): to generate a trajectory for the DCM;
* [Gazebo](http://gazebosim.org/): for the simulation (tested Gazebo 8, 9 and 10);
* [Catch2](https://github.com/catchorg/Catch2): to compile the tests.

# :hammer: Build the suite
## Linux/macOs

```sh
git clone https://github.com/robotology/walking-controllers.git
cd walking-controllers
mkdir build && cd build
cmake ../
make
[sudo] make install
```
Notice: `sudo` is not necessary if you specify the `CMAKE_INSTALL_PREFIX`. In this case it is necessary to add in the `.bashrc` or `.bash_profile` the following lines:
```sh
export WalkingControllers_INSTALL_DIR=/path/where/you/installed/
export PATH=$PATH:$WalkingControllers_INSTALL_DIR/bin
export YARP_DATA_DIRS=$YARP_DATA_DIRS:$WalkingControllers_INSTALL_DIR/share/yarp
```

# :computer: How to run the simulation
1. Set the `YARP_ROBOT_NAME` environment variable according to the chosen Gazebo model:
   ```sh
   export YARP_ROBOT_NAME="icubGazeboSim"
   ```
2. Run `yarpserver`
   ``` sh
   yarpserver --write
   ```
3. Run gazebo and drag and drop iCub (e.g. icubGazeboSim or iCubGazeboV2_5):

    ``` sh
    gazebo -slibgazebo_yarp_clock.so
    ```
4. Run `yarprobotinterface`

    ``` sh
     YARP_CLOCK=/clock yarprobotinterface --config launch-wholebodydynamics.xml
    ```
5. Reset the offset of the FT sensors. Open a terminal and write

   ```
   yarp rpc /wholeBodyDynamics/rpc
   >> resetOffset all
   ```
6. Run the walking module
    ``` sh
    YARP_CLOCK=/clock WalkingModule
    ```
7. communicate with the `WalkingModule`:
   ```
   yarp rpc /walking-coordinator/rpc
   ```
   the following commands are allowed:
   * `prepareRobot`: put iCub in the home position;
   * `startWalking`: run the controller;
   * `pauseWalking`: the controller is paused, you can start again the
     controller sending `startWalking` command;
   * `stopWalking`: the controller is stopped, in order to start again the
     controller you have to prepare again the robot.
   * `setGoal x y`: send the desired final position, `x` and `y` are doubles expressed in iCub fixed frame, in meters. Send this command after `startWalking`.

   Example sequence:
   ```
   prepareRobot
   startWalking
   setGoal 1.0 0.0
   setGoal 1.0 0.0
   stopWalking
   ```

## How to run the Joypad Module
The Joypad application, called `WalkingJoypadModule`, allows you to send all the rpc commands using the buttons. The application processes the button press events associating them to the pre-defined rpc commands which are then sent through Yarp to the Walking Coordinator module. The joypad keys mapping is as follows:
 * `A` for preparing the robot
 * `B` for start walking
 * `Y` for pause walking
 * `X` for stop walking

Suppose that you want to run the Joypad application, called `WalkingJoypadModule` in the same machine where the physical device is connected. The only thing that you have to do is running the following command from the terminal:

``` sh
YARP_CLOCK=/clock WalkingJoypadModule
```
The application will take care to open an [`SDLJoypad`](http://www.yarp.it/classyarp_1_1dev_1_1SDLJoypad.html) device.


While, if you want to run the `WalkingJoypadModule` in a machine that is different form the one where the physical devce is connected. The
[`JoypadControlServer`](http://www.yarp.it/classyarp_1_1dev_1_1JoypadControlServer.html) -
[`JoypadControlClient`](http://www.yarp.it/classyarp_1_1dev_1_1JoypadControlClient.html)
architecture is required. In details:
1. Run the `JoypadControlServer` device in the computer where the joypad is
   physically connected:

    ``` sh
    YARP_CLOCK=/clock yarpdev --device JoypadControlServer --use_separate_ports 1 --period 10 --name /joypadDevice/xbox --subdevice SDLJoypad --sticks 0

    ```
2. Run the `WalkingJoypadModule` in the other computer

    ```
    YARP_CLOCK=/clock WalkingJoypadModule --device JoypadControlClient --local /joypadInput --remote /joypadDevice/xbox
    ```

## How to dump data
Before running `WalkingModule` check if `dump_data` is set to 1. This parameter is set in a configuration `ini` file depending on the control mode:
* controlling from the joypad: `src/WalkingModule/app/robots/${YARP_ROBOT_NAME}/dcm_walking_with_joypad.ini`. [Example for the model `iCubGazeboV2_5`](src/WalkingModule/app/robots/iCubGazeboV2_5/dcm_walking_with_joypad.ini#L12)
* control using the human retargeting: in the same folder `src/WalkingModule/app/robots/${YARP_ROBOT_NAME}`, the configuration files `dcm_walking_hand_retargeting.ini` and `dcm_walking_joint_retargeting.ini`.

Run the Logger Module `WalkingLoggerModule` before the Walking Module `WalkingModule`.
``` sh
YARP_CLOCK=/clock WalkingLoggerModule
```

All the data will be saved in the current folder inside a `txt` named `Dataset_YYYY_MM_DD_HH_MM_SS.txt`

## Some interesting parameters
You can change the DCM controller and the inverse kinematics solver by editing [these parameters](src/WalkingModule/app/robots/iCubGazeboV2_5/dcm_walking_with_joypad.ini#L22-L57).

### Inverse Kinematics configuration
The Inverse Kinematics block configuration can be set through the file src/WalkingModule/app/robots/iCubGazeboV2_5/dcm_walking/joint_retargeting/inverseKinematics.ini.

The Inverse Kinematics block uses an open source package for large-scale optimisation, IPOPT (Interior Point Optimizer), which requires other packages like BLAS (Basic Linear Algebra Sub-routines), LAPACK (Linear Algebra PACKage) and a sparse symmetric indefinite linear solver ([MAxx, HSLMAxx](http://www.hsl.rl.ac.uk/), MUMPS, PARDISO etc). Further documentation can be found at https://coin-or.github.io/Ipopt and https://coin-or.github.io/Ipopt/INSTALL.html#EXTERNALCODE. The package IPOPT installed with the superbuild (via homebrew or conda) is built with the solver [MUMPS](http://mumps.enseeiht.fr/) by default, which is reflected in the default configuration of the Inverse Kinematics block src/WalkingModule/app/robots/iCubGazeboV2_5/dcm_walking/joypad_control/inverseKinematics.ini#L14-L17:
```sh
# solver paramenters
solver-verbosity        0
solver_name             mumps
max-cpu-time            20
```

For instance, for using MA27 solver instead of MUMPS, replace `mumps` by `ma27`.

:warning: HSL solvers are not compiled with IPOPT by default. Refer to https://coin-or.github.io/Ipopt/INSTALL.html#EXTERNALCODE for further documentation.

In case you encounter issues when starting the Walking Module with the selected options, you can increase the verbosity to 1 for additional debug information.


# :running: How to test on iCub
You can follows the same instructions of the simulation section without using `YARP_CLOCK=/clock`. Make sure your `YARP_ROBOT_NAME` is coherent with the name of the robot (e.g. iCubGenova04)
## :warning: Warning
Currently the supported robots are only:
- ``iCubGenova04``
- ``icubGazeboSim``
- ``icubGazeboV2_5``

Yet, it is possible to use these controllers provided that the robot has V2.5 legs. In this case, the user should define the robot specific configuration files (those of ``iCubGenova04`` are a good starting point).

:warning: The STRAIN F/T sensors normally mounted on iCub may suffer from saturations due to the strong impacts the robot has with the ground, which may lead to a failure of the controller. It is suggested to use these controllers with STRAIN2 sensors only (as in ``iCubGenova04``) to avoid such saturations.
