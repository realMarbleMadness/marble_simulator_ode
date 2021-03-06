# Marble Madness - Simulator Node

- [Marble Madness - Simulator Node](#marble-madness---simulator-node)
    - [How to install all the dependencies](#how-to-install-all-the-dependencies)
    - [Parameters](#parameters)
    - [See the demo](#see-the-demo)
    - [Extra notes about the code](#extra-notes-about-the-code)

The main code we need to care about is stored in the `/ball1` folder. Detailed explanation see `notes.txt`.

## How to install all the dependencies

1. Download [ode](https://sourceforge.net/projects/opende/files/) and put it somewhere (hopefully not your `/Downloads` folder)
2. `cd` into the ode folder, then run `./configure --enable-double-precision`. Note the double precision here that takes me (Yanda) an hour to debug
3. edit `drawstuff/src/drawstuff.cpp`, change line #79 to `#define LIGHTY (-0.4f)`
4. `make` then `sudo make install`
5. `sudo cp -rp include/drawstuff /usr/local/include/`
6. `cd drawstuff/src/`
7. `sudo cp libdrawstuff.la .libs/libdrawstuff.a /usr/local/lib/`

## Parameters

- `time_step`: the time step for the simulator to go forward. If it is too small, it will miss some collision. If it's too large, it will be too slow
- `obstacle_file`: the path to the json file that contains the information about the whole world setup. This json file should be created by the box-2d optimizer.

## See the demo

1. `git clone` this repo into your catkin workspace `/src` folder
2. `cd` into catkin workspace, then `catkin_make` 
3. `roslaunch marble_simulator_ode default_simulator.launch`

## Extra notes about the code
- the `CMakeLists.txt` is a masterpiece