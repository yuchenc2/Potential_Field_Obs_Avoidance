# Project Title

HTC Vive Eye Pro with Mujoco

## Description

Show the location where the user is looking at in mujoco VR.

## Dependencies
* Linux 18.04
* MuJoCo200
* [SteamVR](https://store.steampowered.com/app/250820/SteamVR/)
* [SRanipal Runtime (1.3.2.0)](https://developer.vive.com/resources/vive-sense/eye-and-facial-tracking-sdk/download/latest/): Make sure to install SRanipal after SteamVR!

# Getting Started

## Setup
1. Clone this repo to your computer
2. Install SRanipal and SteamVR
3. Get a [license key for mujoco](https://www.roboti.us/license.html) and put it in the `mujoco200/bin` folder
4. Run makefile in `mujoco-htcvive/src/` to build `.exe` files in the `mujoco200/bin/` folder
    * Open VS2019 x64 Native Tools Command Prompt
    * Navigate to `src/` directory in repo
    * Run `nmake -f makefile`

## Running the code
* Make sure your Vive headset is turned on before running the code
* Navigate to `mujoco200/bin` and double click on the minivive_drivecar.exe file 
* The view from the headset will appear in a window on the screen. You can now use the Vive to interact with the MuJoCo model.

## Notes
* `mjvive.py` currently got a black screen issue, use mjvive.cpp instead if you want to use the Mujoco API
* `minivive.cpp` is `mjvive.cpp` without controllers

## Main code files
* mujoco-htcvive/src/minivive_drivecar.cpp: main file for mujoco and HTC vive setup
* mujoco200/model/one_car.xml: main xml file for robot car and virtual environment

## Acknowledgement
* Based on https://github.com/thomasweng15/vive-mujoco and https://github.com/openai/mujoco-py/blob/master/examples/mjvive.py
* [HTC Vive Pro Eye Documentation](https://developer.vive.com/resources/vive-sense/eye-and-facial-tracking-sdk/)
