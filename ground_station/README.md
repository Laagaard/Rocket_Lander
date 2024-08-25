# `ground_station`

## Dependencies
* `wxWidgets` (taken care of by cloning the `Rocket_Lander` project with `git clone --recurse-submodules https://github.com/Laagaard/Rocket_Lander.git`)
* [`make`](https://gnuwin32.sourceforge.net/packages/make.htm)
* [`cmake`](https://cmake.org/download/)

## Setup
1. Follow the "Using the CMake GUI" steps [here](https://docs.wxwidgets.org/latest/overview_cmake.html#cmake_options)
2. Run `cmake -G "Unix Makefiles"`

## Build & Run
* Run `make build`

    * This will create a `build` directory for the build files (if one does not already exist)
    * The executable will be placed in the `build` directory

* Run `make run`

    * This will run the application