# vins_source

This repository has all drivers needed for all rpng vins systems. Please clone this repository instead of individual repositories. Add launch files into the `vins_launch` package folder for the different launch types.

## Installing

* Clone this repository with submodules:
    * `git clone --recursive https://github.com/rpng/vins_source.git`
* If this repository is already cloned download submodules
    * `cd vins_source`
    * `git submodule update --init --recursive`
* To add a new driver package (as a submodule)
    * `git submodule add <url-to-git-repo>`