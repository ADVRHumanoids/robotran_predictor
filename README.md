robotran_predictor
===============

## Installation ##

### Install walkman_robotran simulator ###

This module is based on the walkman robotran simulator.
To install it you can follow instruction from https://git.immc.ucl.ac.be/habra/walkman_robotran/edit/master/README.md

### compile the module ###

```
mkdir build
cd build
ccmake ..
#indicate the MBSysC libraries path (compiled in the installation of walkman_robotran simulator)
make
```

## Set up the configuration file ###

In order to run this module, you should provide it the path to the walkman_robotran project. To this end you should adapt [robotran_predictor_initial_config.ini](https://gitlab.robotology.eu/walkman-drc/robotran_predictor/blob/master/app/conf/robotran_predictor_initial_config.ini) to indicate your mbs path.

