# This is for PX4 VTOL simulation
```shell
rm ~/PX4-Autopilot/Tools/simulation/gz -fr
git clone git@github.com:SYSU-HILAB/VTOL-gz-simulation.git ~/PX4-Autopilot/Tools/simulation/gz
```
# use
```shell
cmake -Bbuild -S.
cmake --build build
cmake --install build
```
then the libAerodynamicsPlugin.so will be installed in ~/.gz/sim/plugins/
it will be correctly found by gazebo
