# This is just a template 

## Install necessary dependencies
```
sudo apt install ros-humble-rosbridge-server
sudo apt install ros-humble-rosbridge-suite
sudo apt install libyaml-cpp-dev 
```

## Install cpr 
```
cd cpr && mkdir build && cd build
cmake .. -DCPR_USE_SYSTEM_CURL=ON
cmake --build . --parallel
sudo cmake --install .
```

## Remake 
```
./init_all.sh "<maintainer_name>" <maintainer_email>
```# 82-65-73-65-83
# 82-65-73-65-83
