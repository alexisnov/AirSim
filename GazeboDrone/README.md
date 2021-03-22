# General

Derived from http://gazebosim.org/tutorials?tut=topics_subscribed

Make sure you have installed gazebo dependencies

```
sudo apt-get install libgazebo9-dev
```

# AirLib build

This project is built with g++, so AirLib needs to be built with g++ too. Change lines 56 and 57 of AirSim/build.sh with:
```
export CC="gcc-8"
export CXX="g++-8"
```
then run ./setup.sh and ./build.sh

# AirSim plugin
The AirSim UE plugin needs to be built with clang. Clone AirSim again in another folder without the above change. From that one, you can run the Blocks environment.

# Build

```
mkdir build && cd build
cmake ..
make
```

# Run

```
cd build
./GazeboDrone
```

