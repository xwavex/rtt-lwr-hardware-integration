# rtt-lwr-hardware-integration
RTT Component integrating the FRI library, with RTNET providing real-time communication, into the orocos framework

## Features
- POSITION CONTROL MODE (Not implemented yet)
- TORQUE CONTROL MODE
  - Through joint impedance control mode, gravity is compensated through the KRC unit!

## Install

`mkdir build`

`cd build`

// for orocos plugins

`export PKG_CONFIG_PATH=$insert-prefix-here/lib/pkgconfig`

`cmake -DOROCOS-RTT_DIR=$insert-prefix-here/lib/cmake/orocos-rtt ..`

`make -j 4`

// include RTT component in search path

`export RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:$include-path-to/rtt-lwr-hardware-integration/build/orocos`

## TODO

- check FRI interface to make sure communication is done through OROCOS interfaces rather than straight to xenomai
