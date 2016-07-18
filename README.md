# rtt-lwr-hardware-integration
RTT Component integrating the FRI library, with RTNET providing real-time communication, into the orocos framework.
Specifically designed using kinematic chains for use with [rtt-core-extensions](https://github.com/corlab/rtt-core-extensions).

## Features
- POSITION CONTROL MODE
- TORQUE CONTROL MODE
  - Through joint impedance control mode, gravity is compensated through the KRC unit!
- JOINT IMPEDANCE CONTROL MODE

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
- Adapt KRL src file for KRC to work with the components better (Also put KRL src file on here)
- Stop and cleanup component better with proper stopping of the KRC
- Write instructions on how to use
