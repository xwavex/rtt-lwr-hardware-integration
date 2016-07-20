# rtt-lwr-hardware-integration
RTT Component integrating the FRI library, with RTNET providing real-time communication, into the orocos framework.
Specifically designed using kinematic chains for use with [rtt-core-extensions](https://github.com/corlab/rtt-core-extensions).

## Features
- POSITION CONTROL MODE
- TORQUE CONTROL MODE
  - Through joint impedance control mode, gravity is compensated through the KRC unit!
- JOINT IMPEDANCE CONTROL MODE
 
## Requirements
Realtime:
- Ubuntu with Xenomai patched kernel and RTNet
  - (Tested on Ubuntu 14.04 with Xenomai 2.6.4 and RTNet)

Non-realtime:
- Ubuntu only

## Install
KRC Unit:

Copy FRIControl.src and FRIControl.dat onto the KRC

External machine:

`mkdir build`

`cd build`

// for orocos plugins

`export PKG_CONFIG_PATH=$insert-prefix-here/lib/pkgconfig`

`cmake -DOROCOS-RTT_DIR=$insert-prefix-here/lib/cmake/orocos-rtt ..`

`make -j 4`

// include RTT component in search path

`export RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:$include-path-to/rtt-lwr-hardware-integration/build/orocos`

## How To Use
This package comes in two parts in relation to the Kuka hardware. 
The first part involves the KRC unit and setting up the script for the FRI communication.

### KRC Unit
- Edit tool and ip address on the KRL program
- Run the program once until the program reaches a halt point to initialize the LWR arm
- Run the program past this point to enable the communication between the computer and the KRC unit

### External Computer
- Set the IP address of the computer on the lwr_robot component's property ip_addr (and that both the computer and KRC unit are on the same subnet)
- Once the KRL program has started running on the main loop run your orocos program
- (NOTE) There may be a slight delay before the program takes effect as the arm has to switch modes

## TODO
- Adapt KRL src file for KRC to work with the components better (Also put KRL src file on here)
- Stop and cleanup component better with proper stopping of the KRC
- Write instructions on how to use
