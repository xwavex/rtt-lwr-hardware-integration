# rtt-gazebo-lwr-integration
RTT component using a Gazebo model pointer to integrate RTT and Gazebo for the Kuka LWR. Based on https://github.com/kuka-isir/rtt_lwr/tree/master/rtt_lwr_gazebo

## Install

`mkdir build`

`cd build`

// for orocos plugins

`export PKG_CONFIG_PATH=$insert-prefix-here/lib/pkgconfig`

`cmake -Dgazebo_DIR=$insert-prefix-here/lib/cmake/gazebo -Dignition-math2_DIR=$insert-prefix-here/lib/cmake/ignition-math2 -DSDFormat_DIR=$insert-prefix-here/lib/cmake/sdformat -DOROCOS-RTT_DIR=$insert-prefix-here/lib/cmake/orocos-rtt ..`

`make -j 4`

// include RTT component in search path

`RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:$include-path-to/rtt-gazebo-lwr-integration/build/orocos`

## TODO

- Exclude Eigen folder
