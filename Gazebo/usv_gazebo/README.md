# usv_gazebo

## Installation and Tutorials

[Gazebo Simulation of usv/Heron USV](https://wiki.nps.edu/pages/viewpage.action?pageId=818282511)

## Branches
Currently we maintain separate development branches for ROS releases.  The major differences between the Indigo and Kinetic branches are...

 * The xacro files for Indigo define parameters in the root xacro file, as opposed to Kinetic defining parameters in sub-xacro calls.
 * For worlds and models that have floating things (buoys, etc.), in Indigo (Gazebo 2) we use a [port](https://github.com/bsb808/buoyancy_gazebo_plugin) of the [buoyancy plugin](http://gazebosim.org/tutorials?tut=hydrodynamics&cat=plugins) that is backported for Gazebo 2.  In Kinetic we should use the standard buoyancy plugin, but still needs to be implemented.



