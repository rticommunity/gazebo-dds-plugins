# DDS Gazebo Plugins

This repository includes a number of plugins to integrate the 
[Gazebo simulator](http://gazebosim.org) in the DDS Global Data Space. 

DDS Gazebo Plugins enable DDS-based robotic systems to leverage Gazebo's
simulation capabilities, such as 3D simulation and computer vision.

## Building DDS Gazebo Plugins

### Preparing your Environment

Before building the DDS Gazebo Plugins, you will need to install Gazebo
and RTI Connext DDS on your system.

#### Installing Gazebo

```
# Debian-based systems (e.g., Ubuntu, Debian, or Linux Mint)
$ sudo apt install gazebo

# Fedora
$ sudo dnf gazebo gazebo-devel
```

#### Installing RTI Connext DDS
To RTI Connext DDS (host and target), follow the the 
[Core Libraries Getting Started Guide](https://community.rti.com/static/documentation/connext-dds/current/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_GettingStarted/Content/GettingStarted/Installing_ConnextDDS.htm).

### Building Plugins

Configure the build project using CMake. You will need to provide the path to
your RTI Connext DDS installation and the desired target platform using the
``-DCONNEXTDDS_DIR`` and ``-DCONNEXTDDS_ARCH`` arguments.

Note that in this case, we create a build directory within the source tree
to store the build results.

```
$ cd dds-gazebo-plugins; mkdir build; cd build
$ cmake -DCONNEXTDDS_DIR=/path/to/rti_connext_dds-x.y.z -DCONNEXTDDS_ARCH=<arch_name> ..
```

Once you have configured your build project, simply run ``make`` to build the
plugins.

```
$ make
```

## List of plugins

This project contains multiple Gazebo plugins. These plugins are:

* ***Bumper scan*** : It allows us to obtain the information a bumper scan. You can obtain more information [here](src/bumper_scan/README.md)
* ***Camera*** : It allows us to obtain the information of a camera. You can obtain more information [here](src/camera/README.md)
* ***Differential drive*** : It allows us to manage a differential drive robot and obtain its information. You can obtain more information [here](src/diff_drive/README.md)
* ***Elevator*** : It allows us to manage an elevator. You can obtain more information [here](src/elevator/README.md)
* ***Imu scan*** : It allows us to obtain the information of an imu sensor. You can obtain more information [here](src/imu_scan/README.md)
* ***Laser scan*** : It allows us to obtain the information of a laser sensor. You can obtain more information [here](src/laser_scan/README.md)
* ***Skid steer drive*** : It allows us to manage a skid steer drive robot and obtain its information. You can obtain more information [here](src/skid_steer_drive/README.md)
* ***Stereo camera*** : It allows us to manage two synchronized cameras and obtain its information. You can obtain more information [here](src/stereo_camera/README.md)