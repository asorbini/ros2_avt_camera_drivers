# ROS 2 drivers for cameras by Allied Vision Technologies

This repo contains a ROS2 driver for cameras manufactured by [Allied Vision Technologies](https://www.alliedvision.com).  

The driver relies on libraries provided by AVT as part of their [Vimba SDK](https://www.alliedvision.com/en/products/software.html). 

This driver was ported from the ROS(1) driver at [AutonomouStuff GitHub](https://github.com/astuff/avt_vimba_camera) to ROS2, as of the Foxy release.

At the moment, the repository only includes the "mono" version of the driver.

## Building the driver

This driver relies on the [Vimba SDK](https://www.alliedvision.com/en/products/software.html)
which is exposed to ROS 2 by the [`vimba_sdk_vendor`](https://github.com/asorbini/vimba_sdk_vendor) package.

The SDK must be installed by hand on the build system, and the installation
location must be specified with variable `VIMBA_DIR`.

Once you have installed the SDK and a recent version of ROS 2 (Foxy or later),
you can clone the required repositories and build them in a workspace like
any other ROS 2 package. Note that you will also make sure to have the packages
from the [ros-perception/image_common](https://github.com/ros-perception/image_common)
repository.

```sh
# Create a workspace directory
mkdir ws-avt-cameras
cd ws-avt-cameras

# Clone the required repositories (alternatively, you may also use vcs, see below)
git clone https://github.com/asorbini/vimba_sdk_vendor.git
git clone https://github.com/asorbini/ros2_avt_camera_drivers.git

# Make sure image_common packages are installed (e.g. for Foxy)
sudo apt-get install -y ros-foxy-image-common

# Load ROS 2 (e.g. Foxy)
source /opt/ros/foxy/setup.bash

# Configure location of Vimba SDK
export VIMBA_DIR=/opt/Vimba_4.2

# Build driver using colcon
colcon build --symlink-install
```

If you prefer, you can use the provided file [avt_camera_drivers.repos](avt_camera_drivers.repos)
to clone all required repositories using `vcs`:

```sh
mkdir ws-avt-cameras
cd ws-avt-cameras

curl https://raw.githubusercontent.com/asorbini/ros2_avt_camera_drivers/main/avt_camera_drivers.repos | vcs import .
```

## Running the driver

The driver can be run with `ros2 run` by passing a YAML file to configure all  parameter:

```sh
ros2 run ros2_avt_cameras mono_camera_node --ros-args --params-file ros2_avt_cameras/params/mono_c1.yaml
```
