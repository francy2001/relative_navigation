# REACSA description

This repository contains the robot description of REACSA to use in a ROS2 environment.

## Usage

### As a submodule

Embedding this repo as a sumbodule allows commiting and building changes on the fly.
From the src folder of the target project, run:

```console
git submodule add https://gitlab.esa.int/orl/platforms/reacsa/reacsa-description.git reacsa_description
```

The package can be run using:

```console
ros2 launch reacsa_description robot_description.launch.py
```

The following descritption topics will be published: `/reacsa/tf`, `/reacsa/tf_static`, `/reacsa/robot_description`. Gazebo resources path will also be updated.
