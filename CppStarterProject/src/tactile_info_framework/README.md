## Tactile info framework

ROS packages in charge of acquiring and processing the tactile information from the HARC tactile sensors installed on RHPS1.

### Installation

This package is developed using ROS 2. So, it is meant to run on Uuntu up to version 22.04.

First, create a colcon workspace (e.g., jst_mirai_tacctile_demo_ws) and source its `install/setup.bash` in `.bashrc`.

Within this colcon workspace, clone this repository:

```bash
git clone git@github.com:isri-aist/tactile_info_framework

```

Now the package can be built as any colcon project with `colcon build`

### Execution

The whole pipeline (acquisition + processing nodes) can be started with the following command:

```bash
ros2 launch tactile_info_framework nodelets.launch.py config:=<sensored_link_conf>

```

Where `<sensored_link_conf>` refers to the configuration file defining the configuration of the tactile patches corresponding to that link.
The available configuration files can be found at the `config` folder of this package, using the following name convention: `harc_<sensored_link_conf>.yaml`.

Each state of the pipeline is published as ROS topic.
Use `rqt_image_view` to see the image corresponding to each ROS topic.
