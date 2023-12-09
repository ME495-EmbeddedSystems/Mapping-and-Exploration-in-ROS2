# `nubot_nav` package

The `nubot_nav` package enables mapping and exploration for the [`nubot`](https://github.com/m-elwin/nubot) through the `slam_toolbox` and `nav2` stack.

### Launch manual explorer
The manual exploration allows the user to move the nubot in `gazebo` (using the '`teleop_twist_keyboard` package or by giving 2-D goal poses) to explore its 2-D environment while building a map of it.

To launch manual exploration:
```bash
ros2 launch nubot_nav manual_explore.launch.xml
```

### Launch autonomous explorer
The autonomous explorer node (nicknamed `dora` given the efficacy of the algorithm) explores the frontiers of the map by using what is essentially a naked (single-branched) Rapidly exploring Random Tree. The nubot is given random goal poses within its immediate vicinity to slowly map the entire region after a sufficiently long duration. This algorithm draws inspiration from Finding Nemo's Dory's philosophy: ["Just keep swimming"](https://www.youtube.com/watch?v=zya40MmN9I4), and the [`crazy_turtle`](https://github.com/m-elwin/crazy_turtle) package. As long as the nubot does not stop, it should eventually map the entire region.

To launch autonomous exploration:
```bash
ros2 launch nubot_nav explore.launch.xml
```

Author: Aditya Nair
