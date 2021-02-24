# Ball Chaser
This repository contain the code and launch file for ball chaser node for controlling the robot. The robot will chase after any white pixels on the robot camera

# Dependcies
This ball chaser node needs to work with the robot design in Drive Bot Respository ([Link][link-id]).

[link-id]: https://github.com/wang2700/drive_bot.git

# Build
- Copy the files to your catkin workspace under src folder.
- Build the package using `catkin_make` command.

# Run the Node
- Make sure the gazebo is launched with the robot and world inside.
- Run the ball chaser node by running this command:
  - `roslaunch ball_chaser ball_chaser.launch`
