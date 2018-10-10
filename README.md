# dabot
Integration of the JACO2, the DATable, and a DA


## Setting up the dev environmentet
1. Clone the Dockerfile [here](https://github.com/hrc2da/rosdocked), courtesy of jbohren. This includes scripts to build a docker image with ros-indigo, with shared username, directories, and X11 (I haven't tested this yet). For more info: [rosdocked](https://github.com/jbohren/rosdocked).
2. Make sure that you are in the docker group BEFORE you build the docker image (otherwise, if you sudo, the shared user will be root, and you'll have to sudo everything). `usermod -aG docker <user>`.
3. Run `./build.sh <image name of your choice>` to build the Dockerfile.
4. Run `./run.sh <same image name>` to start a Docker container with your image. It will start in-place in your console. If you want to make sure it's running, you can try running some tool that's installed on your host but not the Docker container, e.g. `code` for VSCode.
5. Make a catkin_ws in the rosdocked directory (don't push to the repo, please) and cd into it.
6. Make a src directory and cd into it.
7. Clone https://github.com/hrc2da/dabot.
8. Cd back into catkin_ws and catkin_make. Make sure the srv files, etc. get created. Don't forget to `source devel/setup.bash`
9. Run the docker image in another terminal and start a ROS master with `roscore`
10. You can run `python -m unittest discover <path to the tests directory, e.g. dabot/test> -v` in the first terminal to make sure everything is working.
