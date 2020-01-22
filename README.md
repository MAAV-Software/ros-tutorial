# ros-tutorial

## Docker Installation

### Windows
- Windows 10 64-bit: Pro, Enterprise, or Education
    - [Docker Desktop on Windows](https://docs.docker.com/docker-for-windows/install/)
- Other Windows Versions
    - [Docker Toolbox on Windows](https://docs.docker.com/toolbox/toolbox_install_windows/)

#### Installing Docker Toolbox
1. Assuming you meet the requirements in *Step 1: Check your version* of the [Install Docker Toolbox on Windows](https://docs.docker.com/toolbox/toolbox_install_windows/#step-1-check-your-version) guide, download the latest **.exe** file from [Toolbox Releases](https://github.com/docker/toolbox/releases).
2. Run the **Docker Toolbox Installer**
    1. Choose an install location (default is fine)
    2. On *Select Components* page
        - If you already have VirtualBox installed, uncheck it on this page
        - If you already have Git for Windows, no need to install it through the Docker Toolbox installer
    3. On the *Select Additional Tasks* page, use the default checked options, then click Install and then Finish
3. Run **VirtualBox as Administrator**
4. Run **Docker Quickstart Terminal as Administrator**
5. After some installation steps, you should be ready to start!

#### GUIs in Docker
To enable the use of GUIs in Docker, follow the instructions on the [MAAV Website](https://sites.google.com/umich.edu/maav/team/software/docker/guis-on-docker?pli=1&authuser=1) and [this article](https://dev.to/darksmile92/run-gui-app-in-linux-docker-container-on-windows-host-4kde).
1. Install **VcXsrv**
    - If you are using Chocolatey: `choco install vcxsrv`
    - Otherwise, download from [Source Forge](https://www.google.com/url?q=https%3A%2F%2Fsourceforge.net%2Fprojects%2Fvcxsrv%2F&sa=D&sntz=1&usg=AFQjCNFZLtdIHK50nhjd-rYEgBPhHWYt4A)
2. Run **XLaunch** and choose the settings shown on the [MAAV Website](https://sites.google.com/umich.edu/maav/team/software/docker/guis-on-docker?pli=1&authuser=1) or [this article](https://dev.to/darksmile92/run-gui-app-in-linux-docker-container-on-windows-host-4kde)
3. Before clicking **Finish**, save the configuration file to one of the following locations:
    - `%AppData%\Xming`
    - `%UserProfile%\Desktop`
    - `%UserProfile%`
4. Open **PowerShell** and run `ipconfig` to determine IP address (look at [MAAV Website](https://sites.google.com/umich.edu/maav/team/software/docker/guis-on-docker?pli=1&authuser=1) for help)
5. In the **docker-compose.yml** file, under environment, add the IP Address to the display variable:
    - `DISPLAY=<IP Address>:0.0` where "\<IP Address>" is replaced with your IP Adress from step 4
    - docker-compose.yml should look something like this now
        ```yml
        version: "2"

        services:
        ros-demo:
            image: ros-demo
            privileged: true
            volumes:
            # Mount the current directory do everything in /tutorial within docker
            - .:/tutorial:rw
            environment:
            - DISPLAY=1.1.1.1:0.0 # Change 1.1.1.1 to your IP Address
            network_mode: "host"
            container_name: ros-demo
            command: "/bin/bash --init-file scripts/source-ros.sh" #source ros automatically
        ```
6. Follow instructions in ROS Tutorial Steps below

#### Suggested Improvement (Optional): Docker Toolbox + WSL + tmux
As you are working through the ROS Tutorials, you will see that they suggest running processes in one terminal, then opening a new window to run other processes. In Docker, this can be troublesome. You can run a process in the background: e.g. `roscore &`, but this solution doesn't work for every situation. When you want to control the movement of the turtle in turtlesim using the arrow keys, you cannot run that in the background. A solution that I have found is to use WSL (Windows Subsystem for Linux) and tmux.

1. [Install Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/install-win10)
2. [Set up Docker Toolbox for Windows Home 10 and WSL](https://medium.com/@joaoh82/setting-up-docker-toolbox-for-windows-home-10-and-wsl-to-work-perfectly-2fd34ed41d51)
3. Install tmux in WSL
    - `sudo apt-get install tmux`
4. Follow the steps in ROS Tutorial below to run the docker container
5. When inside docker, open tmux and run roscore in one window
    - `tmux`
    - You should now see `[0] 0:bash*` at the bottom of your terminal
    - `roscore`
6. Create a new pane with tmux
    - `Ctrl + b` then `"` to open a new pane vertically below
    - `Ctrl + b` then `o` to switch between panes
7. Run other commands in 2nd pane while `roscore` runs in 1st pane
8. Check out [Getting started with Tmux](https://linuxize.com/post/getting-started-with-tmux/) for more information on commands
9. Check out [Making tmux Pretty and Usable](https://www.hamvocke.com/blog/a-guide-to-customizing-your-tmux-conf/) for more information on customization

#### Troubleshooting
If you encounter VirtualBox errors when running the Docker Quickstart Terminal, here are some suggestions:
* If you use the latest version of VirtualBox for personal use, backup your files
  * Uninstall Docker and VirtualBox
  * Restart computer
  * Install latest version of VirtualBox
  * Install Docker and uncheck VirtualBox in the installer
* If you don't need VirtualBox for personal use, the VirtualBox install option in the Docker installer should make everything work correctly
* If you see an error regarding having multiple host-only adapters with the same IP
  * Open VirtualBox -> File -> Host Network Manager
  * Remove one of the duplicate adapters
  * Re-open Docker Quickstart Terminal 

### Ubuntu
Follow the instructions on the Docker website for [Docker Engine - Community](https://docs.docker.com/install/linux/docker-ce/ubuntu/).

### Mac
Follow the instructions on the Docker website for [Docker Desktop for Mac](https://docs.docker.com/docker-for-mac/).

### Other
Check the [Docker website](https://www.docker.com/).

## Docker Essential Vocab
- **Image:** This is essentially the “installation” of something that you want to run using
Docker. An image contains all the data necessary to run containers. Images 
are hierarchical and a new image that shares information with an older 
one will not reproduce this information and instead just re-use it (i.e. if 
you have two Ubuntu based images with different software installed, they 
will both refer to the same base Ubuntu image rather than copy its contents). 
This is what people mean when they say that Docker’s filesystem is layered.

- **Container:** An instance of a particular image. This is equivalent to running,
say Firefox, twice. In that scenario Firefox is installed once but can be 
launched multiple times and each instance refers back to the same installation.
when a container is running, it can’t make any changes to the underlying image 
(images are read only!) but gets assigned a new filesystem for storage of 
new information. Images may be ephemeral (that is they are removed once you 
stop them and any new data is destroyed), or persistent (containers are 
kept either in an on or off state until they are explicitly removed). 
Note that it is possible to mount external volumes onto the containers 
and all changes made to the host filesystem this way are persistent!

- **Dockerfile:** Simple configuration file that defines how your container is 
built. You can start FROM a base container, RUN a series of shell commands, 
set up ENVironment variables, ADD or COPY things from the host filesystem, 
specify what command runs when your container is started, and more! These 
files can start from scratch or from an existing image. Popular Linux 
distribution provide various images in an official capacity on [Docker Hub](https://hub.docker.com/search?category=os&source=verified&type=image&image_filter=official).

## ROS Essential Vocab
- **Packages:** Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.

- **Manifests (package.xml):** A manifest is a description of a package. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...

- **Nodes:** A node is an executable that uses ROS to communicate with other nodes.

- **Messages:** ROS data type used when subscribing or publishing to a topic.

- **Topics:** Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.

## New Terminal Window in Docker
As you work through the ROS Tutorial, you will find that it is sometimes necessary to run a process (e.g. `roscore`) in one window and run another process (e.g. `rosrun turtlesim turtlesim_node`) in another window. This process is slightly more complicated than simply opening a new window due to our use of Docker containers. There are a couple of options depending on how much time you would like to invest into setting up and learning new tools. The ideal option is tmux (see **Docker Toolbox + WSL + tmux** above), but if you would rather not go through the process of seting that up, you can do the following:
1. Run your docker container using the steps in **ROS Tutorial Steps** below
2. Open a new terminal (e.g. Docker Quickstart Terminal, WSL, etc.) and run `docker ps` to see the currently running containers
3. Copy the container ID
4. Run `docker exec -it <container_id> bash` where `<container_id>` is replaced with the container name you copied in step 3
5. Now you should have a new terminal connected to the same Docker container
6. Make sure to run `cd /tutorial` and `source /opt/ros/melodic/setup.bash` in your new terminal

## ROS Tutorial Steps
**NOTE:** If you are using **vcxsrv** as mentioned in **GUIs in Docker** above, **XLaunch** needs to be running and **docker-compose.yml** needs to have the DISPLAY variable set with your IP Address before running the Docker container. If XLaunch is not running, find **config.xlaunch** that you saved and run it.

**NOTE:** If you are using **Docker Toolbox + WSL** as mentioned in in the suggested improvement for Windows, you will need to run **Docker Quickstart Terminal** atleast once to create the **default** virtual machine in VirtualBox. Once this has been created, you will need to manually start the VM since we are not using the quickstart terminal which would have taken care of this for you. In WSL, run `docker-machine.exe start default` and then follow the steps below.

1. In your docker terminal, navigate to the local repository folder
    - `cd /PATH/TO/REPO`
2. Create a docker image called **ros-demo**
    - `docker build -t ros-demo .`
3. Run a container using the ros-demo image
    - `docker-compose run --rm ros-demo`
4. Navigate to the tutorial folder inside your running docker container
    - `cd /tutorial`
5. Set source
    - `source /opt/ros/melodic/setup.bash`

Do all the work inside your **tutorial** folder. This is the only folder linked 
to your host computer. Complete the following [ROS tutorials](https://wiki.ros.org/ROS/Tutorials).

1. [Installing and Configuring your ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
    - Skip **1. Install ROS** and **2. Managing Your Environment** as these have already been taken care of in the docker container.
    - For **3. Create a ROS Workspace**, follow the instructions for **catkin** as we are using ROS Melodic which is newer than ROS Groovy.
    - As mentioned previously, all your work should be done inside the **tutorial** folder. In **3. Create a ROS Workspace**, create the catkin workspace inside the tutorial folder rather than the home directory.
        - `mkdir -p catkin_ws/src` instead of `mkdir -p ~/catkin_ws/src`
2. [Navigating the ROS Filesystem](https://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
    - The ROS tutorial files have already been installed through the **Dockerfile**.
3. [Creating a ROS Package](https://wiki.ros.org/ROS/Tutorials/CreatingPackage)
4. [Building a ROS Package](https://wiki.ros.org/ROS/Tutorials/BuildingPackages)
5. [Understanding ROS Nodes](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
    - Run roscore in the background so you don't need a new terminal window or use tmux as mentioned in **Suggested Improvement (Optional): Docker Toolbox + WSL = tmux**.
        - `roscore &`
    - To run turtlesim, you will need to enable **GUIs for Docker**. See GUIs in Docker section above.
6. [Understanding ROS Topics](https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
7. [Understanding ROS Services and Parameters](https://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
8. [Using rqt_console and roslaunch](https://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
9.  [Using rosed to edit files in ROS](https://wiki.ros.org/ROS/Tutorials/UsingRosEd)
10. [Creating a ROS msg and srv](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
11. [Writing a Simple Publisher and Subscriber (C++)](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

## ROS Commands
- rospack = ros + pack(age) : provides information related to ROS package or stack
- roscd = ros + cd : **c**hanges **d**irectory to a ROS package or stack
- rosls = ros + ls : **l**ist**s** files in a ROS package
- roscp = ros + cp : **c**o**p**ies files from/to a ROS package
- rosmsg = ros + msg : provides information related to ROS message definitions
- rossrv = ros + srv : provides information related to ROS service definitions
- catkin_make : makes (compiles) a ROS package
  - rosmake = ros + make : makes (compiles) a ROS package (if you're not using a catkin workspace)