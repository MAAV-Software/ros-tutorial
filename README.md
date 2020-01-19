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
To enable the use of GUIs in Docker, follow the instructions on the [MAAV Website](https://sites.google.com/umich.edu/maav/team/software/docker/guis-on-docker?pli=1&authuser=1).
- Install VcXsrv
- Open PowerShell and run `ipconfig` to determine ip address
- Build docker container (see ROS Tutorial Steps below)
- Run docker container with `docker-compose run --rm -e DISPLAY=<IP Address>:0.0 ros-demo` to enable GUIs in your docker container

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

- **Packages:** Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.

- **Manifests (package.xml):** A manifest is a description of a package. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...

- **Nodes:** A node is an executable that uses ROS to communicate with other nodes.

- **Messages:** ROS data type used when subscribing or publishing to a topic.

- **Topics:** Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.


## ROS Tutorial Steps
1. Clone (or download) this repository to your computer
2. In your docker terminal, navigate to the local repository folder
    - `cd /PATH/TO/REPO`
3. Create a docker image called "ros-demo"
    - `docker build -t ros-demo .`
4. Run a container using the ros-demo   
    - `docker-compose run --rm ros-demo`
5. Navigate to the tutorial folder inside your running docker container
    - `cd /tutorial`
6. Set source
    - `source /opt/ros/melodic/setup.sh`
7. Run roscore as a background task so you don't need a new terminal
    - `roscore &`

Do all the work inside your **tutorial** folder. This is the only folder linked 
to your host computer. Complete the following [ROS tutorials](https://wiki.ros.org/ROS/Tutorials).
1. [Installing and Configuring your ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
    - Skip **1. Install ROS** and **2. Managing Your Environment** as these have already been taken care of in the docker container.
    - For **3. Create a ROS Workspace**, follow the instructions for **catkin** as we are using ROS Melodic which is newer than ROS Groovy.
    - As mentioned previously, all your work should be done inside the **tutorial** folder. In **3. Create a ROS Workspace**, create the catkin workspace inside the tutorial folder rather than the home directory.
        - `mkdir -p catkin_ws/src` instead of `mkdir -p ~/catkin_ws/src`
2. [Navigating the ROS Filesystem](https://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
    - We are using ROS Melodic Morenia so you should run `sudo apt-get install ros-melodic-ros-tutorials` to install the required packages.
    - Review
        - rospack = ros + pack(age)
        - roscd = ros + cd
        - rosls = ros + ls
3. [Creating a ROS Package](https://wiki.ros.org/ROS/Tutorials/CreatingPackage)
4. [Building a ROS Package](https://wiki.ros.org/ROS/Tutorials/BuildingPackages)
5. [Understanding ROS Nodes](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
    - To run turtlesim, you will need to enable the GUI for docker. See GUIs in Docker section above.
    - Review
        - roscore = ros + core: master (provides name service for ROS) + rosout (stdout/stderr) + parameter server.
        - rosnode = ros + node: ROS tool to get information about a node.
        - rosrun = ros + run: runs a new node from a given package.
6. [Understanding ROS Topics](https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
7. [Understanding ROS Services and Parameters](https://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
8. [Using rqt_console and roslaunch](https://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
9.  [Using rosed to edit files in ROS](https://wiki.ros.org/ROS/Tutorials/UsingRosEd)
10. [Creating a ROS msg and srv](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
11. [Writing a Simple Publisher and Subscriber (C++)](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)