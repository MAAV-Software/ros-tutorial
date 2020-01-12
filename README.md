# ros-tutorial

## Docker setup

### DOCKER ESSENTIAL VOCAB

Image: This is essentially the “installation” of something that you want to run using
Docker. An image contains all the data necessary to run containers. Images 
are hierarchical and a new image that shares information with an older 
one will not reproduce this information and instead just re-use it (i.e. if 
you have two Ubuntu based images with different software installed, they 
will both refer to the same base Ubuntu image rather than copy its contents). 
This is what people mean when they say that Docker’s filesystem is layered.

Container: An instance of a particular image. This is equivalent to running,
say Firefox, twice. In that scenario Firefox is installed once but can be 
launched multiple times and each instance refers back to the same installation.
when a container is running, it can’t make any changes to the underlying image 
(images are read only!) but gets assigned a new filesystem for storage of 
new information. Images may be ephemeral (that is they are removed once you 
stop them and any new data is destroyed), or persistent (containers are 
kept either in an on or off state until they are explicitly removed). 
Note that it is possible to mount external volumes onto the containers 
and all changes made to the host filesystem this way are persistent!

Dockerfile: Simple configuration file that defines how your container is 
built. You can start FROM a base container, RUN a series of shell commands, 
set up ENVironment variables, ADD or COPY things from the host filesystem, 
specify what command runs when your container is started, and more! These 
files can start from scratch or from an existing image. Popular Linux 
distribution provide various images in an official capacity on …

### Tutorial Steps

first create a docker image called "ros-demo"  


`docker build -t ros-demo . `


now you can run a container using the ros-demo   


`docker-compose run --rm ros-demo`


Now you are inside your running docker container.  


`cd /tutorial`


Do all the work inside your /tutorial folder. This is the only folder linked 
to your host computer. Follow the ROS tutorials [here](https://wiki.ros.org/ROS/Tutorials) 
and complete up to and including ROS Pub/Sub \#11 

