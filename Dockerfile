FROM ros:melodic

RUN sudo apt-get update && apt-get install -y \
	tmux \
	curl \
	wget \
	vim \
	unzip \
	ros-melodic-ros-tutorials

CMD ["bash"]
