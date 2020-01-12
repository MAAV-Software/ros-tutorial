FROM ros:melodic

RUN  apt-get update && apt-get install -y \
	tmux \
	curl \
	wget \
	vim \
	unzip
	
CMD ["bash"]
