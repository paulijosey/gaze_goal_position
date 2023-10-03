FROM ros:humble-perception
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME
RUN useradd --uid $USER_UID --gid $USER_GID -m $USERNAME
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update
RUN apt-get install -y \
    	python3-pip \
		ffmpeg \
		git \
    	libboost-all-dev \
    	build-essential \
    	libglib2.0-dev \
    	lua5.3 \
    	liblua5.3 \
    	libjchart2d-java \
    	default-jdk \
    	cmake \
    	ninja-build \
    	stow \
    	python3-dev \
    	python3-pip \
		python3-colcon-common-extensions \
		libsm6 \
		libxext6 \
		ros-${ROS_DISTRO}-rviz2 && \
	rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp
RUN pip install -r /tmp/requirements.txt

# fix wierd ROS docker memory issue
RUN echo "ulimit -n 524388" >> /home/${USERNAME}/.bashrc
RUN echo "ulimit -Hn 524388" >> /home/${USERNAME}/.bashrc

# add ros setup file to bashrc for convinience
RUN echo source /opt/ros/${ROS_DISTRO}/setup.bash >> /home/${USERNAME}/.bashrc

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
