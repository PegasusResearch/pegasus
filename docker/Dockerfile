ARG ros_version=foxy
FROM osrf/ros:$ros_version-desktop
LABEL maintainer="Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>"

# -------------------------------
# Create the container user
# -------------------------------
ARG user=pegasus
ARG passwd=pegasus
ARG uid=1000
ARG gid=1000
ENV USER=$user
ENV PASSWD=$passwd
ENV UID=$uid
ENV GID=$gid
RUN groupadd $USER && \
    useradd --create-home --no-log-init -g $USER $USER && \
    usermod -aG sudo $USER && \
    echo "$PASSWD:$PASSWD" | chpasswd && \
    chsh -s /bin/bash $USER && \
    usermod --uid $UID $USER && \
    groupmod --gid $GID $USER

ENV HOME=/home/$USER
WORKDIR $HOME

# --------------------
# Install requirements
# --------------------
#COPY install_requirements.sh install_requirements.sh
#RUN ./install_requirements.sh
#RUN rm install_requirements.sh

# ----------------------------------------------------
# Get the script to use as entrypoint to the container
# ----------------------------------------------------
COPY docker/entrypoint.sh /home/pegasus/entrypoint.sh 
RUN chmod +x /home/pegasus/entrypoint.sh

# -----------------------------------
# Change the User to the pegasus user
# -----------------------------------
USER $USER

# -----------------------
# Create a ROS2 workspace
# -----------------------
ENV CATKIN_ROOT=${HOME}
ENV ROS_WORKSPACE=${CATKIN_ROOT}/pegasus_ws
RUN mkdir $ROS_WORKSPACE

# Adding ROS2 elements to the .bashrc file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# -----------------------------------------------------------------
# Setup the image entry point to be anything specified as arguments
# -----------------------------------------------------------------
ENTRYPOINT ["./entrypoint.sh"]
CMD ["bash"]