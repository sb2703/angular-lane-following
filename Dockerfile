FROM duckietown/dt-core:backup
WORKDIR ${CATKIN_WS_DIR}/src/dt-core
ARG PACKAGES
COPY ./${PACKAGES} ./${PACKAGES}
RUN \cp -rf ./${PACKAGES}/* ./packages 
RUN rm -r ./${PACKAGES}
RUN python3 -m pip install simple_pid
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/
