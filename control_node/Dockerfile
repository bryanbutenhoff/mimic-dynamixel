FROM ros:kinetic-ros-base

ENV PACKAGE_LIST="curl vim unzip" \
#    ROS_PACKAGE_LIST="ros-kinetic-dynamixel-sdk" \
    ROS_PACKAGE_LIST="" \
    APP_ROOT="/opt/catkin-ws"

RUN apt-get update && \
    apt-get install -y $PACKAGE_LIST $ROS_PACKAGE_LIST && \
    rm -rf /var/lib/apt/lists/

# Install dynamixel-sdk package
ENV DYNAMIXEL_SDK_VERSION="3.6.2" \
    DYNAMIXEL_LIB_DIR=${APP_ROOT}/lib/dynamixel-sdk
RUN curl https://github.com/ROBOTIS-GIT/DynamixelSDK/archive/${DYNAMIXEL_SDK_VERSION}.tar.gz -JLo dynamixel.tar.gz && \
    tar -xvf dynamixel.tar.gz && \
    rm -f dynamixel.tar.gz && \
    mkdir -p ${DYNAMIXEL_LIB_DIR} && \
    cp -R DynamixelSDK-${DYNAMIXEL_SDK_VERSION}/python/* ${DYNAMIXEL_LIB_DIR} && \
    rm -rf DynamixelSDK-*
WORKDIR ${DYNAMIXEL_LIB_DIR}
RUN python setup.py install

# Install servowrapper package
ENV SERVOWRAPPER_VERSION="master" \
    SERVOWRAPPER_LIB_DIR=${APP_ROOT}/lib/servowrapper
RUN curl https://github.com/bryanbutenhoff/servowrapper/archive/${SERVOWRAPPER_VERSION}.zip -JLo servowrapper.zip
RUN unzip servowrapper.zip
RUN rm -f servowrapper.zip
RUN mkdir -p ${SERVOWRAPPER_LIB_DIR}
RUN cp -R servowrapper-${SERVOWRAPPER_VERSION}/* ${SERVOWRAPPER_LIB_DIR}
RUN rm -rf servowrapper-*
WORKDIR ${SERVOWRAPPER_LIB_DIR}
RUN python setup.py install

# Install application source
WORKDIR ${APP_ROOT}

RUN mkdir -p ${APP_ROOT}/src
COPY src ${APP_ROOT}/src

RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash; catkin_make"]

RUN echo "source /opt/ros/kinetic/setup.bash; source ${APP_ROOT}/devel/setup.bash" >> /etc/bash.bashrc

COPY entrypoint.sh ${APP_ROOT}
