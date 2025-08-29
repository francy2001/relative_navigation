ARG ROS_DISTRO=jazzy
ARG UBUNTU_DISTRO=noble
ARG FROM_IMAGE=ros:${ROS_DISTRO}
ARG OVERLAY_WS=/opt/overlay_ws
ARG PKG_NAME=reacsa_description
ARG VERSION_ORL_COMMON=0.1.1

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
ARG PKG_NAME
WORKDIR $OVERLAY_WS/src/${PKG_NAME}
#RUN vcs import ./ < ../overlay.repos
COPY . .

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building dependencies
# Some dependencies here are only available for AMD64, thus this stage is not used for exporting packages to the package registry. Any true dependencies for the packages to export shall be added within the release stage itself.
FROM ${FROM_IMAGE} AS dependencies

ARG DEBIAN_FRONTEND=noninteractive
ARG OVERLAY_WS
ARG ROS_DOMAIN_ID
ARG ROS_DISTRO
ARG VERSION_ORL_COMMON
ARG UBUNTU_DISTRO

# Install dependencies
RUN apt-get -q update && apt-get -qy install --no-install-recommends \
        ca-certificates \
        gnupg \
        lsb-release \
        wget \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ORL common
RUN ARCH=`dpkg --print-architecture` \
    && wget -q "https://gitlab.esa.int/api/v4/projects/12054/packages/generic/orl_common/${VERSION_ORL_COMMON}/ros-${ROS_DISTRO}-orl-common_${VERSION_ORL_COMMON}-0${UBUNTU_DISTRO}_${ARCH}.deb" -O "orl-common.deb" \
    && dpkg -i "orl-common.deb" && rm "orl-common.deb"

# Install ROS dependencies without storing the source
WORKDIR ${OVERLAY_WS}
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN apt-get -q update && . /opt/ros/$ROS_DISTRO/setup.sh && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# multi-stage for building source
FROM dependencies AS builder

# build overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher $OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --symlink-install

# Add overlay sourcing to bashrc
RUN echo 'source "$OVERLAY_WS/install/setup.bash"' >> /etc/bash.bashrc

# Run
CMD [ "/bin/bash" ]
