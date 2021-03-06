# fog-sw BUILDER
FROM ros:foxy-ros-base as fog-sw-builder

ARG BUILD_NUMBER
ARG COMMIT_ID
ARG GIT_VER

# workaround for ROS GPG Key Expiration Incident
RUN rm /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y curl && \
    curl http://repo.ros2.org/repos.key | sudo apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update

# RUN echo "deb [trusted=yes] https://artifactory.ssrc.fi/artifactory/debian-public-local focal fog-sw" >> /etc/apt/sources.list

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    build-essential \
    dh-make debhelper \
    cmake \
    git-core \
    fakeroot \
    python3 \
    pip \
    python3-bloom \
    dh-python \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /build

COPY . .

RUN params="-m $(realpath .) " \
    && [ ! "${BUILD_NUMBER}" = "" ] && params="$params -b ${BUILD_NUMBER}" || : \
    && [ ! "${COMMIT_ID}" = "" ] && params="$params -c ${COMMIT_ID}" || : \
    && [ ! "${GIT_VER}" = "" ] && params="$params -g ${GIT_VER}" || : \
    && ./packaging/common/package.sh $params

FROM scratch
COPY --from=fog-sw-builder /ros-foxy-phidget-spatial_*.deb /packages/