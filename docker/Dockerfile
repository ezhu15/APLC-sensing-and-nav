FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz \
    liburdfdom-tools \
    && rm -rf /var/lib/apt/lists/*

# Cache package dependencies separately from source changes.
COPY src/aplc_description/package.xml \
    /tmp/dependencies/aplc_description/package.xml
COPY src/aplc_control/package.xml \
    /tmp/dependencies/aplc_control/package.xml
COPY src/aplc_gazebo/package.xml \
    /tmp/dependencies/aplc_gazebo/package.xml
COPY src/aplc_localization/package.xml \
    /tmp/dependencies/aplc_localization/package.xml

RUN source /opt/ros/jazzy/setup.bash \
    && rosdep update \
    && apt-get update \
    && rosdep install \
         --from-paths /tmp/dependencies \
         --ignore-src --rosdistro jazzy -r -y \
    && rm -rf /var/lib/apt/lists/*

ARG LOCAL_UID=1000
ARG LOCAL_GID=1000

# Reuse existing IDs (Ubuntu images may already have a user with UID 1000).
RUN test "$LOCAL_UID" -gt 0 && test "$LOCAL_GID" -gt 0 \
    && if ! getent group "$LOCAL_GID" > /dev/null; then \
         groupadd --gid "$LOCAL_GID" dev; \
       fi \
    && existing_user="$(getent passwd "$LOCAL_UID" | cut -d: -f1)" \
    && if [ -n "$existing_user" ]; then \
         if [ "$existing_user" != dev ]; then usermod --login dev "$existing_user"; fi; \
         usermod --gid "$LOCAL_GID" --home /home/dev --shell /bin/bash dev; \
       else \
         useradd --uid "$LOCAL_UID" --gid "$LOCAL_GID" \
           --home-dir /home/dev --shell /bin/bash dev; \
       fi \
    && mkdir -p /home/dev /ws \
    && chown "$LOCAL_UID:$LOCAL_GID" /home/dev /ws

ENV HOME=/home/dev
USER dev
WORKDIR /ws
CMD ["bash"]
