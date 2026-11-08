FROM ros:jazzy-ros-base-noble AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential cmake git python3-colcon-common-extensions python3-pip python3-wheel \
      python3-opencv python3-numpy python3-scipy python3-matplotlib python3-pil \
      python3-yaml python3-bitstring python3-serial \
      ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher ros-jazzy-teleop-twist-joy ros-jazzy-joy \
      ros-jazzy-image-proc ros-jazzy-cv-bridge ros-jazzy-vision-msgs \
      ros-jazzy-tf-transformations ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-usb-cam ros-jazzy-camera-ros ros-jazzy-topic-tools \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      xvfb x11-utils x11vnc openbox novnc websockify fonts-dejavu-core \
      libgl1-mesa-dri libglx-mesa0 mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Keep cv_bridge and OpenCV on Noble's NumPy ABI.
RUN python3 -m pip install --no-cache-dir --no-deps --target /opt/robot-python \
      filterpy==1.4.5 pyapriltags==3.4.3.1
ENV PYTHONPATH=/opt/robot-python

FROM dependencies AS development
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      gosu python3-pytest ros-jazzy-ament-flake8 ros-jazzy-ament-pep257 \
      ros-jazzy-ament-copyright \
    && rm -rf /var/lib/apt/lists/* \
    && if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && useradd --uid 1000 --create-home --user-group --shell /bin/bash robot
COPY docker/ros-env.sh /usr/local/bin/ros-env
COPY docker/sim.sh /usr/local/bin/robofetz-sim
COPY docker/smoke.sh /usr/local/bin/robofetz-smoke
COPY docker/dev-entrypoint.sh /usr/local/bin/robofetz-dev-entrypoint
RUN install -d -m 1777 /tmp/.X11-unix \
    && sed -i 's/\r$//' /usr/local/bin/{ros-env,robofetz-sim,robofetz-smoke,robofetz-dev-entrypoint} \
    && chmod 755 /usr/local/bin/{ros-env,robofetz-sim,robofetz-smoke,robofetz-dev-entrypoint} \
    && printf '\nsource /opt/ros/jazzy/setup.bash\nif [ -f /ws/install/local_setup.bash ]; then source /ws/install/local_setup.bash; fi\n' >> /etc/bash.bashrc
WORKDIR /ws
ENV HOME=/home/robot
ENTRYPOINT ["/usr/local/bin/robofetz-dev-entrypoint"]
CMD ["sleep", "infinity"]

FROM dependencies AS simulation
WORKDIR /ws
COPY src/ src/
COPY docker/ros-env.sh /usr/local/bin/ros-env
COPY docker/sim.sh /usr/local/bin/robofetz-sim
COPY docker/smoke.sh /usr/local/bin/robofetz-smoke
RUN install -d -m 1777 /tmp/.X11-unix \
    && sed -i 's/\r$//' /usr/local/bin/{ros-env,robofetz-sim,robofetz-smoke} \
    && chmod 755 /usr/local/bin/{ros-env,robofetz-sim,robofetz-smoke} \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --merge-install --executor sequential \
    && useradd --create-home --shell /bin/bash robot \
    && chown -R robot:robot /ws
USER robot
ENV HOME=/home/robot
RUN ros-env robofetz-smoke
ENTRYPOINT ["/usr/local/bin/ros-env"]
CMD ["robofetz-sim"]
