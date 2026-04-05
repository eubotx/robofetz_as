# syntax=docker/dockerfile:1
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
# Erlaubt pip-Installationen systemweit (umgeht PEP 668)
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# 1. System-Tools & SSH Client installieren
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    python3-pip \
    python3-colcon-common-extensions \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# 2. GitHub zu bekannten Hosts hinzufügen (verhindert Bestätigungs-Prompt)
RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# 3. Alle ROS 2 Pakete (wie zuvor)
RUN apt-get update && apt-get install -y \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-rqt* \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-usb-cam \
    ros-jazzy-camera-ros \
    python3-pydantic \
    ros-jazzy-camera-calibration \
    ros-jazzy-tf-transformations \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-topic-tools \
    ros-jazzy-apriltag-ros \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# 4. Python-Packages (Torch GPU, Ultralytics, SAM2 Dependencies & Fixes)
# Wir installieren PyTorch separat mit dem CUDA-Index, damit pip nicht die CPU-Version zieht
# 4. Python-Packages (Torch GPU, Ultralytics, SAM2 Dependencies & Fixes)

# 4.1 PyTorch separat mit dem CUDA-Index installieren
RUN pip3 install --no-cache-dir \
    torch \
    torchvision \
    torchaudio \
    --index-url https://download.pytorch.org/whl/cu121

# 4.2 Debian-Konflikte umgehen: Numpy & SciPy zwingend "drüber" installieren
RUN pip3 install --no-cache-dir --ignore-installed \
    "numpy>=2.2.3" \
    scipy

# 4.3 Restliche Pip-Pakete
RUN pip3 install --no-cache-dir \
    filterpy \
    pyapriltags \
    ultralytics \
    "setuptools<80" \
    bitstring \
    "scenedetect[opencv]>=0.6.2" \
    "yt-dlp>=2024.0.0"
# 5. Klonen via SSH (Nutzt den gemounteten SSH-Agent)
WORKDIR /ros2_ws/src
RUN --mount=type=ssh git clone --recursive git@github.com:eubotx/robofetz_as.git

# 6. Build
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/local_setup.bash" >> ~/.bashrc

CMD ["bash"]