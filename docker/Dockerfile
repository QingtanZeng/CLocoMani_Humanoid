FROM osrf/ros:jazzy-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
ARG PYTHON_VERSION=3.12
ARG ROS_DISTRO=jazzy
ARG Humanoid_LocoManipulation_DIR="/Humanoid_LocoManipulation_dev"

# Layer for stable base dependencies that rarely change
# Install system dependencies, ROS, Python, and development tools in a single step
# Core toolchain
# analysis tools
    # SSH and utilities
    # 图形测试工具
    # Recommended C++ tools
    # basis library
    # Python
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    g++ \
    gdb \
    cmake \
    valgrind \
    cppcheck \
    openssh-server \
    git \
    vim \
    curl \
    unzip \
    zip \
    wget \    
    ca-certificates \    
    sudo \
    gosu \
    gnupg2 \
    lsb-release \ 
    locales \
    x11-apps \
    mesa-utils \
    gettext-base \
    software-properties-common \
    linux-tools-common \
    libeigen3-dev \
    python3 \
    python3-pip \    
    && rm -rf /var/lib/apt/lists/*

COPY dependencies.txt /tmp/dependencies.txt
RUN apt-get update && \
    if [ -s /tmp/dependencies.txt ]; then \
        envsubst < /tmp/dependencies.txt | xargs apt-get install -y --no-install-recommends; \
    fi && \
    rm -rf /var/lib/apt/lists/*


# 4. 配置 NVIDIA 运行时环境变量
# 即使没有 CUDA Toolkit，这两个变量告诉 nvidia-container-toolkit 注入驱动和图形库
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,display

# Create a non-root user for development
ARG HOST_UID=1000
ARG HOST_GID=1000

# Set up SSH daemon directory
RUN mkdir -p /var/run/sshd

# Modified to use ARG and add to root group
RUN (getent passwd $HOST_UID | cut -d: -f1 | xargs -r userdel -f) && \
    (getent group $HOST_GID | cut -d: -f1 | xargs -r groupdel) && \
    groupadd -g $HOST_GID dev && \
    useradd -m -s /bin/bash -u $HOST_UID -g $HOST_GID -G root,sudo dev && \
    echo 'root:root' | chpasswd && \
    echo 'dev:dev' | chpasswd && \
    echo 'dev ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Configure SSH to permit root login with a password
# [修改点] 将默认监听端口从 22 改为 2222
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config

EXPOSE 2222

# Layer for frequently updated libraries.
# Place any library you expect to update often here.
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     && rm -rf /var/lib/apt/lists/*

# 5. 创建启动脚本 (Entrypoint) 
# 逻辑：Root 启动 SSH -> 降权切换 dev -> 执行命令
RUN echo '#!/bin/bash\n\
# [新增] 自动设置 XDG_RUNTIME_DIR 以解决 GLFW/MuJoCo 报错\n\
export XDG_RUNTIME_DIR=/tmp/runtime-dev\n\
mkdir -p $XDG_RUNTIME_DIR\n\
chmod 700 $XDG_RUNTIME_DIR\n\
\n\
# 将环境变量保存到 /etc/environment，供 SSH 使用\n\
env | grep "^DISPLAY" >> /etc/environment\n\
# [新增] 也将 XDG 变量保存给 SSH\n\
echo "XDG_RUNTIME_DIR=/tmp/runtime-dev" >> /etc/environment\n\
\n\
# add dev for vscode-server\n\
chown -R dev:dev /home/dev/.vscode-server 2>/dev/null || true \n\
# [修复] 自动 source ROS 环境变量\n\
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" /home/dev/.bashrc; then\n\
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/dev/.bashrc\n\
fi\n\
# Start SSH service\n\
service ssh start\n\
\n\
# update dev .bashrc\n\
echo "" >> /home/dev/.bashrc && \n\
echo "# 只保留路径的最后两个层级" >> /home/dev/.bashrc && \n\
echo "export PROMPT_DIRTRIM=2" >> /home/dev/.bashrc \n\
\n\
# Switch to dev user and execute command\n\
exec gosu dev "$@"' > /usr/local/bin/entrypoint.sh && \
    chmod +x /usr/local/bin/entrypoint.sh

# Set up environment variables
ENV CMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH}"
# 这行命令等同于在所有 shell 启动时 export 了这个变量
ENV PROMPT_DIRTRIM=2


WORKDIR ${Humanoid_LocoManipulation_DIR}
RUN mkdir -p ${Humanoid_LocoManipulation_DIR}
RUN chown -R dev:dev $Humanoid_LocoManipulation_DIR

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]
