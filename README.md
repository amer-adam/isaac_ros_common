# Isaac ROS Common

Dockerfiles and scripts for development using the Isaac ROS suite.

## On x86_64 platforms:

1. Install the ``nvidia-container-toolkit`` using the [instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
    1. Configure the production repository:
        ```bash
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
        ```

    2. Update the packages list from the repository:
        ```bash
        sudo apt-get update
        ```

    3. Install the NVIDIA Container Toolkit packages:
        ```bash
        sudo apt-get install -y nvidia-container-toolkit
        ```

2. Configure ``nvidia-container-toolkit`` for Docker using the [instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)
    1. Configure the container runtime by using the nvidia-ctk command:
        ```bash
        sudo nvidia-ctk runtime configure --runtime=docker
        ```
        > The nvidia-ctk command modifies the /etc/docker/daemon.json file on the host. The file is updated so that Docker can use the NVIDIA Container Runtime.


3. Restart Docker:
   ```bash
   sudo systemctl daemon-reload && sudo systemctl restart docker
   ```

4. Install [Git LFS](https://git-lfs.github.com/) to pull down all large files:
      ```bash
      sudo apt-get install git-lfs
      ```
      ```bash
      git lfs install --skip-repo
      ```

5. Create a ROS 2 workspace for experimenting with Isaac ROS:
    ```bash
    mkdir -p  ~/workspaces/isaac_ros-dev/src
    ```
    ```bash
    echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
    ```
    ```bash
    source ~/.bashrc
    ```
    > We expect to use the ``ISAAC_ROS_WS`` environmental variable to refer to this ROS 2 workspace directory, in the future.

6. Clone the repos:
    1. isaac_ros_common:
    ```bash
    git clone --depth 1 -b main https://github.com/amer-adam/isaac_ros_common.git ~/workspaces/isaac_ros-dev/src/isaac_ros_common
    ```
    2. docker files:
    ```bash
    git clone --depth 1 -b main https://github.com/amer-adam/docker.git ~/workspaces/isaac_ros-dev/src/docker
    ```
