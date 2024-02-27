# Isaac ROS Common

Dockerfiles and scripts for development using the Isaac ROS suite.

## On x86_64 platforms:
1. Install Docker using the [instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
    1. Set up Docker's apt repository
        ```bash
        # Add Docker's official GPG key:
        sudo apt-get update
        sudo apt-get install ca-certificates curl
        sudo install -m 0755 -d /etc/apt/keyrings
        sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        sudo chmod a+r /etc/apt/keyrings/docker.asc

        # Add the repository to Apt sources:
        echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
        $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        sudo apt-get update
        ```
    2. To install the latest version, run:
        ```bash
        sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
        ```
    3. Verify that the Docker Engine installation is successful by running the hello-world image.
        ```bash
        sudo docker run hello-world
        ```
        >This command downloads a test image and runs it in a container. When the container runs, it prints a confirmation message and exits.
    4. Create the docker group.
        ```bash
        sudo groupadd docker
        ```
    5. Add your user to the docker group.
        ```bash 
        sudo usermod -aG docker $USER
        ```
    6. **Log out** and log back in so that your group membership is re-evaluated.
    7. Verify that you can run docker commands without sudo.
        ```bash
        docker run hello-world
        ```


2. Install the ``nvidia-container-toolkit`` using the [instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
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

3. Configure ``nvidia-container-toolkit`` for Docker using the [instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)
    1. Configure the container runtime by using the nvidia-ctk command:
        ```bash
        sudo nvidia-ctk runtime configure --runtime=docker
        ```
        > The nvidia-ctk command modifies the /etc/docker/daemon.json file on the host. The file is updated so that Docker can use the NVIDIA Container Runtime.


4. Restart Docker:
   ```bash
   sudo systemctl daemon-reload && sudo systemctl restart docker
   ```

5. Install [Git LFS](https://git-lfs.github.com/) to pull down all large files:
      ```bash
      sudo apt-get install git-lfs
      ```
      ```bash
      git lfs install --skip-repo
      ```

6. Create a ROS 2 workspace for experimenting with Isaac ROS:
    ```bash
    mkdir -p  ~/workspaces/isaac_ros-dev/src
    echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
    source ~/.bashrc
    ```
    > We expect to use the ``ISAAC_ROS_WS`` environmental variable to refer to this ROS 2 workspace directory, in the future.

7. Clone the repos:
    1. isaac_ros_common:
        ```bash
        git clone --depth 1 -b main https://github.com/amer-adam/isaac_ros_common.git ~/workspaces/isaac_ros-dev/src/isaac_ros_common
        ```
    2. docker files:
        ```bash
        git clone --depth 1 -b master https://github.com/amer-adam/docker.git ~/workspaces/isaac_ros-dev/src/docker
        ```

8. Build and run:
    ```bash
    cd ~/workspaces/isaac_ros-dev && ./src/isaac_ros_common/scripts/run_dev.sh
    ```
    > First time building will take some time (_5456s on my laptop_). After that will take less than one minute
    > _~~good luck soldier~~_

9. Run some tests:
    1. test microros with usb 
        ```bash
        cd /workspaces/isaac_ros-dev/
        source install/local_setup.bash 
        ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6
        ```
    1. test yolov8:
        ```bash
        cd /tmp
        yolo export model=yolov8n.pt format=engine imgsz=32
        ```