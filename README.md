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

1. Configure ``nvidia-container-toolkit`` for Docker using the [instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)
    1. Configure the container runtime by using the nvidia-ctk command:
        ```bash
        sudo nvidia-ctk runtime configure --runtime=docker
        ```
        > The nvidia-ctk command modifies the /etc/docker/daemon.json file on the host. The file is updated so that Docker can use the NVIDIA Container Runtime.


1. Restart Docker:
   ```bash
   sudo systemctl daemon-reload && sudo systemctl restart docker
   ```

1. Install [Git LFS](https://git-lfs.github.com/) to pull down all large files:
      ```bash
      sudo apt-get install git-lfs
      ```
      ```bash
      git lfs install --skip-repo
      ```

1. Create a ROS 2 workspace for experimenting with Isaac ROS:
    ```bash
    mkdir -p  ~/workspaces/isaac_ros-dev/src
    echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
    source ~/.bashrc
    ```
    > We expect to use the ``ISAAC_ROS_WS`` environmental variable to refer to this ROS 2 workspace directory, in the future.

1. Clone the repos:
    1. isaac_ros_common:
        ```bash
        git clone --depth 1 -b main https://github.com/amer-adam/isaac_ros_common.git ~/workspaces/isaac_ros-dev/src/isaac_ros_common
        ```
    2. docker files:
        ```bash
        git clone --depth 1 -b master https://github.com/amer-adam/docker.git ~/workspaces/isaac_ros-dev/src/docker
        ```

1. Build and run:
    ```bash
    cd ~/workspaces/isaac_ros-dev && ./src/isaac_ros_common/scripts/run_dev.sh
    ```
    > First time building will take some time (_5456s on my laptop_). After that will take less than one minute
    > _~~good luck soldier~~_ :fearful:

1. Run some tests:
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


## On Jetson:
1. Confirm that you have installed `Jetpack 5.1.2` by running the following command. Confirm that the output has the terms `R35 (release), REVISION: 4.1`.
    ```bash
    cat /etc/nv_tegra_release
    ```
1. Run the following command to set the GPU and CPU clock to max. See [Maximizing Jetson Performance](https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonOrinNxSeriesAndJetsonAgxOrinSeries.html?highlight=maxn#maximizing-jetson-orin-performance) for more details.
    ```bash
    sudo /usr/bin/jetson_clocks
    ```
1. Run the following command to set the to power to MAX settings. See [Power Mode Controls](https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonOrinNxSeriesAndJetsonAgxOrinSeries.html?highlight=maxn#power-mode-controls) for more details.
    ```bash
    sudo /usr/sbin/nvpmodel -m 0
    ```
1. Add your user to the `docker` group.
    ```bash
    sudo usermod -aG docker $USER
    newgrp docker
    ```

1. Setup Docker. From the official Docker install instructions ([here](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)), install the `docker-buildx-plugin`.
    ```bash
    # Add Docker's official GPG key:
    sudo apt-get update
    sudo apt-get install ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    # Add the repository to Apt sources:
    echo \
    "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update

    sudo apt install docker-buildx-plugin
    ```
1. Change the ownership of the `/ssd` directory.
    ```bash
    sudo chown ${USER}:${USER} /ssd
    ```
1. Stop the Docker service.
    ```bash
    sudo systemctl stop docker
    ```
1. Move the existing Docker folder.
    ```bash
    sudo du -csh /var/lib/docker/ && \
    sudo mkdir /ssd/docker && \
    sudo rsync -axPS /var/lib/docker/ /ssd/docker/ && \
    sudo du -csh  /ssd/docker/
    ```
1. Edit `/etc/docker/daemon.json`
    ```bash
    sudo vi /etc/docker/daemon.json
    ```
    insert `"data-root"` line similar to the following:
    ```json
    {
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia",
    "data-root": "/ssd/docker"
    }
    ```
1. Rename the old Docker data directory.
    ```bash
    sudo mv /var/lib/docker /var/lib/docker.old
    ```
1. Restart the Docker daemon.
    ```bash
    sudo systemctl daemon-reload && \
    sudo systemctl restart docker && \
    sudo journalctl -u docker
    ```
1. Test Docker on SSD:
    1. [Terminal 1] Open a terminal to monitor the disk usage while pulling a Docker image.
        ```bash
        watch -n1 df
        ```
    1. [Terminal 2] Open a new terminal and begin the Docker pull.
        ```bash
        docker pull nvcr.io/nvidia/l4t-base:r35.2.1
        ```
    1. [Terminal 1] Observe that the disk usage on `/ssd` goes up as the container image is downloaded and extracted.
        ```bash
        ~$ docker image ls
        REPOSITORY                  TAG       IMAGE ID       CREATED        SIZE
        nvcr.io/nvidia/l4t-base     r35.2.1   dc07eb476a1d   7 months ago   713MB
        ```
1. Final Verification. Reboot your Jetson, and verify that you observe the following:
    ```bash
    ~$ sudo blkid | grep nvme
    /dev/nvme0n1: UUID="9fc06de1-7cf3-43e2-928a-53a9c03fc5d8" TYPE="ext4"

    ~$ df -h
    Filesystem      Size  Used Avail Use% Mounted on
    /dev/mmcblk1p1  116G   18G   94G  16% /
    none            3.5G     0  3.5G   0% /dev
    tmpfs           3.6G  108K  3.6G   1% /dev/shm
    tmpfs           734M   35M  699M   5% /run
    tmpfs           5.0M  4.0K  5.0M   1% /run/lock
    tmpfs           3.6G     0  3.6G   0% /sys/fs/cgroup
    tmpfs           734M   88K  734M   1% /run/user/1000
    /dev/nvme0n1    458G  824M  434G   1% /ssd

    ~$ docker info | grep Root
    Docker Root Dir: /ssd/docker

    ~$ sudo ls -l /ssd/docker/
    total 44
    drwx--x--x  4 root root 4096 Mar 22 11:44 buildkit
    drwx--x---  2 root root 4096 Mar 22 11:44 containers
    drwx------  3 root root 4096 Mar 22 11:44 image
    drwxr-x---  3 root root 4096 Mar 22 11:44 network
    drwx--x--- 13 root root 4096 Mar 22 16:20 overlay2
    drwx------  4 root root 4096 Mar 22 11:44 plugins
    drwx------  2 root root 4096 Mar 22 16:19 runtimes
    drwx------  2 root root 4096 Mar 22 11:44 swarm
    drwx------  2 root root 4096 Mar 22 16:20 tmp
    drwx------  2 root root 4096 Mar 22 11:44 trust
    drwx-----x  2 root root 4096 Mar 22 16:19 volumes

    ~$ sudo du -chs /ssd/docker/
    752M    /ssd/docker/
    752M    total

    ~$ docker info | grep -e "Runtime" -e "Root"
    Runtimes: io.containerd.runtime.v1.linux nvidia runc io.containerd.runc.v2
    Default Runtime: nvidia
    Docker Root Dir: /ssd/docker
    ```
    >Check out our [troubleshooting](https://nvidia-isaac-ros.github.io/troubleshooting/dev_env.html) section for issues with setting up your development environment


1. Restart Docker:
   ```bash
   sudo systemctl daemon-reload && sudo systemctl restart docker
   ```

1. Install [Git LFS](https://git-lfs.github.com/) to pull down all large files:
      ```bash
      sudo apt-get install git-lfs
      ```
      ```bash
      git lfs install --skip-repo
      ```

1. Create a ROS 2 workspace for experimenting with Isaac ROS:
    ```bash
    mkdir -p  /ssd/workspaces/isaac_ros-dev/src
    echo "export ISAAC_ROS_WS=/ssd/workspaces/isaac_ros-dev/" >> ~/.bashrc
    source ~/.bashrc
    ```
    > We expect to use the ``ISAAC_ROS_WS`` environmental variable to refer to this ROS 2 workspace directory, in the future.

1. Clone the repos:
    1. isaac_ros_common:
        ```bash
        git clone --depth 1 -b main https://github.com/amer-adam/isaac_ros_common.git ~/ssd/workspaces/isaac_ros-dev/src/isaac_ros_common
        ```
    2. docker files:
        ```bash
        git clone --depth 1 -b master https://github.com/amer-adam/docker.git ~/ssd/workspaces/isaac_ros-dev/src/docker
        ```

1. Build and run:
    ```bash
    cd ~/ssd/workspaces/isaac_ros-dev/ && ./src/isaac_ros_common/scripts/run_dev.sh
    ```
1. Run some tests:
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
