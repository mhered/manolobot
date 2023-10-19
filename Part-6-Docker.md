## Docker for robotics

See Articulated Robotics video: https://www.youtube.com/watch?v=XcJzOYe3E6M

### What is docker?

Vocabulary: 

* Run an image to create a container

* Start and stop containers 

* Destroy and Run again to reset 

### 6 reasons to use docker in robotics

1. Running incompatible software/OS/hardware
2. Standardized build/test environment
3. Standardized development environment
4. Simplified deployment
5. Infrastructure as code
6. Cloud development

### Docker demo

Currently using ROS2 Foxy in Ubuntu 20.04:

```bash
$ lsb_release -a
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 20.04.6 LTS
Release:	20.04
Codename:	focal
$ ls /opt/ros
foxy  noetic
```

Pull a docker image of ROS humble:

```bash
$ docker image pull ros:humble
...
Status: Downloaded newer image for ros:humble
docker.io/library/ros:humble
```

```bash
$ docker image ls
REPOSITORY   TAG      IMAGE ID       CREATED      SIZE
ros          humble   a2cfb6110191   4 days ago   752MB
...
```

Run the image (this creates container `f17bdfd41c3e`  and runs as `root`):

```bash
$ docker run -it ros:humble
root@f17bdfd41c3e:/# 
```

Note inside the container we run Ubuntu 22.04 and ROS2 Humble:

```bash
# lsb_release -a
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 22.04.3 LTS
Release:	22.04
Codename:	jammy
# ls /opt/ros
humble
```

We can list containers running:

```bash
(Terminal 2)$ docker container ls
CONTAINER ID   IMAGE        COMMAND                  CREATED         STATUS         PORTS     NAMES
f17bdfd41c3e   ros:humble   "/ros_entrypoint.sh …"   3 minutes ago   Up 3 minutes             festive_roentgen
```

If we create a file, destroy the container with `^d` or typing `exit` then run it again with the same command the file is no longer there.

## Docker 101

https://www.youtube.com/watch?v=SAMPOK_lazw

### Installation

He recommends installing the docker engine using the convenience script from here: https://docs.docker.com/engine/install/ubuntu/

```bash
$ curl -fsSL https://get.docker.com -o get-docker.sh
$ sudo sh ./get-docker.sh
```

Then to prevent docker from running as `root` by default:

```bash
$ sudo groupadd docker
groupadd: group 'docker' already exists
$ sudo usermod -aG docker $USER
$ systemctl is-enabled docker
enabled
```

To enable if needed:

```bash
$ sudo systemctl enable docker.service
$ sudo systemctl enable containerd.service
```

Note adding user to the docker group requires logout and login / reboot to take effect

Note common commands have short versions.

### Working with images

List images:

```bash
$ docker image ls
$ docker images
```

Download an image `<name>:<tag>` from registry:

```bash
$ docker image pull ros:humble
$ docker pull ros:humble
```

If you don't specify image it will download the latest. If you try to run an image you don't have it will download it first

Delete an image:

```bash 
$ docker image rm -f hello-world
```

`-f`  force modifier needed if ever a container was run from this image

### Working with containers

Run a container:

```bash
$ docker container run ros:humble
$ docker run ros:humble
```

This runs and stops the container, if you want a terminal you add `-it` (give me a terminal and make it interactive) modifier:

```bash
$ docker run -it ros:humble 
```

List containers:

```bash
$ docker container ls
$ docker ps
```

Stop with `^d` or `# exit` from inside or `$ docker container stop container_name` from outside

List all containers even stopped ones:

```bash
$ docker ps -a
```

