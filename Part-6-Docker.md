# Docker for robotics

## Part 1: Docker for robotics

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

## Part2: Docker 101

See Articulated Robotics video: https://www.youtube.com/watch?v=SAMPOK_lazw

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

Note adding user to the docker group requires logout and login / reboot to take effect.

And now it works:

```bash
$ docker run hello-world
```

General syntax is `$ docker base-command child-command` -. Some  common commands have short versions.



### Working with images with `docker image`

#### List images

```bash
$ docker image ls
$ docker images
```

#### Download an image from registry

```bash
$ docker image pull ros:humble
$ docker pull ros:humble
```

Identify the image as `<name or repository>:<tag or variant>` 

If you don't specify the tag -> it will download the latest. 

Default registry is DockerHub but can be changed

```bash
$ docker image pull ros:humble
$ docker pull ros:humble
```

If you try to run an image you don't have -> it will download it first

#### Delete an image

```bash 
$ docker image rm hello-world
$ docker rmi hello-world
```

need `-f`  force modifier needed if ever a container was run from this image:

```bash
$ docker image rm hello-world
```

### Working with containers with `docker container`

#### Run an image in a container

```bash
$ docker container run ros:humble
$ docker run ros:humble
```

This runs in a container, and by default quits when it finishes, which is OK for production. For development you want may a terminal, so add `-it` modifiers (give me a terminal and make it interactive):

```bash
$ docker run -it ros:humble 
```

#### List containers

```bash
$ docker container ls
$ docker ps
```

#### Stop a container

Type `^d` or `# exit` from inside or from outside:

```bash
$ docker container stop container_name
$ docker stop container_name
```

#### List all containers, even stopped ones

```bash
$ docker ps -a
```

#### Start a stopped container again

```bash
$ docker container start -i container_name
$ docker start -i container_name
```

Note this way previous changes persist

This is not the same as running another container from the same image, which starts anew (in a reset state)

```bash
$ docker run -it ros:humble 
```

#### Delete containers one by one

```bash
$ docker container rm container_name
```

#### Delete all stopped containers

```bash
$ docker container prune
```

#### Tips

* Run in container with custom name (`--name`) and delete automatically when finished (`--rm`):

```bash
$ docker run --rm --name my_ros_humble -it ros:humble
```

Network settings and passing hardware devices must be set up when creating the container

* Run another terminal of the same container

 Use `$ docker exec -it container_name command_to_execute` e.g. use a call to bash to simply open a terminal

```bash
$ docker exec -it my_ros /bin/bash
```

Or run a specific command inside the container e.g. 

```bash
$ docker exec -it my_ros ls
```

#### Docker files

How do we make permanent changes to an image so we don't lose config everytime we 

With Dockerfile we create a new custom image derived from an existing one.

Define the base image with `FROM`

Run terminal commands using `RUN` 

For example you can install things with `apt` (no need for `sudo` as everything runs by default as root). Note that images tend to be totally naked, without even a text editor.

To copy our custom files (code, sources, config files, dependencies etc) use `COPY`. Do not use `ADD` 

See example [here](./docker/Dockerfile).`

#### Build an image

`-t` to give the image a name

```bash
$ docker image build -t image_name path_to_dockerfile
$ docker build -t image_name path_to_dockerfile
```

In our example after building we have the image available and can run it in a container and inside it both the file  `my_config.yaml` and the app `nano` are available

```bash
$ docker build -t nano_image .
$ docker images
$ docker run -it nano_image
(root@container)$ nano robot_config/my_config.yaml
```

#### To share outside files 

Docker uses Volumes and Bind mounts (naming is ocassionally mixed causing confusion) which are kind of shared drives. 

He recommends Bind mounts.

Mount a host path to make it accessible from inside the container:

```bash
$ docker run -it -v absolute_path_on_host:absolute_path_on_image image_name
```

Now we can edit files in the host from the container!!!

The problem is that files created inside docker will be owned by `root` and wil be difficult to handle by the user...

## Part 3

See Articulated Robotics video:  https://www.youtube.com/watch?v=RbP5cARP-SM

There are other types of ROS images that support graphics: `ros:humble` is lightweight and OK for command line. Use instead `osrf/ros:humble-desktop-full` to support graphics

You can change the user to run commands of the Dockerfile using `USER`. Bear in mind the default user when container is run will be the last one set. Useful e.g. to create files with certain permissions. Good practice to finish always with `USER root`.

You can override this defining the user running with the run command. This runs a `ros` (different name but same UID so same user for Linux) and mounts the path as mentioned earlier:

```bash
$ docker run -it --user ros -v $PWD/code/:/my_source_code nano_image
```

Can identify the user with either one of:

```
--user <username>
--user <username>:<group>
--user <uid>
--user <uid>:<gid>
```

Now files created from docker in the mount area will be owned by the default user in the host

Note: it is not necessary, but sometimes it is useful to define the username inside ros the same as in the host e.g. if we use paths that contain the username.

#### Set up `sudo`

Long command installs `sudo`, gives user `sudo` permission and allows using it without password:

```dockerfile
# Set up sudo: install sudo, add user to sudo group, allow sudo without password
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

```



#### About `apt-get install` commands

* Why start every install with an `apt-get update`? Rebuilding docker images no guarantee all Dockerfile lines will be run so keep lines self sufficient. Also forced by the good practice of deleting the package lists after install.
* Why `apt-get` instead of `apt`? `apt` more user friendly not for automation. `apt-get` clunkier but more reliable for scripts

* Why `-y` on `apt-get-install`? to avoid prompting the user  for (Y/N) questions

* Why split commands in multiple lines separated by `\` ? so version control lets you track changes (otherwise it highlights the whole line!!). Recommended to order them alphabetically for easier reading

* why delete the package lists after install command? to save space in the image and forces people to run `apt-get update` before any install
* why all this these commands in a single `RUN` command? keeps image smaller and prevents errors
* sometimes you'll see also added setting`DEBIAN_FRONTEND=noninteractive` to prevent user prompts during install. He's not convinced does not add it but in any case not recommended to set it with `ENV`

### Basic Networking

To add basic networking capabilities just add to the docker run command the modifiers: `--network=host` to share networking with the host`--ipc=host` share shared memory with host.

```bash
$ docker image build -t nano_image . #rebuild image
$ docker run -it --user ros --network=host --ipc=host -v $PWD/code/:/my_source_code nano_image #run in container
```

## Specifying entry points and commands

When making a Dockerfile we can specify an entry point and a command

An entry point is a bash script in the same directory as the Dockerfile used to setup the runtime environment.

In the Dockerfile we `COPY` the file `entrypoint.sh` into the image, execute it with `ENTRYPOINT` and we can define default commands with `CMD`

If we provide extra arguments to the `docker run` command they replace the default command set with `CMD`:

```bash
$ docker run -it --user ros --network=host --ipc=host -v $PWD/code/:/my_source_code nano_image ros2 topic list
Provided arguments: ros2 topic list
/parameter_events
/rosout
$ 
```

It starts the container, execute the command and exit 
This allows to run a program being oblivious to the fact it is running in a container in a different OS and ROS version!!

## Graphics

It is complicated. There are different options for different applications

First need permission to access X. If you issue from inside the container issue you can give permission during the current session:

```bash
$ xhost + # give permission to all users
$ xhost +local: # give permission to all local users
$ xhost +local:root # give permission to root
```

Same commands with `-` revoke these permissions

Add a new volume and set environment variable same as for the host:

```bash
$ docker run -it --user ros \
--network=host --ipc=host \
-v $PWD/code/:/my_source_code  \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
--env=DISPLAY \
nano_image 
```



Things to try if it does not work:

```bash
--gpus all
--runtime nvidia
--env="QT_X11_NO_MITSHM=1"
--env="NVIDIA_DRIVER_CAPABILITIES=all"
--env="NVIDIA_VISIBLE_DEVICES=all"
--device=/dev/dri:/dev/dri
```

Careful with the following, they are a security risk and should not be needed:

```bash
 --privileged
 --security-opt apparmor:unconfined
```

## Setting Locale and Time zones

Not needed if using official ros images as basis

Otherwise install language support and time zone support

## Autocompletion

Autocompletion should work when sourcing ROS but does not work if we do the source from the entrypoint, you actually need to type it manually which is inconvenient. 

A workaround: adding a simple `.bashrc` (which executes when launching a terminal) that includes the source commands for ROS then executing `entrypoint.sh`.

Create a `bashrc` in the same folder as the Dockerfile in host (without `.` for convenience) and then copy it into `/$HOME/.bashrc` in the container (the naming and location is compulsory)

Note there is another script you can also source from `.bashrc` to get autocompletion for `colcon` as well

Note it helps also installing `bash-completion` and `python3-argcomplete` (with a `RUN` command in Dockerfile)

## Part #4 Devices

https://www.youtube.com/watch?v=uf4zOigzTFo

Passing devices safely may be complex. 

"map all devices and run the container in `--privileged` mode" is bad advice, it exposes to security risks

### How linux handles devices

On Linux (almost) everything is a file, including (most) devices, i.e. Linux r/w from devices as if they were files. These files live in `/dev` folder

e.g. with `xxd` you this a text output when moving the mouse

```bash
$ sudo xxd /dev/input/mouse2
00000000: 18fe 0118 fa04 18f6 0918 f40c 18f2 0e18  ................
00000010: f110 18f3 0d18 f20e 18f7 0b18 fc08 0800  ................
00000020: 0808 0203 0801 0108 0100 18ff 0038 fffd  .............8..
00000030: 38fe fb38 fcf6 38fe f938 fef9 38fd f938  8..8..8..8..8..8
00000040: fdf6 38fd f838 fdf4 38fd f638 fff8 38ff  ..8..8..8..8..8.
00000050: fc28 00fb 2800 fb28 02fd 0801 0008 0200  .(..(..(........
00000060: 0804 0208 0503 0806 0408 0503 0807 0508  ................
00000070: 0906 0805 0308 0403 0807 0508 0502 0802  ................
...
```

You can have more then one of these `dev` files acting as different interfaces to the same device, e.g. the mouse is also available at:

```bash
$ sudo xxd /dev/input/event15
00000000: 1746 3c65 0000 0000 09e6 0d00 0000 0000  .F<e............
00000010: 0200 0000 ffff ffff 1746 3c65 0000 0000  .........F<e....
00000020: 09e6 0d00 0000 0000 0200 0100 ffff ffff  ................
00000030: 1746 3c65 0000 0000 09e6 0d00 0000 0000  .F<e............
00000040: 0000 0000 0000 0000 1746 3c65 0000 0000  .........F<e....
00000050: 860d 0e00 0000 0000 0200 0000 f6ff ffff  ................
...
```

You can access them using a slightly more descriptive links:

```bash
$ ls -l /dev/input/by-id
total 0
lrwxrwxrwx 1 root root  9 Mar 27  2023 usb-Chicony_Electronics_Co._Ltd._LG_Camera_0001-event-if00 -> ../event4
lrwxrwxrwx 1 root root 10 Oct 27 23:34 usb-Logitech_USB_Receiver-if01-event-kbd -> ../event14
lrwxrwxrwx 1 root root 10 Oct 27 23:34 usb-Logitech_USB_Receiver-if01-event-mouse -> ../event15
lrwxrwxrwx 1 root root  9 Oct 27 23:34 usb-Logitech_USB_Receiver-if01-mouse -> ../mouse2

```







### Connecting a gamepad



### Connecting a depth camera



### Connecting a serial device 

