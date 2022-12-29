Tutorial: https://larrylisky.com/2016/11/24/enabling-raspberry-pi-camera-v2-under-ubuntu-mate/ rather old, from 2016

https://wesleych3n.medium.com/enable-camera-in-raspberry-pi-4-with-64-bit-ubuntu-21-04-d97ce728db9d

Note: Connect ribbon with blue side facing the USB connectors

```bash
(RPi)$ sudo apt-get update
(RPi)$ sudo apt-get upgrade
```

Installation of `raspi-config` failed

```bash
(RPi)$ sudo apt-get install raspi-config rpi-update
Reading package lists... Done
Building dependency tree       
Reading state information... Done
E: Unable to locate package raspi-config
E: Unable to locate package rpi-update
```

On the other hand the file `/boot/firmware/config.txt` already had the lines:

```
start_x=1
gpu_mem=128
```

`/dev/video0/` was detected:

```bash
$ ls -ltrh /dev/video*
crw-rw----+ 1 root video 81, 0 Apr 21  2022 /dev/video13
crw-rw----+ 1 root video 81, 1 Apr 21  2022 /dev/video10
crw-rw----+ 1 root video 81, 2 Apr 21  2022 /dev/video14
crw-rw----+ 1 root video 81, 4 Apr 21  2022 /dev/video16
crw-rw----+ 1 root video 81, 3 Apr 21  2022 /dev/video15
crw-rw----+ 1 root video 81, 7 Apr 21  2022 /dev/video12
crw-rw----+ 1 root video 81, 6 Apr 21  2022 /dev/video0
crw-rw----+ 1 root video 81, 5 Apr 21  2022 /dev/video11

```

And `cheese` was already installed

I tested with `cheese`  (cfr. https://linuxconfig.org/how-to-test-webcam-on-ubuntu-20-04-focal-fossa) and the camera works but is very slow, has low resolution, and weird colors.

This article: https://raspberrypi.stackexchange.com/questions/114035/picamera-and-ubuntu-20-04-arm64 recommends using `opencv` but mentions there is active development to support `libcamera`? in ubuntu arm 64b.

However this guy claims `raspiconfig` can be installed in Ubuntu Mate: https://ubuntu-mate.community/t/install-raspi-config-on-ubuntu-mate-20-10-and-higher/23974

To be continued... 

