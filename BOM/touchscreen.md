# 5inch HDMI LCD (B) Waveshare

## Setup in RPi running Ubuntu Mate

Note the [official wiki](https://www.waveshare.com/wiki/5inch_HDMI_LCD_(B)?Amazon ) refers to Raspbian and there are minor changes in Ubuntu Mate

RPi does not detect automatically the screen configuration, need to add the values to a configuration file:

```bash
(RPi)$ sudo nano /boot/firmware/usercfg.txt
```

Add this section at the end of the file and save:

```
# Waveshare HDMI touchscreen configuration added by MH 21.11
hdmi_group=2
hdmi_mode=87
hdmi_cvt 800 480 60 6 0 0 0
hdmi_drive=1
```

Reboot with `$ sudo reboot`

## Screen calibration

Install calibration software:

```bash
(RPi)$ sudo apt-get install xinput-calibrator
(RPi)$ sudo apt-get install xserver-xorg-input-evdev
(RPi)$ sudo cp -rf /usr/share/X11/xorg.conf.d/10-evdev.conf /usr/share/X11/xorg.conf.d/45-evdev.conf
(RPi)$ sudo reboot
```

This will add a new option in the menu **System Settings -> Administration -> Calibrate Touchscreen**. 

Follow the instructions to calibrate the screen with the stylus. At the end of the process the following message is shown:   
```
Calibrating EVDEV driver for "WaveShare ws170120" id=6
	current calibration values (from XInput): min_x=135, max_x=3925 and min_y=333, max_y=3913

Doing dynamic recalibration:
	Setting calibration data: 129, 3929, 310, 3925
	--> Making the calibration permanent <--
  copy the snippet below into '/etc/X11/xorg.conf.d/99-calibration.conf' (/usr/share/X11/xorg.conf.d/ in some distro's)
Section "InputClass"
	Identifier	"calibration"
	MatchProduct	"WaveShare ws170120"
	Option	"Calibration"	"129 3929 310 3925"
	Option	"SwapAxes"	"0"
EndSection
```
To make the settings permanent create a new config file: 
```bash
(RPi)$ sudo nano /usr/share/X11/xorg.conf.d/99-calibration.conf
```
Paste this section to the file and save it. Note the values will be unique for each device and setup.
```
Section "InputClass"
	Identifier	"calibration"
	MatchProduct	"WaveShare ws170120"
	Option	"Calibration"	"135 3925 333 3913"
	Option	"SwapAxes"	"0"
EndSection
```

Reboot with `$ sudo reboot`

## Flip screen

To flip the screen run:

```bash
(RPi):$ DISPLAY=:0 xrandr --output HDMI-1 --rotate inverted
```

This only works if you are already logged in in the robot GUI, otherwise it throws an error `Invalid MIT-MAGIC-COOKIE-1 key` therefore I enabled autologin (cfr. https://techpiezo.com/linux/enable-or-disable-automatic-login-in-ubuntu-20-04/):

1. The display manager is `lightdm` (to find out, run `(RPI):$ systemctl status display-manager`)

2. Edit the config file with `(RPi):$ sudo nano /etc/lightdm/lightdm.conf` and append:

```
[Seat:*]
autologin-user=mhered
autologin-user-timeout=0
```

3. reboot

To flip also the touch input:

```bash
(RPi):$ DISPLAY=:0 xinput set-prop "WaveShare ws170120" --type=float "Coordinate Transformation Matrix" -1 0 1 0 -1 1 0 0 1
```

To run both automatically at startup run `(RPi):$ sudo nano /etc/xdg/autostart/setscreen.desktop`  and add the following lines:

```
[Desktop Entry]
Name=Set Screen Rotation
Exec=/bin/bash -c 'sleep 10 && xrandr -o inverted && xinput set-prop "WaveShare ws170120" --type=float "Coordinate Transformation Matrix" -1 0 1 0 -1 1 0 0 1'
Type=Application
```

This waits 10s after login, then executes. 
