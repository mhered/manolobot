# UUGEAR MEGA4 - 4-Port USB Hub

The UUGear MEGA4 is a 4-Port USB 3.1 PPPS (Per-Port Power Switching) hub designed for theRPi

adds 4 USB 3.1 ports

power on each USB port can be individually turned on/off  via software or via UUGear Web Interface (UWI) on PC, Tablet or Smartphone. This allows resetting devices or saving energy (e.g. the Lidar)

can be externally powered connecting a 5V power supply to its  USB-C connector to increase its current output ability up to 5A, and 2.5A per port

there is a protection to prevent the hub from back-powering the RPi.

## Install software

The software is needed to turn on/off the power on each USB port.

Download and run the installation script to install the software in `mega4` directory and the UUGear Web Interface (UWI) in `uwi` directory in the RPi:

```bash
(RPi):$ wget https://www.uugear.com/repo/MEGA4/install.sh
(RPi):$ sudo sh install.sh
```

Note: `sudo` is needed to run the script because it attempts to disable USB auto-suspend in `/boot/cmdline.txt`.

After installing the software, reboot the RPi so the UWI server will run in the background. 

With the default  configuration you should be able to access the MEGA4 via UWI on address https://raspberrypi:8000/mega4/ (Note:???)

## Specs

* Dimensions: 85 x 56 x 9 mm (USB plug protrudes 7mm)

* Weight: 60g (29g without accesories)

* Output Current

  - Bus-powered only: up to 1.2A in total (limited by Raspberry Pi 4B)

  - With supplemental power supply: up to 5A in total, and 2.5A per port (also depends on the power supply)

## References

https://www.uugear.com/product/mega4-4-port-usb-3-ppps-hub-for-raspberry-pi-4b/

https://thepihut.com/products/mega4-4-port-usb-3-1-ppps-hub-for-raspberry-pi-4

https://www.uugear.com/doc/MEGA4_UserManual.pdf

The software is a wrapper of utility uhubctl: https://github.com/mvp/uhubctl

## First try to switch off Lidar

1. install `uhubctl`:

```bashç
(RPi):$ sudo apt install uhubctl
```

2. find devices:

```bash
(RPi):$ sudo uhubctl
Current status for hub 2-2 [2109:0817 VIA Labs, Inc. USB3.0 Hub, USB 3.10, 4 ports]
  Port 1: 02a0 power 5gbps Rx.Detect
  Port 2: 02a0 power 5gbps Rx.Detect
  Port 3: 02a0 power 5gbps Rx.Detect
  Port 4: 02a0 power 5gbps Rx.Detect
Current status for hub 2 [1d6b:0003 Linux 5.4.0-1079-raspi xhci-hcd xHCI Host Controller 0000:01:00.0, USB 3.00, 4 ports]
  Port 1: 02a0 power 5gbps Rx.Detect
  Port 2: 0263 power 5gbps U3 enable connect [2109:0817 VIA Labs, Inc. USB3.0 Hub, USB 3.10, 4 ports]
  Port 3: 02a0 power 5gbps Rx.Detect
  Port 4: 02a0 power 5gbps Rx.Detect
Current status for hub 1-1.2 [2109:2817 VIA Labs, Inc. USB2.0 Hub, USB 2.10, 4 ports]
  Port 1: 0103 power enable connect [0eef:0005 WaveShare ws170120 WaveShare ws170120]
  Port 2: 0103 power enable connect [10c4:ea60 Silicon Labs CP2102 USB to UART Bridge Controller 0001]
  Port 3: 0103 power enable connect [1a86:7523 USB2.0-Ser!]
  Port 4: 0100 power
Current status for hub 1 [1d6b:0002 Linux 5.4.0-1079-raspi xhci-hcd xHCI Host Controller 0000:01:00.0]
  Port 1: 0503 power highspeed enable connect [2109:3431 USB2.0 Hub, USB 2.10, 4 ports]
```

3. switch off (but does not seem to work):

```bash
(RPi):$ sudo uhubctl -a off -l 1-1.2 -p 2
Current status for hub 1-1.2 [2109:2817 VIA Labs, Inc. USB2.0 Hub, USB 2.10, 4 ports]
  Port 2: 0103 power enable connect [10c4:ea60 Silicon Labs CP2102 USB to UART Bridge Controller 0001]
Sent power off request
New status for hub 1-1.2 [2109:2817 VIA Labs, Inc. USB2.0 Hub, USB 2.10, 4 ports]
  Port 2: 0000 off
```



