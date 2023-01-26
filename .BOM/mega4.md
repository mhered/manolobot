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

Note: `sudo` is needed because to disable USB auto-suspend in `/boot/cmdline.txt`.

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