
# Based on code written by Ho Yun "Bobby" Chan @ SparkFun Electronics
# and released under the MIT License (http://opensource.org/licenses/MIT)
#
#        https://learn.sparkfun.com/tutorials/raspberry-pi-safe-reboot-and-shutdown-button/

# Tutorial to adjust the "rc.local" file and run at startup:
#
#        https://learn.sparkfun.com/tutorials/raspberry-pi-safe-reboot-and-shutdown-button
#

import time
import RPi.GPIO as GPIO

# Pin definition
shutdown_pin = 17

# Suppress warnings
GPIO.setwarnings(False)

# Use "GPIO" pin numbering
GPIO.setmode(GPIO.BCM)

# Use built-in internal pullup resistor so the pin is not floating
# if using a momentary push button without a resistor.
GPIO.setup(shutdown_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# modular function to shutdown Pi
def shut_down():
    print("Shutting down")
    command = "/usr/bin/sudo /sbin/shutdown -h now"
    import subprocess
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = process.communicate()[0]
    print(output)

# modular function to shutdown Pi
def reboot():
    print("Rebooting")
    command = "/usr/bin/sudo /sbin/shutdown -r now"
    import subprocess
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = process.communicate()[0]
    print(output)

def press_button():
    start_time=time.time()
    time_pressed=0
    noise_time=1
    hold_time=4

    while GPIO.input(shutdown_pin) == False and (time_pressed < hold_time) :
        time_pressed=time.time()-start_time

    if time_pressed < noise_time:
        print("Ignoring short button press")
    elif noise_time < time_pressed < hold_time:
        print("Detected a short button press: Rebooting the RPi")
        reboot()
    else:
        print("Detected a long button press: Shutting down the RPi")
        shut_down()
    

# Main loop
while True:
    #short delay, otherwise this code will take up a lot of the Pi's processing power
    time.sleep(0.5)

    # uncomment for troubleshooting: 
    # print('GPIO state is = ', GPIO.input(shutdown_pin))
    if GPIO.input(shutdown_pin) == False:
        press_button()
