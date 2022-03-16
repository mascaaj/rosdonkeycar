Started pigpiod as system service:

```
sudo systemctl enable pigpiod
```

Check if this comes up automatically on boot, this disables the need for sudo pigpiod

Need to test with bring up and teardown along with all effect sensor, seems to work ok, needs to be rossified


Study this service code using pigpio for context on cleanup and callbacks
https://roboticsbackend.com/raspberry-pi-ros-service-example-with-gpios/

https://roboticsbackend.com/raspberry-pi-gpio-interrupts-tutorial/
