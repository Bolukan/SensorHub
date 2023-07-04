# SensorHub
![Travis build](https://app.travis-ci.com/Bolukan/SensorHub.svg)

SensorHub is responsible for interpreting readings from multiple sensors and transmitting them via MQTT. It can serve as an example of how to solve such a problem.

### Situation
In my house there are six wired sensors, including four PIR sensors and two contact sensors. They need 12V to operate and separately wires are used for the signal. The signal resistance is 4.7K ohm when inactive and 9.4K ohm when active.

### Solution
I designed a PCB that supplies 12V and can accommodate an ESP32 S2 mini. The MCU delivers 3.3V to one side of the six signal lines. At the other side a voltage divider is created by placing a 4.7K ohm resistor between the signal output and ground. Thus theoretically 1.65V will be measured when inactive and 1.1V when active.  

Measurements are taken at a rate of 25 per second, and the values are smoothed and evaluated. Any change in status is sent via MQTT. The current status is repeated at least every five minutes.

A total of eight LEDs can be controlled, with two of them reserved for signaling connection and MQTT traffic, while the remaining six LEDs can be utilized to display the status of the sensors. 

