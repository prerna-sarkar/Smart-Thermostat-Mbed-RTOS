# Smart-Thermostat-Mbed-RTOS

The Thermostat displays Temperature, Pressure and Humidity measurements and allows the user to set the desired temperature using their smartphone. There are several different modes the Thermostat can be in:
The temperature can be displayed in either degC or degF mode.
The temperature, pressure and humidity measurements can either display current readings or average readings.
Depending on the desired temperature the Thermostat can either be in heating or cooling mode.
The thermostat can be also be set to a mode that allows a user to change the current temperature -> used to demo functionality

Hardware Used
● Mbed LPC1768
● uLCD-144G2 – LCD for Display
● BME280 – Temperature, Pressure, Humidity sensor
● Bluetooth module
● 2 Pushbuttons – to change units and to see rolling average of values
● RGB Led – to show status (Heating(Red)/Cooling(Blue))
● Speaker
● Jumper wires
● Breadboard
● Power Switch
● Fan

Software Implementation
● Thread 1(Main): Display values from Temperature, Pressure, Humidity sensor, based on set temp, display whether Heating or Cooling (along with speaker beep and led indicator)
● Thread 2: Display the Desired Temp
● Thread 3: mbed's LED (heartbeat)
● Thread 4: Read bluetooth control commands – Set Mode/Desired Temp values
● PushButton interrupts
