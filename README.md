# Smart-Thermostat-Mbed-RTOS

The Thermostat displays Temperature, Pressure and Humidity measurements and allows the user to set the desired temperature using their smartphone. There are several different modes the Thermostat can be in:
The temperature can be displayed in either degC or degF mode.
The temperature, pressure and humidity measurements can either display current readings or average readings.
Depending on the desired temperature the Thermostat can either be in heating or cooling mode.
The thermostat can be also be set to a mode that allows a user to change the current temperature -> used to demo functionality

# Hardware Used
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

# Software Implementation

This is a program written in C++ for an embedded system. It includes various libraries such as mbed.h, uLCD_4DGL.h, rtos.h, PinDetect.h, BME280.h, and wave_player.h. These libraries provide functions and classes for controlling various hardware peripherals and performing various tasks.

1. The program starts by defining some constants and variables. It then defines a class RGBLed that controls an RGB LED using three PWM pins. It also defines various objects such as uLCD, led, speaker, and pb1, among others. These objects represent various hardware peripherals such as a display, LED, speaker, and pushbutton.

2. The program also defines various interrupt routines such as pb_hit, pb_hit2, heating_on_audio, cooling_on_audio, and tock. These interrupt routines are called when certain events occur, such as when a pushbutton is pressed or when a timer expires.

3. The program then defines several threads, each of which performs a certain task. These threads are executed in parallel, with the operating system scheduling them based on their priorities.

4. The main function starts by initializing various hardware peripherals and setting up various interrupts and threads. It then enters an infinite loop, which continuously reads the current temperature, humidity, and pressure from a BME280 sensor, updates the display, and checks for control commands from Bluetooth.

5. If the desired temperature is higher than the current temperature, the program turns on a heating mode and plays an audio signal.

6. If the desired temperature is lower than the current temperature, the program turns on a cooling mode and plays an audio signal.

7. The program also allows the user to switch between Celsius and Fahrenheit modes and between sliding window average values and instantaneous values [PushButton interrupts].

# RTOS

The RTOS (Real-Time Operating System) in this code is used to create multiple threads, which allows multiple tasks to be performed simultaneously. Specifically, this code is using the ARM mbed OS RTOS library to create and manage threads.

The following are the threads created using the RTOS library in this code:

Display_Desired_Temp(): This thread is responsible for displaying the desired temperature on the uLCD screen.

mbedLED(): This thread is responsible for flashing the LED1 on the board, providing a visual indication that the program is running.

ControlCommands(): This thread is responsible for receiving control commands over Bluetooth from a remote device.

sample_audio(): This thread is responsible for playing audio samples when the heating or cooling system is turned on.

Each of these threads runs concurrently, allowing the program to perform multiple tasks at the same time. The threads are created using the Thread class provided by the RTOS library, and the threads communicate with each other using shared variables and mutexes. For example, the dtemp variable is accessed by both the Display_Desired_Temp() thread and the ControlCommands() thread, so access to this variable is synchronized using a mutex.

The RTOS library also provides other features, such as semaphores, timers, and message queues, which can be used to implement more complex synchronization and communication between threads. However, in this code, only mutexes are used to synchronize access to shared variables.
