# mbed_exam2

### 1. How to set up our program
##### (1) move to the directory
  `cd ~/ee2405/mbed_exam2/src/model_deploy`
##### (2) compile the main.cpp
  `sudo mbed compile --source . --source ~/ee2405new/mbed-os-build2/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -f`
##### (3) Open the screen 
  `sudo screen /dev/ttyACM0`
##### (4) Open another terminal
##### (5) move to the directory
  `cd ~/ee2405/mbed_exam2/src/model_deploy/wifi_mqtt`
##### (6) compile the mqtt_client.py
  `sudo python3 mqtt_client.py`

### 2. What are the results
##### (1) Compile commend
  `sudo mbed compile --source . --source ~/ee2405new/mbed-os-build2/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -f`
##### (2) After compile, open the screen with the commend 
  `sudo screen /dev/ttyACM0`
##### (3) When we type /1/run on the screen, RPC call the gesture mode and LED1 will turn on
###### When the mbed sense the gesture, the angle-threshold will +5 degree. If it arrives 180 degree, it will turn back to 30 degree.
###### The angle will show on the uLCD.

##### (4) Press the userbutton to determine the threshold angle and send the event to broker

##### (5) When we type /2/run on the screen, RPC call the tile_angle_detect mode and LED2 will turn on
###### The mode will turn on the LED3 for 3 second to remind the user keeps the mbed board flat for measuring the reference gravity value.
###### Then restore the reference value into ref_pDataXYZ
###### After LED3 turn off, it means we can start tilting the board and keep measuring the tilt angle per 50ms
###### The tilt angle which is the angle between present vector and the reference vector is calculated by the law of cosine.
###### If the angle is greaater than the threshold angle for 10 times continously, the angle detect function will turn off and send the events to python by MQTT

##### (6) Python will receive the 10 events that the tilt angle is greater than the threshold angle at ten times measurement continuously. 
