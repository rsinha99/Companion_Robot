# Companion_Robot
To run the serial test, run driver.py.
You'll need to change the port to the one you're using in particular.
For Linux, you would go to the root folder and look into the /dev/ folder. There should be something like ttyUSBX, where X is the number. If the Arduino is the first thing you've plugged into the Nvidia, the port is probably /dev/ttyUSB0, which is what is already in the code. The last thing you'll need to do is set the "isWindowsOS" variable to False in the driver.py file.

Serial Communication:
The way it is set up right now is, the Nvidia is sending bytes over to the Arduino and then Arduino reads each byte and puts it into a queue (the cppQueue Library).

Dependencies:
Arduino:
You may need to install the cppQueue library for the Arduino
