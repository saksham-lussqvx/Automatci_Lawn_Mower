# Automatic_Lawn_Mower

## How it Works?
This is a program which will control a sophisticated system of lawn mower. It can detect and avoid obstacles through a method never made before. It has a simple GUI which is easy to use. The program has 2 modes, out of which one is Manual Mode --> In this mode the user has to create path by maneuvering the mower around the field through GUI. They can get current stats of sensors through GUI too including lat and long given by GPS sensor. Then this path will be used on later for automatic maneuvering. Now, the other mode is Automatic mode --> Here the saved path will be used for Automatic maneuvering. Along side this, live feed from the camera will be shown in order to get the exact position and work around of the mower. If the sensors detects any kind off obstacle is detected from right, left or back then it will shut itself down for 10 seconds and will pause all motors and on going functions, also it turn on the alarm for 1 sec and will turn it off to the user let know. After the obstacle is removed it will continue its maneuver. The main feature of the mode is that along side maneuver it can detect obstacles and can surpass them easily. 

# All Features

## Manual Mode
1. Controls - Stop, Forward, Backward, Left, Right, Reverse Right (Commented), Reverse Left (Commented).
2. Creation of a template file for automatic mode.
3. Get Live Status - this will provide you with live values of each sensor onboard. 
4. You can also switch to automatic mode.

## Automated Mode
1. Live Feed - You'll get live feed mode where internal camera is mainly used (USB Cam can also be used).
2. Real time sensors value checking. Three thread will be created out of which two are for sensor value checking and other one for maneuvering.
3. Controls - Stop (all others are automatic and you can't control it for now).
4. You can also switch to Manual Mode.

## Obstacle Avoiding
If it finds anything in front of it, it will automatically stop itself for 10 seconds and if the obstacle is still there then it will first of all check if there is an obstacle on the right or not, if there is then it will check for left and if there is too then it will set the alarm on until it is removed. Else if there is not any one on the right, it will take a turn on right, move forward till the corner of the obstacle, and will also store the time it took to reach the corner. After this it will take a left and move forward until it reaches the end of the obstacle, after that it will take left again and will move forward till the time it took it at the start to reach corner of the obstacle. After this it will take a right and will then resume the process of maneuvering. Also if there is something on left it will repeat the same process from the left side.
