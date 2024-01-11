# Aerial support for the rescue force

In this mission, we had to keep the vehicle focused on a car with an ArUco marker mounted on the top of it. In order to do that, we used basically three main tools: OpenCV, Simple PID and Dronekit.

## OpenCV

OpenCV is the most commom library in terms of Computer Vision, and here in Skyrats we use it all the time. In this mission, it was used to identify the AruCo marker, and also return its corners positions.

## Simple PID


PID is a widely used control technique, its name stands for Proportional, Integral and Derivative, what concerns the different gains of the system.
The main reason PID is commomly used, is that you don't need to know precisely the model of your system, so you can fit the parameters of your controller by testing and analyzing the results.
This library allowed us to make a simple controller to keep the drone centralized with the center of the AruCo, implementing a PID controller for the drone velocity. 
The information of the difference between the marker center and the drone is processed here, and sent to Dronekit.

## Dronekit

Dronekit was the library responsible for all of the drone movement commands. So, besides the commands of taking of and landing, Dronekit had to get the target velocity from the PID controller and set it in the flight controller.
The entire process happens at a medium rate about 10 Hz, because of the high speed of the AruCo detection. Is importante to notice that if you have a different and more complex reference marker, it can seriously affect the controller response.

