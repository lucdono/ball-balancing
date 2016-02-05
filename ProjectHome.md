# Ball Balancing Plate #

This is a simple project that aims to build a simple control system that will set a ball in the center of a plate or force it to move along a given path.

The embedded control is based on ChibiOS-RT running on an STM32F0 Discovery board: the firmware controls each servos and receives controls from serial port through a client
running on a PC.
The client uses OpenCV libraries in order to capture the ball image and find its coordinates. It also is responsible for the control algorithm.

See:
  * http://www.youtube.com/watch?v=PGyC0zuaesg (Final)
  * http://www.youtube.com/watch?v=I6QR00xgxWM (First Test)

## Algorithm ##
The control algorithm is very simple: it only uses a Kalman filter in order to estimate the real position of the center of the ball; then two PID controller generate the signals for each motors and send this values over serial.
Tha ball can be forced to move along a path (e.g. a circle) changing the set point coordinates.

## BOM ##

  * HS325HB servos
  * STM32F0 Discovery
  * ChibiOS-RT
  * OpenCV
  * RS232/Usart board
  * A webcam
  * Construction material

## Credits ##
Luca D'Onofrio (c) 2013