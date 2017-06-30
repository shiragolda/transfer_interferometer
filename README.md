# transfer_interferometer
Arduino servo to lock transfer interferometer

pid_interferometer contains code to use a chipkit uc32 to lock
a) the interferometer to the stable 780 nm reference laser
b) the 423 nm laser to the interferometer
c) the 453 nm laser to the interferometer
Error and correction signals are published on the computer's serial port. 

transfer_interferometer contains code to read the data from the serial port and publish on a zeroMQ socket. 

feedforward contains code to use an arduino uno to measure the voltage being sent to the laser piezo and feedforward to the laser current. 
