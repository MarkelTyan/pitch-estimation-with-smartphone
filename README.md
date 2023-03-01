# Estimating Vehicle Pitch Using Gyroscopic Smartphone Data
Final project for Bachelor's degree thesis
The goal of this project is to obtain the tilt of a vehicle using just the data
provided by a smartphone. In order to achieve this I used two
three-dimensional-sensors that usually come in most smartphones nowadays,
the accelerometer and the gyroscope.
The accelerometer provides the acceleration and the gyroscope provides the
angular velocity both along the x, y, z body axes. To combine both of these
values I implemented the Kalman filter, which is the most popular technique for
this purpose. However I went over some other techniques, which I discarded
and will be explained in the document.
The objective of obtaining the tilt angle is to calculate the power needed by the
Bosch vehicle and supply it to the car.

Bosch wants an automated vehicle and one of the variables needed to
automatically supply the power to the train is the tilt angle of the car. My part is
to make it possible to obtain this angle with just a smartphone.
The smartphone has some sensors which we could take advantage of, those
being the accelerometer and the gyroscope. The accelerometer provides the
acceleration and the gyroscope provides the angular velocity along the x,y,z
body axis.
I am going to merge the data from both of these sensors using the
complementary and the kalman filters and choose one with best results. The
key is to combine the advantages of both sensors to minimize the downsides of
each. This technique is called sensor fusion and is often used to determine the
attitude of a body.
Also, a calibration button is needed, since we want to know the inclination of the
vehicle independently of the inclination of the phone.
After obtaining the tilt angle in degrees we need to send it to the server via
MQTT protocol where it can be read by other components.
