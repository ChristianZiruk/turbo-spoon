// #include <Arduino.h>
// #include <SharpDistSensor.h>
//
// int irSensorPin=A9; // IR Sensor Anolog 9
// SharpDistSensor irsensor(irSensorPin, 3);
//
// /* Set the power fit curve coefficients and range
//  * C and P: Coefficients in Distance = C*A^P relation
//  * where A is the analog value read from the sensor.
//  */
// const float C = 90373.;
// const float P = -1.027;
//
// /*
//  * Minimum and maximum analog values for which to return a distance
//  * These should represent a range of analog values within which the
//  * power fit curve is valid.
//  */
// const unsigned int minVal = 90; // ~800 mm
// const unsigned int maxVal = 875; // ~50mm
// // Functions.
