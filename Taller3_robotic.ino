#include <Servo.h>
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
int anterior1;
int anterior2;
int anterior3;
int anterior4;

void setup()
{
  Serial.begin(230400);
  motor1.attach(9);
  motor2.attach(10);
  motor3.attach(11);
  motor4.attach(6);
  motor1.write(90);
  motor2.write(90);
  motor3.write(90);
  motor4.write(90); // Valores iniciales
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming string
    String integer_string = Serial.readStringUntil('\n');
    int integer_array[4];
    int i = 0;
    char* token = strtok((char*)integer_string.c_str(), ",");
    while (token != NULL && i < 4) {
      integer_array[i] = atoi(token);
      i++;
      token = strtok(NULL, ",");
    }
      motor1.write(integer_array[0]);
      motor2.write(integer_array[1]);
      motor3.write(integer_array[2]);
      motor4.write(integer_array[3]);
    i = 0;
    Serial.flush();
  }

}
// r1 = 3.5cm
// r2 = 8cm
// r3 = 8cm
// r4 = 3.5
// r5 = 8cm
// (x,y) = 8,8 inicial con 180, 90
// Convertir mi x, yz, a polares y usar ese angulo como mi angulo de la base
