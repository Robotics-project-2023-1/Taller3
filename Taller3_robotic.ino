#include <Servo.h>
// Dato de entrada: [selector, motor1, motor2, eje1, eje2, eje3, eje4]

// Pines Brazo
#define Servo1_pin 8 // PWM
#define Servo2_pin 9 // PWM
#define Servo3_pin 10 // PWM
#define Servo4_pin 11 // PWM
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Variable brazo
int inicial1 = 90;
int inicial2 = 90;
int inicial3 = 90;
int inicial4 = 90;

// Funciones generales --------------------------------------------------------

void setup()
{
  // Protocolos de comunicacion
  Serial.begin(230400);
  // Brazo
  motor1.attach(Servo1_pin);
  motor2.attach(Servo2_pin);
  motor3.attach(Servo3_pin);
  motor4.attach(Servo4_pin);
  motor1.write(inicial1);
  motor2.write(inicial2);
  motor3.write(inicial3);
  motor4.write(inicial4);
}

void loop() {

  if (Serial.available() > 0) {
    // Read the incoming string
    String integer_string = Serial.readStringUntil('\n');
    int integer_array[7]; //motor1, motor2, Servo1, servo2, servo3, servo4
    int i = 0;
    char* token = strtok((char*)integer_string.c_str(), ",");
    while (token != NULL && i < 7) {
      integer_array[i] = atoi(token);
      i++;
      token = strtok(NULL, ",");
    }
    if (integer_array[0] == 2) { // Mover brazo
      motor1.write(integer_array[3]);
      motor2.write(integer_array[4]);
      motor3.write(integer_array[5]);
      motor4.write(integer_array[6]);
    }

  }
  i = 0;

  }
