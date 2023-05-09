#include <Servo.h>
Servo motor1;
Servo motor2;
Servo motor3;
int anterior1;
int anterior2;
int anterior3;

void setup()
{
  Serial.begin(230400);
  motor1.attach(8);
  motor2.attach(9);
  motor3.attach(10);
  anterior1 = 30;
  anterior2 = 40;
  anterior3 = 165; //Garra cerrada
  motor1.write(anterior1);
  motor2.write(anterior2);
  motor3.write(anterior3);
  Serial.println(motor1.read());
  Serial.println(motor2.read());
  Serial.println(motor3.read());
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming string
    String integer_string = Serial.readStringUntil('\n');
    int integer_array[3];
    int i = 0;
    char* token = strtok((char*)integer_string.c_str(), ",");
    while (token != NULL && i < 3) {
      integer_array[i] = atoi(token);
      i++;
      token = strtok(NULL, ",");
    }
    if (integer_array[0] != 0 ) { //Si el comando es "on"

      motor1.write(motor1.read() + integer_array[0]);
      anterior1 = motor1.read();
      Serial.println("MOTOR1 MUEVE");
      Serial.println(motor1.read());

    }
    else if (integer_array[1] != 0 ) {
      motor2.write(motor2.read() + integer_array[1]);
      anterior2 = motor2.read();
      Serial.println("MOTOR2 MUEVE");
      Serial.println(motor2.read());


    }
    else if (integer_array[2] != 0 ) { // Funciona
      motor3.write(motor3.read() + integer_array[2]);
      anterior3 = motor3.read();
      Serial.println("MOTOR3 MUEVE");
        Serial.println(motor3.read());


    }

    i = 0;
    Serial.flush();
  }

}
