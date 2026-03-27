#include <Servo.h>

String cmd;

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;

int pos = 0;
int inverse_of_speed = 5;

void setup() {
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(3);
  myservo4.attach(4);
  myservo5.attach(5);
  myservo6.attach(6);

  Serial.begin(9600);

  // =====【新增 1】上电提示 =====
  Serial.println("Arduino CMD ready");

  myservo1.write(0); 
  myservo2.write(180);
  myservo3.write(0); 
  myservo4.write(180);
  myservo5.write(0); 
  myservo6.write(180);
}

void loop() {

  if (Serial.available()) {

    cmd = Serial.readStringUntil('\n');

    // =====【新增 3】打印收到的命令 =====
    Serial.print("Received cmd: ");
    Serial.println(cmd);

    if (cmd == "left_close") {
      Serial.println("Executing left_close");  // 新增反馈

      for (pos = 0; pos <= 180; pos++) {
        myservo1.write(pos);
        myservo2.write(180 - pos);
        delay(inverse_of_speed);
      }
      Serial.println("left_close done");
    }

    else if (cmd == "left_open") {
      Serial.println("Executing left_open");

      for (pos = 180; pos >= 0; pos--) {
        myservo1.write(pos);
        myservo2.write(180 - pos);
        delay(inverse_of_speed);
      }
      Serial.println("left_open done");
    }

    else if (cmd == "right_close") {
      Serial.println("Executing right_close");

      for (pos = 0; pos <= 180; pos++) {
        myservo5.write(pos);
        myservo6.write(180 - pos);
        delay(inverse_of_speed);
      }
      Serial.println("right_close done");
    }

    else if (cmd == "right_open") {
      Serial.println("Executing right_open");

      for (pos = 180; pos >= 0; pos--) {
        myservo5.write(pos);
        myservo6.write(180 - pos);
        delay(inverse_of_speed);
      }
      Serial.println("right_open done");
    }

    else {
      // =====【新增 4】未识别命令 =====
      Serial.println("Unknown command");
    }
  }
}
