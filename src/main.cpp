#include <Arduino.h>
#include <Servo.h>

#define PIN_DISC1 3
#define PIN_DISC2 5
#define PIN_TILTER 6

#define PIN_SW1 4
#define PIN_SW2 7
#define PIN_LED1 8
#define PIN_LED2 10
#define PIN_A_FB 0
#define PIN_A_SPEED 1
#define PIN_A_THETA 2

#define servo_zero 90

Servo disc1;
Servo disc2;
Servo tilter;

int A_FB, A_Speed, A_Theta;
int val_discs_speed;
int val_tilter;

void readAnalogs()
{
  A_FB = analogRead(PIN_A_FB);
  A_Speed = analogRead(PIN_A_SPEED);
  A_Theta = analogRead(PIN_A_THETA);
}
void setup()
{
  Serial.begin(9600);
  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_DISC1, OUTPUT);
  pinMode(PIN_DISC2, OUTPUT);
  pinMode(PIN_TILTER, OUTPUT);

  disc1.attach(PIN_DISC1);
  disc2.attach(PIN_DISC2);
  tilter.attach(PIN_TILTER);
}

void loop()
{
  readAnalogs();
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  delay(10);

  val_discs_speed = servo_zero + A_Speed / 13;
  val_tilter = servo_zero + (A_FB - A_Theta) / 2;
  if (val_tilter > 170)
    val_tilter = 170;
  if (val_tilter < 10)
    val_tilter = 10;

  disc1.write(val_discs_speed);
  disc2.write(val_discs_speed);
  tilter.write(val_tilter);
}

//Serial.println (val_discs_speed);   Serial.print (" ");
//Serial.print (A_Speed);   Serial.print (" ");
//Serial.print (A_Theta);   Serial.println (" ");
