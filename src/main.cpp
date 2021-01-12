#include <Arduino.h>
#include <Servo.h>
// pins allocation
#define PIN_DISC1 3
#define PIN_DISC2 5
#define PIN_TILTER 6
#define PIN_SW1 4
#define PIN_SW2 7
#define PIN_REDLED 8
#define PIN_GRNLED 10
#define PIN_A_FB 0
#define PIN_A_SPEED 2
#define PIN_A_tilt_W 1
// motors
#define servo_zero 90
#define tilt_zero_value 497
#define tilt_lower_limit -0.33
#define tilt_upper_limit 0.37
#define POT2RAD 0.0035
#define FF 0
#define Kp 200
#define max_tilter_pwr 80
#define cycle_time 1 // milisec

Servo disc1;
Servo disc2;
Servo tilter;

int A_FB, A_Speed, A_tilt_W;
int val_discs_speed;
float val_tilter;
float tilt_A, tilt_W, prev_tilt_W, vel_W, error, last_tilt_updated_LED;
long last_time, lastGRNblink;
bool GRN_LED;

void readAnalogs()
{
  A_FB = analogRead(PIN_A_FB);
  A_Speed = analogRead(PIN_A_SPEED);
  A_tilt_W = analogRead(PIN_A_tilt_W);
}

void print_tele()
{
  Serial.print("valT ");
  Serial.print(val_tilter);
  Serial.print("TW ");
  Serial.println(tilt_W);
}
void toggle(bool &flag)
{
  flag = 1 - flag;
}

void toggle_GRNLED(uint32_t delay_time)
{
  if (millis() - lastGRNblink > delay_time)
  {
    toggle(GRN_LED);
    lastGRNblink = millis();
    digitalWrite(PIN_GRNLED, GRN_LED);
  }
}

void control_LEDs()
{
  if (GRN_LED)
    toggle_GRNLED(1);
  else
    toggle_GRNLED(50 + (1023 - A_Speed) / 2);
  if (abs(last_tilt_updated_LED - tilt_W) < 0.03)
  {
    digitalWrite(PIN_REDLED, LOW);
  }
  else
  {
    digitalWrite(PIN_REDLED, HIGH);
    last_tilt_updated_LED = tilt_W;
  }
}
void setup()
{
  Serial.begin(9600);
  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);
  pinMode(PIN_REDLED, OUTPUT);
  pinMode(PIN_GRNLED, OUTPUT);
  pinMode(PIN_DISC1, OUTPUT);
  pinMode(PIN_DISC2, OUTPUT);
  pinMode(PIN_TILTER, OUTPUT);

  disc1.attach(PIN_DISC1);
  disc2.attach(PIN_DISC2);
  tilter.attach(PIN_TILTER);
}

float clipF(float val, float lower, float upper)
{
  if (val > upper)
    val = upper;
  if (val < lower)
    val = lower;
  return (val);
}

void loop()
{
  while (millis() - last_time < cycle_time)
  {
  }
  last_time = millis();
  readAnalogs();
  val_discs_speed = servo_zero + A_Speed / 18;

  tilt_A = float(A_FB - tilt_zero_value) * POT2RAD;
  prev_tilt_W = tilt_W;
  tilt_W = (float(A_tilt_W - 511) * POT2RAD) / 2;
  tilt_W = clipF(tilt_W, tilt_lower_limit, tilt_upper_limit);
  vel_W = 1000 * (tilt_W - prev_tilt_W) / (cycle_time);
  error = tilt_W - tilt_A;
  val_tilter = clipF(servo_zero + error * Kp + vel_W * FF, servo_zero - max_tilter_pwr, servo_zero + max_tilter_pwr);
  disc1.write(val_discs_speed);
  disc2.write(val_discs_speed);
  tilter.write(int(val_tilter));

  control_LEDs();
  // print_tele();
}
