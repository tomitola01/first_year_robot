//PIN definitions
#define R_MOTOR_F 13
#define R_MOTOR_B 12
#define L_MOTOR_F 10
#define L_MOTOR_B 11
#define LENC_A 21
#define LENC_B 20
#define RENC_A 18
#define RENC_B 19
#define leftTrig 3
#define leftEcho 4
#define frontTrig 5
#define frontEcho 6
#define leftSupply 30
#define frontSupply 31

//defining variables
int state;

const int normalSpeed = 90;
const int slowSpeed = 60;
const int fastSpeed = 110;
int leftDistance;
int frontDistance;
long leftDuration;
long frontDuration;
unsigned long checkTime;
int pwmMotor = 100;
int targetTicks = 5;
int timePeriod = 5;
volatile long ticksL = 0;
volatile long ticksR = 0;
volatile long ticked;
int proportionalGain;


long dist;
const int wheel_d = 65; //mm
const float wheel_c = PI * wheel_d;
const int counts_per_rev = 960;
const int drive_distance = 2000; //mm

void setup() {
  Serial.begin(9600);

  pinMode(R_MOTOR_F, OUTPUT);
  pinMode(L_MOTOR_F, OUTPUT);
  pinMode(R_MOTOR_B, OUTPUT);
  pinMode(L_MOTOR_B, OUTPUT);
  pinMode(LENC_A, INPUT_PULLUP);
  pinMode(LENC_B, INPUT_PULLUP);
  pinMode(RENC_A, INPUT_PULLUP);
  pinMode(RENC_B, INPUT_PULLUP);
  digitalWrite(LENC_A, HIGH);
  digitalWrite(RENC_A, HIGH);

  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftSupply, OUTPUT);
  digitalWrite(leftSupply, HIGH);
  pinMode(frontSupply, OUTPUT);
  digitalWrite(frontSupply, HIGH);

  analogWrite(R_MOTOR_F, 0);
  analogWrite(L_MOTOR_F, 0);

  state = 0;

  attachInterrupt(digitalPinToInterrupt(LENC_A), countLEFT, RISING);
  attachInterrupt(digitalPinToInterrupt(RENC_A), countRIGHT, RISING);
  proportionalGain = pwmMotor / targetTicks;
  checkTime = millis();
}

void loop() {
  //Both methods not run in the same loop
  goOneMeter();
  leftHandRule();
}

//Method used for traversing area using lefthand rule
void leftHandRule() {
  int distanceFront = frontSensor();
  int distanceLeft = leftSensor();

  //When the robot is turned on it drives straight until it reaches a wall
  //Continues to turn left until it has returned to original position
  //Records Distance travelled in ticksL
  if(state == 0){
    if (distanceFront < 9 && distanceFront > 5) {
      brake();
      dist = (ticksL);
      delay(500);
      rightTurn(slowSpeed);
      delay(850);
      brake();
      state++;
      delay(500);
    } else {
      driveStraight();
    }
  }else if(state == 1||state == 2||state ==3){
    if (distanceFront < 9) {
      brake();
      delay(500);
      rightTurn(slowSpeed);
      delay(850);
      brake();
      state++;
      delay(500);
    } else {
      driveStraight();
    }
  }else if(state == 4){
    findCentre();
    delay(500);
    rightTurn(slowSpeed);
    delay(850);
    brake();
    delay(500);
    findCentre();
    delay(30000);
  }
}

//method used to find centre of enclosure
void findCentre() {
  long centre = (dist / 2);
  ticksL = 0;
  ticksR = 0;
  while ((ticksL < centre) && (ticksR < centre)) {
    driveStraight();
    Serial.print(ticksL);
    Serial.print  ("            ");
    Serial.println(ticksR);
  }
  brake();
}

//Debug method to record distance travelled by robot
void test() {
  goOneMeter();
  delay(1000);
  dist = (ticksL + ticksR) / 2;
  Serial.print("Target Distance: ");
  long target = (dist / 2);
  Serial.println(target);
  ticksL = 0;
  ticksR = 0;
  while ((ticksL < target) && (ticksR < target)) {
    driveStraight();
    Serial.print(ticksL);
    Serial.print  ("            ");
    Serial.println(ticksR);
  }
  brake();
  delay(30000);
}

//Using encoders to keep a robot straight for one meter
void goOneMeter() {
  float num_rev = drive_distance / wheel_c;
  unsigned long target_count = num_rev * counts_per_rev;

  while ((ticksL < target_count) && (ticksR < target_count)) {
    driveStraight();
  }

  brake();
  Serial.print(ticksL);
  Serial.print  ("            ");
  Serial.println(ticksR);
}

//Methods for turning robot right or left
void leftTurn(int speedL) {
  leftMotor(speedL);
  rightMotor(-speedL);
}

void rightTurn(int speedR) {
  leftMotor(speedR);
  rightMotor(-speedR);
}

//Mehtods for counting encoder ticks
void countRIGHT() {
  ticksR++;
}

void countLEFT() {
  ticksL++;
}

//Methods for velocity of motors
void rightMotor(int right) {
  if (right > 0) {
    analogWrite(R_MOTOR_F, abs(right*1.167));
    analogWrite(R_MOTOR_B, 0);
  } else {
    analogWrite(R_MOTOR_B, abs(right));
    analogWrite(R_MOTOR_F, 0);
  }
}

void leftMotor(int left) {
  if (left > 0) {
    analogWrite(L_MOTOR_F, abs(left));
    analogWrite(L_MOTOR_B, 0);
  } else {
    analogWrite(L_MOTOR_B, abs(left));
    analogWrite(L_MOTOR_F, 0);
  }
}

void brake() {
  analogWrite(L_MOTOR_F, 0);
  analogWrite(L_MOTOR_B, 0);
  analogWrite(R_MOTOR_F, 0);
  analogWrite(R_MOTOR_B, 0);
}

//Method using encoders and simple PID controller to keep a robot travelling straight
void driveStraight() {
  unsigned long initialCount = ticksL;
  checkTime = millis();
  while (millis() - checkTime <= timePeriod);
  unsigned long countDifference = ticksL - initialCount;
  int error = targetTicks - countDifference;
  int newPWM = error * proportionalGain;
  if (newPWM > 0) {
    leftMotor(newPWM);
  }
  else(leftMotor(0));

  initialCount = ticksR;
  checkTime = millis();
  while (millis() - checkTime <= timePeriod);
  countDifference = ticksR - initialCount;
  error = targetTicks - countDifference;
  newPWM = error * proportionalGain;
  if (newPWM > 0) {
    rightMotor(newPWM);
  }
  else(rightMotor(0));

  Serial.print(ticksL);
  Serial.print("          ");
  Serial.println(ticksR);
}

//Methods for returning distance of object from ultrasensors
int leftSensor() {
  digitalWrite(leftTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(leftTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTrig, LOW);

  //Calculation of distance: d = t*v
  //distance = (time taken for ultrasensor response x speed of sound in cm/ms)/2
  //An integer of 2 is used because the time taken is for an ultrasensor response is an echo
  //Therefore the time taken to reach object is half the time for the ultrasensor to pick up a response
  leftDuration = pulseIn(leftEcho, HIGH);
  leftDistance = (leftDuration * 0.0343) / 2;

  return leftDistance;

  Serial.print("Distance: ");
  Serial.print(leftDistance);
  Serial.println(" cm");
}

int frontSensor() {
  digitalWrite(frontTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(frontTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrig, LOW);

  frontDuration = pulseIn(frontEcho, HIGH);
  frontDistance = (frontDuration * 0.0343) / 2;
  return frontDistance;

  Serial.print("Distance: ");
  Serial.print(frontDistance);
  Serial.println(" cm");
}
