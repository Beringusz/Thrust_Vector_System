 #include <Servo.h> 
 
 int joyPinX = 0;                 // slider variable connecetd to analog pin 0
 int joyPinY = 1;                 // slider variable connecetd to analog pin 1
 int joyPinZ = 2;                 // slider variable connecetd to analog pin 2
 int xPos = 70;                  // variable to read the value from the analog pin 0
 int xPosTmp = 1450;
 int yPos = 60;                  // variable to read the value from the analog pin 1
 int yPosTmp = 1450;

 int xServoPin = 6;
 int yServoPin = 5;

 int servoMinPwm = 500;
 int servoMaxPwm = 2400;

 double xHornLength = 15.0;
 double xBarLength = 25.0;
 double xALength = 43.5;
 double xBLength = 37.184;

 double yHornLength = 15.0;
 double yBarLength = 25.0;
 double yALength = 42.0;
 double yBLength = 35.69;

 double xAngleOffset;
 double yAngleOffset;

 double alpha = 0.6;

 Servo xServo;
 Servo yServo;
 

 void setup() {
  Serial.begin(9600);
  xServo.attach(xServoPin, servoMinPwm, servoMaxPwm);
  yServo.attach(yServoPin, servoMinPwm, servoMaxPwm);

  xAngleOffset = atan(24.75/27.75) * 180.0/PI;
  yAngleOffset = atan(22.75/27.5) * 180.0/PI;
  
//  xServo.write(xPos);
//  yServo.write(yPos);
  delay(1000);
 }

 int mapXInput2Angle(int data) {
  return map(data,0,1024,80-xAngleOffset, 100-xAngleOffset);
 }

 int mapYInput2Angle(int data) {
  return map(data,0,1024,80-yAngleOffset, 100-yAngleOffset);
 }

 double mapXAxisOutputToInput(double thetaAB_deg) {
  double A = xALength;
  double B = xBLength;
  double C = xHornLength;
  double D = xBarLength;

  double thetaAB_rad = thetaAB_deg * PI / 180.0;

  double F = sqrt(pow(A,2) + pow(B,2) - 2.0*A*B*cos(thetaAB_rad));

  double thetaBC = acos((pow(B,2) + pow(F,2) - pow(A,2))/(2.0*B*F)) + acos((pow(C,2) + pow(F,2) - pow(D,2))/(2.0*C*F));

  return thetaBC * (180.0/PI);
 }

 double mapYAxisOutputToInput(double thetaAB_deg) {
  double A = yALength;
  double B = yBLength;
  double C = yHornLength;
  double D = yBarLength;

  double thetaAB_rad = thetaAB_deg * PI / 180.0;

  double F = sqrt(pow(A,2) + pow(B,2) - 2.0*A*B*cos(thetaAB_rad));

  double thetaBC = acos((pow(B,2) + pow(F,2) - pow(A,2))/(2.0*B*F)) + acos((pow(C,2) + pow(F,2) - pow(D,2))/(2.0*C*F));

  return thetaBC * (180.0/PI);
 }

 int mapOutputAngle2Micros(double angle) {
  return int(angle/180.0 * double(servoMaxPwm - servoMinPwm)) + servoMinPwm;
 }


 

 void loop() {
  // reads the value of the variable resistor
  xPosTmp = analogRead(joyPinX);
  xPosTmp = mapXInput2Angle(xPosTmp);
  xPos = int(double(xPosTmp)*alpha + double(xPos)*(1.0-alpha));
  
//  xServo.writeMicroseconds(xPos);
xServo.writeMicroseconds(mapOutputAngle2Micros(mapXAxisOutputToInput(double(xPos))));  
  // this small pause is needed between reading
  // analog pins, otherwise we get the same value twice
  delay(10);            
  
  
  yPosTmp = analogRead(joyPinY);
  yPosTmp = mapYInput2Angle(yPosTmp);
  yPos = int(double(yPosTmp)*alpha + double(yPos)*(1.0-alpha));  
  delay(1);

//  yServo.writeMicroseconds(yPos);
yServo.writeMicroseconds(mapOutputAngle2Micros(mapYAxisOutputToInput(double(yPos))));  

  Serial.print("X Output: ");
  Serial.println(xPosTmp);
  Serial.print("X Input: ");
  Serial.println(mapXAxisOutputToInput(double(xPosTmp)));
  Serial.println("");
  Serial.print("Y: ");
  Serial.println(yPosTmp);
  Serial.print("Y: ");
  Serial.println(mapYAxisOutputToInput(double(yPosTmp)));
  Serial.println("");
  delay(100);
 }
