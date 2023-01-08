// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;

// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// Joystick connections
const int pinJoyX = A0;
const int pinJoyY = A1;


void setup() {
  
  Serial.begin(9600);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {

   int Xvalue = 0;
   int Yvalue = 0;

   //read values
   Xvalue = analogRead(pinJoyX);
   delay(100);   // short delay needed between analog readings
   Yvalue = analogRead(pinJoyY);

   //compute motorA, motorB and fwdA, fwdB from analog input Xvalue, Yvalue
   bool fwdA = true;
   bool fwdB = true;
   int motorA = map(Yvalue,0,1023,255,-255) + map(Xvalue,0,1023,-255,255);
   int motorB = map(Yvalue,0,1023,255,-255) - map(Xvalue,0,1023,-255,255);
      
   if (motorA < 0) 
   {
      fwdA = false;
      motorA = - motorA;
   }

   if (motorB < 0) 
   {
      fwdB = false;
      motorB = - motorB;
   }

   if (motorA>255) motorA=255;
   if (motorB>255) motorB=255;

  // avoid humming
  if (motorA<15) motorA = 0;
  if (motorB<15) motorB = 0;
  

   //show values through serial
   Serial.print("(X, Y): (");
   Serial.print(Xvalue);
   Serial.print(", ");
   Serial.print(Yvalue);
   Serial.print(") | motorA: ");
   if (fwdA == false) {
    Serial.print("-");
   }
   Serial.print(map(motorA, 0,255,0,100));
   Serial.print("% | motorB: ");
   if (fwdB == false) {
    Serial.print("-");
   } 
   Serial.print(map(motorB,0,255,0,100));   
   Serial.println("%");      
   // operate motors

  if (fwdA == true)
  {
    // move motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } 
  else {
    // move motor A in reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  if (fwdB == true)
  {
    // move motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } 
  else {
    // move motor B in reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  
   analogWrite(enA, motorA);
   analogWrite(enB, motorB);
   
   delay(100);
}
