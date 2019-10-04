#include <Encoder.h>

char incomingByte; 
int leftpwm = 8, rightpwm = 9;
int leftdir = 22, rightdir = 23;
int leftbrk = 10, rightbrk = 11;

Encoder lEnc(2,3);
Encoder rEnc(18,19);

float get_angle( long enc){
  float angle;
  if(enc > 0)
  angle = float(enc%29520)/29520*6.28;
  else
  angle = float(29520+enc%29520)/29520*6.28;
  /*if(angle <0){
    angle = angle + 6.28;
    }*/
  return angle;
}

void ser()
{
  if(Serial.available()) {
    incomingByte = Serial.read();
    Serial.println(incomingByte);
    if( incomingByte == 'w' ){
      Serial.println(incomingByte);
      digitalWrite(leftdir, LOW);
      digitalWrite(rightdir, HIGH);
      digitalWrite(leftbrk, LOW);
      digitalWrite(rightbrk, LOW);
    } else if( incomingByte == 's' ){
      Serial.println(incomingByte);
      digitalWrite(leftdir, HIGH);
      digitalWrite(rightdir, LOW);
      digitalWrite(leftbrk, LOW);
      digitalWrite(rightbrk, LOW);
    } else if( incomingByte == 'd' ){
      Serial.println(incomingByte);
      digitalWrite(leftdir, LOW);
      digitalWrite(rightdir, LOW);
      digitalWrite(leftbrk, LOW);
      digitalWrite(rightbrk, LOW);
    } else if( incomingByte == 'a' ){
      digitalWrite(leftdir, HIGH);
      digitalWrite(rightdir, HIGH);
      digitalWrite(leftbrk, LOW);
      digitalWrite(rightbrk, LOW);
    } else if( incomingByte == '2' ){
      digitalWrite(leftbrk, HIGH);
      digitalWrite(rightbrk, HIGH);
    }
  }
//  ys=Serial.readStringUntil('\t').toInt();
//  xs=Serial.readStringUntil('\t').toInt();
//  lift = Serial.readStringUntil('\n').toInt();
//  x=map(xs,0,640,-8,15);
//  y=map(ys,480,0,10,18); 
}

void setup() 
{
  Serial.begin(9600);
  pinMode(leftpwm, OUTPUT);
  pinMode(leftbrk, OUTPUT);
  pinMode(leftdir, OUTPUT);
  pinMode(rightpwm, OUTPUT);
  pinMode(rightbrk, OUTPUT);
  pinMode(rightdir, OUTPUT);
  
}


long long oldPosition  = -999;
long i=1;
void loop()
{
 analogWrite(leftpwm, 30);
 analogWrite(rightpwm, 50);
// digitalWrite(leftdir, HIGH);
// digitalWrite(rightdir, HIGH);
// digitalWrite(rightdir, HIGH);
  //ser();
  if(oldPosition>i*-5000){
    digitalWrite(leftdir, HIGH);
      digitalWrite(rightdir, LOW);
      digitalWrite(leftbrk, HIGH);
      digitalWrite(rightbrk, HIGH);
  }
  
  else{
    i++;
    digitalWrite(leftbrk, HIGH);
      digitalWrite(rightbrk, HIGH);
      delay(1000);
      //rEnc.write(0);
  }
  long newPosition = lEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    float a;
    a = get_angle(newPosition);
    Serial.println(a);
    Serial.println(newPosition);
  }
}
