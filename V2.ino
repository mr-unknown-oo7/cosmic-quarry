#include <dht11.h>
#include<SoftwareSerial.h>
////////////////////////////////////mpu/////////////////////////////////////
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include "HX711.h"
#define DHT11PIN 4
////////////////////////////////////variables/////////////////////////////////////
float s0;
float ocv;
float cc;
float soc;
float prev_soc;
float op;
float t_prev = 0;

float incli;
float prev_ema = 0;
float incli_check ;
float ax;
float ay;
Adafruit_MPU6050 mpu;
float voltage;
float inclination=23,ambient_temp=34,speed=0,weight=56,battery_level=0,battery_temp=78,timee=89,battery_rating=10000.0,charging_cycle=1,acceleration=90,battery_voltage=12,battery_current=0,humidity=45.7;
char c = 'a',garbage;
int fc,dc;
String str;
const int LOADCELL_DOUT_PIN = 2,LOADCELL_SCK_PIN = 3;
int i,temp,chk,flag=0;
double adc,myVal;
unsigned long startTime, currentTime;
unsigned long theTime;
char a;
int BTspeed=0,one=7,two=6,three=9,four=10;
/////////////////////////////////////objects//////////////////////////////////////
dht11 DHT11;
HX711 scale;
SoftwareSerial mySerial(12,8);
////////////////////////////////////soc/////////////////////////////////////
float ocvfun(float v,float i){
  float V,v1,v2,v3,v4,v5,op;
  V = v+i*0.6470588235294;
  v1 = (pow(V,0.2)-1.621147)/0.026838;
  v2 = (pow(V,0.4)-2.628839)/0.086274;
  v3 = (pow(V,0.6)-4.264050)/0.208093;
  v4 = (pow(V,0.8)-6.918236)/0.446346;
  v5 = (pow(V,2)-126.8619)/19.507246;
  // Serial.print("V: ");
  // Serial.print(V);
   Serial.print(" v1: ");
   Serial.print(v1);
   Serial.print(" v2: ");
   Serial.print(v2);
   Serial.print(" v3: ");
   Serial.print(v3);
   Serial.print(" v4: ");
   Serial.print(v4);
   Serial.print(" v5: ");
   Serial.print(v5);
  // Serial.print(" op11: ");
  // Serial.print(op11);
  // Serial.print(" op12: ");
  // Serial.print(op12);
  // Serial.print(" op13: ");
  // Serial.print(op13);
  // Serial.print(" op14: ");
  // Serial.print(op14);
  // Serial.print(" op15: ");
  // Serial.print(op15);
  // Serial.print(" op21: ");
  // Serial.print(op21);
  // Serial.print(" op22: ");
  // Serial.print(op22);
  // Serial.print(" op23: ");
  // Serial.print(op23);
   Serial.print(" op: ");
   Serial.print(op);

  op = v1*-29.342491+v2*-17.620155+v3*-7.3668194+v4*4.6083674+v5*73.09605+56.13433;
  return op;
}

float socfun(float v,float i,float t){
  ocv = ocvfun(v,i);
  cc = s0-(i*(t-t_prev)*0.95/1000)*100/5948.342032632134; //
  Serial.print("CC : ");
  Serial.print(cc);
  Serial.print(" OCV : ");
  Serial.println(ocv);
  
  soc = (ocv+cc)/2;
  soc = 0.3*soc+0.7*prev_soc;
  prev_soc = soc;
  t_prev = t;
  if(soc > 100){
    soc = 100;
  }
  else if(soc<0){
    soc = 0;
  }
  else{
    soc = soc;
  }
  Serial.print(" soc: ");
  Serial.println(soc);
  return soc;
}
////////////////////////////////////soc-end/////////////////////////////////////
float volt_to_temp(float v){

  float r2;
  r2 = (v*10.0)/(5.0-v);
  Serial.print("r2 = ");
  Serial.print(r2);
  float temp;
  temp = (-0.0274*r2*r2*r2)+(1.0225*r2*r2)-(14.6615*r2)+97.41;
  return temp;
}

void data(){


  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  incli_check = acos(a.acceleration.z/9.81)*180/PI - 8;
  if (isnan(incli_check)){
    incli = prev_ema;
  }
  else{
    incli = 0.8*incli_check + (1-0.8)*prev_ema;
    prev_ema = incli;
  }
    ax = a.acceleration.x;
    ay = a.acceleration.y;    
}

void setup() {
  // put your setup code here, to run once:
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT); 
  pinMode(11,OUTPUT);
  digitalWrite(5,LOW);
  digitalWrite(11,LOW);
  pinMode(12,INPUT);
  pinMode(8,OUTPUT);
  startTime = millis();
  Serial.begin(115200);
  mySerial.begin(9600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(95.008);
  scale.tare(); 
  while (!Serial)
    delay(10); 

  Serial.println("Adafruit MPU6050 test!");


  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
/////////////////////////////
  for(i=0,myVal=0,temp=0;i<50;i++)
  { 
    temp=analogRead(A3);
    myVal += temp;
  }
  myVal /= 50.0;
  if(myVal>=100)
  {
    battery_voltage = myVal*12.0/1023.000;
    Serial.print("battery_voltage : ");
    Serial.println(battery_voltage);
  }
///////////////////////
  battery_current = (voltage - 2.5) / 0.185;
  if (battery_current < 0.16) {
    battery_current = 0;
  }
/////////////////////////
  s0 = ocvfun(battery_voltage,battery_current);
  Serial.print(" s0: ");
  Serial.print(s0);
}

void loop() {

  //////////////////////////just-something-to-help-the-dht////////////////////////
  if(flag==1){
   startTime = millis();
   flag=0;
  }
  ////////////////////////////getting-temperature/////////////////////////////////
  myVal = analogRead(A2);
  voltage = myVal*5.0/1023;
  battery_temp= volt_to_temp(voltage);
  Serial.print("battery_temperature : ");
  Serial.println(battery_temp);
  ///////////////////////////////getting-voltage//////////////////////////////////
  for(i=0,myVal=0,temp=0;i<50;i++)
  { 
    temp=analogRead(A3);
    myVal += temp;
  }
  myVal /= 50.0;
  if(myVal>=100)
  {
    battery_voltage = myVal*12.0/1023.000;
    Serial.print("battery_voltage : ");
    Serial.println(battery_voltage);
  }
  else{
    Serial.println("battrey voltage malfunction");
  }

  ///////////////////////////////getting-current//////////////////////////////////
  for(i=0,adc=0,temp=0;i<50;i++)
  { 
    temp = analogRead(A7);
    adc += temp;
  }
  adc /= 50.0;
  voltage = adc * 5 / 1023.0;
  battery_current = (voltage - 2.5) / 0.185;
  if (battery_current < 0.16) {
    battery_current = 0;
  }
  Serial.print("Current : ");
  Serial.println(battery_current);
  ////////////////////////getting-temperature&humidity////////////////////////////
  if(millis()-startTime > 2000){
    chk = DHT11.read(DHT11PIN);
    ambient_temp = (float)DHT11.temperature;
    humidity = (float)DHT11.humidity;
    Serial.print("temperature : ");
    Serial.println(temp);
    Serial.print("humidity : ");
    Serial.println(humidity);
    flag=1;
  }
  /////////////////////////////////getting-load///////////////////////////////////

  weight = scale.get_units();
  Serial.print("weight in grams :");
  Serial.println(weight);
  /////////////////////////////getting-acceleration///////////////////////////////
  data();
  Serial.print("Inclination : ");
  inclination = incli;
  Serial.println(inclination);
  acceleration = ay;
  Serial.print("acceleration = ");
  Serial.println(acceleration);

  timee = millis();
  //////////////////////////////////callling soc to get battery level///////////////////
  battery_level = socfun(battery_voltage,battery_current,timee);

  //////////////////////////////////delay&motors////////////////////////////////////////
  str = "&" + (String)inclination + "%" + (String)ambient_temp + "%" + (String)speed + "%" + (String)weight + "%" + (String)battery_level + "%" + (String)battery_temp + "%" + (String)timee + "%" + (String)battery_rating + "%" + (String)charging_cycle + "%" + (String)acceleration + "%" + (String)battery_voltage + "%" + (String)battery_current + "%" + (String)humidity;
  Serial.println(str);

while(mySerial.available()==0){}
a = mySerial.read();
if(a=='0'){
  BTspeed = 0;
}
else if(a=='1'){
  BTspeed = 25;
}
else if(a=='2'){
  BTspeed = 50;
}
else if(a=='3'){
  BTspeed = 76;
}
else if(a=='4'){
  BTspeed = 101;
}
else if(a=='5'){
  BTspeed = 127;
}
else if(a=='6'){
  BTspeed = 158;
}
else if(a=='7'){
  BTspeed = 170;
}
else if(a=='8'){
  BTspeed = 210;
}
else if(a=='9'){
  BTspeed = 230;
}
else if(a=='q'){
  BTspeed = 255;
}
analogWrite(5,255);
analogWrite(11,255);

      if(a=='G')
      {
        digitalWrite(one,LOW);
        digitalWrite(two,LOW);
        digitalWrite(three,HIGH);
        digitalWrite(four,LOW);
      }
      else if(a=='F')
      {
        digitalWrite(one,HIGH);
        digitalWrite(two,LOW);
        digitalWrite(three,HIGH);
        digitalWrite(four,LOW);
      }
      else if(a=='I')
      {
        digitalWrite(one,HIGH);
        digitalWrite(two,LOW);
        digitalWrite(three,LOW);
        digitalWrite(four,LOW);
      }
      else if(a=='L')
      {
        digitalWrite(one,LOW);
        digitalWrite(two,HIGH);
        digitalWrite(three,HIGH);
        digitalWrite(four,LOW);
      }
      else if(a=='S')
      {
        digitalWrite(one,LOW);
        digitalWrite(two,LOW);
        digitalWrite(three,LOW);
        digitalWrite(four,LOW);
      }
      else if(a=='R')
      {
        digitalWrite(one,HIGH);
        digitalWrite(two,LOW);
        digitalWrite(three,LOW);
        digitalWrite(four,HIGH);
      }
      else if(a=='H')
      { 
        digitalWrite(one,LOW);
        digitalWrite(two,LOW);
        digitalWrite(three,LOW);
        digitalWrite(four,HIGH);
      }
      else if(a=='B')
      {
        digitalWrite(one,LOW);
        digitalWrite(two,HIGH);
        digitalWrite(three,LOW);
        digitalWrite(four,HIGH);
      }
      else if(a=='J')
      {
        digitalWrite(one,LOW);
        digitalWrite(two,HIGH);
        digitalWrite(three,LOW);
        digitalWrite(four,LOW);
      }


  while(mySerial.available()>0)
  {garbage=mySerial.read();}
}
