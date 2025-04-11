#include <HCSR04.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

#define PIN_RGB 3
#define TRIGGER_PIN 12
#define ECHO_PIN 13
#define MOTOR_INTERFACE_TYPE 4
#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LiquidCrystal_I2C lcd(0x27,16,2);
HCSR04 hc(TRIGGER_PIN,ECHO_PIN);

const int rgbPin[3] = {28,26,24};
const int buzzerPin = 22;

enum colorState {RED,GREEN,BLUE};

enum AlarmeState {ACTIVE, DESACTIVE};

enum AppState {NORMAL, TROP_LOIN, TROP_PROCHE};

AppState appState = NORMAL;

AlarmeState alarmeState = DESACTIVE;

unsigned long currentTime = 0;
const int lastValue = 400;
int distance = 0;
int distanceAlarme = 15;

int angle = 0;
int minStep;
int maxStep;

const int minDistance = 30;
const int maxDistance = 60; 
const int minAngle = 10;
const int maxAngle = 170;
const int fullStep = 2038;
const int fullAngle = 360;

const String numero = "2347643";
String ligne1 = numero;
String ligne2 = "Labo 4B"; 

void affichageDebut(){
  
  const int rate = 2000;
  lcd.init();
  lcd.setCursor(0,0);
  lcd.print(ligne1);
  lcd.setCursor(0,1);
  lcd.print(ligne2);
  delay(rate);
  ligne1 = "";
  ligne2 = "";
  lcd.clear();
}

#pragma region Modèles

void color (unsigned long now)// the color generating function  
{
  static unsigned long lastTime = 0;
  static colorState color = RED;
  const int delay = 200;

  if(now - lastTime < delay) return;

  lastTime = now;

  offLed();

  digitalWrite(rgbPin[color], HIGH);

  switch(color){
    case RED:
      color = BLUE;
      break;
    case BLUE:
      color = RED;
      break;  
  }
}

void offLed(){
  for(int i = 0; i < PIN_RGB; i++){
    digitalWrite(rgbPin[i], LOW);
  }
}

void actualiserAffichage(unsigned long now){
  const int reset = 100;
  static unsigned long lastTime = 0;

  if(now - lastTime < reset) return;

    lcd.setCursor(0,0);
    lcd.print(ligne1);
    lcd.setCursor(0,1);
    lcd.print(ligne2);

    lastTime = now;

}

void retournerDistance(unsigned long ct) {
  static unsigned long lastTime = 0;
  unsigned long rate = 100;
  static int lastDistance = 0;
  
  if (ct - lastTime < rate) { 
    distance = lastDistance == 0 ? lastValue : lastDistance;
    return;
  }
  
  lastTime = ct;
  
  // Faire le code de la tâche ici
  int distanceBrute = hc.dist();
  
  lastDistance = distanceBrute;
  
  distance = distanceBrute == 0 ? lastValue : distanceBrute;  
}

void normalState(){
  
  static bool firstime = true;

  if( firstime){
    lcd.clear();
    firstime = false;
    myStepper.enableOutputs();
  }

  int distanceLim = constrain(distance, minDistance, maxDistance); 
  int step = map(distanceLim, minDistance, maxDistance, minStep, maxStep);

  if(myStepper.distanceToGo() == 0){
    myStepper.moveTo(step);
  }
  
  angle = map(distanceLim, minDistance, maxDistance, minAngle, maxAngle);
  ligne1 = "Dist : " + (String)distanceLim + " cm  ";
  ligne2 = "Obj  : " + (String)angle +" deg  "; 
 
  if(distance < minDistance){
   appState = TROP_PROCHE;
   firstime = true;

    myStepper.moveTo(minStep);
  }

  else if(distance > maxDistance){
   appState = TROP_LOIN;
   firstime = true;

   myStepper.moveTo(minStep);
  }
}

void nearState(){
  static bool firstTime = true;

  if(firstTime && myStepper.distanceToGo() == 0){
    myStepper.disableOutputs();
    firstTime = false;
  }

  ligne1 = "Dist : " + (String)distance + " cm  ";
  ligne2 = "Obj  : Trop pret"; 

  if(distance >= minDistance){
    appState = NORMAL;
    firstTime = true;
  }
}

void farState(){

  static bool firstTime = true;

  if(firstTime && myStepper.distanceToGo() == 0){
    myStepper.disableOutputs();
    firstTime = false;
  }

  ligne1 = "Dist : " + (String)distance + " cm  ";
  ligne2 = "Obj  : Trop loin"; 
  
  if(distance <= maxDistance){
    appState = NORMAL;
    firstTime = true;
  }
}

void affichagePort(unsigned long now){
  const int rate = 100;
  static unsigned long lastTime = 0;

  if(now - lastTime < rate) return;

  lastTime = now;

  Serial.print("etd:");
  Serial.print(numero);

  Serial.print(",dist:");
  Serial.print(distance);

  Serial.print(",deg:");
  Serial.println(angle);
}

void gererAlarme(unsigned long now){
  static unsigned long lastTime = 0;
  const int rate = 3000;

  if(now - lastTime > rate && distance > distanceAlarme){
    alarmeState = DESACTIVE;
    lastTime = now;  
  }

  if(distance <= distanceAlarme){
    alarmeState = ACTIVE;
    lastTime = now;
  }

}

#pragma endregion

void stateManager() {
  // Adapter selon votre situation!
  switch (appState) {
    case NORMAL:
      normalState();
      break;
    case TROP_PROCHE:
      nearState();
      break;
    default:
      farState();
      break;  
  }
}

void alertManager(unsigned long currentTime) {
  // Adapter selon votre situation!
  switch (alarmeState) {
    case ACTIVE:
      color(currentTime);
      digitalWrite(buzzerPin, HIGH);
      break;
    default:
      digitalWrite(buzzerPin, LOW);
      offLed();
      break;  
  }
}

#pragma region setup-loop

void setup() {
  // put your setup code here, to run once:

  for(int i = 0; i < PIN_RGB; i++){
    pinMode(rgbPin[i], OUTPUT);
  }

  pinMode(buzzerPin, OUTPUT);
  Serial.begin(115200);

  lcd.backlight();
  affichageDebut();

  minStep = minAngle * (fullStep * 1.0 /fullAngle);
  maxStep = maxAngle * (fullStep * 1.0 /fullAngle);
  
  myStepper.setMaxSpeed(1000);  
  myStepper.setAcceleration(250); 

  myStepper.moveTo(minStep);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();

  actualiserAffichage(currentTime);
  retournerDistance(currentTime);
  stateManager();
  gererAlarme(currentTime);
  alertManager(currentTime);
  affichagePort(currentTime);

  myStepper.run();
}
#pragma endregion
