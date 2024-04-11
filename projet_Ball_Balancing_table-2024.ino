//***********************************************************************
// ZERGUIT Nour
// 6ème Electronique INRACI
// Projet ball and plate 2023-2024
// Hardware : ESP32 waroom 32E + servomoteurs+ écran résistif tactile
// 08/04/2024
#include <ESP32_Servo.h>

// Crée un objet Servo pour chaque moteurs
Servo monServoX;
Servo monServoY;

#define Yplus 26  //A0
#define Ymoin 27
#define Xplus 25      //A1
#define Xmoin 4       //A5
#define centreX 1850  // centre souhaité
#define centreY 1966  // centre souhaité

// mersure des Axes
int MesureY;
int MesureX;
// angle des deux servo-moteurs
int AngleX;
int AngleY;

// valeur PID point de fonctionnement
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.0;

// valeurs stockers pour chaque axes
float e_precedentY = 0;
float integralY = 0;
float deriveeY = 0;

float e_precedentX = 0;
float integralX = 0;
float deriveeX = 0;

// Limites des servo-moteurs
#define angleMinY 18
#define angleMaxY 144
#define angleMinX 54
#define angleMaxX 144


hw_timer_t* Ma_minuterie = NULL;  // hw_timer_t_type de donner defini dans le framework de ESP32
                                  // *Timer0_Cfg  variable  qui contient l'adresse  hw_timer_t


void IRAM_ATTR Timer0_ISR()

{

  // Mesure sur l'axe Y
  digitalWrite(Xplus, 1);
  digitalWrite(Xmoin, 0);
  MesureY = analogRead(Yplus);
  if (MesureY < 230)
    MesureY = 0;
  // pinMode(Yplus, OUTPUT);
  // digitalWrite(Yplus, 0);
  AngleY = map(MesureY, 230, 3100, angleMinY, angleMaxY);

  // Mesure sur l'axe X
  pinMode(Xmoin, INPUT);
  digitalWrite(Xplus, HIGH);
  pinMode(Yplus, OUTPUT);
  pinMode(Ymoin, OUTPUT);
  digitalWrite(Yplus, 1);
  digitalWrite(Ymoin, 0);
  MesureX = analogRead(Xmoin);
  if (MesureX == 4095)
    MesureX = 0;
  // pinMode(Xmoin, OUTPUT);
  // digitalWrite(Xmoin, 0);
  AngleX = map(MesureX, 170, 3700, angleMinX, angleMaxX);

  // Calcul des sorties PID pour les axes X et Y
  float erreurY = centreY - MesureY;
  float sortieY = calculerPID(erreurY, e_precedentY, integralY, deriveeY);
  float erreurX = centreX - MesureX;
  float sortieX = calculerPID(erreurX, e_precedentX, integralX, deriveeX);

  // Limitation des sorties PID pour les angles des servomoteurs
  int angleX_limite = constrain(90 + sortieX, angleMinX, angleMaxX);
  int angleY_limite = constrain(90 + sortieY, angleMinY, angleMaxY);

  // Déplacement des servomoteurs
  monServoX.write(angleX_limite);
  monServoY.write(angleY_limite);
}



void setup() {
  Serial.begin(9600);

  // Attache les servomoteurs aux broches correspondantes
  monServoX.attach(12);
  monServoY.attach(13);

  // pinMode(Yplus, OUTPUT);
  // pinMode(Yplus, OUTPUT);
  // pinMode(Xmoin, OUTPUT);
  // pinMode(Xmoin, OUTPUT);

  // Initialise la position initiale des servomoteurs à ? degrés
  monServoX.write(90);  // 99
  monServoY.write(90);  //81


  Ma_minuterie = timerBegin(0, 80, true);                 //j'initialise le timer
  timerAttachInterrupt(Ma_minuterie, &Timer0_ISR, true);  // active l'événement d'interruption Timer et attache sa fonction de rappel à la fonction ISR_Handler
  timerAlarmWrite(Ma_minuterie, 10000, true);             // true=Activer le rechargement automatique  // je choisie le Tout qui est de 10000
  timerAlarmEnable(Ma_minuterie);                         // active la minuterie en continue
}

void loop()

{




  // // // Mesure sur l'axe Y
  // pinMode(Xplus, OUTPUT);
  // pinMode(Xmoin, OUTPUT);
  // pinMode(Yplus, INPUT);
  // pinMode(Ymoin, OUTPUT_OPEN_DRAIN);  // output_open_drain
  // digitalWrite(Ymoin, HIGH);          //Set Y2 in in tri-state mode

  // digitalWrite(Xplus, 1);
  // digitalWrite(Xmoin, 0);

  // MesureY = analogRead(Yplus);

  // if (MesureY < 230)
  //   MesureY = 0;
  // delay(10);
  // pinMode(Yplus, OUTPUT);
  // digitalWrite(Yplus, 0);


  // AngleY = map(MesureY, 230, 3100, angleMinY, angleMaxY);
  // delay(10);



  // // Mesure sur l'axe X
  // pinMode(Xmoin, INPUT);
  // pinMode(Xplus, OUTPUT_OPEN_DRAIN);  // output_open_drain
  // digitalWrite(Xplus, HIGH);          //Set Y2 in in tri-state mode
  // pinMode(Yplus, OUTPUT);
  // pinMode(Ymoin, OUTPUT);

  // digitalWrite(Yplus, 1);
  // digitalWrite(Ymoin, 0);


  // MesureX = analogRead(Xmoin);
  // if (MesureX == 4095)
  //   MesureX = 0;
  // delay(10);
  // pinMode(Xmoin, OUTPUT);
  // digitalWrite(Xmoin, 0);

  // AngleX = map(MesureX, 170, 3700, angleMinX, angleMaxX);
  // delay(10);

  // // Affichage résultat
  // Serial.print("MesureX=");
  // Serial.print(MesureX);
  // Serial.print("   ");
  // Serial.print("MesureY=");
  // Serial.println(MesureY);
  //delay(50);

  // if (MesureX > 170) {
  //   // monServoY.write(AngleY);
  //   monServoX.write(AngleX);
  // } else if (MesureX >= 3700) {

  //   monServoX.write(angleMinX);
  // } else {
  //   monServoX.write(90);
  //   monServoY.write(90);
  // }

  // if (MesureY > 230) {
  //   monServoY.write(AngleY);
  //   // monServoX.write(AngleX);
  // } else if (MesureY >= 3100) {

  //   monServoY.write(angleMinY);
  // } else {
  // monServoX.write(90);
  // monServoY.write(90);
  // }
  // Calcul des sorties PID pour les axes X et Y
}


float calculerPID(float erreur, float& erreurPrecedente, float& integrale, float& derivee)

{
  integrale = integrale + erreur;
  derivee = erreur - erreurPrecedente;

  float sortie = Kp * erreur + Ki * integrale + Kd * derivee;

  erreurPrecedente = erreur;

  return sortie;
}
