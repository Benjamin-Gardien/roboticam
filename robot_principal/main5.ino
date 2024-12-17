//test//
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include "Fourche.h"
#include "Panneau.h"
#include "Camera.h"
#include <Wire.h>
#include <ESP32Encoder.h>
#include <cmath>
double pi = M_PI;
Panneau panneau;
Fourche fourche;
#define SLAVE_ADDRESS 0x04



SemaphoreHandle_t odriveSemaphore; TaskHandle_t xHandleStrategy = NULL; 
SemaphoreHandle_t action1Semaphore; SemaphoreHandle_t action2Semaphore; SemaphoreHandle_t action3Semaphore; SemaphoreHandle_t action4Semaphore; SemaphoreHandle_t action5Semaphore; SemaphoreHandle_t action6Semaphore; SemaphoreHandle_t action7Semaphore; SemaphoreHandle_t action8Semaphore; SemaphoreHandle_t action9Semaphore; SemaphoreHandle_t action10Semaphore; SemaphoreHandle_t action11Semaphore; SemaphoreHandle_t encoderSemaphore;

// DECLARATION CODEURS
ESP32Encoder encoder;
ESP32Encoder encoder2;
volatile float pos1 = 0;
volatile float pos2 = 0;
volatile float x = 0;
volatile float y = 0;
volatile float a = 0;
volatile float Currentorientation = 0;  // Orientation en radians
float L = 280; // Distance entre les deux roues codeur
float lastPos1 = 0; // Dernières positions des encodeurs (pour calculer les déplacements)
float lastPos2 = 0;

//DECLARATION DES FONCTIONS
void TaskInit(void *parameter);
void encoderTask(void *pvParameters);
void executeStrategy(void *parameter);
void executeAction1(void *parameter);
void executeAction2(void *parameter);
void executeAction3(void *parameter);
void executeAction4(void *parameter);
void executeAction5(void *parameter);
void executeAction6(void *parameter);
void executeAction7(void *parameter);
void executeAction8(void *parameter);
void executeAction9(void *parameter);
void executeAction10(void *parameter);
void executeAction11(void *parameter);
void executeAction12(void *parameter);


//DECLARATION DES MOTEURS
HardwareSerial odriveSerial1(1); // Utilisez le second port série sur l'ESP32
HardwareSerial odriveSerial2(2); // Utilisez le second port série sur l'ESP32
ODriveArduino odrive1(odriveSerial1);
ODriveArduino odrive2(odriveSerial2);

// DECLARATION DE LA STRATÉGIE
const int NB_STRATEGIES = 23 ;
const int NB_PARAMS = 8 ;

float Strategieopp[23][8] ={{1,1,2000,200,180,100,3,1},{2,1,2000,200,4,100,3,1},{3,1,1000,200,4,100,3,1},{4,1,2000,200,4,1000,2,1},{5,1,2400,705,4,2000,1,1},{6,1,2160,705,1,1500,1,1},{7,0,2200,705,4,1000,1,0},{8,1,2060,705,7,1000,1,0},{9,0,2400,705,4,1500,3,0},{10,1,2230,1925,4,1500,10,1},{11,0,2230,1700,6,1000,1,0},{12,1,2960,1800,4,2000,1,1},{13,0,2620,1800,4,1500,1,0},{14,1,2860,1300,4,2000,3,1},{15,0,2750,1550,4,1500,1,0},{16,1,2800,1400,4,1500,1,1},{17,1,2900,1400,3,1500,10,1},{18,0,2600,1305,4,1500,10,0},{19,1,2200,1300,5,1500,10,1},{20,0,165,1200,4,1000,10,1},{21,0,168,1200,4,1000,3,0},{22,1,100,1200,12,1000,3,0},{23,1,100,1200,12,1000,3,1},}; 

float Strategiecote[19][8] ={{1,1,2000,200,180,1000,3,1},{2,1,1500,200,4,3000,3,1},{3,1,2000,200,4,3000,3,1},{4,1,2000,500,4,3000,2,1},{5,1,2000,200,4,3000,3,1},{6,0,1250,705,1,1500,3,1},{7,1,1100,705,2,1500,3,1},{8,1,1000,705,0,1500,3,1},{9,1,250,518,4,1500,3,1},{10,1,200,490,9,1500,3,1},{11,0,100,880,4,1500,3,0},{12,1,200,650,4,1500,3,1},{13,1,0,650,9,1500,3,1},{14,0,300,650,0,1500,3,0},{15,0,700,1300,5,1500,3,1},{16,1,850,1300,2,1500,3,1},{17,0,200,1000,4,1500,3,1},{18,0,100,1000,4,1500,3,0},{19,0,0,1000,12,1500,3,0},}; float Strategie[23][8] ;

//Evitement 
int compteur = 1;
int DIST_AV = 0;
int DIST_AR = 0;
int dist;
int angle;

//Calcul positionnement
float Distance = 0;
float entraxe = 135.0;
float diametre = 103.0*1.015;
int sens = 0;
float o = 0;
float xPoint = 0;
float yPoint = 0;
float angleDifference = 0;

//Tirette
int Tir = 0;

//Fin de course
int FC_AvD = 0;
int FC_AvG = 0;
int FC_ArD = 0; 
int FC_ArG = 0;

//Switch stratégie
const int pinStrat = 35;
int switchStrat;

void receiveData(int byteCount) {
  while (Wire.available()) {
    String receivedData = "";
    receivedData.reserve(64);

    // Lecture des données complètes envoyées
    while (Wire.available()) {
      char c = Wire.read();
      receivedData += c;
    }
  
    //Serial.println(receivedData);  // Affichage pour debug
    Serial.print(angle);
    Serial.print("   ");
    Serial.println(dist);

    // Extraction des données pour chaque tag
    int idx = receivedData.indexOf("ESP32:") + 6; // Trouver le début des données après "ESP32:"
    if (idx != -1) 
    {
    sscanf(receivedData.c_str(), "ESP32: dist=%d, angle=%d",
           &dist, &angle);
    }
    //switchStrat = digitalRead(pinStrat);
    }   
  }

void requestEvent() {
  static int value = 1;
  Wire.write(value); // Répond avec une valeur croissante
  Serial.print("Sent: ");
  Serial.println(value);
  value = value % 10 + 1; // Cycles from 1 to 10
}
void TaskInit(void *parameter) { 
  // Initialisation des semaphores
    odriveSemaphore = xSemaphoreCreateMutex();
    action1Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 1
    action2Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 2
    action3Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 3
    action4Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 4
    action5Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 5
    action6Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 6
    action7Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 7
    action8Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 8
    action9Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 9
    action10Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 10
    action11Semaphore = xSemaphoreCreateBinary(); // Créer un sémaphore binaire pour l'action 11
    panneau.setup();

    //Init I2C
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData);
    Wire.onRequest(requestEvent);
    Serial.begin(115200);

    // INIT TIRETTE
    pinMode(15, INPUT);


    // INIT ENCODER
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachHalfQuad(14, 13);
    encoder2.attachHalfQuad(26, 27);
    encoder.clearCount();
    encoder2.clearCount();

    // INIT MOTEURS
    odriveSerial1.begin(115200, SERIAL_8N1, 33, 25); 
    vTaskDelay(pdMS_TO_TICKS(200));
    odriveSerial2.begin(115200);                     
    while (odrive1.getState() == AXIS_STATE_UNDEFINED) {
        delay(100);
    }
    while (odrive1.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrive1.clearErrors();
        odrive1.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
    }
    while (odrive2.getState() == AXIS_STATE_UNDEFINED) {
        delay(100);
    }
    while (odrive2.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrive2.clearErrors();
        odrive2.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
    }
    odriveSerial1.println("w axis0.controller.config.input_mode 2"); // 2 correspond à INPUT_MODE_TRAP_TRAJ
    odriveSerial2.println("w axis0.controller.config.input_mode 2"); // 2 correspond à INPUT_MODE_TRAP_TRAJ
    odriveSerial1.println("w axis0.motor.config.current_lim 40.0"); // Exemple pour 40 A sur l'axe 0
    odriveSerial2.println("w axis0.motor.config.current_lim 40.0"); // Exemple pour 40 A sur l'axe 0

    //SÉLECTION STRATÉGIE
    switchStrat = digitalRead(pinStrat);
    Serial.println(switchStrat);
    if (switchStrat == 1) {
    // Copie des éléments de Strategiebleue dans Strategie
    for (int i = 0; i < NB_STRATEGIES; i++) {
        for (int j = 0; j < NB_PARAMS; j++) {
            Strategie[i][j] = Strategieopp[i][j];
            Serial.println("Bleue choisie");
        }
    }
    } 
    else if (switchStrat == 0) {
        // Copie des éléments de Strategiejaune dans Strategie
        for (int i = 0; i < NB_STRATEGIES; i++) {
            for (int j = 0; j < NB_PARAMS; j++) {
                Strategie[i][j] = Strategiecote[i][j];
                Serial.println("Jaune choisie");
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Donner du temps pour que le message soit envoyé
    
    // Démarrage des autres tâches...
    xTaskCreate(Tirette, "Tirette", 4096, NULL, 1, NULL);
    xTaskCreatePinnedToCore(
    encoderTask,         // Fonction de la tâche
    "Encoder Task",      // Nom de la tâche (pour le debug)
    2048,                // Taille de la pile allouée à la tâche
    NULL,                // Paramètres à passer à la tâche (ici, rien)
    1,                   // Priorité de la tâche (1 est bas)
    NULL,                // Handle de la tâche (pas nécessaire ici)
    1                    // Noyau sur lequel exécuter la tâche (0 ou 1 pour les ESP32 dual-core)
    );
    vTaskDelete(NULL); // Effacer cette tâche de la planification
}
void Tirette(void *parameter) {
    while (Tir == 0) {
        // Lire l'état du bouton sur le pin 25
        int buttonState = digitalRead(15);
        
        // Si le bouton est appuyé (LOW pour un bouton avec PULLUP interne), on change l'état de Tir
        if (buttonState == HIGH) {
            Tir = 1;
        }
        // Affiche "ready to start" tant que Tir est à 0
        Serial.println("ready to start");
        // Attendre 20 ms avant de lire à nouveau
        vTaskDelay(pdMS_TO_TICKS(20));
    }
        // Créer les autres tâches une fois que Tir est à 1
  xTaskCreate(executeStrategy, "ExecuteStrategy", 4096, NULL, 1, NULL);
  xTaskCreate(Timer, "Timer", 4096, NULL, 1, NULL);

  // Effacer cette tâche de la planification
  vTaskDelete(NULL);
}
void Timer(void *parameter) { 
  vTaskDelay(pdMS_TO_TICKS(100000)); // Attendre 100 secondes
  Serial.println("200 secondes c'est 2 minutes ????");
  odriveSerial1.println("w axis0.requested_state 1");  // Met l'axe 0 en idle
  odriveSerial2.println("w axis0.requested_state 1");  // Met l'axe 1 en idle
  vTaskDelete(xHandleStrategy);
  vTaskDelete(NULL); // Effacer cette tâche de la planification
}
void executeStrategy(void *pvParameters) {
  const TickType_t tickInterval = pdMS_TO_TICKS(100);
  TickType_t lastWakeTime;
  float targetM1 = 0;
  float targetM2 = 0;
  int inde = 1;
  a = 0;

  for (int i = 0; i < sizeof(Strategie) / sizeof(Strategie[0]); i++) {
    lastWakeTime = xTaskGetTickCount();

    sens = Strategie[i+1][1];
    float xtarget = Strategie[i + 1][2];
    float ytarget = Strategie[i + 1][3];

    //Parametres des moteurs
    float velocityrot_limit = 0.50000;
    float velocitylin_limit = Strategie[i + 1][6];
    float acceleration_limit = 10;
    float deceleration_limit = 10;

    //Parametres des taches
    int actionId = (int)Strategie[i + 1][4]; // Action ID
    int tache = (int)Strategie[i+1][0];
    float delay = Strategie[i + 1][5];
    inde = Strategie[i + 1][7];

    //LOG
    Serial.println("---------------------");
    Serial.print("En route vers : Point"); Serial.print(tache); Serial.print(" x: "); Serial.print(xtarget); Serial.print(" y: "); Serial.print(ytarget);Serial.print(" Sens: "); Serial.println(sens);

    float distance = sqrt(pow(xtarget - x, 2) + pow(ytarget - y, 2));
    float angle = atan2(ytarget - y, xtarget - x) * 180 / pi; // Convert to degrees

    // Détection des mouvements inutile (utilisé pour réaliser plusieurs actions à la suite)
    if (distance < 0.1 ) {
        Serial.println("Mouvement ignoré");
        performAction(actionId);
    continue; // Passe au prochain mouvement
    }

    // Mouvement pur en ignorant les écarts de position
    if (inde == 0) {
        targetM1 += (distance / (pi * diametre)) * (sens == 1 ? -1 : 1);
        targetM2 -= (distance / (pi * diametre)) * (sens == 1 ? -1 : 1);
        odrive2.setPosition(targetM2);
        odrive1.setPosition(targetM1);
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(Strategie[i + 1][5])); // Délai entre les actions
        // Maintenant, appelez performAction() avec l'action appropriée
        performAction(actionId);
    continue; // Passe au prochain mouvement
    }

    float angleDifference = Currentorientation - angle;


    //Conditions pour avoir toujours l'angle le plus court
    if (angleDifference > 180) angleDifference -= 360;
    if (angleDifference < -180) angleDifference += 360;

    targetM1 += (angleDifference * entraxe) / (360 * diametre); 
    targetM2 += (angleDifference * entraxe) / (360 * diametre); // Opposite for the other wheel

    ODriveFeedback feedback1 = odrive1.getFeedback();
    ODriveFeedback feedback2 = odrive2.getFeedback();

      // Setup the ODrive parameters for trajectory movement
    odrive1.setParameter("axis0.trap_traj.config.vel_limit", 2);
    odrive1.setParameter("axis0.trap_traj.config.accel_limit", 1);
    odrive1.setParameter("axis0.trap_traj.config.decel_limit", 1);
    odrive1.setParameter("axis0.controller.config.pos_gain", 40);
    odrive1.setParameter("axis0.controller.config.vel_gain", "0.32");
    odrive1.setParameter("axis0.controller.config.vel_integrator_gain", "0.4");

    odrive2.setParameter("axis0.trap_traj.config.vel_limit", 2);
    odrive2.setParameter("axis0.trap_traj.config.accel_limit", 1);
    odrive2.setParameter("axis0.trap_traj.config.decel_limit", 1);
    odrive2.setParameter("axis0.controller.config.pos_gain", 40);
    odrive2.setParameter("axis0.controller.config.vel_gain", "0.32");
    odrive2.setParameter("axis0.controller.config.vel_integrator_gain", "0.4");

    // Start the movement
    odrive2.trapezoidalMove(targetM2);
    odrive1.trapezoidalMove(targetM1);

    unsigned long startTime1 = millis(); // Début du timer

    // Monitoring loop to check if the target positions are reached
    bool isTargetReached1 = false, isTargetReached2 = false;
    while (!isTargetReached1 || !isTargetReached2) {
      ODriveFeedback feedback1 = odrive1.getFeedback();
      ODriveFeedback feedback2 = odrive2.getFeedback();
      if (fabs(feedback1.pos - targetM1) < 0.1) {
        isTargetReached1 = true;
      }
      if (fabs(feedback2.pos - targetM2) < 0.1) {
        isTargetReached2 = true;
      }
      //Système de sortie si le robot est coincé
      if (millis() - startTime1 >= 10000) 
      {
          isTargetReached1 = true;
          isTargetReached2 = true;
          Serial.println(" I'm Stuck");
      }
        // Ajouter vTaskDelayUntil dans la boucle while
        vTaskDelayUntil(&lastWakeTime, tickInterval);
      }
      Serial.print("Orientation souhaitée :");Serial.print(angle); Serial.print(" Réalisé :");Serial.println(Currentorientation);

      vTaskDelayUntil(&lastWakeTime, delay); // Délai entre les action
      
      targetM1 += (distance / (pi * diametre)) * (sens == 1 ? -1 : 1);
      targetM2 -= (distance / (pi * diametre)) * (sens == 1 ? -1 : 1);

      odrive1.setParameter("axis0.trap_traj.config.vel_limit", 2);
      odrive1.setParameter("axis0.trap_traj.config.accel_limit", 1);
      odrive1.setParameter("axis0.trap_traj.config.decel_limit", 1);
      odrive1.setParameter("axis0.controller.config.pos_gain", 40);
      odrive1.setParameter("axis0.controller.config.vel_gain", "0.5");
      odrive1.setParameter("axis0.controller.config.vel_integrator_gain", "0.4");

      odrive2.setParameter("axis0.trap_traj.config.vel_limit", 2);
      odrive2.setParameter("axis0.trap_traj.config.accel_limit", 1);
      odrive2.setParameter("axis0.trap_traj.config.decel_limit", 1);
      odrive2.setParameter("axis0.controller.config.pos_gain", 40);
      odrive2.setParameter("axis0.controller.config.vel_gain", "0.7");
      odrive2.setParameter("axis0.controller.config.vel_integrator_gain", "0.4");

      odrive2.trapezoidalMove(targetM2);
      odrive1.trapezoidalMove(targetM1);
      

      Serial.print("Position souhaitée : x=");Serial.print(xtarget);Serial.print(" y=");Serial.println(ytarget);

      isTargetReached1 = false;
      isTargetReached2 = false;

      unsigned long startTime2 = millis(); // Début du timer


      while (!isTargetReached1 || !isTargetReached2) {
          ODriveFeedback feedback1 = odrive1.getFeedback();
          ODriveFeedback feedback2 = odrive2.getFeedback();
          if (fabs(feedback1.pos - targetM1) < 0.1) {
            isTargetReached1 = true;
          }
          if (fabs(feedback2.pos - targetM2) < 0.1) {
            isTargetReached2 = true;
          }
          if (millis() - startTime2 >= 15000) {
            // if (sens == 0){
            //   Serial.println("je recule");
            //   targetM1 = feedback1.pos - 0.5;
            //   targetM2 = feedback1.pos + 0.5;
            //   odrive1.setPosition(targetM1);
            //   odrive2.setPosition(targetM2);
            //   vTaskDelayUntil(&lastWakeTime, 3000); // Délai entre les actions
            // }
            // else {
            //   Serial.println("j'avance");
            //   targetM1 = feedback1.pos + 0.5;
            //   targetM2 = feedback1.pos - 0.5;
            //   odrive1.setPosition(targetM1);
            //   odrive2.setPosition(targetM2);
            //   vTaskDelayUntil(&lastWakeTime, 3000); // Délai entre les actions
            // }
            isTargetReached1 = true;
            isTargetReached2 = true;
            Serial.println("I'm Stuck");  
        }
          if (FC_AvD == 1 || FC_AvG == 1) {
            isTargetReached1 = true;
            isTargetReached2 = true;
            Serial.println("opopopop");
            odriveSerial1.println("w axis0.requested_state 1");  // Met l'axe 0 en idle
            odriveSerial2.println("w axis0.requested_state 1");  // Met l'axe 1 en idle
            vTaskDelay(pdMS_TO_TICKS(100));
            odriveSerial1.println("w axis0.requested_state 8");
            odriveSerial2.println("w axis0.requested_state 8");
            isTargetReached1 = true;
            Serial.println(isTargetReached1);
            isTargetReached2 = true;
            Serial.println(isTargetReached2);
            Serial.println("ipipip");
          }
          if ((/*(DIST_AV == 2 && sens == 1) ||*/ (DIST_AR == 1 && sens == 0))) {
            Serial.println("Obstacle detected - Suspending strategy task...");
            // Arrêt des moteurs en envoyant une vitesse de 0
            odriveSerial1.println("w axis0.requested_state 1");  // Met l'axe 0 en idle
            odriveSerial2.println("w axis0.requested_state 1");  // Met l'axe 1 en idle
            vTaskDelay(pdMS_TO_TICKS(100));
            odriveSerial1.println("w axis0.requested_state 8");
            odriveSerial2.println("w axis0.requested_state 8");
            compteur = 0;
            // Attendre ici ou dans une boucle séparée jusqu'à ce que la voie soit dégagée
            while (/*(DIST_AV == 1 && sens == 1) || */(DIST_AR == 1 && sens == 0)){
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            Serial.println("Obstacle cleared - Resuming strategy task...");
            odrive1.setPosition(targetM1);
            odrive2.setPosition(targetM2);
            isTargetReached1 = isTargetReached2 = false;  // Reset the flag to continue checking the target reach
            compteur = 1;
          }
        vTaskDelayUntil(&lastWakeTime, tickInterval);
      }
      //Serial.println("Délai");
      vTaskDelayUntil(&lastWakeTime, delay); // Délai entre les action
      Serial.println(" ");Serial.print("Réalisé : x=");Serial.print(x);Serial.print(" y=");Serial.println(y);
      
      // Le code effectue l'action définie dans la stratégie
      performAction(actionId);
  }
  vTaskDelete(NULL); // End the task after completing all movements
}

void encoderTask(void *pvParameters) {
    Currentorientation = Strategie[0][4];
    x = Strategie[0][2];
    y = Strategie[0][3];
    float a = Currentorientation * (PI / 180);
    while (1) {
        // Lecture des positions actuelles des encodeurs en mm
        float currentPos1 = (int64_t)encoder.getCount() / 20.0; // mm
        float currentPos2 = (int64_t)encoder2.getCount() / 20.0; // mm

        // Calcul des distances parcourues depuis la dernière lecture
        float dLeft = currentPos1 - lastPos1;   // Distance parcourue par la roue gauche
        float dRight = currentPos2 - lastPos2;  // Distance parcourue par la roue droite

        // Mise à jour des dernières positions
        lastPos1 = currentPos1;
        lastPos2 = currentPos2;

        // Calcul de la distance moyenne parcourue
        float dCenter = (dLeft + dRight) / 2.0;

        // Calcul du changement d'orientation (delta theta)
        float deltaTheta = (dRight - dLeft) / L;

        // Mise à jour de l'orientation (theta)
        a+= deltaTheta;
        Currentorientation += deltaTheta * (180.0 / PI); // Convertir deltaTheta en degrés

        if (Currentorientation > 180) {
            Currentorientation -= 360;
        }
        if (Currentorientation < -180) {
            Currentorientation += 360;
        }

        if (a > PI) {
          a -= 2 * PI;
        }
        if (a < -PI) {
            a += 2 * PI;
        }

        // Mise à jour des coordonnées x et y
        x -= dCenter * cos(a);
        y -= dCenter * sin(a);

        // Envoi des données sous format CSV pour réglage PID
        /*
        Serial.print(currentPos1);
        Serial.print(",");
        Serial.print(currentPos2);
        Serial.print(",");
        Serial.print(a);  // CSV sans retour à la ligne supplémentaire
        Serial.print(";");
        */

        // Pause pour ne pas occuper 100% du processeur et permettre l'enregistrement à intervalles réguliers
        vTaskDelay(pdMS_TO_TICKS(10)); // Attente de 10 ms (10 Hz)
    }
}


void performAction(int actionId) { // Réalisation des actions
  switch (actionId) {
    case 0: //position initiale
        xTaskCreate(
              executeAction1, // Fonction de la tâche pour l'action 1
              "Action 1 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
      );
      xSemaphoreTake(action1Semaphore, portMAX_DELAY); // Attendre que l'action 1 soit terminée
    break;

    case 1: //fourche1 haut
        xTaskCreate(
              executeAction2, // Fonction de la tâche pour l'action 2
              "Action 2 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
      );
      xSemaphoreTake(action2Semaphore, portMAX_DELAY); // Attendre que l'action 2 soit terminée
    break;
          
    case 2: //fourche1 basse
        xTaskCreate(
              executeAction3, // Fonction de la tâche pour l'action 1
              "Action 3 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
      );
      xSemaphoreTake(action3Semaphore, portMAX_DELAY); // Attendre que l'action 3 soit terminée
    break;

    case 3://fourche1 millieu
      xTaskCreate(
              executeAction4, // Fonction de la tâche pour l'action 1
              "Action 4 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
      );
      xSemaphoreTake(action4Semaphore, portMAX_DELAY); // Attendre que l'action 4 soit terminée
    break;
      
    case 4:
    break;

    case 5:// fourche2 haute
      xTaskCreate(
                  executeAction5, // Fonction de la tâche pour l'action 1
                  "Action 5 Task", // Nom de la tâche
                  2048, // Taille de la pile (en mots)
                  NULL, // Paramètre de la tâche
                  2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
                  NULL // Handle de la tâche
      );
      xTaskCreate(
              executeAction6, // Fonction de la tâche pour l'action 1
              "Action 6 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
      );
      xSemaphoreTake(action5Semaphore, portMAX_DELAY);
      xSemaphoreTake(action6Semaphore, portMAX_DELAY);
    break;  

    case 6: // fourche_2_basse
      xTaskCreate(
                  executeAction7, // Fonction de la tâche pour l'action 1
                  "Action 7 Task", // Nom de la tâche
                  2048, // Taille de la pile (en mots)
                  NULL, // Paramètre de la tâche
                  2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
                  NULL // Handle de la tâche
      );
      xTaskCreate(
              executeAction8, // Fonction de la tâche pour l'action 1
              "Action 8 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
      );
      xSemaphoreTake(action7Semaphore, portMAX_DELAY);
      xSemaphoreTake(action8Semaphore, portMAX_DELAY);
    break;  

    case 7: // fourche_2_depose
      xTaskCreate(
                  executeAction9, // Fonction de la tâche pour l'action 1
                  "Action 9 Task", // Nom de la tâche
                  2048, // Taille de la pile (en mots)
                  NULL, // Paramètre de la tâche
                  2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
                  NULL // Handle de la tâche
      );
      xTaskCreate(
              executeAction10, // Fonction de la tâche pour l'action 1
              "Action 10 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
      );
      xSemaphoreTake(action9Semaphore, portMAX_DELAY);
      xSemaphoreTake(action10Semaphore, portMAX_DELAY);
    break;    

    case 8:
              xTaskCreate(
              executeAction11, // Fonction de la tâche pour l'action 1
              "Action 11 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
            );
      xSemaphoreTake(action11Semaphore, portMAX_DELAY); // Attendre que l'action 15 soit terminée
    break;  

      case 9 : 
      xTaskCreate(
              executeAction12, // Fonction de la tâche pour l'action 1
              "Action 12 Task", // Nom de la tâche
              2048, // Taille de la pile (en mots)
              NULL, // Paramètre de la tâche
              2, // Priorité de la tâche (plus haute que la priorité de la tâche principale)
              NULL // Handle de la tâche
      );
      break; 

    case 12:
      odriveSerial1.println("w axis0.requested_state 1");  // Met l'axe 0 en idle
      odriveSerial2.println("w axis0.requested_state 1");  // Met l'axe 1 en idle
      //vTaskDelete(xHandleStrategy);
      delay(50000000000000);
    break;
  }
}


void executeAction1(void *parameter) {
    // Code de l'action 1
    fourche.mise_en_position_initiale_1();
    xSemaphoreGive(action1Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction2(void *parameter) {
    fourche.fourche_1_haute();
    xSemaphoreGive(action2Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}
void executeAction3(void *parameter) {
    fourche.fourche_1_basse();
    xSemaphoreGive(action3Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction4(void *parameter) {
    fourche.fourche_1_depose();
    xSemaphoreGive(action4Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction5(void *parameter) {
    fourche.fourche_2_hauteD();
    xSemaphoreGive(action5Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction6(void *parameter) {
    fourche.fourche_2_hauteG();
    xSemaphoreGive(action6Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction7(void *parameter) {
    fourche.fourche_2_basseD();
    xSemaphoreGive(action7Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}
void executeAction8(void *parameter) {
    fourche.fourche_2_basseG();
    xSemaphoreGive(action8Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction9(void *parameter) {
    fourche.fourche_2_deposeD();
    xSemaphoreGive(action9Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction10(void *parameter) {
    fourche.fourche_2_deposeG();
    xSemaphoreGive(action10Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction11(void *parameter) {
    panneau.start();
    xSemaphoreGive(action11Semaphore); // Libérer le sémaphore lorsque l'action est terminée
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void executeAction12(void *parameter) {
    panneau.roule();
    vTaskDelete(NULL); // Supprimer la tâche une fois l'action terminée
}

void setup() {
    Serial.begin(115200);

    xTaskCreate(
        TaskInit,       // Fonction de la tâche
        "Init Task",    // Nom de la tâche
        2048,           // Taille de la pile (en mots)
        NULL,           // Paramètre de la tâche
        1,              // Priorité de la tâche
        NULL            // Handle de la tâche
    );

    Wire.begin(SLAVE_ADDRESS); // Démarre la communication I2C en tant qu'esclave avec l'adresse 8
    Wire.onReceive(receiveData); // Définit la fonction de réception des données
    Wire.onRequest(requestEvent);
    encoderSemaphore = xSemaphoreCreateMutex();
    pinMode(35, INPUT);
    fourche.setup();

}

void loop() {}
