#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>

void TaskInit(void *parameter);
void executeStrategy(void *parameter);

HardwareSerial odriveSerial1(1); // Utilisez le second port série sur l'ESP32
HardwareSerial odriveSerial2(2); // Utilisez le second port série sur l'ESP32
ODriveArduino odrive1(odriveSerial1);
ODriveArduino odrive2(odriveSerial2);

float Strategie[48][5] ={
{1,-0.93,0.93,2,5},{0,-0.93,0.93,0,1},{1,-1.70,1.70,0,5},{0,-2.03,1.37,0,1},{1,-1.10,0.44,0,5},{0,-1.26,0.28,0,1},{1,-0.61,-0.37,0,5},{0,-0.61,-0.37,0,1},{1,-0.17,-0.81,0,5},{0,0.19,-0.45,0,1},{1,3.28,-3.54,0,5},{0,3.41,-3.41,0,1},{1,3.66,-3.66,0,5},{0,3.66,-3.66,0,1},{1,3.04,-3.04,0,5},{0,2.71,-3.37,0,1},{1,3.79,-4.45,0,5},{0,3.79,-4.45,0,1},{1,2.87,-3.52,0,5},{0,3.19,-3.19,0,1},{1,3.87,-3.87,0,5},{0,3.87,-3.87,0,1},{1,4.18,-4.18,0,5},{0,4.18,-4.18,0,1},{1,2.02,-2.02,0,5},{0,2.62,-1.42,0,1},{1,3.26,-2.05,0,5},{0,3.15,-2.17,0,1},{1,3.59,-2.60,0,5},{0,4.15,-2.03,0,1},{1,6.31,-4.19,0,5},{0,6.23,-4.26,0,1},{1,5.61,-3.64,0,5},{0,5.24,-4.02,0,1},{1,6.66,-5.44,0,5},{0,6.71,-5.40,0,1},{1,7.33,-6.02,0,5},{0,7.33,-6.02,0,1},{1,7.94,-6.63,0,5},{0,7.84,-6.74,0,1},{1,12.40,-11.30,0,5},{0,12.34,-11.36,0,1},{1,11.91,-10.92,0,5},{0,11.91,-10.92,0,1},{1,11.47,-10.49,0,5}};

void TaskInit(void *parameter) {

  odriveSerial1.begin(115200, SERIAL_8N1, 4, 2); // Baud rate, format 8N1, RX sur GPIO4, TX sur GPIO2
  vTaskDelay(pdMS_TO_TICKS(5000)); // Attendre 1000 ms
  odriveSerial2.begin(115200); // Assurez-vous de définir RX_PIN et TX_PIN avec vos pins
  while (odrive1.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }
  while (odrive1.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive1.clearErrors();
    odrive1.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  // INIT ODRIVE 2
    while (odrive2.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }

  while (odrive2.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive2.clearErrors();
    odrive2.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  vTaskDelay(pdMS_TO_TICKS(500)); // Donner du temps pour que le message soit envoyé

  // Démarrage des autres tâches...
  vTaskDelete(NULL); // Effacer cette tâche de la planification
}

void executeStrategy(void *pvParameters) {
    const TickType_t tickInterval = pdMS_TO_TICKS(100);
    TickType_t lastWakeTime;

    for (int i = 0; i < sizeof(Strategie) / sizeof(Strategie[0]); i++) {
        lastWakeTime = xTaskGetTickCount();
        float targetM1 = -Strategie[i][1];
        float targetM2 = -Strategie[i][2];
        float velocity_limit = Strategie[i][4];
        float acceleration_limit = 1;
        float deceleration_limit = 1;

        odrive1.setParameter("axis0.trap_traj.config.vel_limit", velocity_limit);
        odrive1.setParameter("axis0.trap_traj.config.accel_limit", acceleration_limit);
        odrive1.setParameter("axis0.trap_traj.config.decel_limit", deceleration_limit);

        odrive2.setParameter("axis0.trap_traj.config.vel_limit", velocity_limit);
        odrive2.setParameter("axis0.trap_traj.config.accel_limit", acceleration_limit);
        odrive2.setParameter("axis0.trap_traj.config.decel_limit", deceleration_limit);

        odrive1.trapezoidalMove(targetM1);
        odrive2.trapezoidalMove(targetM2);

        // Boucle de surveillance pour vérifier l'état du mouvement
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
            
            vTaskDelayUntil(&lastWakeTime, tickInterval);
        }
      vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000)); // Délai de 1 seconde entre les actions
    }
    vTaskDelete(NULL); // Terminer la tâche après avoir complété les mouvements
}

void performAction(int actionId) {
    switch (actionId) {
        case 0:
            Serial.println("Etape 1");
            break;
        case 1:
            Serial.println("Etape 2");
            break;
        case 2:
            Serial.println("Etape 3");
          break;
      case 3:
            Serial.println("Etape 4");
            break;
        default:
            break;
    }
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
}

void loop() {
    // Laisser la boucle loop vide dans un environnement FreeRTOS
}