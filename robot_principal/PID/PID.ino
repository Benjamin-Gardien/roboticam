#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include "Pinces.h"
#include "Panneau.h"
#include "Camera.h"

SemaphoreHandle_t odriveSemaphore;
TaskHandle_t xHandleStrategy;

void TaskInit(void *parameter);
void executeStrategy(void *parameter);

HardwareSerial odriveSerial1(1); // Utilisez le second port série sur l'ESP32
HardwareSerial odriveSerial2(2); // Utilisez le second port série sur l'ESP32
ODriveArduino odrive1(odriveSerial1);
ODriveArduino odrive2(odriveSerial2);

int Strategie[48][5] ={{0,0.0,0.0,0,1},{1,0.0,0.0,1,5},{0,0.0,0.0,1,1},{1,-9.3,9.3,2,5},{0,-9.3,9.3,0,1},{1,-17.0,17.0,0,5},{0,-20.3,13.7,0,1},{1,-11.0,4.4,0,5},{0,-12.6,2.8,0,1},{1,-6.1,-3.7,0,5},{0,-6.1,-3.7,0,1},{1,-1.7,-8.1,0,5},{0,1.9,-4.5,0,1},{1,32.8,-35.4,0,5},{0,34.1,-34.1,0,1},{1,36.6,-36.6,0,5},{0,36.6,-36.6,0,1},{1,30.4,-30.4,0,5},{0,27.1,-33.7,0,1},{1,37.9,-44.5,0,5},{0,37.9,-44.5,0,1},{1,28.7,-35.2,0,5},{0,31.9,-31.9,0,1},{1,38.7,-38.7,0,5},{0,38.7,-38.7,0,1},{1,41.8,-41.8,0,5},{0,41.8,-41.8,0,1},{1,20.2,-20.2,0,5},{0,26.2,-14.2,0,1},{1,32.6,-20.5,0,5},{0,31.5,-21.7,0,1},{1,35.9,-26.0,0,5},{0,41.5,-20.3,0,1},{1,63.1,-41.9,0,5},{0,62.3,-42.6,0,1},{1,56.1,-36.4,0,5},{0,52.4,-40.2,0,1},{1,66.6,-54.4,0,5},{0,67.1,-54.0,0,1},{1,73.3,-60.2,0,5},{0,73.3,-60.2,0,1},{1,79.4,-66.3,0,5},{0,78.4,-67.4,0,1},{1,124.0,-113.0,0,5},{0,123.4,-113.6,0,1},{1,119.1,-109.2,0,5},{0,119.1,-109.2,0,1},{1,114.7,-104.9,0,5},};
const float acceleration = 5; // Accélération en unités par seconde²
bool mode = 1;

void setParameter(HardwareSerial& serial, const String& command) {
    serial.println(command);
}
void TaskInit(void *parameter) {
  odriveSemaphore = xSemaphoreCreateMutex();
  // Petite pause pour s'assurer que la communication série est prête
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
  xTaskCreate(executeStrategy, "ExecuteStrategy", 4096, NULL, 1, NULL);
  vTaskDelete(NULL); // Effacer cette tâche de la planification
}

void executeStrategy(void *pvParameters) {
    const TickType_t tickInterval = pdMS_TO_TICKS(100);
    TickType_t lastWakeTime;

    for (int i = 0; i < sizeof(Strategie) / sizeof(Strategie[0]); i++) {
        lastWakeTime = xTaskGetTickCount();
        float targetM1 = -Strategie[i][1];
        float targetM2 = -Strategie[i][2];

        while (true) {
            errorM1 = targetM1 - odrive1.getPosition(0); // Get current motor position
            integralM1 += errorM1 * (tickInterval / 1000.0);
            derivativeM1 = (errorM1 - prevErrorM1) / (tickInterval / 1000.0);
            float controlM1 = kp * errorM1 + ki * integralM1 + kd * derivativeM1;

            errorM2 = targetM2 - odrive2.getPosition(0); // Get current motor position
            integralM2 += errorM2 * (tickInterval / 1000.0);
            derivativeM2 = (errorM2 - prevErrorM2) / (tickInterval / 1000.0);
            float controlM2 = kp * errorM2 + ki * integralM2 + kd * derivativeM2;

            odrive1.setPosition(0, controlM1);
            odrive2.setPosition(0, controlM2);

            prevErrorM1 = errorM1;
            prevErrorM2 = errorM2;

            if (fabs(errorM1) < 0.1 && fabs(errorM2) < 0.1) break; // Check if error is small enough

            vTaskDelayUntil(&lastWakeTime, tickInterval);
        }

        performAction(Strategie[i][3]);  // Execute the action associated with the current step
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000));
    }
    vTaskDelete(NULL); // Terminate the task after completing the motion
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

  setParameter(odriveSerial1, "w axis0.controller.config.pos_gain 100.0");
  setParameter(odriveSerial1, "w axis0.controller.config.vel_gain 0");
  setParameter(odriveSerial1, "w axis0.controller.config.vel_integrator_gain 0");

  setParameter(odriveSerial2, "w axis0.controller.config.pos_gain 100.0");
  setParameter(odriveSerial2, "w axis0.controller.config.vel_gain 0");
  setParameter(odriveSerial2, "w axis0.controller.config.vel_integrator_gain 0");

}

void loop() {
    // Laisser la boucle loop vide dans un environnement FreeRTOS
}