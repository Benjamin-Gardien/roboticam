#include "Panneau.h"
#include <Arduino.h>

#define DIR_PIN   13
#define STEP_PIN  14

Panneau::Panneau() : stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN) {}

void Panneau::setup() {
    Serial.begin(115200);
    servobras.attach(27);
    stepper.setMaxSpeed(15000);
    stepper.setAcceleration(10000);
}

void Panneau::start() {
  position0();
    stepper.setCurrentPosition(0);
     stepper.move(450);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
     }
     }

void Panneau::roule(){
position2();}


void Panneau::position1() {
    servobras.write(0);
}

void Panneau::position0() {
    servobras.write(58);
}

void Panneau::position2() {
    servobras.write(20);
}