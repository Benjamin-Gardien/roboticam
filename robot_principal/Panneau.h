#ifndef PANNEAU_H
#define PANNEAU_H

#include <ESP32Servo.h>
#include <AccelStepper.h>

class Panneau {
public:
    Panneau();
    void setup();
    void start(); 
    void roule ();
private:
    Servo servobras;
    AccelStepper stepper; // Déclaration de stepper dans le fichier d'en-tête

    void position1();
    void position0();
    void position2();
};

#endif
