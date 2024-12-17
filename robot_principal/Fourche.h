#ifndef FOURCHE_H
#define FOURCHE_H

#include <Arduino.h>
#include <ESP32Servo.h>

class Fourche {
public:
    Fourche();
    void setup();
    void mise_en_position_initiale_1();
    void fourche_1_haute();
    void fourche_1_basse();
    void fourche_1_depose();
    void fourche_2_hauteD();
    void fourche_2_hauteG();
    void fourche_2_basseD();
    void fourche_2_basseG();
    void fourche_2_deposeD();
    void fourche_2_deposeG();


private:
    Servo servoElevateur1;
    Servo servoElevateur2;
    Servo servoElevateur3;

    const int IR_1 = 35;
    const int IR_2 = 39;
    const int IR_3 = 34;

  
    void elevateur_1_bas();
    void elevateur_1_milieu();
    void elevateur_1_haut();
    void elevateur_2_bas();
    void elevateur_2_milieu();
    void elevateur_2_haut();
    void elevateur_3_bas();
    void elevateur_3_milieu();
    void elevateur_3_haut();
};

#endif
