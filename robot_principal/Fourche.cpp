#include "Fourche.h"

Fourche::Fourche() {
}

void Fourche::setup() {
    //Serial.begin(115200);
    

    Serial.println("Pinces setup() called");

    // Pin pour chaque servomoteur
    servoElevateur1.attach(19);
    servoElevateur2.attach(23);
    servoElevateur3.attach(5);

}

void Fourche::mise_en_position_initiale_1() {
elevateur_1_bas();
elevateur_2_bas();
elevateur_3_bas();
}

void Fourche::fourche_1_haute() {
  elevateur_2_haut();
}

void Fourche::fourche_1_basse() {
  elevateur_2_bas();
}

void Fourche::fourche_2_hauteG() {
  elevateur_1_haut();
  delay(1500);
}
void Fourche::fourche_2_hauteD() {
  elevateur_3_haut();
  delay(1500);
}

void Fourche::fourche_2_basseG() {
  elevateur_1_bas();
  delay(1500);
}
void Fourche::fourche_2_basseD() {
  elevateur_3_bas();
  delay(1500);
}

void Fourche::fourche_1_depose() {
  elevateur_2_milieu();
}

void Fourche::fourche_2_deposeG() {
  elevateur_1_milieu();
  delay(1500);
}

void Fourche::fourche_2_deposeD() {
  elevateur_3_milieu();
  delay(1500);
}

void Fourche::elevateur_1_bas() {
  servoElevateur1.write(1);
}
void Fourche::elevateur_1_milieu() {
  servoElevateur1.write(38);
}
void Fourche::elevateur_1_haut() {
  servoElevateur1.write(90);
}

void Fourche::elevateur_2_bas() {
  servoElevateur2.write(1);
}
void Fourche::elevateur_2_milieu() {
  servoElevateur2.write(38);
}
void Fourche::elevateur_2_haut() {
  servoElevateur2.write(95);
}

void Fourche::elevateur_3_bas() {
  servoElevateur3.write(1);
}
void Fourche::elevateur_3_milieu() {
  servoElevateur3.write(38);
}
void Fourche::elevateur_3_haut() {
  servoElevateur3.write(93);
}
