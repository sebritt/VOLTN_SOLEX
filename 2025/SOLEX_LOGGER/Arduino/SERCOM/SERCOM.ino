#include <Arduino.h>
#include "wiring_private.h" // Nécessaire pour pinPeripheral()

// Création du port série matériel sur SERCOM3 (RX=1, TX=0)
Uart mySerial(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void setup() {
  Serial.begin(115200);     // USB serial monitor
  mySerial.begin(9600);     // GPS UART

  // Configuration des pins pour le SERCOM3
  pinPeripheral(1, PIO_SERCOM); // RX
  pinPeripheral(0, PIO_SERCOM); // TX

  delay(2000); // Attente du démarrage

  Serial.println("Lecture brute des trames GPS via UART matériel (SERCOM3 sur pins 1/0)");
}

void loop() {
  // Lecture des données du GPS et affichage dans le moniteur série USB
  while (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c); // Affiche la trame NMEA brute
  }
}

// Handler d'interruption obligatoire pour le SERCOM
void SERCOM3_Handler() {
  mySerial.IrqHandler();
}