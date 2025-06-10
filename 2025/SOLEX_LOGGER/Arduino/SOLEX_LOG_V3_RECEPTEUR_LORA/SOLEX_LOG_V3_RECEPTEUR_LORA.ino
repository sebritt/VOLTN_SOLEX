#include <SPI.h>
#include <LoRa.h>

void setup() {
  Serial.begin(9600);
  while (!Serial); // Attendre que le port série soit prêt

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(868E6)) { // Fréquence Europe (868 MHz)
    Serial.println("Erreur d'initialisation LoRa !");
    while (1);
  }

  Serial.println("LoRa prêt !");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Paquet reçu (");
    Serial.print(packetSize);
    Serial.print(" octets) : ");

    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    Serial.println();
  }
}
