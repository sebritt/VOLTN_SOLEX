int receivedNumber =0;
void setup() {
  // Initialise le port série à 9600 bauds
  Serial.begin(9600);
  
}

void loop() {
  if (Serial.available() > 0) {
    // Attendre un peu pour que tout le nombre arrive
    delay(100);

    // Lire le nombre entier envoyé sur le port série
    receivedNumber = Serial.parseInt();

    // Affiche le nombre reçu sur le moniteur série
    Serial.print("Nombre reçu : ");
    Serial.println(receivedNumber);
  }
  analogWrite(10,receivedNumber);
}
