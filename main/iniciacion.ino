void iniciacion_general()
{

  // LEDs
  pinMode(A0, OUTPUT); // Led rojo 
  pinMode(A1, OUTPUT); // Led amarillo
  pinMode(A2, OUTPUT); // Led verde
  pinMode(A3, OUTPUT); // Segundo led rojo


// Protocolo de comunicación I2C

Wire.begin(); //  Inicia la librería Wire e inicia el bus I2C como maestro o esclavo.
TWBR = 24; // Ajusta la frecuencia del I2C a 400kHz (default = 100kHz lo cual nos da tiempos de lectura de acelerómetro demasiado grandes)


// Declararacion de pines para señal ESC
  pinMode(3, OUTPUT);  //Motor 2
  pinMode(4, OUTPUT);  //Motor 1
  pinMode(5, OUTPUT);  //Motor 4
  pinMode(6, OUTPUT);  //Motor 3

 // Declaramos las interrupciones para el mando RC
  pinMode(7, INPUT_PULLUP);                  // ROLL
  enableInterrupt(7, InterrupcionRoll, CHANGE);
  pinMode(8, INPUT_PULLUP);                 // PITCH
  enableInterrupt(8, InterrupcionPitch, CHANGE);
  pinMode(9, INPUT_PULLUP);                  // POTENCIA
  enableInterrupt(9, InterrupcionPotencia, CHANGE);
  pinMode(10, INPUT_PULLUP);                  // YAW
  enableInterrupt(10, InterrupcionYaw, CHANGE);

 
}
