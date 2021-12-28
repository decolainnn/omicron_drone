

//////////////////////////////////////////MANDO RC//////////////////////////////////////////////////////


// MANDO POTENCIA
const int max_potencia_mando = 1900;
const int min_potencia_mando = 1000;
const int max_pulso_ESC = 1750;  // 
const int min_pulso_ESC = 1000;  //  

// MANDO YAW
const int max_yaw_mando = 1980;
const int min_yaw_mando = 990;
const int max_velocidad_rotacion_z = 60; //
const int min_velocidad_rotacion_z = -60;  // 


// MANDO PITCH
const int max_pitch_mando = 1940;
const int min_pitch_mando = 1000;
const int max_velocidad_rotacion_x = -75; //
const int min_velocidad_rotacion_x = 75;  //  

// MANDO ROLL
const int max_roll_mando = 1940;
const int min_roll_mando = 970;
const int max_velocidad_rotacion_y = 60; //
const int min_velocidad_rotacion_y = -60;  // 

// SEGURIDAD
int contador_seguridad;



//=============================================//////////////////// INTERRUPCIONES ////////////////////=============================================
volatile long roll_inicial; // LEER MANDO RC --> ROLL (ALABEO)
volatile int pulso_roll;
void InterrupcionRoll() {
  if (digitalRead(7) == HIGH) 
  roll_inicial = micros();
  if (digitalRead(7) == LOW) 
  pulso_roll = micros()-roll_inicial;
}

volatile long pitch_inicial; // LEER MANDO RC --> PITCH (CABECEO)
volatile int pulso_pitch;
void InterrupcionPitch() {
  if (digitalRead(8) == HIGH) 
  pitch_inicial = micros();
  if (digitalRead(8) == LOW) 
  pulso_pitch = micros()-pitch_inicial;
}

volatile long potencia_inicial; // LEER MANDO RC --> POTENCIA (Throttle)
volatile int pulso_potencia;
void InterrupcionPotencia() {
  if (digitalRead(9) == HIGH) 
  potencia_inicial = micros();
  if (digitalRead(9) == LOW) 
  pulso_potencia = micros()-potencia_inicial;
}

volatile long yaw_inicial; // LEER MANDO RC --> YAW (GUIÑADA)
volatile int pulso_yaw;
void InterrupcionYaw() {
  if (digitalRead(10) == HIGH) 
  yaw_inicial = micros();
  if (digitalRead(10) == LOW) 
  pulso_yaw = micros()- yaw_inicial;
}







void generar_senal_mando()
{
//mapeamos las señales de manera lineal ajustando los maximos y los minimos 
pulso_potencia_consigna= map(pulso_potencia, min_potencia_mando, max_potencia_mando,min_pulso_ESC,max_pulso_ESC);
velocidad_rotacion_yaw_consigna= map(pulso_yaw, min_yaw_mando, max_yaw_mando, min_velocidad_rotacion_z, max_velocidad_rotacion_z);
velocidad_rotacion_pitch_consigna= map(pulso_pitch, min_pitch_mando, max_pitch_mando, min_velocidad_rotacion_x, max_velocidad_rotacion_x); 
velocidad_rotacion_roll_consigna= map(pulso_roll, min_roll_mando, max_roll_mando, min_velocidad_rotacion_y, max_velocidad_rotacion_y); 


//if(ModoVuelo == 1) //modo estable pasamos la consigna de grados por segundo a grados de inclinacion, esto es dividiendo entre 3 para tener un control mas suave
//  {
//velocidad_rotacion_pitch_consigna = velocidad_rotacion_pitch_consigna/3;
//velocidad_rotacion_roll_consigna = velocidad_rotacion_roll_consigna/3;
//velocidad_rotacion_yaw_consigna = velocidad_rotacion_yaw_consigna/3;
//  }




  /////////////////////PRUEBAS DE ESTABILIDAD //////////////////

  velocidad_rotacion_pitch_consigna =  0;
  velocidad_rotacion_roll_consigna = 0;

  // Detectar perdida mando: si la lectura del Mando RC es indentica a la medida anterior, apagamos los motores
 
  if (pulso_potencia_anterior == pulso_potencia)contador_seguridad++;
  else contador_seguridad = 0;
  if (contador_seguridad > 300) { // Perdida conexion con mando RC
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(A3, HIGH);

    while(1);
  }
  pulso_potencia_anterior = pulso_potencia;


}
