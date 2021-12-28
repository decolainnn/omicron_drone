bool M1, M2, M3, M4 = false;
float pulso_m1, pulso_m2, pulso_m3, pulso_m4;

void calculo_pulsos()
{

  if (pulso_potencia_consigna < 1250) //por debajo de 1300 de señal de potencia desactivamos el PID, la salida de los motores es directamente la del mando
  {
    senal1 = pulso_potencia_consigna;
    senal2 = pulso_potencia_consigna;
    senal3 = pulso_potencia_consigna;
    senal4 = pulso_potencia_consigna;
    if (senal1 < 1150) senal1 = 1000; 
    if (senal2 < 1150) senal2 = 1000;
    if (senal3 < 1150) senal3 = 1000;
    if (senal4 < 1150) senal4 = 1000;
  }
  if (pulso_potencia_consigna >= 1250)  //por encima activamos el PID 
  {
    senal1 = pulso_potencia_consigna - PID_x_ang - PID_y_ang; //+ PID_z_vel;
    senal2 = pulso_potencia_consigna + PID_x_ang - PID_y_ang; //- PID_z_vel;
    senal3 = pulso_potencia_consigna - PID_x_ang + PID_y_ang; //- PID_z_vel;
    senal4 = pulso_potencia_consigna + PID_x_ang + PID_y_ang; //+ PID_z_vel;

    if (senal1 < 1100) senal1 = 1150; //Evitamos mandar una señal menor a 1100 a los motores
    if (senal2 < 1100) senal2 = 1150;
    if (senal3 < 1100) senal3 = 1150;
    if (senal4 < 1100) senal4 = 1150;
    if (senal1 > 2000) senal1 = 2000; //Evitamos mandar una señal mayor a 2000ms a los motores
    if (senal2 > 2000) senal2 = 2000;
    if (senal3 > 2000) senal3 = 2000;
    if (senal4 > 2000) senal4 = 2000;
  }
}

void generador_pulsos ()
{

  digitalWrite(3, HIGH); //Motor 2 HIGH
  digitalWrite(4, HIGH); //Motor 1 HIGH
  digitalWrite(5, HIGH); //Motor 4 HIGH
  digitalWrite(6, HIGH); //Motor 3 HIGH
  
  M1 = M2 = M3 = M4 = true;

  //calculamos el pulso de cada motor (salida pid + consigna mando)
  pulso_m1= senal1 + tiempo_bucle;
  pulso_m2= senal2 + tiempo_bucle;
  pulso_m3= senal3 + tiempo_bucle;
  pulso_m4= senal4 + tiempo_bucle;


  
//suponemos que el tiempo entre que encendemos la señal y llegamos al bucle es menor de 1ms, de otra manera no podriamos mandar señales de dicha magnitud

  while (M1 || M2 || M3 || M4 == true) {
    tiempo_bucle_general_variadores = micros();
    if (pulso_m2 <= tiempo_bucle_general_variadores) { // Motor 2 LOW
      M2 = false;
      digitalWrite(3, LOW);
    }
    tiempo_bucle_general_variadores = micros();
    if (pulso_m1 <= tiempo_bucle_general_variadores) { // Motor 1 LOW
      M1 = false;
      digitalWrite(4, LOW);
    }
    tiempo_bucle_general_variadores = micros();
    if (pulso_m4 <= tiempo_bucle_general_variadores) { // Motor 4 LOW
      M4 = false;
      digitalWrite(5, LOW);
    }
    tiempo_bucle_general_variadores = micros();
    if (pulso_m3 <= tiempo_bucle_general_variadores) { // Motor 3 LOW
      M3 = false;
      digitalWrite(6, LOW);
    }
  }


}



void calibrar_motores() {

  while (pulso_roll < 1700) {
    
    digitalWrite(3, HIGH); //Motor 1
    digitalWrite(4, HIGH); //Motor 2
    digitalWrite(5, HIGH); //Motor 3
    digitalWrite(6, HIGH); //Motor 4
    delayMicroseconds(1000);
    digitalWrite(3, LOW); //Motor 1
    digitalWrite(4, LOW); //Motor 2
    digitalWrite(5, LOW); //Motor 3
    digitalWrite(6, LOW); //Motor 4
    delayMicroseconds(tiempociclo - 1000);
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
  }
digitalWrite(A0, 0);
digitalWrite(A1, 0);
digitalWrite(A2, 0);
digitalWrite(A3, 0);


}
