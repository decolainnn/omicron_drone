////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////                    TFG- Proyecto final                                                               ////////
/////                     Diego Suárez González                                                             ////////
/////                     Drone con Arduino                                                                 ////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
//====================================== TIEMPOS =======================================//

//bucle general
long tiempo_bucle;

//bucle acelerometro
long tiempo_bucle_acelerometro, tiempo_ejecucion_acelerometro;

//motores
long tiempo_bucle_general_variadores;

//pruebas 
long tiempo1, tiempo2, tiempo3;



//====================================== ACELEROMETRO =======================================//
#define direccion_acelerometro 0x68
int gx, gy, gz;
float ax, ay, az, temperatura;
float ax_cal, ay_cal, az_cal, gx_cal, gy_cal, gz_cal; //variables de coma flotante para la calibracion del acelerometro y giroscopio
float velocidad_rotacion_x, velocidad_rotacion_y, velocidad_rotacion_z;
float angulo_inclinacion_x, angulo_inclinacion_y, modulo_aceleracion, angulo_inclinacion_x_acelerometro, angulo_inclinacion_y_acelerometro;



//====================================== MANDO =======================================//

long int pulso_potencia_consigna, pulso_potencia_anterior, velocidad_rotacion_yaw_consigna, velocidad_rotacion_pitch_consigna, velocidad_rotacion_roll_consigna;


//====================================== MOTORES =======================================//
float senal1, senal2, senal3, senal4; 

//====================================== PID =======================================//

float error_x_vel, termino_integral_x_vel, termino_derivativo_x_vel, velocidad_rotacion_x_anterior, PID_x_vel;

float error_y_vel, termino_integral_y_vel, termino_derivativo_y_vel, velocidad_rotacion_y_anterior, PID_y_vel;

float error_z_vel, termino_integral_z_vel, termino_derivativo_z_vel, velocidad_rotacion_z_anterior, PID_z_vel;

float error_x_ang, termino_integral_x_ang, termino_derivativo_x_ang, angulo_inclinacion_x_anterior , PID_x_ang;

float error_y_ang, termino_integral_y_ang, termino_derivativo_y_ang, angulo_inclinacion_y_anterior , PID_y_ang;



//====================================== LIBRERIAS =======================================//
#include <Wire.h>
#include <EnableInterrupt.h>
#include <I2Cdev.h>






//====================================== VARIABLES GLOBALES =======================================//

// VARIABLES IMPORTANTES //
int ModoVuelo = 1;    //0 = estable, 1 = Acro

// //


//===================================== OTROS ====================================================== //
#define tiempociclo 6000 //tiempo máximo de ciclo en microsegundos




void setup() {
  
  iniciacion_general();
  iniciacion_acelerometro();
  calibracion_acelerometro();
  calibrar_motores();
  tiempo_bucle = micros();
}

void loop() {

  generar_senal_mando();
  if(ModoVuelo == 1) 
  {
    PID_angulo_inclinacion();
  }
//  PID_velocidad_rotacion();
  calculo_pulsos();

  while(micros()- tiempo_bucle < tiempociclo);          //esperamos hasta finalizar el tiempo de bucle                                                                                                                                  
  tiempo_bucle = micros(); //comienzo del bucle 
  generador_pulsos(); //generamos las señales para los motores

// como el generador de pulsos puede tardar entre 1000 y 2000 ms esperamos un tiempo prudencial (2200) para leer el acelerometro siempre en el mismo momento
  while (micros() - tiempo_bucle < 2200);
  
  leer_acelerometro(); // Leer MPU6050

  procesar_acelerometro();
 

Serial.begin(115200);
Serial.print(senal1);
Serial.print("\t");
Serial.print(senal2);
Serial.print("\t");
Serial.print(senal3);
Serial.print("\t");
Serial.print(senal4);
Serial.print("\t");
Serial.println(tiempo2-tiempo1);
//
//  Serial.print(PID_x_vel);
//  Serial.print("\t");
//  Serial.print(PID_y_vel);
//  Serial.print("\t");
//  Serial.println(PID_z_vel);
//
//  Serial.print(PID_x_ang);
//  Serial.print("\t");
//  Serial.println(PID_y_ang);



//Serial.print(velocidad_rotacion_x);
//Serial.print("\t");
//Serial.print(velocidad_rotacion_y);
//Serial.print("\t");
//Serial.print(velocidad_rotacion_z);
//Serial.print("\t");
//Serial.print(angulo_inclinacion_x);
//Serial.print("\t");
//Serial.println(angulo_inclinacion_y);  

//Serial.print(ax/4096);
//Serial.print("\t");
//Serial.print(ay/4096);
//Serial.print("\t");
//Serial.print(az/4096);
//Serial.print("\t");
//Serial.print(gx/65.5);
//Serial.print("\t");
//Serial.print(gy/65.5);
//Serial.print("\t");
//Serial.println(gz/65.5);



}
