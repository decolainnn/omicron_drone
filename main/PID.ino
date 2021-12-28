//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////                       PID                                 ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////// Parametros de control PID para todos los lazos de control ///////////////////////////

// PID de velocidad de rotacion eje x
float Kp_x_vel = 2, Ki_x_vel = 0.07, Kd_x_vel = 12;
float integral_x_vel_min = -400;
float integral_x_vel_max = 400;
float PID_x_vel_min = -400;
float PID_x_vel_max =  400;

// PID de velocidad de rotacion eje y
float Kp_y_vel = 2, Ki_y_vel = 0.07, Kd_y_vel = 12;
float integral_y_vel_min = -400;
float integral_y_vel_max = 400;
float PID_y_vel_min = -400;
float PID_y_vel_max =  400;

// PID de velocidad de rotacion eje z
float Kp_z_vel = 1.5, Ki_z_vel = 0.05, Kd_z_vel = 0;
float integral_z_vel_min = -400;
float integral_z_vel_max = 400;
float PID_z_vel_min = -400;
float PID_z_vel_max =  400;



// PID de angulo de inclinacion eje x
float Kp_x_ang = 2.2, Ki_x_ang = 0.06, Kd_x_ang = 15;
float integral_x_ang_min = -400;
float integral_x_ang_max = 400;
float PID_x_ang_min = -400;
float PID_x_ang_max =  400;

// PID de angulo de inclinacion eje y
float Kp_y_ang = 2.2, Ki_y_ang = 0.06, Kd_y_ang = 15;
float integral_y_ang_min = -400;
float integral_y_ang_max = 400;
float PID_y_ang_min = -400;
float PID_y_ang_max =  400;


// Lazo PID de angulo de inclinacion

void PID_angulo_inclinacion() {
tiempo1=micros();

tiempo3= tiempo1-tiempo2;
  //////////////////////////////////////////////////  EJE X ////////////////////////////////////////////////////////
  //calculamos el error
  error_x_ang = velocidad_rotacion_pitch_consigna  - angulo_inclinacion_x;
  //calculo parte integral
  termino_integral_x_ang += (tiempo3 * error_x_ang);
  termino_integral_x_ang = constrain(termino_integral_x_ang, integral_x_ang_min, integral_x_ang_max); //limitamos parte integral

  //calculo parte derivativa
  termino_derivativo_x_ang = (angulo_inclinacion_x - angulo_inclinacion_x_anterior)/tiempo3;
  angulo_inclinacion_x_anterior = angulo_inclinacion_x;

  // Calculo parte proporcional y total
  PID_x_ang = Kp_x_ang * error_x_ang + Ki_x_ang*termino_integral_x_ang + Kd_x_ang*termino_derivativo_x_ang;
  PID_x_ang = constrain(PID_x_ang, PID_x_ang_min, PID_x_ang_max); //limitamos salida total por si ha habido algun error


  //////////////////////////////////////////////////  EJE Y ////////////////////////////////////////////////////////
  //calculamos el error
  error_y_ang = velocidad_rotacion_roll_consigna  - angulo_inclinacion_y;
  //calculo parte integral
  termino_integral_y_ang += (tiempo3 * error_y_ang);
  termino_integral_y_ang = constrain(termino_integral_y_ang, integral_y_ang_min, integral_y_ang_max); //limitamos parte integral

  //calculo parte derivativa
  termino_derivativo_y_ang = (angulo_inclinacion_y - angulo_inclinacion_y_anterior)/tiempo3;
  angulo_inclinacion_y_anterior = angulo_inclinacion_y;

  // Calculo parte proporcional y total
  PID_y_ang = Kp_y_ang * error_y_ang + Ki_y_ang*termino_integral_y_ang + Kd_y_ang*termino_derivativo_y_ang;
  PID_y_ang = constrain(PID_y_ang, PID_y_ang_min, PID_y_ang_max); //limitamos salida total por si ha habido algun error


tiempo2=tiempo1;

}








// Lazo PID de velocidad de rotacion


void PID_velocidad_rotacion () {


//if(ModoVuelo == 1) 
//  {
//velocidad_rotacion_pitch_consigna = PID_x_ang;
//velocidad_rotacion_roll_consigna = PID_y_ang;
//  }
  //////////////////////////////////////////////////  EJE X ////////////////////////////////////////////////////////
  //calculamos el error
  error_x_vel = velocidad_rotacion_pitch_consigna - velocidad_rotacion_x;
  //calculo parte integral
  termino_integral_x_vel +=  (Ki_x_vel * error_x_vel);
  termino_integral_x_vel = constrain(termino_integral_x_vel, integral_x_vel_min, integral_x_vel_max); //limitamos parte integral

  //calculo parte derivativa
  termino_derivativo_x_vel = Kd_x_vel * (velocidad_rotacion_x - velocidad_rotacion_x_anterior);
  velocidad_rotacion_x_anterior = velocidad_rotacion_x;

  // Calculo parte proporcional y total
  PID_x_vel = Kp_x_vel * error_x_vel + termino_integral_x_vel - termino_derivativo_x_vel;
  PID_x_vel = constrain(PID_x_vel, PID_x_vel_min, PID_x_vel_max); //limitamos salida total por si ha habido algun error

  //la salida es en grados por segundo


  //////////////////////////////////////////////////  EJE Y ////////////////////////////////////////////////////////
  //calculamos el error
  error_y_vel = velocidad_rotacion_roll_consigna - velocidad_rotacion_y;
  //calculo parte integral
  termino_integral_y_vel += (Ki_y_vel * error_y_vel);
  termino_integral_y_vel = constrain(termino_integral_y_vel, integral_y_vel_min, integral_y_vel_max);

  //calculo parte derivativa
  termino_derivativo_y_vel = Kd_y_vel * (velocidad_rotacion_y - velocidad_rotacion_y_anterior);
  velocidad_rotacion_y_anterior = velocidad_rotacion_y;

  // Calculo parte proporcional y total
  PID_y_vel = Kp_y_vel * error_y_vel + termino_integral_y_vel - termino_derivativo_y_vel;
  PID_y_vel = constrain(PID_y_vel, PID_y_vel_min, PID_y_vel_max);


  //////////////////////////////////////////////////  EJE Z ////////////////////////////////////////////////////////
  //calculamos el error
  error_z_vel = velocidad_rotacion_yaw_consigna - velocidad_rotacion_z;
  //calculo parte integral
  termino_integral_z_vel += (Ki_z_vel * error_z_vel);
  termino_integral_z_vel = constrain(termino_integral_z_vel, integral_z_vel_min, integral_z_vel_max);

  //calculo parte derivativa
  termino_derivativo_z_vel = Kd_z_vel * (velocidad_rotacion_z - velocidad_rotacion_z_anterior);
  velocidad_rotacion_z_anterior = velocidad_rotacion_z;

  // Calculo parte proporcional y total
  PID_z_vel = Kp_z_vel * error_z_vel + termino_integral_z_vel - termino_derivativo_z_vel;
  PID_z_vel = constrain(PID_z_vel, PID_z_vel_min, PID_z_vel_max);


}
