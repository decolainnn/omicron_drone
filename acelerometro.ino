
bool calibracionOk = false;


//////////////////////////////////INICIAR ACELEROMETRO/////////////////////////////

void iniciacion_acelerometro() {

  //Habilitamos configuración del giroscopio/acelerometro

  Wire.beginTransmission(direccion_acelerometro);
  Wire.write(0x6B);                           // Registro 6B para acceder a la configuración y poder cambiarla (This register allows the user to configure the power mode and clock source)
  Wire.write(0x00);                           // Internal 8MHz oscillator
  Wire.endTransmission();

  //Configuramos giroscopio
  Wire.beginTransmission(direccion_acelerometro);
  Wire.write(0x1B);                           //1B configuración del giroscopio
  Wire.write(0x08);                           //Girscopio a 500dps
  Wire.endTransmission();

  //Configuramos acelerometro
  Wire.beginTransmission(direccion_acelerometro);
  Wire.write(0x1C);                          //1C configuración del acelerometro
  Wire.write(0x10);                          // 0x10 = +/- 8g
  Wire.endTransmission();

  //Configuramos el filtro paso bajo tanto para el acelerometro como para el  giroscopio
  Wire.beginTransmission(direccion_acelerometro);
  Wire.write(0x1A);                        // 1A para el Low-pass filter
  Wire.write(0x06);                        // 0x03 = 42 Hz
  Wire.endTransmission();

  //Giroscopio Self-Test
//  Wire.beginTransmission(direccion_acelerometro);
//  Wire.write(0x1B); //nos colocamos en el registro 27 (configuraicon del giroscopio) y pedimos el siguiente byte   (This register is used to trigger gyroscope self-test)
//  Wire.endTransmission();
//  Wire.requestFrom(direccion_acelerometro, 1); // le pedimos un byte
//  while (Wire.available() < 1); //devuelve el numero de bytes recuperados, esperamos hasta que sea 1
//  if (Wire.read() != 0x08) { // leemos el byte, si es distinto de 8 damos el error, si no contunuamos.
//    digitalWrite(A0, HIGH);
//    digitalWrite(A1, HIGH);
//    digitalWrite(A2, HIGH);
//    digitalWrite(A3, HIGH);
//    // si detectamos un error encendemos todos los leds y dejamos el programa en bucle
//    while (1);
//  }

  //Acelerómetro Self-Test
}



//////////////////////////////////CALIBRAR ACELEROMETRO/////////////////////////////


void calibracion_acelerometro()
{

  for (int calibracion = 1; calibracion <= 2000; calibracion++) //tomamos 2000 muestras
  {
    leer_acelerometro();
    ax_cal += ax; //sumamos todos los valores obtenidos
    ay_cal += ay;
    az_cal += az;
    gx_cal += gx;
    gy_cal += gy;
    gz_cal += gz;
    delayMicroseconds(1000); //esperamos 1ms entre toma y toma
  }

  //dividimos entre el numero de muestras para obtener el offset (media) de las medidas

  ax_cal = ax_cal / 2000;
  ay_cal = ay_cal / 2000;
  az_cal = az_cal / 2000;
  gx_cal = gx_cal / 2000;
  gy_cal = gy_cal / 2000;
  gz_cal = gz_cal / 2000;

  calibracionOk = true;

  for (int p=0; p<3; p++){
  digitalWrite(A0, HIGH); //Hacemos parpadear los leds para saber que la calibración ha sido completada
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  delay(200);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(200);
  }

}


//////////////////////////////////LEER ACELEROMETRO/////////////////////////////

void leer_acelerometro()
{
  digitalWrite(3, LOW);  //señal de los motores siempre en low por seguridad
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);

  Wire.beginTransmission(direccion_acelerometro);     //Empezar comunicacion con MPU6050
  Wire.write(0x3B); // registro 59, que se corresponde con los datos de salida del acelerometro, luego la temperatura y luego el giroscopio
  Wire.endTransmission();
  Wire.requestFrom(direccion_acelerometro, 14); // requerimos 14 bytes correspondientes a los datos del acelerometro, temperatura y giroscopio

  while (Wire.available() < 14); //esperamos hasta recibirlos
  tiempo_ejecucion_acelerometro = (micros() - tiempo_bucle_acelerometro) / 1000; //anotamos el tiempo que hemos tardado en conseguir los datos para utilizarlo al procesar los datos y dividimos para pasar a milisegundos
  tiempo_bucle_acelerometro = micros(); //anotamos el tiempo para el siguiente bucle


  ax = Wire.read() << 8 | Wire.read(); //obtenemos los valores en RAW (sin procesar) del osciloscopio, que son variables globales
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  temperatura = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}



//////////////////////////////////PROCESAR ACELEROMETRO/////////////////////////////

void procesar_acelerometro() //procesamos los valores en RAW  para obtener los grados de inclinación y la velocidad angular del drone
{
  //si ya lo hemos calibrado anteriormente entonces le restamos los valores del offset automaticamente
  if (calibracionOk == true)
  {
    ax -= ax_cal;
    ay -= ay_cal;
    az -= az_cal;
    az += 4096;// le sumamos la gravedad al eje z  para ajustarlo sobre 0
    gx -= gx_cal;
    gy -= gy_cal;
    gz -= gz_cal;
  }
  else
  {
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    // si detectamos un error encendemos todos los leds y dejamos el programa en bucle
    while (1);
  }

  //pasamos a calcular la velocidad de rotacion a partir de los datos del giroscopio
  velocidad_rotacion_x = gx / 65.5; //ya que 65,5º en raw son 1º/s (ver datasheet)
  velocidad_rotacion_y = gy / 65.5;
  velocidad_rotacion_z = gz / 65.5;


  //pasamos a calcular el angulo de inclinación
  angulo_inclinacion_x += velocidad_rotacion_x * tiempo_ejecucion_acelerometro / 1000; //el angulo sera el angulo anterior mas la velocidad de rotacion por el tiempo que ha estado rotando a esa velocidad dividido entre 1000 para pasarlo a segundos
  angulo_inclinacion_y += velocidad_rotacion_y * tiempo_ejecucion_acelerometro / 1000;

  // esta medida induce un error de deriva  o drift, vamos  a corregirla con la velocidad de giro del eje z explciacion aqui: http://robologs.net/2014/10/15/tutorial-de-arduino-y-mpu-6050/
  angulo_inclinacion_x += angulo_inclinacion_y * sin(velocidad_rotacion_z * tiempo_ejecucion_acelerometro  * 0.00001745);      //
  angulo_inclinacion_y -= angulo_inclinacion_x * sin(velocidad_rotacion_z * tiempo_ejecucion_acelerometro  * 0.00001745);


  //estimaremos el angulo de inclinacion a partir de los datos del acelerometro
  modulo_aceleracion = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2)); //calculamos el modulo de la aceleracion  (aceleracion total)
  angulo_inclinacion_x_acelerometro = asin((float)ay / modulo_aceleracion) * 57.2958;     // 57.2958 = Conversion de radianes a grados
  angulo_inclinacion_y_acelerometro = asin((float)ax / modulo_aceleracion) * -57.2958;

  //esta estimacion no hace uso del tiempo aunque no es tan precisa, por lo que le daremos un pequeño peso (2%) al valor total del calculo por si obtenemos un error en alguna de las medidad del giroscopio (se acumula) para que se vaya disipando
 // filtro complementario
  angulo_inclinacion_x = angulo_inclinacion_x * 0.98 + angulo_inclinacion_x_acelerometro * 0.02;
  angulo_inclinacion_y = angulo_inclinacion_y * 0.98 + angulo_inclinacion_y_acelerometro * 0.02;




}
