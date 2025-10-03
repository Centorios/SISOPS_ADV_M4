
/*
Mira el tutorial en 
Programador Novato: https://www.programadornovato.com/esp32-con-servo-girar-servomotor/
Youtube: https://www.youtube.com/watch?v=5mlnl0CkNHg&list=PLCTD_CpMeEKTvjzabAvLGHakg-ql6t0q6&index=9

En este tutorial trabajaremos en el Esp32 con servo y haremos 
que un servomotor gire 180 grados y después regrese a 0 grados.
*/

//Agregamos las libreriras del servo
#include <ESP32Servo.h>
//Instanciamos nuestro servo
Servo servo;
int pinServo=33;
void setup()
{
  //Inicializamos la posicion del servo
  servo.attach(pinServo, 500, 2500);
}
//Inicializamos la posicion (en grados) del servo
int pos = 0;
void loop() {
  //Ciclo que posicionara el servo desde 0 hsta 180 grados
  for (pos = 0; pos <= 180; pos += 1) {
    //Movemos el servo a los grados que le entreguemos
    servo.write(pos);
    //Esperamos 15 milisegundos
    delay(15);
  }
  //Ciclo que posicionara el servo desde 180 hsta 0 grados
  for (pos = 180; pos >= 0; pos -= 1) {
    //Movemos el servo a los grados que le entreguemos
    servo.write(pos);
    //Esperamos 15 milisegundos
    delay(15);
  }
  
  /*
  //Movemos el servo a 0 grados
  servo.write(0);
  //Esperamos 1.5 segundos
  delay(1500);
  //Movemos el servo a 180 grados
  servo.write(180);
  //Esperamos 1.5 segundos
  delay(1500);
  */
}
