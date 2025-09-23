const int triggerPin = 5;
const int echoPin = 4;

//defino la velocidad del sonido
#define SOUND_SPEED 0.034

long duration;
float distanceCm;
float distanceInch;

void setup() {
  Serial.begin(115200); // configuro el monitor serial
  pinMode(triggerPin, OUTPUT); // seteo el pin 5 del esp32 como salida
  pinMode(echoPin, INPUT); // seteo el pin4 del esp32 como entrada
}


long readUltrasonicSensor()
{
 
  //Desactivo el trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
  //Activo el Trigger por 10 microsegundos
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  
   //Desactivo el trigger
  digitalWrite(triggerPin, LOW);
  
  //Leo el pin de Echo, y retorno el tiempo en
  //microsegundos en donde se encuentra el objeto
  return pulseIn(echoPin, HIGH);
}



void loop() 
{

  duration =readUltrasonicSensor();

  //calcula la distancia del objeto en cm
  distanceCm = duration * SOUND_SPEED/2;
  
  //imprime la distnacia
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  
  delay(100);
}