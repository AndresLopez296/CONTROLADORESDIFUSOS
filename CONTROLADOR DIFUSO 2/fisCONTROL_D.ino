//Librerías 
#include <SoftwareSerial.h> 
SoftwareSerial Serie2(10,11); //CREAMOS UN NUEVO PUERTO SERIE (RX, TX)
SoftwareSerial Serie3(6,7); //RX TX
#include <RTClib.h>   // incluye libreria para el manejo del modulo RTC
RTC_DS3231 rtc;     // crea objeto del tipo RTC_DS3231
#include <Wire.h>   // incluye libreria para interfaz I2C
#include <OneWire.h>             // Librería para sensor de Temperatura DS18B20
#include <DallasTemperature.h>   // Librería para sensor de Temperatura DS18B20
OneWire ourWire(8); //Se establece el pin declarado como bus para la comunicación OneWire
DallasTemperature sensors(&ourWire); //Se instancia la librería DallasTemperature
#include <DHT.h>   // Librería para sensor de Temperatura Ambiente
#include <DHT_U.h>
int SENSOR = 12;
int TEMPERATURA;
int HUMEDAD; 
DHT dht (SENSOR, DHT22);
//Declaración de Variables 
int i = 0; 
int x = 0;
int y = 0;
int z=0;
int vector1 [5]; // HUMEDAD SUSTARTO
int vector2 [5]; // TEMPERATURA SUSTRATO
int vector3 [5]; // TEMPERATURA AMBIENTE
int vector4 [5]; // PH 
int vector5 [6]; // CONDUCTIVIDAD ELECTRICA 
int cont=0;
float Po;            // Variable valor de PH
int conductividad;   // Variable valor de Conductividad 
int tempsustrato;    // Variable Temperatura Sustrato  
int tempambiente;    // Variable Temperatura Ambiente 
int humsustrato;     // Variable Humedad Sustarto
// CONDUCTIVIDAD ELECTRICA
const int analogInPin = A1; 
const int analogOutPin = 9; 
int sensorValue = 0; 
int outputValue = 0; 
// PH
int ph_pin = A0; 
/////////////////////
#include "fis_header.h"
// Numero de entradas del controlador 
const int fis_gcI = 2;
// Numero de salidas del controlador
const int fis_gcO = 1;
// Numero de reglas difusas
const int fis_gcR = 20;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

void setup()
{
  //Se inician los sensores
  Wire.begin();
  sensors.begin(); 
  dht.begin ();
  // Se declaran los pines como Entrada/Salida
  pinMode(2 , OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(0 , INPUT);
  pinMode(1 , INPUT);
  pinMode(12, INPUT);
  pinMode(3 , OUTPUT);
  pinMode(4 , OUTPUT);
  pinMode(5 , OUTPUT);
  pinMode(8, OUTPUT);
  // inicializa comunicacion serie a 9600 bps
  Serial.begin(9600); 
  Serie2.begin(9600);
  Serie3.begin(9600); 
  ///////////////////////////////////////////////////////////////////7
  if (! rtc.begin()) {       // si falla la inicializacion del modulo
  Serial.println("Modulo RTC no encontrado !");  // muestra mensaje de error
  while (1);         // bucle infinito que detiene ejecucion del programa
  }
  //rtc.adjust(DateTime(__DATE__, __TIME__));  // funcion que permite establecer fecha y horario
  //Al momento de la compilacion. Comentar esta linea y volver a subir para el normal funcionamiento
  }

void loop()
{
  if (i==0)// Condición para poner los pines con señal en alto
  {
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    digitalWrite(4,HIGH);
    digitalWrite(5,HIGH);
    i=1;
  }
DateTime fecha = rtc.now();   // funcion que devuelve fecha y hora
//Serial.print(fecha.hour());      // funcion que obtiene la hora de la fecha completa
//Serial.print(":");       // caracter dos puntos como separador
//Serial.print(fecha.minute());      // funcion que obtiene los minutos de la fecha completa
//Serial.print(":");       // caracter dos puntos como separador
//Serial.println(fecha.second());    // funcion que obtiene los segundos de la fecha completa
// Condición para verificar si existen datos desde el otro controlador
   if (Serie3.available()>0)
 {
 char dato= Serie3.read();//Guardamos en la variable dato el valor leido
 if(dato=='A')
 {
 Serial.print("RESULTADOOOOOOOOOOOOOO");
 // Condición para verificar si conductividad está alta 
 if (((analogRead(1)* 5.00 / 1024)*1000) > 100)
 {
  int measure = analogRead(ph_pin);
  double voltage = 5 / 1024.0 * measure; //classic digital to voltage conversion
  Po = 7 + ((2.5 - voltage) / 0.18);
    g_fisInput[0] = Po;
    g_fisInput[1] = ((analogRead(1)* 5.00 / 1024)* 1000);
    g_fisOutput[0] = 0;
    fis_evaluate();
    analogWrite(2 , g_fisOutput[0]);
    y=0;
    x=0;
 }
 }
 }
 // Condición para activar el controlador difuso 
 if (fecha.hour()== 7 && fecha.minute()== 0 || fecha.hour()== 7 && fecha.minute()== 45 || fecha.hour()== 8 && fecha.minute()== 50 || fecha.hour()== 9 && fecha.minute()== 10 ||
 fecha.hour()== 10 && fecha.minute()== 1 || fecha.hour()== 10 && fecha.minute()== 45 || fecha.hour()== 11 && fecha.minute()== 31 || fecha.hour()== 12 && fecha.minute()== 15 ||
 fecha.hour()== 13 && fecha.minute()== 1 || fecha.hour()== 13 && fecha.minute()== 45 || fecha.hour()== 14 && fecha.minute()== 31 || fecha.hour()== 15 && fecha.minute()== 15 ||
 fecha.hour()== 16 && fecha.minute()== 40 || fecha.hour()== 16 && fecha.minute()== 45 || fecha.hour()== 17 && fecha.minute()== 5 || fecha.hour()== 20 && fecha.minute()== 21)
 {
  //Almacenar valores de sensores en cada vector para envío 
  sensors.requestTemperatures(); //Prepara el sensor para la lectura
  vector1 [0]= analogRead(A3);
  ////////////////////////////////////////////////////
  vector2 [0] = sensors.getTempCByIndex(0);
  ////////////////////////////////////////////////////
  vector3 [0] = dht.readTemperature ();
  ////////////////////////////////////////////////////
  int measure = analogRead(ph_pin);
  double voltage = 5 / 1024.0 * measure; 
  Po = 7 + ((2.5 - voltage) / 0.18);
  vector4 [0] = Po;
  ////////////////////////////////////////////////////
  sensorValue = analogRead(analogInPin);
  outputValue = map(sensorValue, 0, 1023, 0, 5000);
  analogWrite(analogOutPin, outputValue);
  conductividad=(analogRead(1)* 5.00 / 1024)*1000;
  vector5 [0] = conductividad;
  // Proceso para obtener cada digito
  if (vector1[0]>=1000)
  {
  vector1[1]= vector1[0]/1000;
  vector1[2]= (vector1[0]-(vector1[1]*1000))/100;
  vector1[3]= (vector1[0]-(vector1[1]*1000))/10;
  vector1[4]= (vector1[0]-(vector1[1]*1000))-(vector1[3]*10);
  /////////////////////////////////////////////////////////////////////////
  vector2[1]= vector2[0]/1000;
  vector2[2]= (vector2[0]-(vector2[1]*1000))/100;
  vector2[3]= (vector2[0]-(vector2[1]*1000))/10;
  vector2[4]= (vector2[0]-(vector2[1]*1000))-(vector2[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector3[1]= vector3[0]/1000;
  vector3[2]= (vector3[0]-(vector3[1]*1000))/100;
  vector3[3]= (vector3[0]-(vector3[1]*1000))/10;
  vector3[4]= (vector3[0]-(vector3[1]*1000))-(vector3[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector4[1]= vector4[0]/1000;
  vector4[2]= (vector4[0]-(vector4[1]*1000))/100;
  vector4[3]= (vector4[0]-(vector4[1]*1000))/10;
  vector4[4]= (vector4[0]-(vector4[1]*1000))-(vector4[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector5[1]= vector5[0]/1000;
  vector5[2]= (vector5[0]-(vector5[1]*1000))/100;
  vector5[3]= (vector5[0]-(vector5[1]*1000))/10;
  vector5[4]= (vector5[0]-(vector5[1]*1000))-(vector5[3]*10);
  vector5[5] = 1;
  //Proceso para enviar cada digito por medio de XBee emisor
  Serie2.write(byte (vector1[1]));
  delay (10);
  Serie2.write(byte (vector1[2]));
  delay(10);
  Serie2.write(byte (vector1[3]));
  delay (10);
  Serie2.write(byte (vector1[4]));
  delay (10);
  //////////////////////////////////
  Serie2.write(byte (vector2[1]));
  delay (10);
  Serie2.write(byte (vector2[2]));
  delay(10);
  Serie2.write(byte (vector2[3]));
  delay (10);
  Serie2.write(byte (vector2[4]));
  delay (10);
  //////////////////////////////////
  Serie2.write(byte (vector3[1]));
  delay (10);
  Serie2.write(byte (vector3[2]));
  delay(10);
  Serie2.write(byte (vector3[3]));
  delay (10);
  Serie2.write(byte (vector3[4]));
  delay (10);
  //////////////////////////////////
  Serie2.write(byte (vector4[1]));
  delay (10);
  Serie2.write(byte (vector4[2]));
  delay(10);
  Serie2.write(byte (vector4[3]));
  delay (10);
  Serie2.write(byte (vector4[4]));
  delay (10);
  /////////////////////////////////
  Serie2.write(byte (vector5[1]));
  delay (10);
  Serie2.write(byte (vector5[2]));
  delay(10);
  Serie2.write(byte (vector5[3]));
  delay (10);
  Serie2.write(byte (vector5[4]));
  delay (10);
  Serie2.write(byte (vector5[5]));
  delay (10);
  /////////////////////////////////
  }
  // Proceso para obtener cada digito si el número es menor 1000
  if (vector1[0]<1000)
  {
  vector1[1]= vector1[0]/1000;
  vector1[2]= (vector1[0]-(vector1[1]*1000))/100;
  vector1[3]= (vector1[0]-(vector1[2]*100))/10;
  vector1[4]= (vector1[0]-(vector1[2]*100))-(vector1[3]*10);
   //////////////////////////////////////////////////////////
  vector2[1]= vector2[0]/1000;
  vector2[2]= (vector2[0]-(vector2[1]*1000))/100;
  vector2[3]= (vector2[0]-(vector2[1]*1000))/10;
  vector2[4]= (vector2[0]-(vector2[1]*1000))-(vector2[3]*10);
  ///////////////////////////////////////////////////////////
  vector3[1]= vector3[0]/1000;
  vector3[2]= (vector3[0]-(vector3[1]*1000))/100;
  vector3[3]= (vector3[0]-(vector3[1]*1000))/10;
  vector3[4]= (vector3[0]-(vector3[1]*1000))-(vector3[3]*10);
  ///////////////////////////////////////////////////////////
  vector4[1]= vector4[0]/1000;
  vector4[2]= (vector4[0]-(vector4[1]*1000))/100;
  vector4[3]= (vector4[0]-(vector4[1]*1000))/10;
  vector4[4]= (vector4[0]-(vector4[1]*1000))-(vector4[3]*10);
  ///////////////////////////////////////////////////////////
  vector5[1]= vector5[0]/1000;
  vector5[2]= (vector5[0]-(vector5[1]*1000))/100;
  vector5[3]= (vector5[0]-(vector5[1]*1000))/10;
  vector5[4]= (vector5[0]-(vector5[1]*1000))-(vector5[3]*10);
  vector5[5] = 1;
  //Envío de datos 
  Serie2.write(byte (vector1[1]));
  delay (10);
  Serie2.write(byte (vector1[2]));
  delay(10);
  Serie2.write(byte (vector1[3]));
  delay (10);
  Serie2.write(byte (vector1[4]));
  delay (10);
  ////////////////////////////////
  Serie2.write(byte (vector2[1]));
  delay (10);
  Serie2.write(byte (vector2[2]));
  delay(10);
  Serie2.write(byte (vector2[3]));
  delay (10);
  Serie2.write(byte (vector2[4]));
  delay (10);
  ////////////////////////////////
  Serie2.write(byte (vector3[1]));
  delay (10);
  Serie2.write(byte (vector3[2]));
  delay(10);
  Serie2.write(byte (vector3[3]));
  delay (10);
  Serie2.write(byte (vector3[4]));
  delay (10);
  ////////////////////////////////
  Serie2.write(byte (vector4[1]));
  delay (10);
  Serie2.write(byte (vector4[2]));
  delay(10);
  Serie2.write(byte (vector4[3]));
  delay (10);
  Serie2.write(byte (vector4[4]));
  delay (10);
  ////////////////////////////////
  Serie2.write(byte (vector5[1]));
  delay (10);
  Serie2.write(byte (vector5[2]));
  delay(10);
  Serie2.write(byte (vector5[3]));
  delay (10);
  Serie2.write(byte (vector5[4]));
  delay (10);
  Serie2.write(byte (vector5[5]));
  delay (10);
  ////////////////////////////////
  }
 }
// Proceso para enviar los datos mientras el otro controlador está en operacion 
//PH
  int measure = analogRead(ph_pin);
  double voltage = 5 / 1024.0 * measure; //classic digital to voltage conversion
  Po = 7 + ((2.5 - voltage) / 0.18);
  vector4 [0]=Po; 
//CONDUCTIVIDAD ELECTRICA 
  sensorValue = analogRead(analogInPin);
 outputValue = map(sensorValue, 0, 1023, 0, 5000);
 analogWrite(analogOutPin, outputValue);
 conductividad=(analogRead(1)* 5.00 / 1024)*1000;
 vector5 [0]=conductividad; 
//TEMPERATURA SUSTRATO
  sensors.requestTemperatures();
  tempsustrato = sensors.getTempCByIndex(0);
  vector2 [0]=tempsustrato;
//HUMEDAD SUSTRATO 
  humsustrato = analogRead(A3);
  vector1 [0]=humsustrato;
//TEMPERATURA AMBIENTE
  tempambiente = dht.readTemperature ();
  vector3 [0]=tempambiente;
  delay(1000);

  if (vector1[0]>=1000)
  {
  vector1[1]= vector1[0]/1000;
  vector1[2]= (vector1[0]-(vector1[1]*1000))/100;
  vector1[3]= (vector1[0]-(vector1[1]*1000))/10;
  vector1[4]= (vector1[0]-(vector1[1]*1000))-(vector1[3]*10);
  /////////////////////////////////////////////////////////////////////////
  vector2[1]= vector2[0]/1000;
  vector2[2]= (vector2[0]-(vector2[1]*1000))/100;
  vector2[3]= (vector2[0]-(vector2[1]*1000))/10;
  vector2[4]= (vector2[0]-(vector2[1]*1000))-(vector2[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector3[1]= vector3[0]/1000;
  vector3[2]= (vector3[0]-(vector3[1]*1000))/100;
  vector3[3]= (vector3[0]-(vector3[1]*1000))/10;
  vector3[4]= (vector3[0]-(vector3[1]*1000))-(vector3[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector4[1]= vector4[0]/1000;
  vector4[2]= (vector4[0]-(vector4[1]*1000))/100;
  vector4[3]= (vector4[0]-(vector4[1]*1000))/10;
  vector4[4]= (vector4[0]-(vector4[1]*1000))-(vector4[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector5[1]= vector5[0]/1000;
  vector5[2]= (vector5[0]-(vector5[1]*1000))/100;
  vector5[3]= (vector5[0]-(vector5[1]*1000))/10;
  vector5[4]= (vector5[0]-(vector5[1]*1000))-(vector5[3]*10);
  vector5[5]= 0;
  ////////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector1[1]));
  delay (10);
  Serie2.write(byte (vector1[2]));
  delay(10);
  Serie2.write(byte (vector1[3]));
  delay (10);
  Serie2.write(byte (vector1[4]));
  delay (10);
  ///////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector2[1]));
  delay (10);
  Serie2.write(byte (vector2[2]));
  delay(10);
  Serie2.write(byte (vector2[3]));
  delay (10);
  Serie2.write(byte (vector2[4]));
  delay (10);
  ///////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector3[1]));
  delay (10);
  Serie2.write(byte (vector3[2]));
  delay(10);
  Serie2.write(byte (vector3[3]));
  delay (10);
  Serie2.write(byte (vector3[4]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector4[1]));
  delay (10);
  Serie2.write(byte (vector4[2]));
  delay(10);
  Serie2.write(byte (vector4[3]));
  delay (10);
  Serie2.write(byte (vector4[4]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector5[1]));
  delay (10);
  Serie2.write(byte (vector5[2]));
  delay(10);
  Serie2.write(byte (vector5[3]));
  delay (10);
  Serie2.write(byte (vector5[4]));
  delay (10);
  Serie2.write(byte (vector5[5]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  }
  if (vector1[0]<1000)
  {
  vector1[1]= vector1[0]/1000;
   Serial.println(vector1[1]);
  vector1[2]= (vector1[0]-(vector1[1]*1000))/100;
   Serial.println(vector1[2]);
  vector1[3]= (vector1[0]-(vector1[2]*100))/10;
   Serial.println(vector1[3]);
  vector1[4]= (vector1[0]-(vector1[2]*100))-(vector1[3]*10);
   Serial.println(vector1[4]);
   /////////////////////////////////////////////////////////////////////////
  vector2[1]= vector2[0]/1000;
  Serial.println(vector2[1]);
  vector2[2]= (vector2[0]-(vector2[1]*1000))/100;
  Serial.println(vector2[2]);
  vector2[3]= (vector2[0]-(vector2[2]*100))/10;
  Serial.println(vector2[3]);
  vector2[4]= (vector2[0]-(vector2[2]*100))-(vector2[3]*10);
  Serial.println(vector2[4]);
  ////////////////////////////////////////////////////////////////////////
  vector3[1]= vector3[0]/1000;
  Serial.println(vector3[1]);
  vector3[2]= (vector3[0]-(vector3[1]*1000))/100;
  Serial.println(vector3[2]);
  vector3[3]= (vector3[0]-(vector3[2]*100))/10;
  Serial.println(vector3[3]);
  vector3[4]= (vector3[0]-(vector3[2]*100))-(vector3[3]*10);
  Serial.println(vector3[4]);
  ////////////////////////////////////////////////////////////////////////
  vector4[1]= vector4[0]/1000;
  Serial.println(vector4[1]);
  vector4[2]= (vector4[0]-(vector4[1]*1000))/100;
  Serial.println(vector4[2]);
  vector4[3]= (vector4[0]-(vector4[2]*100))/10;
  Serial.println(vector4[3]);
  vector4[4]= (vector4[0]-(vector4[2]*100))-(vector4[3]*10);
  Serial.println(vector4[4]);
  ////////////////////////////////////////////////////////////////////////
  vector5[1]= vector5[0]/1000;
  Serial.println(vector5[1]);
  vector5[2]= (vector5[0]-(vector5[1]*1000))/100;
  Serial.println(vector5[2]);
  vector5[3]= (vector5[0]-(vector5[2]*100))/10;
  Serial.println(vector5[3]);
  vector5[4]= (vector5[0]-(vector5[2]*100))-(vector5[3]*10);
  Serial.println(vector5[4]);
  vector5[5]= 0;
  ////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector1[1]));
  delay (10);
  Serie2.write(byte (vector1[2]));
  delay(10);
  Serie2.write(byte (vector1[3]));
  delay (10);
  Serie2.write(byte (vector1[4]));
  delay (10);
  ///////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector2[1]));
  delay (10);
  Serie2.write(byte (vector2[2]));
  delay(10);
  Serie2.write(byte (vector2[3]));
  delay (10);
  Serie2.write(byte (vector2[4]));
  delay (10);
  ///////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector3[1]));
  delay (10);
  Serie2.write(byte (vector3[2]));
  delay(10);
  Serie2.write(byte (vector3[3]));
  delay (10);
  Serie2.write(byte (vector3[4]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector4[1]));
  delay (10);
  Serie2.write(byte (vector4[2]));
  delay(10);
  Serie2.write(byte (vector4[3]));
  delay (10);
  Serie2.write(byte (vector4[4]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector5[1]));
  delay (10);
  Serie2.write(byte (vector5[2]));
  delay(10);
  Serie2.write(byte (vector5[3]));
  delay (10);
  Serie2.write(byte (vector5[4]));
  delay (10);
  Serie2.write(byte (vector5[5]));
  delay (10);
  /////////////////////////////////////////////////////////////////////
  }
 ////////////////////////////////////////////////////////////////////////////
 delay(10000);
}

///////////LOGICA DIFUSA/////////////////// 
//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b)
{
    return (a * b);
}

FIS_TYPE fis_probor(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b - (a * b));
}

FIS_TYPE fis_sum(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 5, 4 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 5 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0, 2, 4 };
FIS_TYPE fis_gMFI0Coeff2[] = { 2.4, 3.9, 5.5 };
FIS_TYPE fis_gMFI0Coeff3[] = { 5, 5.9, 6.8 };
FIS_TYPE fis_gMFI0Coeff4[] = { 6.5, 8.8, 11 };
FIS_TYPE fis_gMFI0Coeff5[] = { 10, 12, 14 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 200, 400 };
FIS_TYPE fis_gMFI1Coeff2[] = { 200, 450, 700 };
FIS_TYPE fis_gMFI1Coeff3[] = { 600, 800, 1000 };
FIS_TYPE fis_gMFI1Coeff4[] = { 800, 900, 1000 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3, fis_gMFI1Coeff4 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 0 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 0, 120 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0, 0, 180 };
FIS_TYPE fis_gMFO0Coeff4[] = { 0, 0, 280 };
FIS_TYPE fis_gMFO0Coeff5[] = { 0, 0, 330 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4, fis_gMFO0Coeff5 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1};

// Output membership function set

int* fis_gMFO[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1 };
int fis_gRI1[] = { 1, 2 };
int fis_gRI2[] = { 1, 4 };
int fis_gRI3[] = { 1, 3 };
int fis_gRI4[] = { 2, 1 };
int fis_gRI5[] = { 2, 2 };
int fis_gRI6[] = { 2, 4 };
int fis_gRI7[] = { 2, 3 };
int fis_gRI8[] = { 3, 1 };
int fis_gRI9[] = { 3, 2 };
int fis_gRI10[] = { 3, 3 };
int fis_gRI11[] = { 3, 4 };
int fis_gRI12[] = { 4, 1 };
int fis_gRI13[] = { 4, 2 };
int fis_gRI14[] = { 4, 3 };
int fis_gRI15[] = { 4, 4 };
int fis_gRI16[] = { 5, 1 };
int fis_gRI17[] = { 5, 2 };
int fis_gRI18[] = { 5, 3 };
int fis_gRI19[] = { 5, 4 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19 };

// Rule Outputs
int fis_gRO0[] = { 1 };
int fis_gRO1[] = { 1 };
int fis_gRO2[] = { 5 };
int fis_gRO3[] = { 4 };
int fis_gRO4[] = { 1 };
int fis_gRO5[] = { 1 };
int fis_gRO6[] = { 5 };
int fis_gRO7[] = { 4 };
int fis_gRO8[] = { 1 };
int fis_gRO9[] = { 1 };
int fis_gRO10[] = { 4 };
int fis_gRO11[] = { 5 };
int fis_gRO12[] = { 1 };
int fis_gRO13[] = { 1 };
int fis_gRO14[] = { 4 };
int fis_gRO15[] = { 5 };
int fis_gRO16[] = { 1 };
int fis_gRO17[] = { 1 };
int fis_gRO18[] = { 4 };
int fis_gRO19[] = { 5 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 14, 1000 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 1 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
// None for Sugeno

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = 1;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = 0;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            FIS_TYPE sWI = 0.0;
            for (j = 0; j < fis_gOMFCount[o]; ++j)
            {
                fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][fis_gcI];
                for (i = 0; i < fis_gcI; ++i)
                {
                    fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i];
                }
            }

            for (r = 0; r < fis_gcR; ++r)
            {
                index = fis_gRO[r][o] - 1;
                sWI += fuzzyFires[r] * fuzzyOutput[o][index];
            }
if (x==y)
{        
            g_fisOutput[o] = sWI / sW;
            Serial.println ("EL RESULTADO ES");
            Serial.println (g_fisOutput[o]);
             while (z<g_fisOutput[o])
            {
            digitalWrite(3,LOW);
            digitalWrite(4,LOW);
            Serial.println (z);
            delay (1000);
            z++;
            if (z>g_fisOutput[o])
            {
            digitalWrite(3,HIGH);
            }
               }
         if (z>(g_fisOutput[o]))
            {
              for (int cont=0;cont<300;cont++)
              {
               // Proceso para enviar los datos mientras el otro controlador está en operacion 
//PH
  int measure = analogRead(ph_pin);
  double voltage = 5 / 1024.0 * measure; //classic digital to voltage conversion
  Po = 7 + ((2.5 - voltage) / 0.18);
  vector4 [0]=Po; 
//CONDUCTIVIDAD ELECTRICA 
  sensorValue = analogRead(analogInPin);
 outputValue = map(sensorValue, 0, 1023, 0, 5000);
 analogWrite(analogOutPin, outputValue);
 conductividad=(analogRead(1)* 5.00 / 1024)*1000;
 vector5 [0]=conductividad; 
//TEMPERATURA SUSTRATO
  sensors.requestTemperatures();
  tempsustrato = sensors.getTempCByIndex(0);
  vector2 [0]=tempsustrato;
//HUMEDAD SUSTRATO 
  humsustrato = analogRead(A3);
  vector1 [0]=humsustrato;
//TEMPERATURA AMBIENTE
  tempambiente = dht.readTemperature ();
  vector3 [0]=tempambiente;
  delay(1000);

  if (vector1[0]>=1000)
  {
  vector1[1]= vector1[0]/1000;
  vector1[2]= (vector1[0]-(vector1[1]*1000))/100;
  vector1[3]= (vector1[0]-(vector1[1]*1000))/10;
  vector1[4]= (vector1[0]-(vector1[1]*1000))-(vector1[3]*10);
  /////////////////////////////////////////////////////////////////////////
  vector2[1]= vector2[0]/1000;
  vector2[2]= (vector2[0]-(vector2[1]*1000))/100;
  vector2[3]= (vector2[0]-(vector2[1]*1000))/10;
  vector2[4]= (vector2[0]-(vector2[1]*1000))-(vector2[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector3[1]= vector3[0]/1000;
  vector3[2]= (vector3[0]-(vector3[1]*1000))/100;
  vector3[3]= (vector3[0]-(vector3[1]*1000))/10;
  vector3[4]= (vector3[0]-(vector3[1]*1000))-(vector3[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector4[1]= vector4[0]/1000;
  vector4[2]= (vector4[0]-(vector4[1]*1000))/100;
  vector4[3]= (vector4[0]-(vector4[1]*1000))/10;
  vector4[4]= (vector4[0]-(vector4[1]*1000))-(vector4[3]*10);
  ////////////////////////////////////////////////////////////////////////
  vector5[1]= vector5[0]/1000;
  vector5[2]= (vector5[0]-(vector5[1]*1000))/100;
  vector5[3]= (vector5[0]-(vector5[1]*1000))/10;
  vector5[4]= (vector5[0]-(vector5[1]*1000))-(vector5[3]*10);
  vector5[5]= 0;
  ////////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector1[1]));
  delay (10);
  Serie2.write(byte (vector1[2]));
  delay(10);
  Serie2.write(byte (vector1[3]));
  delay (10);
  Serie2.write(byte (vector1[4]));
  delay (10);
  ///////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector2[1]));
  delay (10);
  Serie2.write(byte (vector2[2]));
  delay(10);
  Serie2.write(byte (vector2[3]));
  delay (10);
  Serie2.write(byte (vector2[4]));
  delay (10);
  ///////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector3[1]));
  delay (10);
  Serie2.write(byte (vector3[2]));
  delay(10);
  Serie2.write(byte (vector3[3]));
  delay (10);
  Serie2.write(byte (vector3[4]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector4[1]));
  delay (10);
  Serie2.write(byte (vector4[2]));
  delay(10);
  Serie2.write(byte (vector4[3]));
  delay (10);
  Serie2.write(byte (vector4[4]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector5[1]));
  delay (10);
  Serie2.write(byte (vector5[2]));
  delay(10);
  Serie2.write(byte (vector5[3]));
  delay (10);
  Serie2.write(byte (vector5[4]));
  delay (10);
  Serie2.write(byte (vector5[5]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  }
  if (vector1[0]<1000)
  {
  vector1[1]= vector1[0]/1000;
   Serial.println(vector1[1]);
  vector1[2]= (vector1[0]-(vector1[1]*1000))/100;
   Serial.println(vector1[2]);
  vector1[3]= (vector1[0]-(vector1[2]*100))/10;
   Serial.println(vector1[3]);
  vector1[4]= (vector1[0]-(vector1[2]*100))-(vector1[3]*10);
   Serial.println(vector1[4]);
   /////////////////////////////////////////////////////////////////////////
  vector2[1]= vector2[0]/1000;
  Serial.println(vector2[1]);
  vector2[2]= (vector2[0]-(vector2[1]*1000))/100;
  Serial.println(vector2[2]);
  vector2[3]= (vector2[0]-(vector2[2]*100))/10;
  Serial.println(vector2[3]);
  vector2[4]= (vector2[0]-(vector2[2]*100))-(vector2[3]*10);
  Serial.println(vector2[4]);
  ////////////////////////////////////////////////////////////////////////
  vector3[1]= vector3[0]/1000;
  Serial.println(vector3[1]);
  vector3[2]= (vector3[0]-(vector3[1]*1000))/100;
  Serial.println(vector3[2]);
  vector3[3]= (vector3[0]-(vector3[2]*100))/10;
  Serial.println(vector3[3]);
  vector3[4]= (vector3[0]-(vector3[2]*100))-(vector3[3]*10);
  Serial.println(vector3[4]);
  ////////////////////////////////////////////////////////////////////////
  vector4[1]= vector4[0]/1000;
  Serial.println(vector4[1]);
  vector4[2]= (vector4[0]-(vector4[1]*1000))/100;
  Serial.println(vector4[2]);
  vector4[3]= (vector4[0]-(vector4[2]*100))/10;
  Serial.println(vector4[3]);
  vector4[4]= (vector4[0]-(vector4[2]*100))-(vector4[3]*10);
  Serial.println(vector4[4]);
  ////////////////////////////////////////////////////////////////////////
  vector5[1]= vector5[0]/1000;
  Serial.println(vector5[1]);
  vector5[2]= (vector5[0]-(vector5[1]*1000))/100;
  Serial.println(vector5[2]);
  vector5[3]= (vector5[0]-(vector5[2]*100))/10;
  Serial.println(vector5[3]);
  vector5[4]= (vector5[0]-(vector5[2]*100))-(vector5[3]*10);
  Serial.println(vector5[4]);
  vector5[5]= 0;
  ////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector1[1]));
  delay (10);
  Serie2.write(byte (vector1[2]));
  delay(10);
  Serie2.write(byte (vector1[3]));
  delay (10);
  Serie2.write(byte (vector1[4]));
  delay (10);
  ///////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector2[1]));
  delay (10);
  Serie2.write(byte (vector2[2]));
  delay(10);
  Serie2.write(byte (vector2[3]));
  delay (10);
  Serie2.write(byte (vector2[4]));
  delay (10);
  ///////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector3[1]));
  delay (10);
  Serie2.write(byte (vector3[2]));
  delay(10);
  Serie2.write(byte (vector3[3]));
  delay (10);
  Serie2.write(byte (vector3[4]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector4[1]));
  delay (10);
  Serie2.write(byte (vector4[2]));
  delay(10);
  Serie2.write(byte (vector4[3]));
  delay (10);
  Serie2.write(byte (vector4[4]));
  delay (10);
  //////////////////////////////////////////////////////////////////////
  Serie2.write(byte (vector5[1]));
  delay (10);
  Serie2.write(byte (vector5[2]));
  delay(10);
  Serie2.write(byte (vector5[3]));
  delay (10);
  Serie2.write(byte (vector5[4]));
  delay (10);
  Serie2.write(byte (vector5[5]));
  delay (10);
  /////////////////////////////////////////////////////////////////////
  }
  delay(800);
  Serial.print(cont);
  if (cont>=290 && cont<=300)
      {
      digitalWrite(4,HIGH);
      }
      if (cont==301) 
      {
      digitalWrite(5,LOW);
      }
      delay (1000);
      }
      digitalWrite(5,HIGH);
      }
      z=0;
}
        }
    }
}
