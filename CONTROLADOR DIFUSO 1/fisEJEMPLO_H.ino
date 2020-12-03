//LIBRERIAS
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial Serie3(10,11);
SoftwareSerial Serie4(2,3);
#include <Wire.h>   // incluye libreria para interfaz I2C
#include <RTClib.h>   // incluye libreria para el manejo del modulo RTC
RTC_DS3231 rtc;
#include <OneWire.h> 
#include <DallasTemperature.h>
OneWire ourWire(8); //Se establece el pin declarado como bus para la comunicación OneWire
DallasTemperature sensors(&ourWire); //Se instancia la librería DallasTemperature
int thresholdValue = 800;
int rainPin = A2;
////////////////////
#include <DHT.h>
#include <DHT_U.h>
int SENSOR = 13;
int TEMPERATURA;
int HUMEDAD; 
DHT dht (SENSOR, DHT22);
//DECLARACION VARIABLES 
float dato1;
int dato2;
int dato3; 
int b=0;
int x=0;
int y=0;
int z=0;
int a=0;
int i=1000;
int j=1000;
int k=1000;
int l=0;
int cont=0;
int cont1=0;
int cont2=0;
int cont3=0;
int cont4=0;
int cont5=0;
int cont6=0;
int contador=0;//contador para sistema temporizdo
int vector1 [25];
float vector2 [5];
float vector3 [5];
int suma1;
int suma2;
int suma3;
int suma4;
int suma5;

#include "fis_header.h"

// Numero de entradas del controlador 
const int fis_gcI = 3;
// Numero de salidas del controlador
const int fis_gcO = 1;
// Numero de reglas del controlador
const int fis_gcR = 60;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

void setup()
{
  //inicialización de sensores 
  Wire.begin(); 
  sensors.begin(); //Se inician los sensores
  dht.begin ();
  //inicializa comunicacion serie a 9600 bps
  Serial.begin(9600);
  Serie3.begin(9600);
  Serie4.begin(9600);
  //inicializa pantalla lcd 
  lcd.init();
  lcd.backlight();
  lcd.clear();
 //Declaración de pines como entrada y salida
    pinMode(0 , INPUT);
    pinMode(8 , INPUT);
    pinMode(2 , OUTPUT);
    pinMode(3 , OUTPUT);
    pinMode(4 , OUTPUT);
    pinMode(5 , OUTPUT);
    pinMode(6 , OUTPUT);
    pinMode(3 , OUTPUT);
    pinMode(13 , OUTPUT);
}
void loop()
{
  //condición para dejar en alto los pines 
  if (b==0)
 {
  digitalWrite(2,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  b=1;
 }
//condicion para verificar si existen datos en XBee recepetor
if (Serie3.available()>0)
{
  delay(200);
  //ciclo para llenar el vector1
  while(cont3<=20)
  {
  dato1 = Serie3.read();
  delay(10);
  vector1 [cont1] = dato1;
  Serial.println(vector1[cont1]);
  cont1=cont1+1;
  cont3++;
  }
  //condición para agrupar los dígitos en numeros enteros de cada variable
  if (cont1==21)
  {
  suma1=(vector1[0]*1000 + vector1[1]*100 + vector1[2]*10 +vector1[3]);  //HUMEDAD SUSTRATO 
  suma2=(vector1[4]*1000 + vector1[5]*100 + vector1[6]*10 +vector1[7]);  //TEMPERATURA DEL SUSTARTO
  suma3=(vector1[8]*1000 + vector1[9]*100 + vector1[10]*10 +vector1[11]); //TEMPERATURA AMBIENTE
  suma4=(vector1[12]*1000 + vector1[13]*100 + vector1[14]*10 +vector1[15]); //PH
  suma5=(vector1[16]*1000 + vector1[17]*100 + vector1[18]*10 +vector1[19]); //CONDUCTIVIDAD ELECTRICA
  cont1=0;
  cont3=0; 
  //Imprimir en la pantalla LCD
  lcd.setCursor(0,0); //columnas/filas
  lcd.print("HUMSUSTRATO");
  lcd.setCursor(12,0);
  lcd.print(suma1);
  lcd.setCursor(0,1);
  lcd.print("TEMPSUSTRATO");
  lcd.setCursor(13,1);
  lcd.print(suma2);
  delay (1000);
  lcd.clear();
  delay (1000);
  lcd.setCursor(0,0);
  lcd.print("TEMPAMBIENTE");
  lcd.setCursor(14,0);
  lcd.print(suma3); 
  lcd.setCursor(0,1);
  lcd.print("PH");
  lcd.setCursor(5,1);
  lcd.print(suma4);
  delay (1000);
  lcd.clear();
  delay(1000);
  lcd.setCursor(0,0);
  lcd.print("CONDUCTIVDAD");
  lcd.setCursor(13,0);
  lcd.print(suma5);
  delay(1000);
  lcd.clear();
  delay (1000); 
  //condición que permite verificar si activa el controlador 
  if(vector1[20]==1)
  {
    cont=2;
    vector1[20]=0;
  }
  }
}
//Condición para ejecutar las reglas del controlador difuso 
if(cont==2)
{
if  ( suma2>24 || suma1>600 )  
    {
     g_fisInput[0] = suma1;
     g_fisInput[1] = suma2;
     g_fisInput[2] = suma3;
     Serial.println(g_fisInput[2]);
     g_fisOutput[0] = 0;

    fis_evaluate();
    analogWrite(3 , g_fisOutput[0]);
    x=1;
    y=1;
    cont=0;
    }
}

}
///////LOGICA DIFUSA////////////
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
int fis_gIMFCount[] = { 5, 3, 5 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 5 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { -300, 0, 300 };
FIS_TYPE fis_gMFI0Coeff2[] = { 0, 300, 600 };
FIS_TYPE fis_gMFI0Coeff3[] = { 350, 525, 700 };
FIS_TYPE fis_gMFI0Coeff4[] = { 600, 812, 1024 };
FIS_TYPE fis_gMFI0Coeff5[] = { 800, 912, 1024 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5 };
FIS_TYPE fis_gMFI1Coeff1[] = { 14, 18, 22 };
FIS_TYPE fis_gMFI1Coeff2[] = { 16, 21, 26 };
FIS_TYPE fis_gMFI1Coeff3[] = { 20, 25.5, 31 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 0, 11 };
FIS_TYPE fis_gMFI2Coeff2[] = { 0, 8.5, 17 };
FIS_TYPE fis_gMFI2Coeff3[] = { 12, 16, 20 };
FIS_TYPE fis_gMFI2Coeff4[] = { 17, 22, 27 };
FIS_TYPE fis_gMFI2Coeff5[] = { 23, 27.5, 32 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3, fis_gMFI2Coeff4, fis_gMFI2Coeff5 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 0, 796 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 0, 0, 580 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0, 0, 0, 400 };
FIS_TYPE fis_gMFO0Coeff4[] = { 0, 0, 0, 240 };
FIS_TYPE fis_gMFO0Coeff5[] = { 0, 0, 0, 0 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4, fis_gMFO0Coeff5 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0 };
int fis_gMFI2[] = { 0, 0, 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};

// Output membership function set

int* fis_gMFO[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1, 1 };
int fis_gRI1[] = { 1, 2, 1 };
int fis_gRI2[] = { 1, 3, 1 };
int fis_gRI3[] = { 2, 1, 1 };
int fis_gRI4[] = { 2, 2, 1 };
int fis_gRI5[] = { 2, 3, 1 };
int fis_gRI6[] = { 3, 1, 1 };
int fis_gRI7[] = { 3, 2, 1 };
int fis_gRI8[] = { 3, 3, 1 };
int fis_gRI9[] = { 4, 1, 1 };
int fis_gRI10[] = { 4, 2, 1 };
int fis_gRI11[] = { 4, 3, 1 };
int fis_gRI12[] = { 5, 1, 1 };
int fis_gRI13[] = { 5, 2, 1 };
int fis_gRI14[] = { 5, 3, 1 };
int fis_gRI15[] = { 1, 1, 2 };
int fis_gRI16[] = { 1, 2, 2 };
int fis_gRI17[] = { 1, 3, 2 };
int fis_gRI18[] = { 2, 1, 2 };
int fis_gRI19[] = { 2, 2, 2 };
int fis_gRI20[] = { 2, 3, 2 };
int fis_gRI21[] = { 3, 1, 2 };
int fis_gRI22[] = { 3, 2, 2 };
int fis_gRI23[] = { 3, 3, 2 };
int fis_gRI24[] = { 4, 1, 2 };
int fis_gRI25[] = { 4, 2, 2 };
int fis_gRI26[] = { 4, 3, 2 };
int fis_gRI27[] = { 5, 1, 2 };
int fis_gRI28[] = { 5, 2, 2 };
int fis_gRI29[] = { 5, 3, 2 };
int fis_gRI30[] = { 1, 1, 3 };
int fis_gRI31[] = { 1, 2, 3 };
int fis_gRI32[] = { 1, 3, 3 };
int fis_gRI33[] = { 2, 1, 3 };
int fis_gRI34[] = { 2, 2, 3 };
int fis_gRI35[] = { 2, 3, 3 };
int fis_gRI36[] = { 3, 1, 3 };
int fis_gRI37[] = { 3, 2, 3 };
int fis_gRI38[] = { 3, 3, 3 };
int fis_gRI39[] = { 4, 1, 3 };
int fis_gRI40[] = { 4, 2, 3 };
int fis_gRI41[] = { 4, 3, 3 };
int fis_gRI42[] = { 5, 1, 3 };
int fis_gRI43[] = { 5, 2, 3 };
int fis_gRI44[] = { 5, 3, 3 };
int fis_gRI45[] = { 1, 1, 4 };
int fis_gRI46[] = { 1, 2, 4 };
int fis_gRI47[] = { 1, 3, 4 };
int fis_gRI48[] = { 2, 1, 4 };
int fis_gRI49[] = { 2, 2, 4 };
int fis_gRI50[] = { 2, 3, 4 };
int fis_gRI51[] = { 3, 1, 4 };
int fis_gRI52[] = { 3, 2, 4 };
int fis_gRI53[] = { 3, 3, 4 };
int fis_gRI54[] = { 4, 1, 4 };
int fis_gRI55[] = { 4, 2, 4 };
int fis_gRI56[] = { 4, 3, 4 };
int fis_gRI57[] = { 5, 1, 4 };
int fis_gRI58[] = { 5, 2, 4 };
int fis_gRI59[] = { 5, 3, 4 };
int fis_gRI60[] = { 1, 1, 5 };
int fis_gRI61[] = { 1, 2, 5 };
int fis_gRI62[] = { 1, 3, 5 };
int fis_gRI63[] = { 2, 1, 5 };
int fis_gRI64[] = { 2, 2, 5 };
int fis_gRI65[] = { 2, 3, 5 };
int fis_gRI66[] = { 3, 1, 5 };
int fis_gRI67[] = { 3, 2, 5 };
int fis_gRI68[] = { 3, 3, 5 };
int fis_gRI69[] = { 4, 1, 5 };
int fis_gRI70[] = { 4, 2, 5 };
int fis_gRI71[] = { 4, 3, 5 };
int fis_gRI72[] = { 5, 1, 5 };
int fis_gRI73[] = { 5, 2, 5 };
int fis_gRI74[] = { 5, 3, 5 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19, fis_gRI20, fis_gRI21, fis_gRI22, fis_gRI23, fis_gRI24, fis_gRI25, fis_gRI26, fis_gRI27, fis_gRI28, fis_gRI29, fis_gRI30, fis_gRI31, fis_gRI32, fis_gRI33, fis_gRI34, fis_gRI35, fis_gRI36, fis_gRI37, fis_gRI38, fis_gRI39, fis_gRI40, fis_gRI41, fis_gRI42, fis_gRI43, fis_gRI44, fis_gRI45, fis_gRI46, fis_gRI47, fis_gRI48, fis_gRI49, fis_gRI50, fis_gRI51, fis_gRI52, fis_gRI53, fis_gRI54, fis_gRI55, fis_gRI56, fis_gRI57, fis_gRI58, fis_gRI59, fis_gRI60, fis_gRI61, fis_gRI62, fis_gRI63, fis_gRI64, fis_gRI65, fis_gRI66, fis_gRI67, fis_gRI68, fis_gRI69, fis_gRI70, fis_gRI71, fis_gRI72, fis_gRI73, fis_gRI74 };

// Rule Outputs
int fis_gRO0[] = { 5 };
int fis_gRO1[] = { 5 };
int fis_gRO2[] = { 3 };
int fis_gRO3[] = { 5 };
int fis_gRO4[] = { 5 };
int fis_gRO5[] = { 4 };
int fis_gRO6[] = { 5 };
int fis_gRO7[] = { 5 };
int fis_gRO8[] = { 4 };
int fis_gRO9[] = { 3 };
int fis_gRO10[] = { 3 };
int fis_gRO11[] = { 2 };
int fis_gRO12[] = { 1 };
int fis_gRO13[] = { 2 };
int fis_gRO14[] = { 1 };
int fis_gRO15[] = { 5 };
int fis_gRO16[] = { 5 };
int fis_gRO17[] = { 4 };
int fis_gRO18[] = { 5 };
int fis_gRO19[] = { 5 };
int fis_gRO20[] = { 4 };
int fis_gRO21[] = { 4 };
int fis_gRO22[] = { 4 };
int fis_gRO23[] = { 3 };
int fis_gRO24[] = { 4 };
int fis_gRO25[] = { 3 };
int fis_gRO26[] = { 2 };
int fis_gRO27[] = { 2 };
int fis_gRO28[] = { 3 };
int fis_gRO29[] = { 1 };
int fis_gRO30[] = { 5 };
int fis_gRO31[] = { 5 };
int fis_gRO32[] = { 4 };
int fis_gRO33[] = { 4 };
int fis_gRO34[] = { 5 };
int fis_gRO35[] = { 5 };
int fis_gRO36[] = { 4 };
int fis_gRO37[] = { 4 };
int fis_gRO38[] = { 2 };
int fis_gRO39[] = { 2 };
int fis_gRO40[] = { 2 };
int fis_gRO41[] = { 1 };
int fis_gRO42[] = { 2 };
int fis_gRO43[] = { 1 };
int fis_gRO44[] = { 2 };
int fis_gRO45[] = { 5 };
int fis_gRO46[] = { 5 };
int fis_gRO47[] = { 4 };
int fis_gRO48[] = { 5 };
int fis_gRO49[] = { 5 };
int fis_gRO50[] = { 4 };
int fis_gRO51[] = { 5 };
int fis_gRO52[] = { 5 };
int fis_gRO53[] = { 4 };
int fis_gRO54[] = { 5 };
int fis_gRO55[] = { 5 };
int fis_gRO56[] = { 4 };
int fis_gRO57[] = { 5 };
int fis_gRO58[] = { 5 };
int fis_gRO59[] = { 4 };
int fis_gRO60[] = { 4 };
int fis_gRO61[] = { 4 };
int fis_gRO62[] = { 2 };
int fis_gRO63[] = { 4 };
int fis_gRO64[] = { 3 };
int fis_gRO65[] = { 2 };
int fis_gRO66[] = { 3 };
int fis_gRO67[] = { 4 };
int fis_gRO68[] = { 2 };
int fis_gRO69[] = { 2 };
int fis_gRO70[] = { 2 };
int fis_gRO71[] = { 2 };
int fis_gRO72[] = { 1 };
int fis_gRO73[] = { 2 };
int fis_gRO74[] = { 1 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19, fis_gRO20, fis_gRO21, fis_gRO22, fis_gRO23, fis_gRO24, fis_gRO25, fis_gRO26, fis_gRO27, fis_gRO28, fis_gRO29, fis_gRO30, fis_gRO31, fis_gRO32, fis_gRO33, fis_gRO34, fis_gRO35, fis_gRO36, fis_gRO37, fis_gRO38, fis_gRO39, fis_gRO40, fis_gRO41, fis_gRO42, fis_gRO43, fis_gRO44, fis_gRO45, fis_gRO46, fis_gRO47, fis_gRO48, fis_gRO49, fis_gRO50, fis_gRO51, fis_gRO52, fis_gRO53, fis_gRO54, fis_gRO55, fis_gRO56, fis_gRO57, fis_gRO58, fis_gRO59, fis_gRO60, fis_gRO61, fis_gRO62, fis_gRO63, fis_gRO64, fis_gRO65, fis_gRO66, fis_gRO67, fis_gRO68, fis_gRO69, fis_gRO70, fis_gRO71, fis_gRO72, fis_gRO73, fis_gRO74 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 1024, 35, 35 };

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
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, };
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
            Serial.println ("El VALOR DE LAS VARIABLES DE ENTRADA ES");
            Serial.print ("HUMEDAD SUSTRATO = ");
            Serial.println (suma1);
            Serial.print ("TEMPERATURA SUSTRATO = ");
            Serial.println (suma2);
            Serial.print ("TEMPERATURA AMBIENTE = ");
            Serial.println (suma3);
            Serial.print ("EL RESULTADO DEL TIEMPO DE RIEGO ES = ");
            Serial.println (g_fisOutput[o]);
            delay(1000);
             //CICLO PARA ACTIVAR ELECTOBOMBA Y ELECTROVALVULAS 
             while (z<=g_fisOutput[o]) 
            {
            digitalWrite(4,LOW);
            digitalWrite(5,LOW);
            digitalWrite(6,HIGH);
            Serial.println (z);
////////////////////////////////////////////////////////////////
if (Serie3.available()>0)
{
  delay(100);
  while(cont3<=20)
  {
  dato1 = Serie3.read();
  //delay(10);
  vector1 [cont1] = dato1;
  Serial.println(vector1[cont1]);
  cont1=cont1+1;
  cont3++;
  }
  if (cont1==21)
  {
  suma1=(vector1[0]*1000 + vector1[1]*100 + vector1[2]*10 +vector1[3]);  //TEMPERATURA SUSTRATO 
  suma2=(vector1[4]*1000 + vector1[5]*100 + vector1[6]*10 +vector1[7]);  //HUMEDAD DEL SUSTARTO
  suma3=(vector1[8]*1000 + vector1[9]*100 + vector1[10]*10 +vector1[11]); //TEMPERATURA AMBIENTE
  suma4=(vector1[12]*1000 + vector1[13]*100 + vector1[14]*10 +vector1[15]); //PH
  suma5=(vector1[16]*1000 + vector1[17]*100 + vector1[18]*10 +vector1[19]); //CONDUCTIVIDAD ELECTRICA
  cont1=0;
  cont3=0;
  
  lcd.setCursor(0,0); //columnas/filas
  lcd.print("HUMSUSTRATO");
  lcd.setCursor(12,0);
  lcd.print(suma1);
  lcd.setCursor(0,1);
  lcd.print("TEMPSUSTRATO");
  lcd.setCursor(13,1);
  lcd.print(suma2);
  delay (1000);
  lcd.clear();
  delay (1000);
  lcd.setCursor(0,0);
  lcd.print("TEMPAMBIENTE");
  lcd.setCursor(14,0);
  lcd.print(suma3); 
  lcd.setCursor(0,1);
  lcd.print("PH");
  lcd.setCursor(5,1);
  lcd.print(suma4);
  delay (1000);
  lcd.clear();
  delay(1000);
  lcd.setCursor(0,0);
  lcd.print("CONDUCTIVDAD");
  lcd.setCursor(13,0);
  lcd.print(suma5);
  delay(1000);
  lcd.clear();
  delay (1000);
  z=z+8; 
  Serial.println("CONTADOR");
  Serial.println(z);
  }
}           
if (z>=g_fisOutput[o])
 {
 digitalWrite(4,HIGH);
 digitalWrite(5,HIGH);
 digitalWrite(6,LOW);
 }
 }
 if (z>=(g_fisOutput[o]))
 {
 while(cont5<=300)
 {
if (Serie3.available()>0)
{
  while(cont4<=21)
  {
  dato1 = Serie3.read();
  //delay(10);
  vector1 [cont1] = dato1;
  Serial.println(vector1[cont1]);
  cont1=cont1+1;
  cont4++;
  }
  if (cont1==21)
  {
  suma1=(vector1[0]*1000 + vector1[1]*100 + vector1[2]*10 +vector1[3]);  //TEMPERATURA SUSTRATO 
  suma2=(vector1[4]*1000 + vector1[5]*100 + vector1[6]*10 +vector1[7]);  //HUMEDAD DEL SUSTARTO
  suma3=(vector1[8]*1000 + vector1[9]*100 + vector1[10]*10 +vector1[11]); //TEMPERATURA AMBIENTE
  suma4=(vector1[12]*1000 + vector1[13]*100 + vector1[14]*10 +vector1[15]); //TEMPERATURA AMBIENTE
  suma5=(vector1[16]*1000 + vector1[17]*100 + vector1[18]*10 +vector1[19]); //TEMPERATURA AMBIENTE
  cont1=0;
  cont4=0; 
  Serial.println("RESULTADO");
  Serial.println(suma1); 
  Serial.println(suma2); 
  Serial.println(suma3); 
  Serial.println(suma4); 
  Serial.println(suma5); 
  
  lcd.setCursor(0,0); //columnas/filas
  lcd.print("HUMSUSTRATO");
  lcd.setCursor(12,0);
  lcd.print(suma1);
  lcd.setCursor(0,1);
  lcd.print("TEMPSUSTRATO");
  lcd.setCursor(13,1);
  lcd.print(suma2);
  delay (1000);
  lcd.clear();
  delay (1000);
  lcd.setCursor(0,0);
  lcd.print("TEMPAMBIENTE");
  lcd.setCursor(14,0);
  lcd.print(suma3); 
  lcd.setCursor(0,1);
  lcd.print("PH");
  lcd.setCursor(5,1);
  lcd.print(suma4);
  delay (1000);
  lcd.clear();
  delay(1000);
  lcd.setCursor(0,0);
  lcd.print("CONDUCTIVDAD");
  lcd.setCursor(13,0);
  lcd.print(suma5);
  delay(1000);
  lcd.clear();
  delay (1000);
  cont5=cont5+8;
  Serial.println("CONTADOR");
  Serial.println(cont5);
  }
}
////////////////////////////////////////////////////////////////
            }
            Serial.write('A');
            delay(200);
            } 
z=0;
cont=0;
cont1=0;
cont3=0;
cont5=0; 
cont6=1;
}     
}
        }
    }
