#include <SoftwareSerial.h>           //libreria para la comunicación serial secundaria
#include <EEPROM.h>                   //libreria para poder acceder a la memoria EEPROM del arduino


#define KEY_left_shift 225
#define KEY_alt 226                   //se definen las teclas a pulsar en valores decimales obtenidos 
#define KEY_LEFT_CTRL 1               //de las tablas de USB Human Interface Devices table ussage
#define KEY_F13 104                   //|
#define KEY_F14 105                   //|
#define KEY_F15 106                   //|
#define KEY_F16 107                   //|
#define KEY_F17 108                   //|
#define KEY_F18 109                   //|
#define KEY_F19 110                   //|
#define KEY_F20 111                   //|
#define KEY_F21 112                   //|
#define KEY_F22 113                   //|
#define KEY_F23 114                   //|
#define KEY_F24 115                   //|
#define KEY_2 31                      //|
#define KEY_3 32                      //|
#define KEY_4 33                      //|
#define KEY_5 34                      //|
#define KEY_6 35                      //|
#define KEY_0 39                      //|  

#define ledOK 13
#define pulsaA 10                     //se definen los botones fisicos y el la salida de led
#define pulsaB 11
#define pulsaC 12

unsigned long last_change_A = 0;     //variables para almacenar la ultima vez que fue presionado un botón
unsigned long last_change_B = 0;
unsigned long last_change_C = 0;

int correcto_abierto = 0;            //banderas indicar cuando la contraseña fue correcta,  
int corriendo_ahora = 0;             //cuando la alarma está desactivada
int no_push = 0;                     //y para el bloqueo de pulsadores

unsigned long last_ledchange = 0;    //variables para el funcionamiento y control del led
int led_counter = 0;
int led_counter1 = 0;
int led_st = 0;

 const int analogInPin1 = A0;        //variables para el control de los 6 optoswitch
 int sensorValue = 0;     
 int  An1_activo = 0; 
 const int analogInPin2 = A1;
 int  An2_activo = 0; 
 const int analogInPin3 = A2;
 int  An3_activo = 0; 
 const int analogInPin4 = A3;
 int  An4_activo = 0; 
 const int analogInPin5 = A4;
 int  An5_activo = 0; 
  const int analogInPin6 = A5;
 int  An6_activo = 0; 

unsigned long timesnapshot = 0;     //variable que se utilizará para almacenar el tiempo transcurrido
                                    //en milisegundos y comparar durante todo el programa

uint8_t buf[8] = {0};               //Keyboard timing buffer.

int clave[]={2,2,0,2};              //variables para el sistema de contraseña físico
int sec[]={100,100,100,100};
int estado = 0;
int intentosRestantes = 3;

int state = 1;                      //estado base para referencia de los optoswitch.


//variables módulo SIM800L
SoftwareSerial mySerial(3, 2);      //puertos Tx & Rx del SIM800L se conectan a pin #3 & #2 de Arduino

char receivedChar;
boolean newData = false;            //bandera para identificar cuando hay o no un mensaje nuevo a procesar
const byte numChars = 62;

char value[4];
char  eepvalue[4];
int address[4] = {0, 1, 2, 3};      //arreglo para almacenar los datos a almacenar en la EEPROM

String mensaje;                     //variable para almacenar el mensaje a enviar desde el módulo
String imprimnumero;                //variable para almacenar el número al cual se responderá

char guard[64];                     //arreglo en el cual se almacenan todos los carácteres del sms
int aumento;                        //variable para llevar el conteo de cada cáracter ingresado

//variables para control del buzzer
int buzzer = 9;
unsigned long last_buzzchange = 0;
int buzz_counter = 0;
int buzz_st = 0;

boolean co_ab = true;
boolean smstgl = false;


//presencia y proximidad
//#include <NewPing.h>                //libreria para el funcionamiento del sensor ultrasónico
/*Aqui se configuran los pines donde debemos conectar el sensor ultra*/
#define TRIGGER_PIN  5
#define ECHO_PIN     6
#define MAX_DISTANCE 300

/*Crear el objeto de la clase NewPing*/
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned long lastultra = 0;

//movimineto y proximidad
const int PIRPin= 8;                //entrada a la que se conectará el sensor de presencia
const int irPin= 4;                 //entrada a la cual se conectará el optoswitch digital(puerta)
const int alarmPin = 7;             //salida a la bocina para alarma sonora
boolean puertaflag = false;         //bandera para llevar el control de la apertura de puerta
unsigned long lastshot = 0;         //variable para control de tiempo de sensor de presencia
int counterxd = 0;                  //contador de pulsos del sensor de presencia
boolean alarma = false;
boolean panicflag = false;          //distintas banderas de control
boolean alarman2flag = false;
boolean panicbtn = false;
boolean smsalarm = false;
int puertacount = 0;
boolean bipflag = false;            //variables para patrones del buzzer
unsigned long lastbip = 0;
unsigned long last25chk = 0;



void setup()
{
  Serial.begin(9600); //Baud rate para el teclado.

//inicio de comunicación serial entre el módulo SIM800L y Arduino
  mySerial.begin(9600);

  pinMode(ledOK, OUTPUT);
  pinMode(pulsaA,INPUT);            //configuración de las entradas para los pulsadores
  pinMode(pulsaB,INPUT);
  pinMode(pulsaC,INPUT);

  pinMode(buzzer, OUTPUT);
  pinMode(PIRPin, INPUT);           //configuración de las salidas y entradas de los sensores
  pinMode(alarmPin, OUTPUT);        //y actuadores
  digitalWrite(alarmPin, HIGH);

  delay(1000);                      //tiempo de espera para configuración
  modorecibir();                    //llamar a la función que configura el modo de recibir SMS
  }

void loop()
{
    labelRST:
   timesnapshot = millis();         //almacenar el tiempo transcurrido desde el encendido de la tarjeta
                                    //en la variable timesnapshot
                                    
   while(timesnapshot <= 0000001000UL){ //reset de variables de control de tiempo cada 49 dias y un 
    last_change_A = 0;                  //segundo para asegurar funcionamiento continuo
    last_change_B = 0;
    last_change_C = 0;
    last_ledchange = 0;
    last_buzzchange = 0;
    goto labelRST;
   }
if (correcto_abierto == false){         //mientras la alarma se encuentre activa llama a la función
    presencia();                        //de control de sensores de apertura y pesencia
    smstgl = false;
}else{smstgl = true;}

    smsscan();                          //funciones principales del sistema que se mantendrán
    smscheck();                         //ejecutando todo el tiempo en loop
    alarmcheck();                       //|
    sensorcheck();                      //|
    alarman2();                         //|
    alarman3();                         //|
}

//función que configura el módulo GSM para recibir y enviar mensajes
  void modorecibir()
  {
  mySerial.println("AT");               //estableciendo la conexión con Aduino
  delay(150); 
  mySerial.println("AT+CMGF=1");        //configurando el modo de mensajes cortos SMS
  delay(150); 
  mySerial.println("AT+CNMI=1,2,0,0,0");//se configura la el formato en que se recibirán los mensajes
  delay(150); 
  mySerial.write((byte)0x1A);
  delay(50);
  }

//función de comprobación y almacenamiento de nuevos mensajes
void smsscan() {
   static boolean recvInProgress = false;
    char startMarker = 'T';             //configuración de cáracter inicial para comenza a almacenar
    char endMarker = '<';               //configuración de carácter final para terminar de alamcenar
    char rc;                            //variable usada para almacenamiento temporal de carácteres
  
    while (mySerial.available() > 0 && newData == false) {
          rc = mySerial.read();

 if (recvInProgress == true) {          //comienza el almacenamiento de carácteres del SMS
      if(rc != endMarker){              
        receivedChar = rc;
        guard[aumento] = receivedChar;  //se guardan los carácteres en el arreglo llamado "guard"
        aumento ++;
                if (aumento >= 52) {    //si el mensaje SMS contiene mas de 6 carácteres lo descarta
                guard[aumento] = '\0';  //terminación de string
                recvInProgress = false; //termina lel almacenamiento de carácteres
                aumento = 0;
                }
    }
        if(aumento != 0){               //se mantiene guardando carácteres mientras no llegue
        if(rc == endMarker){            //el cáracter de fin de recepción
          guard[aumento] = '\0';        //terminación de string
           recvInProgress = false;
        newData = true;
                aumento = 0;
        }
        }
    }//
    else if(rc == startMarker) {        //comprueba que el carácter de inicio de recepción
            recvInProgress = true;      //ha llegado y activa el almacenamiento
        }
        }
           
}

//función que compara el SMS almacenado con la contraseña y funciones
void smscheck() {
    if (newData == true) {

            eepvalue[0] = EEPROM.read(address[0]); //comprobación de contraseña actual
            eepvalue[1] = EEPROM.read(address[1]); 
            eepvalue[2] = EEPROM.read(address[2]); 
            eepvalue[3] = EEPROM.read(address[3]); 

//almacenamiento del numero remitente para envio de respuestas
     char    imparray[] = {guard[4], guard[5], guard[6], guard[7], guard[8], guard[9], guard[10], guard[11], guard[12], guard[13], guard[14], guard[15],'\0'};
     String  impnum(imparray);
     imprimnumero = impnum;
        
//comprobación de contraseña recivida con la contraseña correcta
        if((guard[48]==eepvalue[3])&&(guard[47]==eepvalue[2])&&(guard[46]==eepvalue[1])&&(guard[45]==eepvalue[0])){
        correctpass();
        if(smstgl == true){
          mensaje = "alarma activa";        //envia un mensaje avisando que la alarma se activó
          modoenviar();
        }

        else{
          mensaje = "alarma inactiva";      //envia un mensaje avisando que la alarma se desactivó
          modoenviar();
          varclean();
        }
        }
    //si la contraseña no es la correcta
         else{
           panicbtn = false;
           if (guard[49] == '^'){         //si el comando para cambio de contraseña fue recibido
           value[0] = guard[45];          //almacena los carácteres en el arreglo "value"
           value[1] = guard[46];
           value[2] = guard[47];
           value[3] = guard[48];
                                                  
            EEPROM.update(address[0], value[0]);  //cambio de contraseña en la memoria EEPROM          
            EEPROM.update(address[1], value[1]);  //por la nueva contraseña
            EEPROM.update(address[2], value[2]);
            EEPROM.update(address[3], value[3]);

            eepvalue[0] = EEPROM.read(address[0]); 
            eepvalue[1] = EEPROM.read(address[1]);//almacenamiento de los nuevos valores en el
            eepvalue[2] = EEPROM.read(address[2]);//arreglo "eepvalue"
            eepvalue[3] = EEPROM.read(address[3]); 

//almacenamiento del arreglo de cáracteres en un string llamado "impeep"
            char    eepvaluearray[] = { eepvalue[0], eepvalue[1], eepvalue[2], eepvalue[3],'\0'};
            String  impeep(eepvaluearray);

            mensaje = ("nueva contraseña: "+impeep); //SMS indicando el cambio de contraseña
            modoenviar();                            //función que envia el SMS
            
            guard[49] = '1';                     //guarda un valor distinto para que no
        }                                        //se vuelva a gatillar el cambio de contraseña
        else{
//si la contraseña es incorrecta envia un mensaje al numero remitente
          mensaje = ("contraseña incorrecta");
          modoenviar();
        }
          
        }
        newData = false;                        //resetea la variable newData
  
    }
}

                    //función para enviar un mensaje de texto
void modoenviar(){
            mySerial.println("AT+CMGF=1");     //configuración como SMS
            delay(150);
            mySerial.println("AT+CMGS=\"+"+imprimnumero+"\"");//configura el numero
            delay(150);
            mySerial.println(mensaje);         //mensaje a enviar
            delay(150);
            mySerial.write(26);
            delay(50);
            mySerial.println();
}

//función que se encarga de escanear la pulsación de los botónes físicos
  void alarmcheck(){
  if ((estado < 4)&&(no_push == 0)){           //revisa mientras no se hayan introducido 4 pulsaciones
  if(timesnapshot > (last_change_A + 500)){    //espera 500ms antes para evitar dobles pulsaciones
   if(digitalRead(pulsaA)){                    //si el botón A es pulsado...
    labelA: while (digitalRead(pulsaA)){       //sistema antirrebote de los pulsadores
      digitalWrite(ledOK, HIGH);               //|
      analogWrite(buzzer,128);                 //| sonido como feedback de pulsación
      goto  labelA;}                           //|
      digitalWrite(ledOK, LOW);
      digitalWrite(buzzer,LOW);
    sec[estado]=0;                            //almacena un 0 en el arreglo "sec" al ser pulsado
    estado++;                                 //el botón A
    last_change_A = timesnapshot;             //actualiza la variable al tiempo actual
  }
  }
  
  if(timesnapshot > (last_change_B + 500)){   //espera 500ms antes para evitar dobles pulsaciones
  if(digitalRead(pulsaB)){                    //si el botón B es pulsado...
    labelB: while (digitalRead(pulsaB)){      //sistema antirrebote de los pulsadores
      digitalWrite(ledOK, HIGH);              //|
      analogWrite(buzzer,64);                 //|
      goto  labelB;}                          //|
      digitalWrite(ledOK, LOW);
      digitalWrite(buzzer,LOW);
    sec[estado]=1;                            //almacena un 1 en el arreglo "sec" al ser pulsado
    estado++;                                 //el botón A
    last_change_B = timesnapshot;             //actualiza la variable al tiempo actual
  }
  }
   if(timesnapshot > (last_change_C + 500)){  //espera 500ms antes para evitar dobles pulsaciones
   if(digitalRead(pulsaC)){                   //si el botón C es pulsado...
    labelC: while (digitalRead(pulsaC)){      //sistema antirrebote de los pulsadores
      digitalWrite(ledOK, HIGH);              //|
      analogWrite(buzzer,96);                 //|
      goto  labelC;}                          //|
      digitalWrite(ledOK, LOW);
      digitalWrite(buzzer,LOW);
    sec[estado]=2;                            //almacena un 2 en el arreglo "sec" al ser pulsado
    estado++;                                 //el botón C
    last_change_C = timesnapshot;             //actualiza la variable al tiempo actual
   }      
   }
  }
  if(estado==4){                              //al almacena cuatro pulsaciones
    if((sec[0]==clave[0])&&(sec[1]==clave[1])&&(sec[2]==clave[2])&&(sec[3]==clave[3])){
                                              //compara las pulsaciones con la contraseña alamcenada
       digitalWrite(buzzer, LOW);             //feedback auditivo de confirmación
       delay(100);                            //|
       analogWrite(buzzer,90);                //|
       delay(100);                            //|
       digitalWrite(buzzer, LOW);             //|
       delay(100);                            //|
       analogWrite(buzzer,90);                //|
       delay(100);                            //|
       digitalWrite(buzzer, LOW);             //|
       correctpass();                         //llama la función de confirmación contraseña correcta
     }
    else {                                    //si la contraseña es incorrecta
        wrongpass();                          //parpadeo de error (buzzer y led)
         }
  } 
  if(intentosRestantes==0){                   //al equivocarse 3 veces seguidas
       no_push = 1;                           //se bloquean las pulsaciones de botónes
    buzzerblock();                            //llama a la función de parpadeo de bloqueo

//configura el tiempo de bloqueo de 16 segundos
    if ((timesnapshot > (last_ledchange + 2000))&&(led_counter1 < 4)&&(led_st == 0)){
      digitalWrite(ledOK,HIGH);
      last_ledchange = timesnapshot;
      led_st = 1;                         //led_counter++;
      }
      if ((timesnapshot > (last_ledchange + 2000))&&(led_counter1 < 4)&&(led_st == 1)){
      digitalWrite(ledOK,LOW);
      last_ledchange = timesnapshot;
      led_st = 0;
      led_counter1++;
      }
   if (led_counter1 == 4){ 
    digitalWrite(ledOK,LOW);
    intentosRestantes=3;
    led_counter1 = 0;
    last_ledchange = 0;
    no_push = 0;
   }
  }
}

//función que monitorea la activación de los optoswitch de las cajas
void sensorcheck(){                 // parte de alarma desactivada / solo registro
  if (correcto_abierto == 1){
    digitalWrite(ledOK,HIGH);       //mantiene el led encendido mientras la alarma está desactivada
    corriendo_ahora = 1;

    varclean();                     //llama a la función que limpia las variables

    sensorValue = analogRead(analogInPin1);  //monitorea el primer sensor optoswitch
  if((sensorValue > 500)&&(An1_activo == 0)) //al detectar cambios ejecuta el script tq12
        { 
            An1_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                  //presiona tecla ctrl.
    buf[2] = KEY_F19;                        //presiona la tecla F19.
    Serial.write(buf, 8);                    //envia los comandos anteriores.
    releaseKey();                            //llama a la función releaseKey.   
  }

  if((sensorValue < 500)&&(An1_activo == 1)) //método de seguridad para que no se
  {         An1_activo = 0;}                 //gatille más de una vez cada iteración

    sensorValue = analogRead(analogInPin2);  //monitorea el segundo sensor optoswitch
  if((sensorValue > 500)&&(An2_activo == 0)) //al detectar cambios ejecuta el script tq22
        { 
          An2_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                  //presiona tecla ctrl.
    buf[2] = KEY_F20;                        //presiona la tecla F20.
    Serial.write(buf, 8);                    //envia los comandos anteriores.
    releaseKey();                            //llama a la función releaseKey.  
  }

  if((sensorValue < 500)&&(An2_activo == 1)) //método de seguridad para que no se
  {         An2_activo = 0;}                 //gatille más de una vez cada iteración

  sensorValue = analogRead(analogInPin3);    //monitorea el tercer sensor optoswitch
  if((sensorValue > 500)&&(An3_activo == 0)) //al detectar cambios ejecuta el script tq32
        { 
          An3_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                  //presiona tecla ctrl.
    buf[2] = KEY_F21;                        //presiona la tecla F21.
    Serial.write(buf, 8);                    //envia los comandos anteriores.
    releaseKey();                            //llama a la función releaseKey. 
  }

  if((sensorValue < 500)&&(An3_activo == 1)) //método de seguridad para que no se
  {         An3_activo = 0;}                 //gatille más de una vez cada iteración
    
    sensorValue = analogRead(analogInPin4);  //monitorea el cuarto sensor optoswitch
  if((sensorValue > 500)&&(An4_activo == 0)) //al detectar cambios ejecuta el script tq42
        { 
          An4_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                  //presiona tecla ctrl.
    buf[2] = KEY_F22;                        //presiona la tecla F22.
    Serial.write(buf, 8);                    //envia los comandos anteriores.
    releaseKey();                            //llama a la función releaseKey. 
  }
  if((sensorValue < 500)&&(An4_activo == 1)) //método de seguridad para que no se
  {         An4_activo = 0;}                 //gatille más de una vez cada iteración

  
  sensorValue = analogRead(analogInPin5);   //monitorea el quinto sensor optoswitch
  if((sensorValue > 500)&&(An5_activo == 0))//al detectar cambios ejecuta el script tq52
        { 
          An5_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                 //presiona tecla ctrl.
    buf[2] = KEY_F23;                       //presiona la tecla F23.
    Serial.write(buf, 8);                   //envia los comandos anteriores.
    releaseKey();                           //llama a la función releaseKey.
  }
  if((sensorValue < 500)&&(An5_activo == 1))//método de seguridad para que no se
    {         An5_activo = 0;}              //gatille más de una vez cada iteración

     sensorValue = analogRead(analogInPin6);//monitorea el sexto sensor optoswitch
  if((sensorValue > 500)&&(An6_activo == 0))//al detectar cambios ejecuta el script tq62
        { 
          An6_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                 //presiona tecla ctrl.
    buf[2] = KEY_F24;                       //presiona la tecla F24.
    Serial.write(buf, 8);                   //envia los comandos anteriores.
    releaseKey();                           //llama a la función releaseKey.
  }
  if((sensorValue < 500)&&(An6_activo == 1))//método de seguridad para que no se
    {         An6_activo = 0;}              //gatille más de una vez cada iteración
    
    }
    else {                                  //parte de alarma activada y registro 
  sensorValue = analogRead(analogInPin1);   //monitorea el primer sensor optoswitch
  if((sensorValue > 500)&&(An1_activo == 0))//al detectar cambios ejecuta el script tq1
        { 
            An1_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                 //presiona tecla ctrl.
    buf[2] = KEY_F13;                       //presiona la tecla F13.
    Serial.write(buf, 8);                   //envia los comandos anteriores.
    releaseKey();                           //llama a la función releaseKey. 

//si la alarma disuasiva o sonora se encuentra ativa gatilla el botón de pánico
    if (puertaflag == true || alarman2flag == true){
      panicbtn = true;
      alarman3();
    }
  }

  if((sensorValue < 500)&&(An1_activo == 1))//método de seguridad para que no se
  {         An1_activo = 0;}                //gatille más de una vez cada iteración

sensorValue = analogRead(analogInPin2);     //monitorea el segundo sensor optoswitch
  if((sensorValue > 500)&&(An2_activo == 0))//al detectar cambios ejecuta el script tq2
        { 
          An2_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                 //presiona tecla ctrl.
    buf[2] = KEY_F14;                       //presiona la tecla F14.
    Serial.write(buf, 8);                   //envia los comandos anteriores.
    releaseKey();                           //llama a la función releaseKey. 

//si la alarma disuasiva o sonora se encuentran activas gatilla el botón de pánico
    if (puertaflag == true || alarman2flag == true){
      panicbtn = true;
      alarman3();
    }
  }

  if((sensorValue < 500)&&(An2_activo == 1))//método de seguridad para que no se
  {         An2_activo = 0;}                //gatille más de una vez cada iteración
                                                       
    sensorValue = analogRead(analogInPin3); //monitorea el tercer sensor optoswitch
  if((sensorValue > 500)&&(An3_activo == 0))//al detectar cambios ejecuta el script tq3
        { 
          An3_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                 //presiona tecla ctrl.
    buf[2] = KEY_F15;                       //presiona la tecla F15.
    Serial.write(buf, 8);                   //envia los comandos anteriores.
    releaseKey();                           //llama a la función releaseKey. 

//si la alarma disuasiva o sonora se encuentra ativa gatilla el botón de pánico
    if (puertaflag == true || alarman2flag == true){
      panicbtn = true;
      alarman3();
    }
  }

  if((sensorValue < 500)&&(An3_activo == 1))//método de seguridad para que no se
  {         An3_activo = 0;}                //gatille más de una vez cada iteración

sensorValue = analogRead(analogInPin4);     //monitorea el cuarto sensor optoswitch
  if((sensorValue > 500)&&(An4_activo == 0))//al detectar cambios ejecuta el script tq4
        { 
          An4_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                 //presiona tecla ctrl.
    buf[2] = KEY_5;                       //presiona la tecla 0.
    Serial.write(buf, 8);                   //envia los comandos anteriores.
    releaseKey();                           //llama a la función releaseKey. 

//si la alarma disuasiva o sonora se encuentra ativa gatilla el botón de pánico
    if (puertaflag == true || alarman2flag == true){
      panicbtn = true;
      alarman3();
    }
  }
  if((sensorValue < 500)&&(An4_activo == 1))//método de seguridad para que no se
  {         An4_activo = 0;}                //gatille más de una vez cada iteración

  
 sensorValue = analogRead(analogInPin5);    //monitorea el quinto sensor optoswitch
  if((sensorValue > 500)&&(An5_activo == 0))//al detectar cambios ejecuta el script tq5
        { 
          An5_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                 //presiona tecla ctrl.
    buf[2] = KEY_F17;                       //presiona la tecla F17.
    Serial.write(buf, 8);                   //envia los comandos anteriores.
    releaseKey();                           //llama a la función releaseKey. 

//si la alarma disuasiva o sonora se encuentra ativa gatilla el botón de pánico
    if (puertaflag == true || alarman2flag == true){
      panicbtn = true;
      alarman3();
    }
  }
  if((sensorValue < 500)&&(An5_activo == 1))//método de seguridad para que no se
    {         An5_activo = 0;}              //gatille más de una vez cada iteración

 sensorValue = analogRead(analogInPin6);    //monitorea el sexto sensor optoswitch
  if((sensorValue > 500)&&(An6_activo == 0))//al detectar cambios ejecuta el script tq6
        { 
          An6_activo = 1;
    buf[0] = KEY_LEFT_CTRL;                 //presiona tecla ctrl.
    buf[2] = KEY_F18;                       //presiona la tecla F18.
    Serial.write(buf, 8);                   //envia los comandos anteriores.
    releaseKey();                           //llama a la función releaseKey.

//si la alarma disuasiva o sonora se encuentra ativa gatilla el botón de pánico
    if (puertaflag == true || alarman2flag == true){
      panicbtn = true;
      alarman3();
    }
  }
  if((sensorValue < 500)&&(An6_activo == 1))//método de seguridad para que no se
    {         An6_activo = 0;}              //gatille más de una vez cada iteración

}
}
//función que se usa para "soltar las teclas" luego de "pulsarlas".
void releaseKey()
{
  buf[0] = 0;                               //resetea todos los valores del buffer a 0.
  buf[1] = 0;                               //|
  buf[2] = 0;                               //|
  Serial.write(buf, 8);                     //envia los comandos anteriores.
  delay(200);
}

void  varclean(){
if (co_ab == false){
    puertaflag = false;             //pone las banderas a 0 para reiniciar los valores de los
    panicflag = false;              //sensores de presencia y proximidad y apaga la alarma
    digitalWrite(alarmPin, HIGH);   //sonora si es que estaba encendida
    digitalWrite(buzzer, LOW);      //|
    alarman2flag = false;           //|
    alarma = false;                 //|
    panicbtn = false;               //|
    puertacount = 0;                //|
    smsalarm = false;               //|
    co_ab = true;
  }
  }

//función que es llamada al confirmarse que la contraseña ingresada fue correcta
  void correctpass() {                                                                                                     
      correcto_abierto = 1;                 //configuración de banderas para comparación
      co_ab = false;
      if (corriendo_ahora == 1){            //|
        correcto_abierto = 0;               //|
        corriendo_ahora = 0;                //|
        }
      intentosRestantes=3;                  //restaura los intentos restantes de pulsaciones
         digitalWrite(ledOK,LOW);
         estado=0;
    }

//función que se encarga de los parpadeos de led y buzzer al introducir una contraseña incorrecta
void wrongpass(){
      if ((timesnapshot > (last_ledchange + 150))&&(led_counter < 7)&&(led_st == 0)){
      digitalWrite(ledOK,HIGH);
      analogWrite(buzzer,60);
      last_ledchange = timesnapshot;
      led_st = 1;     
      }
      if ((timesnapshot > (last_ledchange + 150))&&(led_counter < 7)&&(led_st == 1)){
      digitalWrite(ledOK,LOW);
      digitalWrite(buzzer,LOW);
      last_ledchange = timesnapshot;
      led_st = 0;
      led_counter++;
      }
      if (led_counter == 7){
        intentosRestantes--;
          digitalWrite(ledOK,LOW);
          digitalWrite(buzzer,LOW);
          estado=0;
          led_counter = 0;
          last_ledchange = 0;   
          led_st = 0;   
        }
    }
//función que se en carga de los parpadeos del buzzer al introducir incorrectamente 3 veces la contraseña
void buzzerblock(){
    if((timesnapshot > (last_buzzchange + 300)) && (buzz_counter < 4)&&(buzz_st == 0)){
        analogWrite(buzzer,100);
        last_buzzchange = timesnapshot;
        buzz_st = 1;
      }
      if((timesnapshot > (last_buzzchange + 100)) && (buzz_counter < 4)&&(buzz_st == 1)){
        digitalWrite(buzzer,LOW);
        last_buzzchange = timesnapshot;
        buzz_st = 0;
        buzz_counter++;
      }
      if(buzz_counter == 4){
        digitalWrite(buzzer,LOW);
      }
      if((timesnapshot > (last_buzzchange + 11000)) && (buzz_counter >= 4)&&(buzz_st == 0)){
        buzz_counter = 0;
        last_buzzchange = 0;
      }
      }
//función que se encarga de los sensores de presencia y el optoswitch de apertura de la puerta
void presencia()
{int value= digitalRead(PIRPin);            //guarda la lectura del sensor de presencia en "value"

ultrasonico();                              //se llama la función de monitoreo del
endchk: //label                             //sensor ultrasónico
  if(puertaflag == true){                   //si la puerta ha sido abierta
    bipflag = true;                         
    buzzerbip();                            //buzzer parpadea intermitentemente
    
 if (value == HIGH && timesnapshot >= (lastshot + 3000))
  {                                         //espera 3 segundos para entre cada lectura
    counterxd++;                            //aumenta el contador con cada lectura
    analogWrite(buzzer,45);                 //feedback por medio del buzzer
    lastshot = millis();}                   //actualiza la variable con el tiempo actual

  if(timesnapshot >= last25chk+30000){      //|espera 30 segundos para realizar la comparación
    if (counterxd >= 3 || value == HIGH){   //|si detectó 3 lecturas de movimiento en 30s
       if(timesnapshot >= last25chk+120000){//|se mantendrá leyendo el movimiento durante los
  if (counterxd >= 12 || value == HIGH){    //|proximos 2 minutos y si despúes de los dos minutos
    counterxd = 0;                          //|detectó más de 12 lecturas de movimiento
    alarma = true;                          //|activa la bandera de alarma sonora
    }
else {counterxd = 0;                        //|si despúes de dos minutos no detectaron mas de 12
      puertaflag = false;                   //|lecturas de movimiento se reinician las banderas
      bipflag = false;                      //|
      digitalWrite(buzzer, LOW);            //|
      goto endchk;
      }
}
}
else {
      counterxd = 0;                         //|si despúes de 30s no detectaron mas de 3
      puertaflag = false;                    //|lecturas de movimiento se reinician las banderas
      bipflag = false;                       //|
      digitalWrite(buzzer, LOW);             //|
      }
}
}

else if (puertaflag == false){               //|si la puerta está cerrada las banderas
      bipflag = false;                       //|se mantienen en su estado unicial
      counterxd = 0;                         //|
      buzzerbip();

}
  int irtest = digitalRead(irPin);           //se guarda la lectura del optoswitch en "irtest"
  if (irtest == HIGH && puertaflag == false){
    puertaflag = true;                       //si la puerta se abre se activa la bandera
    puertacount++;                           //aumenta el contador de aperturas
    if (puertacount == 6){alarma = true;}    //si el contador de apertura es > 6 la alarma inicia
    last25chk = millis();                    //se actualiza la variable con el tiempo actual
  }
}

//función que se encarga de activar la alarma nivel 2, alarma sonora
void alarman2(){
   if (alarma == true){                     //al detectar la bandera "alarma" activa
    if(smsalarm == false){                  //|envia un SMS a un número especifico 
    mensaje = "alarma sonora activada";     //|(configuración de mensaje a enviar)
    imprimnumero = "529513337704";          //|(configuración de número)
    modoenviar();                           //|(función para mandar SMS)
    smsalarm = true;                        //activa la bandera indicando que ya fue enviado
                                            //para evitar que se mande más de una vez
    }
    alarman2flag = true;                    //activa la bandera que indica que la alarma está sonando
    digitalWrite(alarmPin, LOW);            //activa el relevador (funcionamiento inverso LOW = encendido)
}
}
//función que se encarga de activar la alarma nivel 3, botón de pánico
void alarman3()
{
  if (panicbtn == true){                   //al detectar la bandera "panicbtn" activa

    if(panicflag == false){                
    delay(500);                            //espera 500ms para que el ordenador termine de registrar
    buf[0] = KEY_LEFT_CTRL;                //presiona la tecla CTRL
    buf[2] = KEY_F16;                       //presiona la telca F16.
    Serial.write(buf, 8);                  //envia los comandos anteriores.
    releaseKey();                          //llama la función releasekey.  
    panicflag = true;                      //activa la bandera para evitar que se mande de nuevo
    mensaje = "boton de panico activado";  //|envia un mensaje a un número especifico indicando
    imprimnumero = "529513337704";         //|que el botón de pánico ha sido activado
    modoenviar();
  }
    digitalWrite(alarmPin, LOW);           //activa la alarma sonora
  }}
//función que se encarga de el la alarma de disuación
void buzzerbip()
{
  if (alarma == false){                   //se desactiva al estar la alarma sonora activa
  if (bipflag == true){
  if ((timesnapshot >= (lastbip + 300)) && (timesnapshot < (lastbip + 600)))
  analogWrite(buzzer,60);
}
  if( timesnapshot >= (lastbip + 600)){
 digitalWrite(buzzer, LOW);
 lastbip = millis();
 }
 }
}
//función que se encarga de monitorear el sensor ultrasónico
void ultrasonico()
{
  if(timesnapshot >= (lastultra+30)){     // Esperar 30 milisegundos entre mediciones
  int uS = sonar.ping_median();           //|Obtener medicion de tiempo de viaje del sonido 
                                          //|y guardar en variable uS
if (((uS / US_ROUNDTRIP_CM)< 100) && ((uS / US_ROUNDTRIP_CM) > 10))
  {                                       //si la distancia es mayor a 10cm y menor a menor a 1m

    alarma = true;                        //se activa la alarma sonora
    panicbtn = true;                      //se activa el botón de pánico
    alarman3();                           //se llama a la función de alarma nivel 3
    }
  lastultra = millis();                   //se actualiza la variable con el tiempo actual
}
}
