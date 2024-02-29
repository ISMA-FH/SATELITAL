/*  **************************************************************************************************************
      Este es un sistema de control para enviar los datos del sensor caudalimetro
      con menu de comandos por terminal, que interactuan con interfaz SW PLUS InfoPro, Speed COM Serial = 115200 bps
      Se utiliza GRIZZLY Ver 0.3 OCT 2023 y TEDDY BEAR para ETHERNET y 2 entradas 4-20 mA
  ==================================================================================================================================================
 
   - Se cambia el tipo de sistema a MEDIDOR (M), se agrega el campo de numero de serie del medidor (NSM) y Unidad verificadora acreditada (UV***)
   - Se debe indicar el modelos de caudalimetro de donde va a tomar los datos
     del sensor de flujo volumetrico. 
   - La velocidad (bit rate) de la conexion USB - intefaz es de 115200 bps
   - Se agregan lectura de memoria SD con el comando #RMEM
   - Comando de obtener de archivo especifico los datos contenidos en la carpeta /DATOS con #FILEnombreArchivo.extension  (archivos 8.3)
   - Se agrega comando #GTME para obtener el tiempo del sistema en formato: AAAMMDD HHMMSS
   - Se agrega comando #STME para modificar el tiempo del sistema y actualizar el RTC, la forma es:
          #STMEAAAAMMDD HHMMSS
   - Comando #SEND, usa la conexion Ethernet (por bridge) para envio de archivo al servidor FTP por conexion alambrica
   - Se agrega funcion de envio por metodo POST HTTP con estructura HTML
   - Se agrega el comando #REDY, se valida primero el puerto de comunicacion y posterior se hace el logeo
   - Se probo con STEP DOWN para poner el voltaje del sistema a 9 VDC
   - Se agrega funcion lectura archivo NSUT
   - Cambio de latitud y longitud de float a String en JSON
   - KER 10 error de conexion servidor FTP y falla de envio FTP
   - Comando #ETHE que muestra en el serial los parametros de IP, GATEWAY y MAC ADDRESS
   - archivo Json de login:  #MDL1{"username":"admin","password":"admin","model":406, "bauds": 9600, "comm": 0}, en infopro 1.3a desktop aun no reconoce comm
   
 ===================================================================================================================================================
  Fecha: NOV 27 de 2023.
  Version 2.3.1
  Modificada  09 ENE 2024 
  Pruebas: EN CURSO
  Escrito por: Héctor C. Sandoval, Ismael Flores, Israel Rivera & Jonatan Gamboa.
 *************************************************************************************************************** 
*/


#include <SoftwareSerial.h>                         // Libreria necesaria para usar software serial.
#include <SPI.h>                                    // Comunicacion SPI
#include <SD.h>                                     // Libreria archivos en Memoria SD
#include <Wire.h>                                   // Comunicación I2C
#include <RTClib.h>                                 // Libreria de Reloj en tiempo real
#include <SimpleModbusMaster.h>                     // Liberia de Modbus RS485
#include <fp64lib.h>                                // Libreria para manejo datos 64 bits
#include <ArduinoJson.h>                            // Libreria para manejo de archivo en formato json
#include <EthernetClient.h>
#include <Ethernet.h>
#include <LiquidCrystal_I2C.h>  
#include <EthernetUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>                    

// Define los Chipselect para SPI del ethernet - CSE - y para el de la Mem SD - CS -
#define CSE   3
#define CS    4  

// Modbus holding registers aforador MC406
#define  MC406_REG_FR     0x0682    //    3:1672 Modbus Register Reading Input Reg (CMD04) - LSB WORD Tasa de Flujo [m3 / hora]    -- Flow Rate instantaneus float 32 bits   LITTLE ENDIAN 
#define  MC406_REG_TV     0x0688    //    3:1666 Modbus Register Reading Input Reg (CMD04) - LSB Tasa de Volumen acumulado [m3]    -- Total Positive m3  float 32 bits

// Modbus holding registers aforador MC608
#define     MC608_REG_FR     0x001A       // 3:0029 - MSB Tasa de Flujo [m3 / seg]    --Flow Rate instantaneus [m3/seg], float 32 bits
#define     MC608_REG_TV     0x0006       // 3:0006 - MSB Volumen total + [m3]    -- Total volume + (accumulated) , float 32 bits

// Modbus holding registers aforador HACH SC200
#define     SC200_REG_FR     0x0001       // 3:0029 - MSB Tasa de Flujo [m3 / seg]    --Flow Rate instantaneus [m3/seg], float 32 bits
#define     SC200_REG_TV     0x0003       // 3:0006 - MSB Volumen total + [m3]    -- Total volume + (accumulated) , float 32 bits

// Modbus holding registers aforador MCCROMETER PC-RA2 (UM06), empieza en el registro 0 para leer 14 Input register
#define  UM06_REG_FR      0x0004    //    3:0005 Modbus Register Reading Input Reg (CMD04) - MSB WORD Tasa de Flujo [m3 / tiempo]  float 32 bits    -- flow rate value in the unit of measure chosen (as can be seen in the display of the instrument)
#define  UM06_REG_TV      0x0008    //    3:0009 Modbus Register Reading Input Reg (CMD04) - MSB WORD Volumen Total + [m3 ], unsigned long 32 bits  -- Totalizer T+ value

// Modbus holding registers aforador MoodMag 1000 or M5000, Comunicacion Modbus RTU RS485   
#define  M1000_REG_TV     0x0216        //     3,4:0206 Modbus Holding Register CMD04 - MSB WORD Total Volumen + [m3]    -- Volume +T
//#define  M1000_REG_FR     0x00ED      //     3,4:0237 Modbus Holding Register CMD04 - MSB WORD Flow Rate [m3/seg]    -- Flow Rate +T
#define   M1000_REG_FR   0x00F1         //     3,4:0241 Modbus Holding Register CMD04 - MSB WORD Flow Rate User units  -- Flow Rate user units

// Modbus holding registers aforador MoodMag 2000, Comunicacion Modbus RTU RS485
#define  M2000_REG_TV     0x00CF        //     3,4:0211 Modbus Holding Register CMD04 - MSB WORD Total Volumen + [m3]    -- Volume +T
//#define  M2000_REG_FR     0x00ED      //     3,4:0237 Modbus Holding Register CMD04 - MSB WORD Flow Rate [m3/seg]    -- Flow Rate +T
#define   M2000_REG_FR    0x00F1         //     3,4:0241 Modbus Holding Register CMD04 - MSB WORD Flow Rate User units  -- Flow Rate user units

// Modbus holding registers aforador ProMag L400, Comunicacion Modbus RTU RS485
#define  PGL400_REG_TV    0x07D8   //    3:2008 Modbus Register Reading Input Reg CMD04 - MSB WORD Flow Rate [l/h]
#define  PGL400_REG_FR    0x0A31   //    3:2609 Modbus Register Reading Input Reg CMD04 - MSB WORD Total Volumen + [m3]   -- Volume +T

// Modbus holding registers aforador ULTRA TT, EquySIS, Comunicacion Modbus RTU RS232
#define  ULTRATT_REG_TV   0x0200    //    3:0512 (0x0200) Modbus Holding Register CMD04 - MSB WORD Total Volumen + [m3]    -- Volume +T
#define  ULTRATT_REG_FR   0x0400   //    3:1024 (0x0400) Modbus Holding Register CMD04 - MSB WORD Flow Rate [m3/seg]    -- Flow Rate +T

// Modbus holding registers aforador SIEMENS_MAG6000
#define  SITRANS6000_REG_FR    0x0BBA         // 4:03003 - MSB Tasa de Flujo [m3 / seg]    --Flow Rate instantaneus [m3/seg], float 32 bits Big Endian
#define  SITRANS6000_REG_TV    0x0BC6         // 4:03014 - LSB Volumen total + [m3]    -- Total volume + (accumulated) , double 64 bits 

// Modbus holding registers aforador KRONE300
#define  KRONE300_REG_FR    0x0002         // 4:30002 - MSB Tasa de Flujo [m3 / seg]    --Flow Rate instantaneus [m3/seg], float 32 bits Big Endian
#define  KRONE300_REG_TV    0x0014         // 4:30020 - MSB Volumen total + [m3]        -- Total volume + (accumulated) , double 64 bits 

// Modbus holding registers aforador ARKON MAGX2
#define  ARKONMX2_REG_FR    0x0096         // 4:0150 - MSB Tasa de Flujo [m3 / seg]  (Modbus Addres = 0149)    --Flow Rate instantaneus [m3/h], float 32 bits Big Endian
#define  ARKONMX2_REG_TV    0x009C         // 4:0156 - MSB Total + [m3]              (Modbus Addres = 0149)                       -- Total volume + (accumulated), float 32 bits Big Endian


//////////////////// Port information MODBUS RTU ///////////////////
#define timeout      200       //Maximum time for slave to respond (ms)
#define polling      50        //Maximum scan rate of master to allow slave to return to idle (ms)
#define retry_count  10     //Maximum retries if slave returns response timeout or error
#define TxEnablePin  2     //Pin to set RS485 interface to transmit or re

// The total amount of available memory on the master to store data
#define TOTAL_NO_OF_REGISTERS 8    // REG[0] = _REG_FR   // This is the easiest way to create new packets, add as many as you want. TOTAL_NO_OF_PACKETS is automatically updated.

//CODIGOS ATRIBUTOS APLICACION ESCRITORIO
#define model_login      "MDL1"
#define cmd_login        "LOGI"
#define cmd_send         "SEND"
#define cmd_save         "SAVE"
#define cmd_stime        "STME"                            // SET TIME for date and time, we use this format AAAAMMDDHHMMSS;      // for TIME we are going to use this command GTME
#define cmd_gtime        "DATE"                             // for TIME we are going to use this command GTME
#define cmd_sign         "SIGN"
#define cmd_wmac         "WMAC"
#define cmd_peth         "ETHE"
#define cmd_rsys         "RSYS"                           // Reinicia el sistema totalmente
#define cmd_vlue         "VLUE"
#define cmd_rmem         "RMEM"                           // Muestra (lista) y da lectura de los archivos contenidos en la MEMORIA SD
#define cmd_dfil         "FILE"                           // Volcado de archivos, muestra la información contenida del archivo por comando #DFIL"archivo.ext"                                                             // (SIN COMILLAS Y MUESTRA LOS ARCHIVOS CONTENIDOS EN LA RAIZ del sistema                                     
#define cmd_envioWebPost  "SWEB"   
#define cmd_obtenNSUT     "NSUT"  
#define cmd_ntpTime       "NTPT"                        // Obtiene el tiempo y fecha de un servidor NTF. 
// defines of the screens to show for case 
#define SCREEN_LETRERO  0
#define SCREEN_VOLUMEN  1
#define SCREEN_FLUJO    2
#define SCREEN_TIEMPO   3
#define SCREEN_MODEM    4
///#define SCREEN_MARCO    5
///////////////////////////////////////////////

//byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE0};   // CAMBIAR LA MAC PARA CADA TEDDY QUE SE CAMBIE, AL MENOS PARA CADA RED LOCAL DEBEN SER DIFERENTES   
int array_pasv[6];
byte outCount;
char outBuf[128];
EthernetClient client;
EthernetClient dclient;
// EthernetUDP    clientNTP;
// NTPClient timeClient(clientNTP,"pool.ntp.org"); //////Protocolo de NTP
bool networkConnected = false;

//Variables globales
 struct Clasif {           
    String loginUsername = "";
    String loginPassword = "";    
    String macAddres= "";
    String uvaUsername = "";
    String uvaPassword = "";
    String uvaID = "";
    String tipo = "";
    String nsm = "";                                        // Número de serie del medidor
    String nsue = "";                                       // Número de serie de la unidad electrónica
    unsigned int dn = 0;                                         // Diámetro nominal de la  13 <= DN <= 800 mm                                 *********
    unsigned int comm = 0;
    //bool eth = false;  
    String nsut = "";                                       // Número de serie de unidad de telemetría                                    *********
    unsigned int modeloAforador = 0;
    unsigned int bauds = 0;
    String ftpServer = "" ;
    String ftpUsername = "";
    String ftpPassword = "";
    String ftpFolder =  "";
    unsigned int port = 0;
    unsigned int hhEnvio = 0;
    unsigned int mmEnvio = 0;
    String rfc = "";                                        
    String tagID = "";                                    // Identificador del nodo o pozo de la concesion                                *********
    String longitud = "0.0";
    String latitud = "0.0";
    unsigned int hhSys = 0;
    unsigned int mmSys = 0;
};

char archivoFTP[512]="";
char *archConf = "CONFIGVE.JSN";  // <- SD library uses 8.3 filenames
char *folderDatosSD = "DATOS/";
char *autologg = "AUTOLOGI.LOG";

Clasif conf;                         // <- global configuration object

bool arranque = false;
File myFile, autoLogg;
File root;
File fileNSUT;
RTC_DS3231 rtc;                                             //DEFINIMOS VARIABLE DEL RELOJ EN TIEMPO REAL RTC   
//Variables funciones
char archivo[13], imei[16];
String inputString = "", content="";         // a string to hold incoming data
boolean stringComplete = false, modbusMsgApp = false;  // whether the string is complete
String commandString = "", cnc_fecha="cnc_fecha", cnc_hour="cnc_hour", cnc_flow="", cnc_vol="", error = "000";
uint8_t  type, counter = 1;
uint16_t hh=0, mm=0, ss=0, AA=0, MM=0, DD=0, controlC = 0, hhNTP=0, mmNTP=0, ssNTP=0, AANTP= 0, MMNTP=0, DDNTP=0;
float totalVol = 0, flowRate = 0, tVolC=0, fRateC=0;

// 
int x = 16,y = 0, x1=16, y1 = 0, RET = 200, i, z, tiempo; 
unsigned long previousMillisTiempo = 0;
unsigned long previousMillisScreen = 0;  
long lcdInterval = 8000;
// screen to show 
uint8_t screen = 0;    
uint8_t screenMax = 4;
bool screenChanged = true;   // initially we have a new screen,  by definition 
bool screenLogin = false, redOK, envioOK, ipOK, passOK, dirOK;

// Variables Modbus

uint32_t tmp, trans; // Change this to suit your needs
bool  estado=false;

// Para recibir paquetes de Modbus RTU con Librería
enum
{
  PACKET1,
  PACKET2,                          
  //PACKET3,                // Every packets its for each variable
  TOTAL_NO_OF_PACKETS       // leave this last entry
};

// Enumeración para identificar el modelo para construir maquina MODBUS
typedef enum 
{
  NONE,                           //  = 0
  MC608,                          //  = 1      
  MC406,
  SC200,                         // HACH SC200
  UM06,                         // MCCROMETER ULTRAMAC PC101
  L400,                  
  M1000,                      // BADGER METER M1000     Model infopro = 100
  M2000,
  ULTRATT,
  SITRANS6000,
  KRONE300,
  ARKONMX2,                   // MAGX2 - ARKON
  LAST
} MODEL;

MODEL ModeloAforador = NONE; 
// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

typedef Packet* packetPointer;

// Masters register array
uint16_t  regs[TOTAL_NO_OF_REGISTERS];

// Create a packetPointer to access each packet individually. This is not required you can access the array explicitly. E.g. packets[PACKET1].id = 2;
// This does become tedious though...
packetPointer packet1 = &packets[PACKET1];
packetPointer packet2 = &packets[PACKET2];
//packetPointer packet3 = &packets[PACKET3];

// FUNCIONES PARA MODBUS 485
float Hex2Float(uint32_t x);
uint32_t Float2Hex(float y);

// FUNCIONES LOGIN

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0

void login(void);
void tiempoActual(bool);
bool saveConfiguration(char *archConf, Clasif &conf);
void loadConfiguration(char *archConf, Clasif &conf);
float Gasto(MODEL ModeloAforador);
float Volumen(MODEL ModeloAforador);
void opciones(void);
bool envioFTP(bool eval, bool mensaje);
bool configSD(void);
void saveDataSD (char *archivoSD, char * datos);
bool setModbus(MODEL ModeloAforador);
void printDirectory(File dir, int numTabs);         //Lista archivos
void dumpFile(String archivo);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t tout = 0);
void setTime(void);
String getData(void);
void getCommand(void);
void updateData(void);
boolean conModbus(float fR, float tV, bool modbusMsgApp);
void envioWebPost(void);
String readFromFile(char * fileName, size_t * fileSize);
void obtenNSUT(bool mesg);
void cableEthernet(void);
void connectToFTP(void);
void loggFTP(void);
void optsFTP(void); 
void dirFTP(void);
void asciiFTP(void);
void storFTP(void);
void salirFTP(void);
String macEtheString(void);
void obtenNTP(void);

StaticJsonDocument<768> JsonDoc;  //almacenar Doc Json 768 bytes en la pila

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Inicia la comunicación por I2C porrowX27, pines asociados al display

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tiempoActual(bool msg){
  char fecha[16], hora[16];
  DateTime now = rtc.now();
  sprintf(fecha, "%02d%02d%02d", now.year(), now.month(), now.day());   // formato AAAAMMDD
  sprintf(hora, "%02d%02d%02d",  now.hour(), now.minute(), now.second());  // formato HHMMSS
  cnc_fecha = fecha; 
  cnc_hour = hora;
  if(!msg){
    Serial.println("");
    Serial.println(F("------------------------- Reloj de Sistema (Hora militar) --------------------------------"));
    Serial.println("\t\tFECHA: " + cnc_fecha + "\t\tTIEMPO: " + cnc_hour);
    Serial.println(F("---------------- ---------------------------------- --------------------------------------"));
  } else 
    Serial.println("#DATE" + cnc_fecha + " " + cnc_hour);
}
//////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////FUNCION PARA DETECTAR CABLE ETHERNET/ /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cableEthernet(void)
{
  Serial.println(F("VERIFICANDO CONEXION ETHERNET... ... ..."));
  if((!Ethernet.begin(mac)== 1))    
  {
    Serial.println(F("NO HAY RED ETHERNET"));
      delay(10);
      error = "012";
      Serial.print(F("DHCP ASIGNADA A IP "));
      Serial.println(Ethernet.localIP());
      redOK = false; 
  } else {
    Serial.println(F("Red Ethernet conectada"));
    Serial.print(F("DHCP asignada a IP "));
    Serial.println(Ethernet.localIP());
    redOK = true; 
    delay(2000); 
    connectToFTP();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////FUNCION PARA CONECTARSE A EL ETHERNET /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void conexionEthernet()
{
  
  Serial.println(F("INICIALIZAR ETHERNET CON DHCP:"));
  if (Ethernet.begin(mac) == 0) 
    {
      Serial.println(F("FALLA EN LA INICIALIZACION ETHERNET USANDO DHCP"));
        if (Ethernet.hardwareStatus() == EthernetNoHardware) 
          {
            Serial.println(F("NO SE ENCONTRÓ EL ETHERNET SHIELD"));
            conf.comm = 0;
          }   
      //Ethernet.begin(mac, ip, myDns);
    } 
      else 
        {
          Serial.print(F("DHCP ASIGNADA A IP "));
          Serial.println(Ethernet.localIP());
          //delay(1000);
          conf.comm = 3;
          Serial.println(F("TEDDY INICIADA CORRECTAMENTE. . . "));
        }
      //delay(1000);
}
// Functions
/* ==================================================================================================================== */

void obtenNSUT(bool mesg) {

  String msg ="" /*, jsonResult = ""*/;
  size_t fileNSUTSize;
  char * archivoNSUT = "SYS/NSUT.txt";
  if(!SD.exists(archivoNSUT)) Serial.println(F("#MSSGVERIFICAR ARCHIVO NSUT.TXT EN CARPETA /SYS\n\rCOMPROBAR NSUT CON SWPLUS TECNOLOGIAS"));
  else{
    conf.nsut = readFromFile(archivoNSUT, &fileNSUTSize);
    msg = "#MSSGNUM. SERIE DE UNIDAD DE TELEMETRIA: " + conf.nsut;
  }
  
  tiempoActual(true);
  if(mesg) Serial.println(msg);
}

// ====================================================================================================================
// FUNCION CONFIGURACION DE MODULOS RF, MODBUS, 
// ====================================================================================================================
boolean setModbus(MODEL ModeloAforador){
  switch(ModeloAforador)
   {
    case MC608:
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, MC608_REG_FR, 2, 1);  // ID=1, CMD=4 (Read Input Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, MC608_REG_TV, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate  
      break;
      
    case MC406: 
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, MC406_REG_FR, 2, 1);  // ID=1, CMD=4 (Read Holding Reg), ModbusAdrr, Reg_read, Index_MasterReg)   FIRMWARE 1.30 EUROMAG
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, MC406_REG_TV, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate
      //modbus_construct(packet1, 1, READ_INPUT_REGISTERS, MC406_REG_FR, 2, 1);  // ID=1, CMD=4 (Read Input Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      //modbus_construct(packet2, 1, READ_INPUT_REGISTERS, MC406_REG_TV, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate             FIRMWARE 1.38  Euromag
      break;
    
    case SC200:                     //
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, SC200_REG_FR, 2, 1);  // ID=1, CMD=4 (Read HOLDING REG), ModbusAdrr, Reg_read, Index_MasterReg)
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, SC200_REG_TV, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate  
      break;
    
    case UM06:            // Modelo MaCrometer UM06 (ProComm Converter)
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, UM06_REG_FR, 2, 1);  // ID=1, CMD=3 (Read Input Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, UM06_REG_TV, 2, 3);  // ID=1, CMD=3 (Read Input Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      break;

    case L400:                // Modelo ProMag L400    Endress + Hauseer  if modAfor == "L400"
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, PGL400_REG_TV, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, PGL400_REG_FR, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate  
      break;
    
    case M1000:                // Modelo MoodMag M1000  Badger    
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, M1000_REG_TV, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg 
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, M1000_REG_FR, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate
      break;

    case M2000:                // Modelo MoodMag M2000  Badger    if modAfor == "MC2000" 
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, M2000_REG_TV, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg 
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, M2000_REG_FR, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate
      break;
   
    case ULTRATT:                // Modelo ULTRA TT  EQUYSIS       if modAfor == "ULTRATT"
        /*
        * Function Code: 0x03    Date: 0x07 0x5B 0xCD 0x15, mean 0X75BCD15 = 123456789
        * Register: 0x0200, #Points: 0x0003   Date: 0x2C 0x01, mean m³-Unit, One fractional part.
        * CRC16: 0x04 0x73    Positive flow volume is 12345678.9m³
        */
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, ULTRATT_REG_TV, 4, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg);  Positive flow volume High-16bit
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, ULTRATT_REG_FR, 4, 5);  // Packet 2 point to next Modbus regiser, in this case Flow rate;         Flow Rate High-16 bit
      break;

    case SITRANS6000:
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, SITRANS6000_REG_FR, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, SITRANS6000_REG_TV, 4, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate  
      break;
    
    case KRONE300:
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, KRONE300_REG_FR, 2, 1);  // ID=1, CMD=3 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, KRONE300_REG_TV, 4, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate  
      break;

    case ARKONMX2:                // Modelo MoodMag M2000  Badger    if modAfor == "MC2000" 
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, ARKONMX2_REG_FR, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg 
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, ARKONMX2_REG_TV, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate
      break;

    default:
      return false;
      break;
   }
   return true;
}

float Gasto(MODEL ModeloAforador){             // GET DATA FROM MODBUS PROTOCOL FOR MODELS SPECIFIED
  uint32_t tmp = 0x00;
  uint32_t trans = 0x00;
  uint8_t fraction;
  //delay(timeout);

  switch (ModeloAforador)
  {
    case MC608:                          // Little endian byte format
    case SC200:
//        Serial.println("_____________FLOW_RATE_______________");
        tmp = regs[2];
        trans = tmp << 16 | regs[1]; // update data to be written to arduino slave
        flowRate = Hex2Float(trans);
        /*Serial.println("Registros ");
        Serial.print(" -- Reg[1]: ");
        Serial.println(regs[1],HEX);
        Serial.print(" -- Reg [2]: ");
        Serial.println(regs[2],HEX);
        Serial.print(" -- Reg[3]: ");
        Serial.println(regs[3],HEX);
        Serial.print(" -- Reg [4]: ");
        Serial.println(regs[4],HEX); */
        //Serial.print("  --> Inst. measured fluid Flow Rate ** MC608 **  [m3/s] = ");
        //Serial.println(flowRate,4);
        //Serial.println();
        break;

    case MC406:       // Little endian byte format
        //Serial.println("_____________FLOW_RATE_______________");
        tmp = regs[2];
        trans = tmp << 16 | regs[1]; // update data to be written to arduino slave
        flowRate = Hex2Float(trans);
      //  Serial.println("Registros ");
      //  Serial.print(" -- Reg[1]: ");
      //  Serial.println(regs[1],HEX);
      //  Serial.print(" -- Reg [2]: ");
      //  Serial.println(regs[2],HEX);
      //  Serial.print("  --> Inst. measured fluid Flow Rate ** MC406 **  [m3/s] = ");
      //  Serial.println(flowRate,4);
      //  Serial.println();
        break;

    case UM06:        // Big endian byte format
    case L400:
    case ARKONMX2:
//        Serial.println("_____________FLOW_RATE_______________");
        tmp = regs[1];
        trans = tmp << 16 | regs[2]; // update data to be written to arduino slave
        flowRate = Hex2Float(trans);
//        Serial.println("Registros ");
//        Serial.print(" -- Reg[1]: ");
//        Serial.println(regs[1],HEX);
//        Serial.print(" -- Reg [2]: ");
//        Serial.println(regs[2],HEX);
//        Serial.print("  --> Inst. measured fluid Flow Rate ** MacCrometer UM06 **  [m3/s] = ");
//        Serial.println(flowRate,4);
//        Serial.println();
        break;
    
     case M1000:
      // Modbus Byte format:  Big endian  HIGH word send it first, then LOW Word
      // Serial.println("_____________FLOW_RATE_______________");
      tmp = regs[3];
      trans = (tmp << 16 | regs[4]) ; // update data to be written to arduino slave
      //Serial.print(" --> transition: ");
      //Serial.println(trans, HEX);
      flowRate = Hex2Float(trans);
      //Serial.println("Registros ");
      // Serial.print(" -- Reg[3]: ");
      // Serial.println(regs[3],HEX);
      // Serial.print(" -- Reg [4]: ");
      // Serial.println(regs[4],HEX);
      // Serial.print("  --> Inst. measured fluid Flow Rate ** ModMag M2000**  [m3/s] = ");
      // Serial.println(flowRate,4);
      break;

    case M2000:
      // Modbus Byte format:  Big endian  HIGH word send it first, then LOW Word
      // Serial.println("_____________FLOW_RATE_______________");
      tmp = regs[3];
      trans = (tmp << 16 | regs[4]) ; // update data to be written to arduino slave
      //Serial.print(" --> transition: ");
      //Serial.println(trans, HEX);
      flowRate = Hex2Float(trans);
      //Serial.println("Registros ");
      // Serial.print(" -- Reg[3]: ");
      // Serial.println(regs[3],HEX);
      // Serial.print(" -- Reg [4]: ");
      // Serial.println(regs[4],HEX);
      // Serial.print("  --> Inst. measured fluid Flow Rate ** ModMag M2000**  [m3/s] = ");
      // Serial.println(flowRate,4);
      break;
    
    case ULTRATT:                // Modelo ULTRA TT  EQUYSIS       if modAfor == "ULTRATT"
        /*
        * Function Code: 0x03    Date: 0x07 0x5B 0xCD 0x15, mean 0X75BCD15 = 123456789
        * Register: 0x0200, #Points: 0x0003   Date: 0x2C 0x01, mean m³-Unit, One fractional part.
        * CRC16: 0x04 0x73    Positive flow volume is 12345678.9m³
        * 
        */
        //Serial.println("_____________FLOW_RATE_______________");
        
        /*Serial.println("Registros ");
        Serial.print(" -- Reg[5] HIGH High Word: ");
        Serial.println(regs[5],HEX);
        Serial.print(" -- Reg [6] LOW High Word: ");
        Serial.println(regs[6],HEX);
        Serial.print(" -- Reg[7] HIGH Low Word: ");
        Serial.println(regs[7],HEX);
        */
        //uint8_t unidades = regs[7] >> 8;
        //Serial.println(unidades,HEX);
        /*
        if(unidades == 0x29) Serial.println(" L ");
        if(unidades == 0x2C) Serial.println(" m3 ");
        if(unidades == 0x32) Serial.println(" L/h ");
        if(unidades == 0x35) Serial.println(" m3/h ");
        else Serial.println("Unidad no reconocida");
             
        fraction = regs[8] | 0x0F ;           // Like if it was and(|) 0x0F
        if (fraction == 0x00) flowRate = (float) trans * 1;
        else if (fraction == 0x01) flowRate = (float) trans * 0.1;
        else if (fraction == 0x02) flowRate = (float) trans * 0.01;
        else if (fraction == 0x03) flowRate = (float) trans * 0.001;
        else if (fraction == 0x04) flowRate = (float) trans * 0.0001;
        else if (fraction == 0x05) flowRate = (float) trans * 0.00001;
        else if (fraction == 0x06) flowRate = (float) trans * 0.000001;
        else if (fraction == 0x07) flowRate = (float) trans * 0.0000001;                
        Serial.print(" -- Reg [8]: fraction");
        Serial.println(fraction,HEX);
        Serial.print("  --> Inst. measured fluid Flow Rate ** ULTRA TT - EQUYSIS ** = ");
        Serial.println(flowRate);
        */
        tmp = regs[5];
        trans = tmp << 16 | regs[6];  
        flowRate = (float) trans * 0.001;       // Like if it was and(|) 0x0F
        break;
      
     case SITRANS6000:
     case KRONE300:
        //Serial.println("_____________FLOW_RATE KRONE_______________");
        tmp = regs[1];
        trans = tmp << 16 | regs[2]; // update data to be written to arduino slave
        flowRate = Hex2Float(trans);
        /* Serial.println("Registros ");
        Serial.print(" -- Reg[1]: ");
        Serial.println(regs[1],HEX);
        Serial.print(" -- Reg [2]: ");
        Serial.println(regs[2],HEX);
        Serial.print("  --> Inst. measured fluid Flow Rate **SIEMENS MAG 6000 **  [m3/s] = ");
        Serial.println(flowRate,4);
        Serial.println();
        */
        break;

      default:
        break;
  }
  return flowRate;
}

float Volumen(MODEL ModeloAforador){             // GET DATA FROM MODBUS PROTOCOL FOR MODELS SPECIFIED
  uint32_t tmp = 0x00;
  uint32_t trans = 0x00;
  uint8_t fraction;
  //delay(100);
  
  switch (ModeloAforador)
  {
     case MC608:                          // Little endian byte format
     case SC200:
        tmp = regs[4];     // HIGH WORD for Total Volumen in m3
        trans = tmp << 16 | regs[3]; // update data to be written to arduino slave
        totalVol = Hex2Float(trans);    
       // Serial.println("____________TOTAL_VOLUMEN_______________");
        // Volumen total m3
        /*Serial.println("Volumen Total [m3]");
        Serial.print(" -- REG[5]: ");
        Serial.println(regs[5],HEX);
        Serial.print(" -- REG[6]: ");
        Serial.println(regs[6],HEX);
        Serial.print(" -- REG[7]: = ");
        Serial.println(regs[7],HEX);
        Serial.print(" -- REG[8]: ");
        Serial.println(regs[8],HEX); */
       // Serial.print(" --Total Volumen [m3] = ");
        //Serial.println(totalVol);
        //Serial.print("  --Total Volumen [m3] float = ");
        //Serial.println(totalVol,4);
        //Serial.println();
        break;

     case MC406:       // Little endian byte format
 /*
        float64_t trans2, tmp2, tmp, trans;
        char *str;
 // Read 4 Input Registers starting at 31673 (MODBUS ADDRESS 1162)
 // Serial.print("  --Inst. measured fluid Flow Rate [m3/h] = ");
 // Serial.println(flowRate);
 // Serial.print("  --Inst. measured fluid Flow Rate [m3/s] float = ");
 // Serial.println(flowRate/3600,5);
 // Serial.print("  --Inst. measured fluid Flow Rate [lts/s] float = ");
 // Serial.println(flowRate/36,5);       
        tmp2 = regs[6];
        trans2 = tmp2 << 48; 
  //    str = fp64_to_string( trans2, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        tmp2 = regs[5];
        tmp2 = tmp2 << 32;
  //    str = fp64_to_string( tmp2, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        trans = regs[4];
  //    trans = trans << 16;
  //    str = fp64_to_string( trans, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        tmp = regs[3];
        trans2 = trans2 + tmp2 + trans + tmp ;
        str = fp64_to_string( trans2, 17, 4);
        totalVol = fp64_ds(trans2);
        Hex2Float(totalVol);
//      Serial.println();   
//        Serial.println("_____________TOTAL_VOLUME_______________"); 
//        Serial.println("Registros ");
//        Serial.print(" -- Reg[6]: ");
//        Serial.println(regs[6],HEX);
//        Serial.print(" -- Reg [5]: ");
//        Serial.println(regs[5],HEX);
//        Serial.print(" -- Reg[4]: ");
//        Serial.println(regs[4],HEX);
//        Serial.print(" -- Reg [3]: ");
//        Serial.println(regs[3],HEX);
//        Serial.print(" String float64: ");
//        Serial.println(str);
//        Serial.print(" Total Vol: ");
//        Serial.println(totalVol,3);
*/
        tmp = regs[4];     // HIGH WORD for Total Volumen in m3
        trans = tmp << 16 | regs[3]; // update data to be written to arduino slave
        totalVol = Hex2Float(trans);    
        /*
         * Serial.println("____________TOTAL_VOLUMEN_______________");
         Volumen total m3
        Serial.println("Volumen Total [m3]");
        Serial.print(" -- MSB WORD Flow Rate: ");
        Serial.println(regs[4],HEX);
        Serial.print(" -- LSB WORD Flow Rate: ");
        Serial.println(regs[3],HEX);
        Serial.print(" --Total Volumen [m3] = ");
        Serial.println(totalVol);
        Serial.print("  --Total Volumen [m3] float = ");
        Serial.println(totalVol,4);
        Serial.println();
         * 
         */
 
        break;

    case UM06:                          // Little endian byte format
        tmp = regs[3];     // HIGH WORD for Total Volumen in m3
        trans = tmp << 16 | regs[4]; // update data to be written to arduino slave
        totalVol = long (trans);    
//        Serial.println("____________TOTAL_VOLUMEN_______________");
        // Volumen total m3
//        Serial.println("Volumen Total [m3]");
//        Serial.print(" -- MSB WORD Flow Rate: ");
//        Serial.println(regs[3],HEX);
//        Serial.print(" -- LSB WORD Flow Rate: ");
//        Serial.println(regs[4],HEX);
//        Serial.print(" --Total Volumen [m3] = ");
//        Serial.println(totalVol);
//        Serial.print("  --Total Volumen [m3] float = ");
//        Serial.println(totalVol,4);
 //       Serial.println();
        break;

    case L400:
    case ARKONMX2:
      tmp = regs[3];     // HIGH WORD for Total Volumen in m3
      trans = tmp << 16 | regs[4]; // update data to be written to arduino slave
      totalVol = Hex2Float(trans);    
      break;

    case M1000:                // Modelo MoodMag M1000  Badger Meter   
        tmp = regs[1];
        trans = tmp  << 16 | regs[2];
        totalVol = Hex2Float(trans);
        //Serial.println("_____________TOTAL_VOLUME_______________"); 
        //Serial.print(" --> transition: ");
        //Serial.println(trans, HEX);
        // Serial.println("Registros ");                 
        // Serial.print(" -- Reg[1]: ");
        // Serial.println(regs[1],HEX);
        // Serial.print(" -- Reg [2]: ");
        // Serial.println(regs[2],HEX);
        // Serial.print(" Total Vol: ");
        // Serial.println(totalVol,4);
        // delay(1000);
         break;

    case M2000:                // Modelo MoodMag M2000  Badger    
        tmp = regs[1];
        trans = tmp  << 16 | regs[2];
        totalVol = Hex2Float(trans);
        //Serial.println("_____________TOTAL_VOLUME_______________"); 
        //Serial.print(" --> transition: ");
        //Serial.println(trans, HEX);
        // Serial.println("Registros ");                 
        // Serial.print(" -- Reg[1]: ");
        // Serial.println(regs[1],HEX);
        // Serial.print(" -- Reg [2]: ");
        // Serial.println(regs[2],HEX);
        // Serial.print(" Total Vol: ");
        // Serial.println(totalVol,4);
        // delay(1000);
         break;

    case ULTRATT:
      tmp = regs[1] << 16 ; 
      trans = tmp | regs[2];
      /*Serial.println("Registros ");
      Serial.print(" -- Reg[1] High Word: ");
      Serial.println(regs[1],HEX);
      Serial.print(" -- Reg [2]Low Word: ");
      Serial.println(regs[2],HEX);
      Serial.print(" -- Reg[3] Fractional Word: ");
      Serial.println(regs[3],HEX);
      Serial.print(" -- Reg[3] Units :");
      */
      /*
      unidades = regs[3] >> 8;
      Serial.println(unidades,HEX);
      if(unidades == 0x29) Serial.println(F(" L "));
      if(unidades == 0x2C) Serial.println(F(" m3 "));
      if(unidades == 0x32) Serial.println(F(" L/h "));
      if(unidades == 0x35) Serial.println(F(" m3/h "));
      else Serial.println(F("Unidad no reconocida")); 
      */ 
      fraction = regs[3];           // Like if it was and(|) 0x0F
      //Serial.print(" -- Reg [3] fraction:");
      //Serial.println(fraction,HEX);
      if (fraction == 0x00) totalVol = (float) trans * 1;
      else if (fraction == 0x01) totalVol = (float) trans * 0.1;
      else if (fraction == 0x02) totalVol = (float) trans * 0.01;
      else if (fraction == 0x03) totalVol = (float) trans * 0.001;
      else if (fraction == 0x04) totalVol = (float) trans * 0.0001;
      /*
      else if (fraction == 0x05) totalVol = (float) trans * 0.00001;
      else if (fraction == 0x06) totalVol = (float) trans * 0.000001;
      else if (fraction == 0x06) totalVol = (float) trans * 0.0000001;
      else if (fraction == 0x07) totalVol = (float) trans * 0.00000001;               
      */
      //  Serial.println(totalVol);
      //  delay(1000);
      break;

    case SITRANS6000:
    case KRONE300:            // __________TOTAL_VOLUME KRONE 300___________ 
  
        float64_t trans2_64, tmp2_64, tmp_64, trans_64;
        //char *str;
        tmp2_64 = regs[3];
        trans2_64 = tmp2_64 << 48; 
  //    str = fp64_to_string( trans2, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        tmp2_64 = regs[4];
        tmp2_64 = tmp2_64 << 32;
  //    str = fp64_to_string( tmp2_64, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        trans_64 = regs[5];
        trans_64 = trans_64 << 16;
  //    str = fp64_to_string( trans64, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        tmp_64 = regs[6];
        trans2_64 = trans2_64 + tmp2_64 + trans_64 + tmp_64 ;
  //    str = fp64_to_string( trans2_64, 17, 4);
        totalVol = fp64_ds(trans2_64);
   
        Hex2Float(totalVol);
        /*Serial.println();   
        Serial.println("_____________TOTAL_VOLUME_______________"); 
        Serial.println("Registros ");
        Serial.print(" -- Reg[3]: ");
        Serial.println(regs[3],HEX);
        Serial.print(" -- Reg [4]: ");
        Serial.println(regs[4],HEX);
        Serial.print(" -- Reg[5]: ");
        Serial.println(regs[5],HEX);
        Serial.print(" -- Reg [6]: ");
        Serial.println(regs[6],HEX);
        */
  //      Serial.print(" String float64: ");
  //      Serial.println(str);
        //Serial.print(" Total Vol: ");
        //Serial.println(totalVol,3);
      break;

    

    default:
      break;
  }
  return totalVol;
}

void configInicial(){
  counter = 1; 

  Serial.println(F("Iniciando Modbus RS-485... "));
  Serial.print(F("modelo: "));
  Serial.println(conf.modeloAforador);
  
  if(conf.modeloAforador == 608)
    ModeloAforador = MC608;
    
  else if(conf.modeloAforador == 406)
    ModeloAforador = MC406;  
    
  else if(conf.modeloAforador == 220)
    ModeloAforador = SC200;  
    
  else if(conf.modeloAforador == 101)
    ModeloAforador = UM06;
    
  else if(conf.modeloAforador == 400)
    ModeloAforador = L400;

  else if(conf.modeloAforador == 100)
    ModeloAforador = M1000;

  else if(conf.modeloAforador == 200)
    ModeloAforador = M2000;

  else if (conf.modeloAforador == 500)
    ModeloAforador = ULTRATT;  
    
  else if (conf.modeloAforador == 600)
    ModeloAforador = SITRANS6000;  

  else if (conf.modeloAforador == 300)
    ModeloAforador = KRONE300;  

  else if (conf.modeloAforador == 700)
    ModeloAforador = ARKONMX2;  

  else 
    Serial.println(F("Modelo de aforador registro MODBUS desconocido"));
      
    //Serial.print(F("Numero de Modelo (enum):"));
    //Serial.println(ModeloAforador);

  if(!setModbus(ModeloAforador)) Serial.println(F("Falla configuracion MODBUS RTU."));
  else Serial.println(F("Configuracion MODBUS exitosa."));

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialize the Modbus Finite State Machine at Serial 3 for MEGA Arduino (TX = 14, RX = 15)   
  modbus_configure(&Serial3, conf.bauds, SERIAL_8N1, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
  obtenNSUT(false);
  saveConfiguration(archConf, conf);
  loadConfiguration(archConf, conf);
//     Serial.println(conf.loginUsername); 
//     Serial.println(conf.loginPassword); 
//    Serial.print("");
//  Serial.println(F("Configuracion iniciada . . .")); 
    
  arranque = true;  
  screenLogin = true;
}

bool configSD() {
  // Initialize SD library
  String jsonResult="";
  pinMode(CSE, OUTPUT);         // Esto es necesario aunque creas que no lo usas.
  digitalWrite(CSE, HIGH);      // ESTO DESACTIVA EL ESCLAVO SPI DEL ETHERNET, CON EL CHIPSELECT
  delay(timeout);
  Serial.print(F("INICIO MEM microSD... "));
  
  if(!SD.begin(CS)){
     Serial.println(F("#MSSG¡ERROR! FALLA DE LECTURA DE CONFIGURACION"));
  }
  while (!SD.begin(CS)) {
    Serial.println(F("FALLA AL INICIAR MEM SD"));
      //delay(timeout);
    }
    // Should load default config if run for the first time
    //Serial.println(F("\nCargando configuracion desde archivo..."));
    loadConfiguration(archConf, conf);
    // Dump config file
    if (SD.exists(archConf)) {
      Serial.println(F("! E X I T O S O !")); 
    } else {
      Serial.println(F("! ERROR DE LECTURA ARCHIVO CONFIGURACION . . . !"));
    }
    if(!SD.exists(autologg)) {
       Serial.println(F("¡ NO HA INICIADO POR 1a VEZ . . . !"));
    } else {
      configInicial();
      jsonResult="";
      //saveConfiguration(archConf, conf); 
      loadConfiguration(archConf, conf);
      serializeJson(JsonDoc, jsonResult);
      Serial.println("#READ" + jsonResult );
      //tiempoActual(true);
    }

    if(!SD.exists(folderDatosSD)) {
      if(!SD.mkdir(folderDatosSD)) {
        Serial.println(F("FALLA AL CREAR FOLDER DE DATOS"));
      }
    }
}

void setTime(){
  String fecha = "", hora = "";
  String tiempo = getData();              // Ingresar tiempo del sistema con formato AAAAMMDD hhmmss 
  fecha = tiempo.substring(0,8); 
  hora = tiempo.substring(8,14);
  cnc_fecha = fecha.substring(0,4);
  AA = (uint16_t) cnc_fecha.toInt();
  cnc_fecha = fecha.substring(4,6);
  MM = (uint16_t) cnc_fecha.toInt();
  cnc_fecha = fecha.substring(6,8);
  DD = (uint16_t) cnc_fecha.toInt();
  cnc_hour = hora.substring(0,2);
  hh = (uint16_t) cnc_hour.toInt();
  cnc_hour = hora.substring(2,4);
  mm = (uint16_t) cnc_hour.toInt();
  cnc_hour = hora.substring(4,6);
  ss = (uint16_t) cnc_hour.toInt();
  rtc.adjust(DateTime(AA, MM, DD, hh, mm, ss));
  Serial.println(F("---------------------------  R E L O J  D E L  S I S T E M A ------------------------------"));
  Serial.print(F("\t\tA&o: ")); Serial.print(AA);
  Serial.print(F(" Mes: ")); Serial.print(MM);
  Serial.print(F(" Dia: ")); Serial.print(DD);
  Serial.print(F("\t\t\tHora: ")); Serial.print(hh);
  Serial.print(F(" Min: ")); Serial.print(mm);
  Serial.print(F(" Seg: ")); Serial.println(ss);
  Serial.println(F("------------------------------------------------------------------------------------------"));  
  cnc_fecha = fecha;
}
// Funciones convertir HEX to Float y viceversa
float Hex2Float(uint32_t x)
{
  return (*(float*)&x);
}

uint32_t Float2Hex(float y) {
  return (*(uint32_t*)&y);
}

// OPCIONES INGRESO DATOS
void opciones() {

  if(stringComplete)
  {
    stringComplete = false;
    getCommand();
    if(commandString.equals(model_login))       { login(); }  
    else if(commandString.equals(cmd_stime))    {setTime(); }
    else if(commandString.equals(cmd_gtime))    { tiempoActual(true);}
    else if(commandString.equals(cmd_send))     { envioFTP(true, true); envioWebPost();}
    else if(commandString.equals(cmd_save))     { updateData(); }
    else if(commandString.equals(cmd_sign))     { signalLevel(); }
    else if(commandString.equals(cmd_wmac))     { macEthernet(true); }
    else if(commandString.equals(cmd_vlue))     {conModbus(flowRate, totalVol, true);}
    else if(commandString.equals(cmd_peth))     {parametrosRed();}
//    else if(commandString.equals(cmd_rmod))     { resetModem(); }
    else if(commandString.equals(cmd_rsys))     { resetFunc(); }
    else if (commandString.equals(cmd_rmem))    {
      Serial.print(F("#RMEM"));
      root = SD.open("/datos");
      printDirectory(root, 0);
    } 
    else if (commandString.equals(cmd_dfil)){
      String replyFile = "";
      dumpFile(replyFile = getData());
      delay(10);
    }
    else if(commandString.equals(cmd_envioWebPost))     {conf.macAddres = macEthernet(false); envioWebPost(); }
    else if(commandString.equals(cmd_obtenNSUT))     { obtenNSUT(true); }
    else  if (commandString.equals(cmd_ntpTime))    {obtenNTP();}
    {
      /* code */
    }
    
    {
      /* code */
    }
    
    inputString = "";
  } 
}

void getCommand()
{
  if(inputString.length()>0)
    commandString = inputString.substring(1,5);
}

String getData()
{
  String value = inputString.substring(5,inputString.length()-1);
  Serial.println(value);
  return value;
}

void login(void) {
  char fechaLog[64];
  DateTime now = rtc.now();
   
  String jsonResult="";
  String json = getData();
  // FORMATO COMANDO SERIAL JSON
  //#MDL1{"username":"admin","password":"admin","model":406, "bauds": 9600, "comm": 0}         comm: 1 (CELULAR) | comm :2  (ethernet)  |  comm: 3 (satelital)
 
  StaticJsonDocument<256> doc;
  deserializeJson(doc, json);

  String username = doc["username"];
  String password = doc["password"];
  //conf.eth = doc["eth"];

  /* Serial.println("------del JSON de la app");
  Serial.println(username);  
  Serial.println(password);    */
  
  conf.modeloAforador  = doc["model"];
  conf.bauds = doc["bauds"];
  conf.comm = doc["comm"];

  conf.loginUsername = username;
  conf.loginPassword = password;
  //if ((username == conf.loginUsername) and (password == conf.loginPassword)) {  
  if ((username == "swplus") and (password == "swplus")) {  

      Serial.println(F("#LOGG1"));
      jsonResult="";
      saveConfiguration(archConf, conf); 
      loadConfiguration(archConf, conf);
      serializeJson(JsonDoc, jsonResult);
      Serial.println("#READ" + jsonResult );

      // CREAR ARCHIVO AUTOLOGI.LOG
      sprintf(fechaLog, "%02d/%02d/%02d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());   // formato ddmmAAA hh:mm:ss
      cnc_fecha = fechaLog;
      if(!SD.exists(autologg)){
        autoLogg = SD.open(autologg, FILE_WRITE);
        // Open file for writing
        if (!autoLogg) 
          Serial.println(F("FALLA AL CREAR EL ARCHIVO DE AUTOLOGGEO"));
        
        autoLogg.print("LOGGEADO POR PRIMERA VEZ:\t");
        autoLogg.print(cnc_fecha);
        autoLogg.close();
        tiempoActual(true);
      } else {
        autoLogg = SD.open(autologg, FILE_WRITE);
        // Open file for writing
        if (!autoLogg) 
          Serial.println(F("FALLA AL CREAR EL ARCHIVO DE AUTOLOGGEO"));
        
        autoLogg.print("SISTEMA REINICIADO:\t\t");
        autoLogg.println(cnc_fecha);
        autoLogg.close(); 
        tiempoActual(true);
      }
      return;
  } else if ((username == "admin") and (password == "admin")) {
      Serial.println(F("#LOGG2"));
    
      jsonResult="";
      saveConfiguration(archConf, conf); 
      loadConfiguration(archConf, conf);
      serializeJson(JsonDoc, jsonResult);
      Serial.println("#READ" + jsonResult );
      tiempoActual(true);
  } else if ((username == conf.uvaUsername) and (password == conf.uvaPassword)) {
      Serial.println(F("#LOGG3"));
    
      jsonResult="";
      saveConfiguration(archConf, conf); 
      loadConfiguration(archConf, conf);
      serializeJson(JsonDoc, jsonResult);
      Serial.println("#READ" + jsonResult );
      tiempoActual(true);

  } else if ((username == "user") and (password == "pass")) {
      Serial.println(F("#LOGG4"));
    
      jsonResult="";
      saveConfiguration(archConf, conf); 
      loadConfiguration(archConf, conf);
      serializeJson(JsonDoc, jsonResult);
      Serial.println("#READ" + jsonResult );
      tiempoActual(true);
  } else {
    Serial.println(F("#LOGG0"));
    Serial.println(F("#MSSG USUARIO INVALIDO"));
  }
  
}

void updateData() {
  // Max 420 caracteres dentro del Json 
  // #SAVE{"med":{"tip":"M","nsm":"AAAAAAAA-AAA","nsue":"BBBBBBBB-BBB","mod":101,"baud":57600,"dn":200},"sys":{"nsut":"1234567-1234","crr":2},"ftp":{"srv":"infopro.swplus.com.mx","usr":"swplus","pwd":"Sw+Tecnologias","dir":"/Pruebas/2023/noviembre","port":21,"h_en":11,"m_en":40},"conc":{"rfc":"STE130213445","id":"TEDDYBEAR BJ CDMX","lat":"19.123456","lon":"-99.654321"},"uva":{"id":"UVA0000","usr":"aguamex","pwd":"aguamex23"}}
  Serial.flush();
  delay(timeout);
  String jsonResult="";
  String json = getData();

  StaticJsonDocument<768> doc;
  
  DeserializationError err = deserializeJson(doc, json);

  if (err) {
    //Si hay un error mandar mendaje a infoPro
      Serial.print(F("#MSSGupdateData "));
      Serial.println(err.f_str());
      return;
  } else {

    //conf.loginUsername = "admin";
    //conf.loginPassword = "admin";

    JsonObject med = doc["med"];
      conf.tipo = med["tip"].as<String>();
      conf.modeloAforador = med["mod"];    
      conf.nsm = med["nsm"].as<String>();
      conf.nsue = med["nsue"].as<String>();
      conf.bauds = med["baud"];
      conf.dn = med["dn"];                                           

    JsonObject sys = doc["sys"];
      conf.nsut = sys["nsut"].as<String>();
      //conf.carrier = sys["crr"];

    JsonObject ftp = doc["ftp"]; 
      conf.ftpServer=ftp["srv"].as<String>();
      conf.ftpUsername=ftp["usr"].as<String>();
      conf.ftpPassword=ftp["pwd"].as<String>();
      conf.ftpFolder = ftp["dir"].as<String>();
      conf.port = ftp["port"] | 21;
      conf.hhEnvio = ftp["h_en"] | 0;
      conf.mmEnvio = ftp["m_en"] | 1;

    JsonObject conc = doc["conc"];
      //conf.cncUsername = conc["usr"].as<String>();
      //conf.cncPassword = conc["pwd"].as<String>();
      conf.rfc = conc["rfc"].as<String>();
      conf.tagID = conc["id"].as<String>();
      conf.latitud = conc["lat"].as<String>();
      conf.longitud = conc["lon"].as<String>();
      
    JsonObject uva = doc["uva"];
      conf.uvaID = uva["id"].as<String>();
      conf.uvaUsername = uva["usr"].as<String>();
      conf.uvaPassword = uva["pwd"].as<String>(); 
      
    if(saveConfiguration(archConf, conf)) {
      loadConfiguration(archConf, conf);
      serializeJson(doc, jsonResult);
      Serial.println("#READ" + jsonResult );
      Serial.println(F("#MSSGCONFIGURACION ACTUALIZADA")); 

    } 
  } 
}

// Write to a file in the SD card
bool writeToFile(char * fileName, char * content) {
   myFile = SD.open(fileName, FILE_WRITE);

  // If the file opened successfully, write to it
  if (myFile) {
    Serial.print(F("Writing to file..."));
    myFile.println(content); // Write the desired content onto the file
    myFile.close(); // Close the file
    Serial.println(F(" done!"));
  }
  else {
    Serial.println(F("Error opening file!"));
    return false;
  }
  
  return true;
}

// *************** FUNCIONES JSON, ARCHIVOS EN SD **********************
// Loads the configuration from a file

void loadConfiguration(char *archConf, Clasif &conf) {
  
  // Open file for reading
  pinMode(CSE, OUTPUT);         // Esto es necesario aunque creas que no lo usas.
  digitalWrite(CSE, HIGH);      // ESTO DESACTIVA EL ESCLAVO SPI DEL ETHERNET, CON EL CHIPSELECT
  delay(timeout);
  myFile = SD.open(archConf);
 
  // Deserialize the JSON document
  //DeserializationError error = deserializeJson(doc, input, MAX_INPUT_LENGTH);
  DeserializationError error = deserializeJson(JsonDoc, myFile);
  
  
  if (error){
    Serial.print(F("Error "));
    Serial.println(error.f_str());
    return;
  } else {             // Copy values from the JsonDocument to the Config
    conf.loginUsername = JsonDoc["login"]["username"].as<String>();        
    conf.loginPassword = JsonDoc["login"]["password"].as<String>();
      
    JsonObject med = JsonDoc["med"];
      conf.tipo = med["tip"].as<String>();
      conf.modeloAforador = med["mod"]; 
      conf.bauds = med["baud"]; 
      conf.nsm = med["nsm"].as<String>();
      conf.nsue = med["nsue"].as<String>();
      conf.dn = med["dn"];                                           

    JsonObject sys = JsonDoc["sys"];
      conf.nsut = sys["nsut"].as<String>();
      //conf.carrier = sys["crr"];
    
    JsonObject ftp = JsonDoc["ftp"]; 
      conf.ftpServer=ftp["srv"].as<String>();
      conf.ftpUsername=ftp["usr"].as<String>();
      conf.ftpPassword=ftp["pwd"].as<String>();
      conf.ftpFolder = ftp["dir"].as<String>();
      conf.port = ftp["port"] | 21;
      conf.hhEnvio = ftp["h_en"] | 0;
      conf.mmEnvio = ftp["m_en"] | 1;

    JsonObject conc = JsonDoc["conc"];
      //conf.cncUsername = conc["usr"].as<String>();
      //conf.cncPassword = conc["pwd"].as<String>();
      conf.rfc = conc["rfc"].as<String>();
      conf.tagID = conc["id"].as<String>();
      conf.latitud = conc["lat"].as<String>();
      conf.longitud = conc["lon"].as<String>();

    JsonObject uva = JsonDoc["uva"];
      conf.uvaID = uva["id"].as<String>();
      conf.uvaUsername = uva["usr"].as<String>();
      conf.uvaPassword = uva["pwd"].as<String>(); 

    myFile.close();
  }
}

// Prints the content of a file to the Serial
String readFile(const char *filename) {
  String Buffer;
  // Open file for reading
  myFile = SD.open(filename);
  if (!myFile) {
    Serial.println(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  while (myFile.available()) {
    Buffer += (char)myFile.read();
  }
  // Close the file
  myFile.close();
  return Buffer;
}


// Saves the configuration to a file in SD

bool saveConfiguration(char * archConf, Clasif &conf) {
  
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
 // Set the values in the document
  JsonObject login = JsonDoc.createNestedObject("login");
   login["username"] = conf.loginUsername;
   login["password"] = conf.loginPassword;
  
  JsonObject med = JsonDoc.createNestedObject("med");
    med["tip"] = conf.tipo;
    med["nsm"] = conf.nsm;
    med["nsue"] = conf.nsue;
    med["mod"] = conf.modeloAforador;
    med["baud"] = conf.bauds;
    med["dn"] = conf.dn;

  JsonObject sys = JsonDoc.createNestedObject("sys");
    sys["nsut"] = conf.nsut;
    //sys["crr"] = conf.carrier;
   
  JsonObject ftp = JsonDoc.createNestedObject("ftp");
    ftp["srv"] = conf.ftpServer;
    ftp["usr"] = conf.ftpUsername;
    ftp["pwd"] = conf.ftpPassword;
    ftp["dir"] = conf.ftpFolder;
    ftp["port"] = conf.port;
    ftp["h_en"] = conf.hhEnvio;
    ftp["m_en"] = conf.mmEnvio;
   
  JsonObject conc = JsonDoc.createNestedObject("conc");
    //conc["usr"] = conf.cncUsername;
    //conc["pwd"] = conf.cncPassword;
    conc["rfc"] = conf.rfc;
    conc["id"] = conf.tagID;
    conc["lat"] = conf.latitud;
    conc["lon"] = conf.longitud;

  JsonObject uva = JsonDoc.createNestedObject("uva");
    uva["id"] = conf.uvaID;
    uva["usr"] = conf.uvaUsername;
    uva["pwd"] = conf.uvaPassword; 

  /*
    Una vez que se termina de generar el objeto JSON ahora si se procede a validarlo para asegurar que se formo correctamente
    Primer habria que serializar el objecto JsonDoc, pero no a un archivo, sino a alguna otra variable y ya que se tenga en una variable la cadena JSON lo validamos deserializandolo
  */
  String tempFile;

  if (serializeJson(JsonDoc, tempFile) == 0) {
    //Si hay un error mandar mendaje a infoPro
      Serial.println(F("#MSSGERROR AL GUARDAR EN MEMORIA"));
      //Serial.println(err.f_str());
      return false;
 } else {
    //Todo OK, proceder a borrar el archivo de la SD y crearlo con la nueva informacion
    // Delete existing file, otherwise the configuration is appended to the file
    pinMode(CSE, OUTPUT);         // Esto es necesario aunque creas que no lo usas.
    digitalWrite(CSE, HIGH);      // ESTO DESACTIVA EL ESCLAVO SPI DEL ETHERNET, CON EL CHIPSELECT
    delay(timeout);
    SD.remove(archConf);
    myFile = SD.open(archConf, FILE_WRITE);
    if (SD.exists(archConf)) {
      // Open file for writing
      if (!myFile) {
        Serial.println(F("Falla al crear el archivo"));
        return;
      }
    } else {
      Serial.println(F("No existe archivo"));
    }
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  // Serialize JSON to file
    if (serializeJson(JsonDoc, myFile) == 0) {
      Serial.println(F("Failed to write to file"));
    }

    // Close the file
    myFile.close();
    return true;
  //
 }
}

void saveDataSD (char *archivoSD, char * datos) {
  pinMode(CSE, OUTPUT);         // Esto es necesario aunque creas que no lo usas.
  digitalWrite(CSE, HIGH);      // ESTO DESACTIVA EL ESCLAVO SPI DEL ETHERNET, CON EL CHIPSELECT
  delay(timeout);
  SD.remove(archivoSD);
  myFile = SD.open(archivoSD, FILE_WRITE);

  if (SD.exists(archivoSD)) {
    // Open file for writing 
   
    if (!myFile) {
      Serial.println(F("Falla al crear el archivo"));
      return;
    }
    myFile.print(datos);
    //myFile.print("\n\r");
    myFile.print(",");
    Serial.println(F("! Escrito en memoria SD exitosamente !")); 
  }
  else{
     Serial.println(F("No existe archivo"));
  }
  // Close the file
  myFile.close();
}

// Read the contents of a file in the SD card
String readFromFile(char *fileName, size_t *fileSize) {
  char incoming = "";
  String salida = "";
  
  pinMode(CSE, OUTPUT);         // Esto es necesario aunque creas que no lo usas.
  digitalWrite(CSE, HIGH);      // ESTO DESACTIVA EL ESCLAVO SPI DEL ETHERNET, CON EL CHIPSELECT
  delay(timeout);
  myFile = SD.open(fileName);
  *fileSize = myFile.size();

  if (myFile) {
    // Read from the file until there's nothing else in it
    while (myFile.available()) {
      //strcat(contentBuff, myFile.read());
      incoming = myFile.read();
      if (incoming != '\n')
        salida += incoming;
    }
    myFile.close();  // Close the file; only 1 can be open at a time
  } else {
    Serial.println(F("Error opening file!"));
  }

  return salida;
}


// *************** FUNCIONES MODBUS **********************
boolean conModbus(float fR, float tV, bool modbusMsgApp){
  String m="";
  Serial.flush();
  Serial3.flush();

  //if((fR == fRateC and tV == tVolC) and (fR !=0)) controlC++;
  if(fR == fRateC and tV == tVolC) controlC++;
  else controlC = 0;

  if ( (fR == 0 and tV == 0) or ((controlC > 50) and (fR !=0) )) {
    estado == false;
    m="#VLUE0;0;0";
    error = "004";
    if(modbusMsgApp) Serial.println(m);
   
  }
  else {
    estado = true;
    if(error == "004") error = "000";
    m="#VLUE"+String(fR,1)+";"+String(tV,3)+";1";
    if(modbusMsgApp) Serial.println(m);
  }
  /*
  Serial.print("Valor de control:");
  Serial.println(controlC);
  */
  if(((controlC > 800) and (fR != 0)) or ((controlC > 31517) and (fR == 0))) resetFunc(); //call reset system;                 // Restart System  en ECOLOGIA Y REUSO planta CONFETEX (mezclilla)
  //if(((controlC > 800) and (fR !=0)) or ((controlC > 800) and (fR ==0))) resetFunc(); //call reset system;                 // Restart System
  
  tVolC = tV;
  fRateC = fR;
  return estado;

}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print(F("\t"));
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println(F("/datos"));
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print(F(" , "));
      Serial.print(entry.size(), DEC);
      Serial.print(F(" ; "));     // Lista en una sola línea
      //Serial.print("\n");     //Lista en lineas independientes
    }
    entry.close();
  }
  Serial.println(F("\r\n"));     //Lista en lineas independientes
}

void dumpFile(String archivo) {
  char  buffArch[32]="";
  static String temp="";
  String contenido;
  pinMode(CSE, OUTPUT);         // Esto es necesario aunque creas que no lo usas.
  digitalWrite(CSE, HIGH);      // ESTO DESACTIVA EL ESCLAVO SPI DEL ETHERNET, CON EL CHIPSELECT
  delay(timeout);
  Serial.print("#FILE" + archivo);
  temp = "datos/" + archivo;
  //temp = "/" + archivo;
  temp.toCharArray(buffArch, 13 + 6);
  //temp.toCharArray(buffArch, 13 + 1);
  File dataFile = SD.open(buffArch);
  //Serial.print("Array:");
  //Serial.println(buffArch);
  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      //contenido = dataFile.read();
      //Serial.print(contenido);
      Serial.write(dataFile.read());
      //dataFile.print(" ; ");
    }
    Serial.println(F("\n\r"));
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(F("Error opening File"));
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////FUNCION PARA CONECTARSE A EL FTP //////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void connectToFTP(void) {

  //const char* ftpServer = "infopro.swplus.com.mx";//conf.ftpServer.c_str();
  //const int ftpPort = 21;//conf.port;  // Puerto FTP estándar
  String responseFTP;
  String comparaFTP; 
  if (client.connect(conf.ftpServer.c_str(), conf.port)) {  
    // Espera la respuesta del servidor
    while (!client.available()) {
      delay(1);
    }
    // Leer, guardar y mostrar la respuesta del servidor
    while (client.available()) {
      char c = client.read();
      responseFTP += c;
      if (c == '\n') {
      break;
      }
    }
    Serial.print(responseFTP); ////////////////////////RESPONDE UN 220 SI ES CORRECTO
  }
/////////////Guarda y compara la respuesta//////////////////////////
  if(responseFTP.length() >= 3)
    comparaFTP = responseFTP.substring(0, 3);
  if(!comparaFTP.equals("220")){
    Serial.println(F("VERIFICA LA IP FTP"));
    ipOK = false;
  } else {
    Serial.println(F("IP DE FTP CORRECTO")); 
    ipOK = true;
    loggFTP();
  }   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////FUNCION PARA AUTENTIFICAR EL FTP MEDIANTE LAS CREDENCIALES/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loggFTP(void) 
{
    ////////////////////CREDENCIALES///////////////
  
  String responseFTP;
  String comparaFTP; 
    ///////////////////DIRECTORIO//////////////////

    // Enviar el comando USER para proporcionar el nombre de usuario
  client.println("USER " + conf.ftpUsername);
    // Dar tiempo para que el servidor responda
  while (!client.available()) {
    delay(1); 
  }  
  // Leer y mostrar la respuesta del servidor
  while (client.available()) {
    Serial.write(client.read()); ///////////////////////REGRESA UN CODIGO 331 FTP SI ES CORRECTO 
  }

  // Enviar el comando PASS para proporcionar la contraseña
  client.println("PASS " + conf.ftpPassword );
  // Espera la respuesta del servidor
  while (!client.available()) {
    delay(1);
  }
  // Leer, guardar y mostrar la respuesta del servidor
  while (client.available()) {
    char c = client.read();
    responseFTP += c;
    if (c == '\n') {
      break;
    }
  }
  Serial.print(responseFTP); ////////////////////////RESPONDE UN 230 SI ES CORRECTO, SI TRAE 530 NO SE LOGUEA
  /////////////Guarda y compara la respuesta//////////////////////////
  if(responseFTP.length() >= 3){
    comparaFTP = responseFTP.substring(0, 3);
  //   Serial.println(comparaFTP);
  }
  if(!comparaFTP.equals("230")){
    Serial.println(F("LOGUEO INCORRECTO AL SERVIDOR FTP")); 
    passOK = false;
  } else {
  Serial.println(F("LOGUEO CORRECTO AL SERVIDOR FTP"));
  passOK = true;
  optsFTP();
  dirFTP();
  }    
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////FUNCION PARA FORMATO DE CODIFICACIÓN DE CARACTERES/////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void optsFTP(void) 
{
  // Enviar el comando OPTS 
  client.println("OPTS UTF8 ON");
  // Dar tiempo para que el servidor responda
  while (!client.available()) {
    delay(1); 
  }
  // Leer y mostrar la respuesta del servidor
  while (client.available()) {
    Serial.write(client.read()); ///////////////////////REGRESA UN CODIGO 200 FTP,
  }
  Serial.println(F("CODIFICACIÓN UTF8 ACTIVADA"));
}

///////////////////////////////FUNCION PARA DIRECTORIO DE TRABAJO/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void dirFTP(void) 
{
  //const char* ftpDir = "/Pruebas/2023/noviembre";//conf.ftpFolder.c_str(); 
  //Serial.println(ftpDir);
  String responseFTP;
  String comparaFTP; 
  ///////////////////DIRECTORIO//////////////////
  // Enviar el comando CWD 
  client.println("CWD " + conf.ftpFolder);
  // Dar tiempo para que el servidor responda
  while (!client.available()) {
    delay(1); 
  }
  // Leer, guardar y mostrar la respuesta del servidor
  while (client.available()) {
    char c = client.read();
    responseFTP += c;
    if (c == '\n') {
      break;
    }
  }
  Serial.print(responseFTP);///////////////////////REGRESA UN CODIGO 250 FTP
  /////////////Guarda y compara la respuesta//////////////////////////
  if(responseFTP.length() >= 3){
    comparaFTP = responseFTP.substring(0, 3);
  }
  if(!comparaFTP.equals("250")){
    Serial.println(F("NO EXISTE EL DIRECTORIO EN EL FTP"));
    dirOK = false;   
  } else {
    Serial.println(F("DIRECTORIO DE TRABAJO LISTO"));
    dirOK = true;
    // Enviar el comando PWD 
    client.println("PWD");
    // Dar tiempo para que el servidor responda
    while (!client.available()) {
      delay(1); 
    }
    // Leer y mostrar la respuesta del servidor
    while (client.available()) {
      Serial.write(client.read()); ///////////////////////REGRESA UN CODIGO 257 FTP,
    }
    asciiFTP();
    storFTP();
  } 
}

///////////////////////////////FUNCION PARA MODO DE TRANSFERENCIA ASCII.  ////////////////////////////////////////////////////////

void asciiFTP(void) 
{
  // Enviar el comando TYPE A 
  client.println("TYPE A");
  // Dar tiempo para que el servidor responda
  while (!client.available()) {
    delay(1); 
  }
  // Leer y mostrar la respuesta del servidor
  while (client.available()) {
    Serial.write(client.read()); ///////////////////////REGRESA UN CODIGO 200 FTP,
  }
  Serial.println(F("MODO DE TRANSFERENCIA ASCII ACTIVADO"));
  // Enviar el comando SYST 
  client.println("SYST");
  // Dar tiempo para que el servidor responda
  while (!client.available()) {
    delay(1); 
    }
  // Leer y mostrar la respuesta del servidor
  while (client.available()) {
    Serial.write(client.read()); ///////////////////////REGRESA UN CODIGO 227 FTP,
  }
  // Enviar el comando PASV 
  client.println("PASV");
  // Dar tiempo para que el servidor responda
  while (!client.available()) {
    delay(1); 
  }
  
  if(!eRcv()) return 0;
  
  char *tStr = strtok(outBuf,"(,");
 
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL,"(,");
    array_pasv[i] = atoi(tStr);
    if(tStr == NULL)
    {
      Serial.println(F("Bad PASV Answer"));    
    }
  }

}

///////////////////////////////FUNCION PARA ENVIO DE ARCHIVOS/////////////////////////////////////////////////////////////////////

void storFTP(void) 
{
  String fileName = archivoFTP;
  //String contenido = "M|20231023|153630|STE130213445|NSUT-CDMX-COCOSAURIO|23102023-96301|19.12341|-99.6543|000|RANASAUdsfdsafsadfsdfafsdafdsasdfRIA";//content;
  String responseFTP;
  String comparaFTP; 
  ///////////////////////////////////////////////

  String contenidoFile;
  int reintentoEnvio = 0;
  unsigned int hiPort,loPort;

  hiPort = array_pasv[4] << 8;
  loPort = array_pasv[5] & 255;

  Serial.print(F("Data port: "));
  hiPort = hiPort | loPort;
  Serial.println(hiPort);
  while (true){
    if (dclient.connect(conf.ftpServer.c_str(),hiPort)) {
      Serial.println(F("LISTO PARA ENVIAR ARCHIVO"));  
      client.println("STOR " + fileName);
      int clientCount = 0;
      dclient.write(content.c_str(),content.length()) /*contenido.c_str(),contenido.length())*/;    
      // Dar tiempo para que el servidor responda
      while (!client.available()) {
        delay(1); 
      }
      // Leer y mostrar la respuesta del servidor
      while (client.available()) {
      Serial.write(client.read()); ///////////////////////REGRESA UN CODIGO 125 FTP,
    }
    
    dclient.stop();
    // Dar tiempo para que el servidor responda
    while (!client.available()) {
      delay(1); 
    }
      // Leer, guardar y mostrar la respuesta del servidor
    while (client.available()) {
      char c = client.read();
      responseFTP += c;
      if (c == '\n') {
        break;
      }
    }       
    break;
    } else {
      //Serial.println(F("INTENTANDO ENVIAR ARCHIVO"));
      Serial.print(F("REINTENTANDO ENVIO"));
      for ( int i = 0; i < 3; i++)
      {
        Serial.print(F("  ..."));
        delay(10);
      }
      Serial.println(F(""));
      reintentoEnvio ++;
    }
    if(reintentoEnvio == 10){
      envioOK = false; 
      break;
    } 
  }  //Serial.println("SALI DEL WHILE");
  Serial.print(responseFTP);///////////////////////REGRESA UN CODIGO 226 FTP
  /////////////Guarda y compara la respuesta//////////////////////////
  if(responseFTP.length() >= 3){
    comparaFTP = responseFTP.substring(0, 3);
    if(!comparaFTP.equals("226")) {
      Serial.println(F("ERROR AL ENVIAR ARCHIVO")); 
      envioOK = false;           
    } else {
      Serial.println(F("SE MANDO EL ARCHIVO"));
      envioOK = true; 
      salirFTP();
    }
  }
}

///////////////////////////////FUNCION PARA CERRAR FTP.  /////////////////////////////////////////////////////////////////////////

void salirFTP(void) 
{
  // Enviar el comando TYPE I 
  client.println("QUIT");
  // Dar tiempo para que el servidor responda
  while (!client.available()) {
    delay(1); 
  }
  // Leer y mostrar la respuesta del servidor
  while (client.available()) {
    Serial.write(client.read()); ///////////////////////REGRESA UN CODIGO 200 FTP,
  }
  Serial.println("SALIENDO DEL FTP");
  client.stop();      
}


byte eRcv() {
  byte respCode;
  byte b;

  while(!client.available()) delay(1);
  respCode = client.peek();
  outCount = 0;
  while(client.available()) {  
    b = client.read();    
    Serial.write(b); 
    if(outCount < 127) {
      outBuf[outCount] = b;
      outCount++;      
      outBuf[outCount] = 0;
    }
  }

  if(respCode >= '4') {
    ftperror();
    return 0;  
  }

  return 1;
}

void ftperror() {
  byte b = 0;
  client.println(F("QUIT"));
  while(!client.available()) delay(1);
  while(client.available()) {  
    b = client.read();    
    Serial.write(b);
  }

  client.stop();
  Serial.println(F("disconnected"));  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool envioFTP(bool eval, bool msg){
  // 

  char volBuff[16] = "", gastoBuff[16] = "";
  char archivoSD[32] = "", carpeta[64] = "";
  boolean result = false;
  
  DateTime now = rtc.now();

  flowRate = Gasto(ModeloAforador);      //     Gasto en m3/seg                                flowRate = Gasto()*1000;   Tasa de Flujo en litros por segundo
  totalVol = Volumen(ModeloAforador);                         // Volumen acumulado total positivo en m3
  Serial.println(F("Validando y generando datos..."));
  
  if (!conModbus(flowRate, totalVol, false)) 
    error = "004"; 
  //else if((fona.getRSSI() == 99)) 
  //  error = "005";
  else {                    
    Serial.print(F("Codigo ***SIN ERRORES***"));
    error = "000";
  }  

  sprintf(archivoSD, "datos/%2d%02d%02d.txt", now.year(),now.month(), now.day());
  sprintf(carpeta,"%s", conf.ftpFolder.c_str());

  dtostrf(totalVol, 1, 3, volBuff); // float_val, min_width, digits_after_decimal, char_buffer
  dtostrf(flowRate, 1, 4, gastoBuff);                                 // Se usa cuando el sistema es QA
  
  tiempoActual(false);
  counter = 1;
  cnc_flow = gastoBuff;
  cnc_vol = volBuff;
  if (conf.tipo == "QA") {                     // structure information  for QA (SISTEMA DE MEDICION)
    
    if(eval == true){
      sprintf(archivoFTP, "%s_%2d%02d%02d_%s_%s.txt", conf.rfc.c_str(), now.year(),now.month(), now.day(), conf.nsut.c_str() , conf.uvaID.c_str());
    //  content = conf.tipo + "|" + cnc_fecha + "|" + cnc_hour +"|"+ conf.rfc + "|" + conf.nsue + "|" + gastoBuff + "|" + volBuff + "|" + latBuff + "|" + lonBuff + "|" + error.c_str() + "|" + uvaIDBuff;
    content = conf.tipo + "|" + cnc_fecha + "|" + cnc_hour +"|"+ conf.rfc + "|" + conf.nsut + "|" + cnc_flow + "|" + cnc_vol + "|" + conf.latitud.c_str() + "|" + conf.longitud.c_str() + "|" + error.c_str() + "|" + conf.uvaID;
    }else {
      
      sprintf(archivoFTP, "%s_%2d%02d%02d_%s.txt", conf.rfc.c_str(), now.year(),now.month(), now.day(), conf.nsue.c_str());
      content = conf.tipo + "|" + cnc_fecha + "|" + cnc_hour +"|"+ conf.rfc + "|" + conf.nsue + "|" + cnc_flow + "|" + cnc_vol + "|" + conf.latitud.c_str() + "|" + conf.longitud.c_str() + "|" + error.c_str() ;
    }
  }
  else if (conf.tipo == "M") {                  // structure information  for M (MEDIDOR)
   if(eval == true){
      sprintf(archivoFTP, "%s_%2d%02d%02d_%s_%s_%s.txt", conf.rfc.c_str(), now.year(),now.month(), now.day(), conf.nsm.c_str(), conf.nsut.c_str() , conf.uvaID.c_str());
     // content = conf.tipo + "|" + cnc_fecha + "|" + cnc_hour +"|"+ conf.rfc + "|" + conf.nsm + "|" + conf.nsue + "|" + volBuff + "|" + latBuff + "|" + lonBuff + "|" + error.c_str() + "|" + uvaIDBuff;
   content = conf.tipo + "|" + cnc_fecha + "|" + cnc_hour +"|"+ conf.rfc + "|" + conf.nsm + "|" + conf.nsut + "|" + conf.latitud.c_str() + "|" + conf.longitud.c_str() + "|" + error.c_str() + "|" + conf.uvaID.c_str();
   }else {
      sprintf(archivoFTP, "%s_%2d%02d%02d_%s_%s.txt", conf.rfc.c_str(), now.year(),now.month(), now.day(), conf.nsm.c_str(),  conf.nsue.c_str());
      content = conf.tipo + "|" + cnc_fecha + "|" + cnc_hour +"|"+ conf.rfc + "|" + conf.nsm + "|" + conf.nsue + "|" + cnc_vol + "|" + conf.latitud.c_str() + "|" + conf.longitud.c_str() + "|" + error.c_str() ;
   }
  } 

  conf.macAddres = macEthernet(false);

  Serial.println(F("=======---------------------------======="));
  Serial.print(F("Nombre de directorio: "));    
  Serial.println(carpeta);
  Serial.print(F("Nombre del archivoSD: "));    
  Serial.println(archivoSD);
  Serial.print(F("Nombre del archivo FTP: "));    
  Serial.println(archivoFTP);
  Serial.println(F("=======---------------------------=======")); 
  //Serial.print(F("ETHERNET: ")); Serial.println(conf.eth);
  Serial.print(F("MAC: ")); Serial.println(conf.macAddres);
  Serial.print(F("Modelo Caudalimetro: ")); Serial.println(conf.modeloAforador);
  Serial.print(F("NSUT: ")); Serial.println( conf.nsut);
  Serial.print(F("Sistema de medicion: ")); Serial.println(conf.tipo);
  Serial.print(F("Fecha: ")); Serial.println(cnc_fecha);
  Serial.print(F("Hora: ")); Serial.println(cnc_hour);
  Serial.print(F("RFC: ")); Serial.println(conf.rfc);
  Serial.print(F("NSM: ")); Serial.println(conf.nsm);
  Serial.print(F("NSUE: ")); Serial.println( conf.nsue);
  Serial.print(F("Gasto [m3/seg]: ")); Serial.println(cnc_flow);
  Serial.print(F("Volumen [m3]: ")); Serial.println(volBuff);
  Serial.print(F("Latitud: ")); Serial.println(conf.latitud);
  Serial.print(F("Longitud: ")); Serial.println(conf.longitud);
  Serial.print(F("ID: ")); Serial.println(conf.tagID.c_str());
  Serial.print(F("kernel error [HEX]:")); Serial.println(error);
  Serial.print(F("Unidad verificadora: ")); Serial.println(conf.uvaID);
  Serial.println(F("=======---------------------------======="));
  
  Serial.println(F("Datos guardados en memoria SD: "));
  Serial.println(content); //
  saveDataSD(archivoSD, content.c_str()); 

  
  cableEthernet();
  
  
  if(redOK == true && envioOK == true){
    Serial.println(F("!FTP ENVIADO CORRECTAMENTE!"));
    Serial.println(F("#MSSGENVIO DE ARCHIVO CON EXITO"));
    if (!error.c_str() == "004")
    {
      error = "000";////SE ENVIO SIN ERROR DE MODBUS
    }  
  } else if (redOK == true && ipOK == false) {
    Serial.println(F("#MSSGEL ACCESO AL SERVIDOR FTP ES INVALIDO, VERIFICAR IP"));
    error = "013";  ///////////////////////ERRO DE ACCESO AL SERVIDOR FTP
  } else if (redOK == true && passOK == false) {
    Serial.println(F("#MSSGEL ACCESO AL SERVIDOR FTP ES INVALIDO, VERIFICAR USUARIO Y/O CONTRASEÑA"));
    error = "014";  ///////////////////////ERRO DE ACCESO AL SERVIDOR FTP
  } else if (redOK == true && dirOK == false) {
    Serial.println(F("#MSSGEL ACCESO AL SERVIDOR FTP ES INVALIDO, VERIFICAR DIRECTORIO"));
    error = "015";  ///////////////////////ERRO DE ACCESO AL SERVIDOR FTP
  } else if (redOK == true && envioOK == false) {
    Serial.println(F("#MSSGFALLA AL DEPOSITAR ARCHIVO AL FTP "));
    error = "009";
  } 
  //Serial.println(archivoFTP);
  //delay(5000);
  return result;
}


void envioWebPost(){
  //Estructura JSON WEB POST 
  client.stop();
  
  flowRate = Gasto(ModeloAforador);      //     Gasto en m3/seg                                flowRate = Gasto()*1000;   Tasa de Flujo en litros por segundo
  totalVol = Volumen(ModeloAforador);                         // Volumen acumulado total positivo en m3
  content = "";
  
  tiempoActual(false);
  Serial.println(F("Enviando datos en Servidor HTTP, metodo POST ..."));

  //if (client.connect("infopro.swplus.com.mx", 80)) {
  if (client.connect("infopro-api.swplus.com.mx", 80)) {
    // Conexión exitosa al servidor
    Serial.println(F("Conexion exitosa al servidor"));
    Serial.print(F("Cadena JSON: "));
    content =  "{\"Tipo\":\"" + conf.tipo + "\",\"Fecha\":\"" + cnc_fecha + "\",\"Hora\":\"" + cnc_hour + "\",\"RFC\":\"" + conf.rfc + "\",\"NSM\":\"" + conf.nsm + "\",\"NSUE\":\"" + conf.nsue + "\",\"lat\":\"" + conf.latitud + "\",\"long\":\"" + conf.longitud + "\",\"gasto\":\"" + String(flowRate) + "\",\"Vol\":\"" + String(totalVol) + "\",\"Modelo\":" + conf.modeloAforador + ",\"NSUT\":\"" + conf.nsut + "\",\"tagID\":\"" + conf.tagID + "\",\"mac\":\"" + conf.macAddres + "\",\"diametro\":" + conf.dn + ",\"ker\":\""  + error + "\"}\r\n";
    Serial.println(content);
    delay(timeout);
   
    // Construir la solicitud POST
    String postRequest = "POST /api/Log HTTP/1.1\r\n";
   
    postRequest += "Host: infopro-api.swplus.com.mx\r\n"; // Reemplaza con tu host
    postRequest += "Content-Type: application/json\r\n";
    postRequest += "Content-Length: " + String(content.length())  + "\r\n\r\n";  
    postRequest += content;

    // Enviar la solicitud POST al servidor
    client.print(postRequest);

    client.println("Connection: close");
       
    Serial.println(F("Data sent status:"));
    // Esperar la respuesta del servidor
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
      }
    }
    //
    client.stop();
  } else {
    // Error al conectar al servidor
    Serial.println(F("Error al conectar al servidor"));
  }
  delay(timeout*10);
}

bool signalLevel()
{
   
  if ((!Ethernet.begin(mac)== 1) || (Ethernet.localIP() == IPAddress(0, 0, 0, 0))) /*or (!Ethernet.begin(mac)== Unknown)) */{
      Serial.println(F("SIN CONEXION A RED ETHERNET"));
      Serial.print(F("IP "));
      Serial.println(Ethernet.localIP());
      return false;
  } else {
      Serial.println(F("RED ETHERNET CONECTADA"));
      Serial.print(F("DHCP IP ASIGNADA "));
      Serial.println(Ethernet.localIP());
      return true;
  }
  
}

String macEthernet(bool msg)
{
  byte macBuffer[6];  // create a buffer to hold the MAC address
  String ethMAC = "";
  Ethernet.MACAddress(macBuffer); // fill the buffer
  Serial.print(F("MAC: "));
  for (byte octet = 0; octet < 6; octet++) {
    Serial.print(macBuffer[octet], HEX);
    char buffer[3];
    sprintf(buffer, "%02X", macBuffer[octet]);
    ethMAC += buffer;
    if (octet < 5) {
      Serial.print(':');
      ethMAC += ':';
    }
  }
  if(msg){
    Serial.println();
    Serial.print(F("#MSSGLA MAC DEL DISPOSITIVO ES: "));
    Serial.print(ethMAC);
    Serial.println();
  }
  
  return ethMAC;
}

char parametrosRed(){
  String paramRed ="#ETHE";
  paramRed += macEtheString();
  
  paramRed += ";" + String(ip2CharArray(Ethernet.localIP())) + ";";
  paramRed += String(ip2CharArray(Ethernet.gatewayIP()));

  Serial.println(paramRed);
  return 0;
}

char* ip2CharArray(IPAddress ip) {
  static char a[16];
  sprintf(a, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  return a;
}

String macEtheString(void){
  byte macBuffer[6];  // create a buffer to hold the MAC address
  String ethMAC = "";
  Ethernet.MACAddress(macBuffer); // fill the buffer
  
  for (byte octet = 0; octet < 6; octet++) {
    //Serial.print(macBuffer[octet], HEX);
    char buffer[3];
    sprintf(buffer, "%02X", macBuffer[octet]);
    ethMAC += buffer;
    if (octet < 5) 
      //Serial.print(':');
      ethMAC += ':';
  }
    return ethMAC;
}

////////////Funciones para mostrar en SCREEN en el LCD.
void LOGO(unsigned int offset, unsigned int row)
{  
  
  //lcd.clear();
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  byte image69[8] = {B00111, B00001, B00011, B00001, B00111, B00000, B00000, B00000};         //Caracteres diseñados para el LCD m3
  
  byte image89[8] = {B00000, B00000, B01111, B01111, B01111, B01110, B01110, B01110};         //Imagen89 
  byte image90[8] = {B00000, B00000, B11111, B11111, B11111, B00000, B00000, B00000};         //Imagen90 -
  byte image91[8] = {B01110, B01110, B01110, B01110, B01110, B01110, B01110, B01110};         //Imagen91 |
  byte image92[8] = {B00000, B00000, B11110, B11110, B11110, B01110, B01110, B01110};         //Imagen92   
  byte image93[8] = {B01110, B01110, B01110, B11110, B11110, B11110, B00000, B00000};         //Imagen93 
  byte image94[8] = {B01110, B01110, B01110, B01111, B01111, B01111, B00000, B00000};         //Imagen94 
  byte image95[8] = {B00000, B00000, B00000, B11111, B11111, B11111, B00000, B00000};         //Imagen95
  
  lcd.createChar(1, image69);
  lcd.createChar(2, image89);
  lcd.createChar(3, image91);
  lcd.createChar(4, image90);
  lcd.createChar(5, image92);
  lcd.createChar(6, image93);
  lcd.createChar(7, image94);
  lcd.createChar(8, image95);
 
  MARCO();
  lcd.setCursor(2, 1);
  lcd.print("SWPLUS ETH 2.3.1");
  lcd.setCursor(5, 2);
  lcd.print("TECNOLOGIAS");
}

void TIEMPO(void)
{
    /*MARCO();
    lcd.clear();
    */
    MARCO();
    //////FECHA/////////
    lcd.setCursor(2,1);
    lcd.print("FECHA");
    lcd.setCursor(8,1);
    if (DD<10)
    {
      lcd.print("0");
    }    
    lcd.print(DD);
    lcd.print("/");
    if (MM<10)
    {
      lcd.print("0");
    }
    lcd.print(MM);
    lcd.print("/");
    lcd.print(AA);
    /////////HORA/////////
    lcd.setCursor(2,2);
    if(ss>59)
    {
      ss=0;
      mm++;
    }
    if(mm>59)
    {
      mm=0;
      hh++;
    }
     if(hh>=24)
    {
      ss=0;
      mm=0;
      hh=0;
    }
    lcd.print("HORA");
    lcd.setCursor(9,2);
    if (hh<10)
    {
      lcd.print("0");
    }
    lcd.print(hh);
    lcd.print(":");
    if(mm<10)
    {
       lcd.print("0");   
    }
    lcd.print(mm);    
    //lcd.print(":");
    if(ss<10)
    {
     // lcd.print("0");
    }
    //lcd.print(ss);
}

void VOLUMEN(void)
{
  
    //lcd.clear();
    MARCO();
    lcd.setCursor(2,1);
    lcd.print("VOLUMEN: "); 
    lcd.setCursor(5,2);
    lcd.print(totalVol,1);
    if(totalVol > 9.999999e+6)
    {
      lcd.setCursor(16,2);
      lcd.print("m");
      lcd.setCursor(17,2);
      lcd.write(byte(1));
    } else {
      lcd.setCursor(15,2);
      lcd.print("m");
      lcd.setCursor(16,2);
      lcd.write(byte(1));
    }

}

void FLUJO(void)
{
    
    //lcd.clear();
    MARCO();
    lcd.setCursor(2,1);
    lcd.print("FLUJO: ");
    lcd.setCursor(6,2);;
    lcd.print(flowRate,2);
    if(flowRate > 9999) {
      lcd.setCursor(15,2);
      lcd.print("m");
      lcd.setCursor(16,2);
      lcd.write(byte(1));
      lcd.print("/h");  
    } else {
      lcd.setCursor(14,2);
      lcd.print("m");
      lcd.setCursor(15,2);
      lcd.write(byte(1));
      lcd.print("/h");
    }
                                        // Se cambio a metros3 / hora a peticion de GADITEC y BIOPAPPEL     24/NOV&/2022
}

void MODEM(void)
{   
    //lcd.clear();
    MARCO(); 
    lcd.setCursor(2,1);
    lcd.print("VERIFICANDO RED ");
    lcd.setCursor(8,2);;
    lcd.print(". . .");

    if (!signalLevel()){
      MARCO(); 
      error = "020";
      lcd.setCursor(2,1);
      lcd.print("DESCONEXION RED");
      lcd.setCursor(5,2);
      lcd.print("ker:");
      lcd.setCursor(11,2);                                      
      lcd.print(error.c_str());  
    }
    else{
      MARCO(); 
      lcd.setCursor(2,1);
      lcd.print("RED SATELITAL OK");
      lcd.setCursor(5,2);
      lcd.print("ker:");
      lcd.setCursor(11,2);
      lcd.print(error.c_str());   
    }
}

void MARCO(void){
    x=1;
    y=0;
    lcd.clear(); 
    lcd.setCursor(0,0);
    lcd.write(byte(2));
    ///////////////////////
    for (i=0; i<=17; i++)
    {
      lcd.setCursor(x,y);
      lcd.write(byte(4)); 
      x=x+1;    
    }
    ////////////////////
    lcd.setCursor(19,0);
    lcd.write(byte(5)); 
    ///////////////////
    x=19;
    y=1;
    for (i=0; i<2; i++)
    {
      lcd.setCursor(x,y);
      lcd.write(byte(3)); 
      y=y+1;    
    }
    ///////////////////
    lcd.setCursor(19,3);
    lcd.write(byte(6));
    /////////////////// 
    x=18;
    y=3;
    for (i=0; i<=17; i++)
    {
      lcd.setCursor(x,y);
      lcd.write(byte(8)); 
      x=x-1;    
    }
    ///////////////////
    lcd.setCursor(0,3);
    lcd.write(byte(7));    
    ///////////////////
    x=0;
    y=1;
    for (i=0; i<2; i++)
    {
      lcd.setCursor(x,y);
      lcd.write(byte(3)); 
      y=y+1;    
    } 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                      FUNCION SETUP                                                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  
  Serial.begin(115200);           // Inicia comunicacion Serial 115200 / 8N1 con la aplicacion INFOPRO
  Wire.begin();                 // Inicia comunicacion I2C para SD Card
  rtc.begin(); 
  // timeClient.begin();                 // Iniciar Reloj tiempo real
  pinMode(CSE, OUTPUT);
  pinMode(CS, OUTPUT);
  digitalWrite(CSE,LOW);
  digitalWrite(CS,HIGH);
  Ethernet.init(CSE);   // Teddy Bear shield 
  tiempoActual(false);
  conexionEthernet();
  configSD();
  //obtenNSUT(false);
  
  //Serial.println(F("INGRESAR CREDENCIALES. . .")); 
  Serial.println(F("Version 2.0.3    MOD DIC 2023 ETHERNET TEDDY")); 
  Serial.println(F("#REDY")); 
 // Serial.println(F("LOGIN . . .")); 
 
  
  /////////////////INICIO DE LCD./////////////////////// 
 
  lcd.begin(20, 4);
  lcd.setBacklightPin(3, POSITIVE);       // Pin 3 del display encenderá y apaga la luz de fondo
  lcd.setBacklight(HIGH);  

  //////////////////////////////////////////////////////
  if(!screenLogin)
  {
    lcd.clear();
    LOGO(5,2);
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                           FUNCION LOOP                                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 


void loop() 
{
  // timeClient.update();
  // Serial.println(timeClient.getFormattedTime());
  delay(1000);
  DateTime now = rtc.now();
  AA = now.year();
  MM = now.month();
  DD = now.day();  
  hh = now.hour();
  mm = now.minute();
  ss = now.second();                                          

  if((ss%20)==0)
  {   
    Serial3.flush();

    if (packet1->failed_requests >= 10) packet1->connection = true;
    if (packet2->failed_requests >= 10) packet2->connection = true;
    delay(timeout-10);
    modbus_update();
    flowRate = Gasto(ModeloAforador);      //     Gasto en m3/seg                                flowRate = Gasto()*1000;   Tasa de Flujo en litros por segundo
    totalVol = Volumen(ModeloAforador);
    conModbus(flowRate, totalVol, false);  
  }

  opciones(); 
  if (hh == 18 and mm == 30 and ss == 10)
  {
    obtenNTP();
  }
  

  if( ( hh == conf.hhEnvio and mm == conf.mmEnvio and (ss%30 == 0)) )
  { 
      envioFTP(false, false);  
  }

  if( ( hh == conf.hhEnvio and mm == conf.mmEnvio and (ss%30 == 0)) or ( hh % 4 == 0 and mm == conf.mmEnvio and (ss%30 == 0)) )
  { 
      envioWebPost();  
  }

  if(screenLogin == true){

    unsigned long currentMillisScreen = millis();

    //////////////////////////SCREEN///////////////////////////////////////
    if (currentMillisScreen - previousMillisScreen > lcdInterval){
      previousMillisScreen = currentMillisScreen;
      screen++;
      if (screen > screenMax) screen = 0;  
      screenChanged = true;  
    }
 
  /////////////////////////////////SCREEN///////////////////////////////////
    if (screenChanged) {
        screenChanged = false;  
        switch(screen)
        {
          case SCREEN_LETRERO:
            LOGO(5,2);
            break;
          case SCREEN_VOLUMEN:
            VOLUMEN();
            break;
          case SCREEN_FLUJO:
            FLUJO();
            break;
          case SCREEN_TIEMPO:
            TIEMPO();
            break;
          case SCREEN_MODEM:
            MODEM();
            break;
      
          default:
            break;
        }
      }     
  }
}
// Funciones para adquirir la hora y la fecha de un servido web
void obtenNTP ()
{
int zonaHoraria = -6*3600;
//setTimeOffset(-6 * 3600);
Serial.println("Conectando a un Server NTP ... ... ...");
EthernetUDP clientNTP;
NTPClient timeClient(clientNTP,"1.mx.pool.ntp.org"); //////Protocolo de NTP 1-1.mx.pool.ntp.org, 2-3.north-america.pool.ntp.org 3-0.north-america.pool.ntp.org
clientNTP.begin(8888);   
timeClient.begin();              // Iniciar Reloj tiempo real
timeClient.update(); 
timeClient.isTimeSet();
if(!timeClient.isTimeSet())
{
  Serial.println("Error en la conexión NTP, Dispisitivo sin RED");
}
else
{
unsigned long formato = timeClient.getEpochTime();
//Serial.println(timeClient.getFormattedTime());
//Serial.println(timeClient.getDay());
//Serial.println(timeClient.getHours() - 6);
//Serial.println(timeClient.getMinutes());
//Serial.println(timeClient.getSeconds());
//Serial.println(timeClient.getEpochTime());
//Serial.println(timeClient.getDay());
setTime(formato + zonaHoraria);
//Serial.println(formato);
String formattedDateTime = String(year()) + "-" + month() + "-" + day() + " " + hour() + ":" + minute() + ":" + second();
Serial.println("Fecha y hora actual: " + formattedDateTime);
AANTP = year();
MMNTP = month();
DDNTP = day();
hhNTP = hour();
mmNTP = minute();
ssNTP = second();
Serial.println("Conexion con exito");
Serial.println("FECHA Y HORA DEL SERVIDOR NTP");
Serial.println(String(AANTP) + "-" + String(MMNTP) + "-" + String(DDNTP) + "  " + String(hhNTP) + ":" + String(mmNTP) + ":" + String(ssNTP));
Serial.println("Modificando Reloj con FECHA Y HORA tomada del servidor NTP");
AA = AANTP, MM = MMNTP, DD = DDNTP, hh = hhNTP, mm = mmNTP, ss = ssNTP;
rtc.adjust(DateTime(AA, MM, DD, hh, mm, ss));
Serial.println(String(AA) + "-" + String(MM) + "-" + String(DD) + "  " + String(hh) + ":" + String(mm) + ":" + String(ss));
}
delay(1000);
timeClient.end();
}
