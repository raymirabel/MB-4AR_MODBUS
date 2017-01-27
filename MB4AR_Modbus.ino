
/*
  MB4AR_Modbus.ino
  28/09/2015 - v1.00
  - Versión inicial
  
  26/01/2017 - v1.01
  - Se añade dirección modbus para lectura de entradas analogicas como digitales
  
  Sketch para usar el módulo MB4AR como esclavo modbus RTU bajo RS485
  Copyright (c) 2015 Raimundo Alfonso
  Ray Ingeniería Electrónica, S.L.
  
  Este sketch está basado en software libre. Tu puedes redistribuir
  y/o modificarlo bajo los terminos de licencia GNU.

  Esta biblioteca se distribuye con la esperanza de que sea útil,
  pero SIN NINGUNA GARANTÍA, incluso sin la garantía implícita de
  COMERCIALIZACIÓN O PARA UN PROPÓSITO PARTICULAR.
  Consulte los terminos de licencia GNU para más detalles.
  
  * CARACTERISTICAS
  - Lectura de antradas analogicas
  - Escritura de reles
  - Estritura de salidas analogicas
  - Parametros de comunicación RTU, 9600 baudios,n,8,1
  
  * MAPA MODBUS
    MODO R: FUNCION 3 - READ BLOCK HOLDING REGISTERS
    MODO W: FUNCION 6 - WRITE SINGLE HOLDING REGISTER
    
  DIRECCION   TIPO    MODO  FORMATO    MAXIMO      MINIMO    UNIDADES    DESCRIPCION
  ---------------------------------------------------------------------------------------------------------
  0x0000      int     R     00000      01023       00000     ---         Entrada analogica 1
  0x0001      int     R     00000      01023       00000     ---         Entrada analogica 2
  0x0002      int     R     00000      01023       00000     ---         Entrada analogica 3  
  0x0003      int     R     00000      01023       00000     ---         Entrada analogica 4
  0x0004      int     R     00000      00015       00000     ---         (reservado)
  0x0005      int     R/W   00000      00015       00000     ---         Salidas reles en formato binario
  0x0006      int     R     00000      00063       00000     ---         Dirección modbus o estado del dipswitch 
  0x0007      int     R/W   00000      04095       00000     ---         Salida analógica 1  
  0x0008      int     R/W   00000      04095       00000     ---         Salida analógica 2
  0x0009      int     R/W   00000      04095       00000     ---         Salida analógica 3
  0x000A      int     R/W   00000      04095       00000     ---         Salida analógica 4 
  0x000B      int     R     00000      00015       00000     ---         Entradas analogicas como digitales y en formato binario
    
*/
#include <ModbusSlave.h>
#include <Wire.h>
#include "mcp4728.h"
#include <avr/wdt.h> 

// Pines E/S asignados:
#define RELE1      7
#define RELE2      8
#define RELE3      9
#define RELE4      10
#define LED1       13
#define LED2       12
#define DIPSW1	   4   
#define DIPSW2	   5    
#define DIPSW3	   6    
#define DIPSW4	   11    
#define DIPSW5	   AN6    
#define DIPSW6	   AN7   

#define MAX_BUFFER_RX  15

// Mapa de registros modbus
enum {        
        MB_AN1,          // Entrada analogica 1
        MB_AN2,          // Entrada analogica 2
        MB_AN3,          // Entrada analogica 3
        MB_AN4,          // Entrada analogica 4
        MB_5,            // reservado
        MB_RELAY,        // Salidas de rele
        MB_DIPSW,        // Estado del dipswitch
        MB_DA1,          // Salida analogica 1        
        MB_DA2,          // Salida analogica 2
        MB_DA3,          // Salida analogica 3
        MB_DA4,          // Salida analogica 4   
        MB_DIG_IN,       // Entradas digitales     
        MB_REGS	 	 // Numero total de registros
};
int regs[MB_REGS];	

// Crea la clase para el modbus...
ModbusSlave modbus;

// Crea instancia DAC...
mcp4728 dac = mcp4728(0); 

void setup()  { 
  wdt_disable();
  
  // Configura entradas y salidas...
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);  
  pinMode(RELE1, OUTPUT);  
  pinMode(RELE2, OUTPUT);  
  pinMode(RELE3, OUTPUT);  
  pinMode(RELE4, OUTPUT);  
  pinMode(DIPSW1, INPUT);
  pinMode(DIPSW2, INPUT);
  pinMode(DIPSW3, INPUT);
  pinMode(DIPSW4, INPUT);  
   
  // configura modbus...
  modbus.config(9600,'n');
  modbus.direccion = leeDIPSW();
  
  // Inicializa DAC...
  dac.begin();  
  dac.eepromReset();
  dac.vdd(5000); // set VDD(mV) of MCP4728 for correct conversion between LSB and Vout
  delay(100);
  dac.setVref(1, 1, 1, 1);
  dac.setGain(1, 1, 1, 1);  

  // Activa WDT cada 4 segundos...   
  wdt_enable(WDTO_4S); 
} 



void loop()  { 
  unsigned long curMillis = millis();          // Get current time

  // Lee entradas analogicas...
  leeAIN();

  // Lee dipswitch...
  regs[MB_DIPSW] = leeDIPSW();
      
  // Espera lectura de tramas modbus
  delay_modbus(100);
  
  // Actualiza reles...
  actualizaReles();
  
  // Actualiza salidas analogicas...
  actualizaSalidasAnalogicas();
  
  // Reset WDT
  wdt_reset();
}

// Rutina de espera que atiende la tarea modbus...
void delay_modbus(int t){
  int n,tt;
  tt = t/10;
  
  for(n=0;n<=tt;n++){
    modbus.actualiza(regs,MB_REGS);
    delay(10);
  }  
}

void actualizaReles(void){
  if((regs[MB_RELAY] & 0x01) == 0x01) digitalWrite(RELE1,HIGH); else digitalWrite(RELE1,LOW);
  if((regs[MB_RELAY] & 0x02) == 0x02) digitalWrite(RELE2,HIGH); else digitalWrite(RELE2,LOW);
  if((regs[MB_RELAY] & 0x04) == 0x04) digitalWrite(RELE3,HIGH); else digitalWrite(RELE3,LOW);
  if((regs[MB_RELAY] & 0x08) == 0x08) digitalWrite(RELE4,HIGH); else digitalWrite(RELE4,LOW); 
}

void actualizaSalidasAnalogicas(void){
  dac.analogWrite(0, regs[MB_DA1]);
  dac.analogWrite(1, regs[MB_DA2]);
  dac.analogWrite(2, regs[MB_DA3]);
  dac.analogWrite(3, regs[MB_DA4]);    
}

// Rutina para leer el dipswitch
byte leeDIPSW(void){
  byte a0,a1,a2,a3,a4,a5;
  
  // Lee dipswitch...
  a0 = !digitalRead(DIPSW1);  
  a1 = !digitalRead(DIPSW2);
  a2 = !digitalRead(DIPSW3);
  a3 = !digitalRead(DIPSW4);
  if(analogRead(A6) < 512)
    a4 = 1;  
  else
    a4 = 0;
  if(analogRead(A7) < 512)
    a5 = 1;  
  else
    a5 = 0;
  // Calcula dirección...
  return(a0 + a1*2 + a2*4 + a3*8 + a4*16 + a5*32);
}

// Rutina para leer entradas analogicas...
void leeAIN(void){
  byte a0,a1,a2,a3;
  int an0,an1,an2,an3;

  an0 = analogRead(A0);
  an1 = analogRead(A1);
  an2 = analogRead(A2);
  an3 = analogRead(A3); 

  regs[MB_AN1] = an0;
  regs[MB_AN2] = an1;
  regs[MB_AN3] = an2;
  regs[MB_AN4] = an3;
  
  // Convierte entradas analogicas a digitales...
  a0 = 0;
  a1 = 0;
  a2 = 0;
  a3 = 0;
  if(an0 > 512) a0 = 1;
  if(an1 > 512) a1 = 1;
  if(an2 > 512) a2 = 1;
  if(an3 > 512) a3 = 1;

  regs[MB_DIG_IN] = (a0 + a1*2 + a2*4 + a3*8);
}

