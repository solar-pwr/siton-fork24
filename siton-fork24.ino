/*
  SOLAR INVERTER 2 - SITON 210
  Program stridace pro fotovoltaicke panely a ohrev bojleru.
  Prevadi DC proud z FV panelu na stridavy obdelnikovy s promennou stridou.
  Vyhledava bod nejvyssiho vykonu fv panelu, zobrazeni hodnot na I2C LCD
  pres expander PCF8574
  mereni napeti a proudu, tlacitka pro rucni zmenu stridy menice
  rucni a automaticky rezim, vypocet vyroby EE, pri zvyseni vyroby o 0,1kWh
  ulozi vyrobu do EEprom, komunikace po RS485, mereni teploty bojleru.

  Poznamka:
  Pri pouziti bootloaderu musi byt v Arduinu novejsi verze bootloaderu (optiboot)
  jinak watchdog nefunguje spravne.
  Pouzita nestandartni inicializace dvou typu komunikaci na jedno sw seriove
  rozhrani.

  Verze 5.4.27(3f)
  file:Solar_inverter54.27.ino
  kompilace Arduino 1.8.5
  Tomas Nevrela; tnweb.tode.cz
  15.2.2022
  Zmeny:
  4.2 - opravena chyba nacitani vyroby
  4.3 - opraveny adresy v eeprom po 90000 zapisech
      - automaticka inicializace Eeprom po prvnim nahrani programu
  4.4 - zvysena zmena pwm na -3 a +4
      - zvysen cas prepnuti z ruc. do aut. rezimu na 10s
  4.5 - tlacitka pres analogovy vstup
      - mereni teploty bojleru
      - periodicka kontrola VA krivky
      - omezeni proudu na 10A
  4.5a- jumper na D5 pro vypnuti funkce kontroly VA krivky
  4.6 - nova verze DPS V2C s nadproudovou ochranou a budici IR2104
  4.7 - svorkovy vstup pro povoleni provozu menice verze DPS V2D
  4.8 - kazdych 60min. se provede zapis vyroby do EEprom
  4.9 - Setting menu, nastaveni max. teploty, max. vykonu,
        krivky, perioda testu VA krivky, ID menice, max.hodnoty + moznost
        nulovat max. hodnoty
  5.0 - pridana komunikace MODBUS
      - run LED behu programu
	    - pridana moznost nastaveni celkove vyroby pri zapnuti
  5.1 - rizeni podsviceni LCD po I2C pres Attinyx5 adresa 0x25
  5.1a- blokovani chodu menice pri nastavovani celkové vyroby kWh
  5.2 - osetreni odpojeneho termocidla; max. podsviceni pri stisku tlacitka
  5.3 - Oprava konfliktu reset watchdogu a komunikace Modbus
        do menu pridana polozka tovarni reset - maze vyrobu a nastaveni
  5.4 - Zmena principu resetu menice po tovarnim resetu pomoci watchdogu

  fork24 (Jakub Strnad)
  =====================
  5.5 - zmena v inicializaci Modbus (kompatibilita s novou verzi knihovny)
      - kalibrace offsetu proudoveho cidla
      - odstraneno readVcc() - nepresnost vnitrni 1.1V reference je vetsi, nez
        tolerance 5V stabilizatoru pouzivanych v Sitonovi, vytvari dodatecnou chybu
      - moznost nastaveni max. efektivniho napeti na vystupu (vhodne pri prepinani
        zatezi s rozdilnym nominalnim vykonem)
      - zmena MPPT algoritmu - misto vykonu sleduje efektivni vystupni napeti
        (vetsi rozliseni, mensi kolisani pri malem vykonu)
      - podpora ovladani jasu pro novou SMD verzi
  5.6 - autodetekce I2C adresy displeje
  5.7 - hotplug displeje
  5.8 - dalsi optimalizace MPPT algoritmu
      - moznost vycitani stridy v Modbus registru 8
      - moznost vycitani VA-krivek pres Modbus
      - odstraneni delay() z hlavni smycky,
        s tim souvisejici uprava funkce obsluhujici tlacitka
      - optimalizace mereni, zvyseni rozliseni proudu
      - odstraneni EEPROMAnything.h (EEPROM.h umi totez)
*/

#include <SoftEasyTransfer.h> //https://github.com/madsci1016/Arduino-SoftEasyTransfer
#include <SoftwareSerial.h>
#include <ModbusRtu.h> //https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>

SoftwareSerial mySerial(8, 6); //Arduino RX - RS485 RO, Arduino TX - RS485 DI
#define TXenableRS485 7 //RE + DE
#define Buttons  A3 //pripojene tlacitka
#define menuTimeExit 10000 //po case ukonci Menu
#define pinA A1 //pin mereni proudu FV
#define pinV A2 //pin mereni napeti FV
#define cidlo_B A0 //cidlo teploty KTY81/210
#define R25 2000.0 //referencni odpor termocidla pri 25st C
#define RDIV 2200.0 // hodnota rezistoru v delici s termocidlem
#define mppt_pin 5 //jumper rezerva 
#define enable_pin 4 // pin povoleni provozu menice
#define backlightPWMpin  3 // Pin D3
#define ochrana_pin 11 // pin stavu nadproudove ochrany
#define ochrana_reset_pin 12 //reset nadproudove ochrany
#define R1 470000.0 //odpory delice napeti FV
#define R2 5600.0
#define maxStrida 247 //maximalni strida (nemenit!)
#define frekvence 50 //frekvence PWM vystupu (nemenit!)
#define Maxproud 1200 //max. proud 12.00A
#define LEDpin 13  // signalizacni LED komunikace
#define runLED 1  // signalizacni LED behu
#define I2C_Address 0x27 // adresa I2C prevodniku PCF8574T,PCF8574P; 0x27 (PCF8574AT; 0x3f)
#define AttinyAddress 0x25 // adresa Attiny desky LCD
byte stupen[8] = {0b01100, 0b10010, 0b10010, 0b01100, 0b00000, 0b00000, 0b00000, 0b00000};
int klavesa;
uint8_t offset, whichkey;
bool showStatus = false; //
int nodeID = 12; // cislo Emon nodu
#define INTERVAL_LCD 800 //cas obnoveni LCD
#define INTERVAL_VA 60000 //interval kontroly VA krivky
#define INTERVAL_WR 3600000 //interval zapisu vyroby - 60 min.
#define INTERVAL_RUC 10000 //cas prepnuti z rucniho zpet do aut. rezimu v ms
#define INTERVAL_KOM 3000 // perioda odesilani dat v ms
unsigned long timNow, timMppt, timKey, timVA, timWR, timRuc, timKom, timDisp, timLed, timVyroba;

/// TLACITKA ///
int keyNow, keyPrev, keyLast, keyState;
enum { KEY_NONE, KEY_FIRST, KEY_REP, KEY_INVALID };
enum { KeyInv, KeyExit, KeyUp, KeyDown, KeySel };

/// MERENI ///
float fNapeti, fProud, fVykon;
float veff, veff_prev;
int napeti, proud, vykon, vykon_prev;
int TeplBojl; //skutecna teplota bojleru (TUV)
#define NSAMPLES 64 // pocet vzorku mereni (64 -> perioda hlavni smycky cca 30ms)
#define VCC 5000.0 // napajeci napeti v mV pro vypocty
#define ADCMAX 1023.0 // maximalni hodnota ADC odpovidajici napajecimu napeti
const float koefProud = VCC / ADCMAX / NSAMPLES; // 10mA/mV; LSB = 10mA
const float koefNapeti = VCC / ADCMAX * (R1 + R2) / R2 / 1000 / NSAMPLES;
float offsetA, rawProud;


unsigned long vyroba = 0;
unsigned long lastvyroba;
float whInc2;
bool nadproud = false;
bool ochrana = false;
bool zapis;
int strida = 8;
long topv;
int amp, nap, valA, valV;
float vyk;
byte restart = 0;
int smer = 1; //smer zmeny stridy 1 = zvysovani 2 = snizovani
bool rucne = false; //rezim rizeni
int addr_vyroba = 45;
int addr_suma_write = 55;
unsigned long SumaWrite; // aktualni pocet zapisu do EEPROM
unsigned long PreviousWrite;// predchozi hodnota zapisu do EEPROM ++90000
unsigned long FirstRun;// hodnota kontroly prvniho spusteni programu
unsigned long menuTime = 0;
unsigned long hodnotaL;
int teplotaMax = 90; //max teplota
int vykonMax = 2800, vykonMaxSts; //max vykon
int veffMax = 260; // max efektivni vystupni napeti
uint16_t timeTestMPPT = 20; //perioda kontroly krivky MPPT
int onVA = 1; // kontrola VA krivky aktivni
int maxV = 0; //max. namerene napeti
int maxA = 0; //max. namereny proud
int maxW = 0; //max. namereny vykon
int kalibV = 0; //kalibr. konstanta napeti
int kalibA = 0; //kalib. konstanta proudu
int KomTyp = 0; // typ komunikace 0 = Modbus RTU, 1 = EasyTransfer
int LCDbacklight = 5; // uroven podsviceni
int LCDbacklightLast; // posledni uroven podsviceni
bool set_kwh = false; // rezim nastaveni kWh

const char string_0[] PROGMEM =   "Exit"; //0
const char string_1[] PROGMEM =   "Max.teplota"; //1
const char string_2[] PROGMEM =   "Max.vykon"; //2
const char string_3[] PROGMEM =   "Max.Veff"; //2
const char string_4[] PROGMEM =   "Perioda testu VA"; //3
const char string_5[] PROGMEM =   "ID menice/adresa"; //4
const char string_6[] PROGMEM =   "Kalibrace V"; //5
const char string_7[] PROGMEM =   "Kalibrace A"; //6
const char string_8[] PROGMEM =   "Komunikace"; //7
const char string_9[] PROGMEM =   "Podsviceni LCD"; //8
const char string_10[] PROGMEM =  "Max. hodnoty"; //9
const char string_11[] PROGMEM =  "Tovarni RESET"; //10 rezerva
const char string_12[] PROGMEM =  " "; //11 rezerva
const char string_13[] PROGMEM =  "  *NASTAVENI*   "; //12
const char string_14[] PROGMEM =  "ukladam..."; //13
const char string_15[] PROGMEM =  "Suma vyroby"; //14


PGM_P const StringTable[] PROGMEM = {
  string_0,
  string_1,
  string_2,
  string_3,
  string_4,
  string_5,
  string_6,
  string_7,
  string_8,
  string_9,
  string_10,
  string_11,
  string_12,
  string_13,
  string_14,
  string_15
};


// struktura pro komunikaci
struct PayloadTX {
  byte nodeID, address, command, func;
  int data1, data2, data3, data4, data5, data6, data7, data8;
  long data9, data10, data11, data12;
};
PayloadTX emontx;   // vytvoreni instance
SoftEasyTransfer ET;

// pole dat pro modbus
unsigned int holdingdata[200];
unsigned int cnt_sl;
Modbus slave(nodeID, mySerial, TXenableRS485); // slave adresa,SoftwareSerial,RS485 enable pin

// Nastav pro LCD piny PCF8574AT a I2C adresu 0x3f(PCF8574T 0x27)
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C *lcd;
int lcdAddr;

void setBacklight(int8_t backlight) {
  Wire.beginTransmission(0x25);
  Wire.write(backlight);
  Wire.endTransmission();
  analogWrite(backlightPWMpin, ((10 - backlight) * 25) + 5);
}

int lcdCheck(int addr) {
  if (addr == -1) return 0;
  Wire.begin();
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void lcdInit(int addr) {
  lcdAddr = addr;
  if (lcdAddr == -1) return;
  if (lcd) {
    lcd->config(lcdAddr, 2, 1, 0, 4, 5, 6, 7);
  } else {
    lcd = new LiquidCrystal_I2C(lcdAddr, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  }
  lcd->begin(16, 2);
  lcd->backlight();
  lcd->createChar(1, stupen);
  setBacklight(LCDbacklight);
}

//==============================================================================
//=================================SETUP========================================
//==============================================================================
void setup() {
  wdt_enable(WDTO_8S);// aktivace watchdogu na 8s
  wdt_reset(); // vynulovani watchdogu
  //==============================================================================
  //nepouzite vyvody sepnout ke GND (omezeni vlivu ruseni)
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

	pinMode(backlightPWMpin, OUTPUT);
	digitalWrite(backlightPWMpin, LOW);

  // sepnout dolni mosfety pro kalibraci offsetA
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  //==============================================================================
  // Zapis vychozich hodnot do Eeprom pri prvnim spusteni programu.

  //nacte kontrolni hodnotu z konce Eeprom
  EEPROM.get(1020, FirstRun);

  //pokud hodnota nesouhlasi provede se prvotni inicializace Eeprom
  if (!(FirstRun == 0x710812AA))
  {
    EEPROM.put(2, addr_vyroba);// int
    EEPROM.put(6, addr_suma_write);// int
    EEPROM.put(8, PreviousWrite);// long
    EEPROM.put(12, nodeID);// int
    EEPROM.put(14, teplotaMax);// int
    EEPROM.put(16, vykonMax);// int
    EEPROM.put(18, timeTestMPPT);// int
    EEPROM.put(20, onVA);// int
    EEPROM.put(22, maxV);// int
    EEPROM.put(24, maxA);// int
    EEPROM.put(26, maxW);// int
    EEPROM.put(28, kalibV);// int
    EEPROM.put(30, kalibA);// int
    EEPROM.put(32, KomTyp);// int
    EEPROM.put(34, LCDbacklight);// int
    EEPROM.put(36, veffMax);
    EEPROM.put(addr_vyroba, vyroba);
    EEPROM.put(addr_suma_write, SumaWrite);
    //zapise kontrolni hodnotu o provedene prvotni inicializaci
    FirstRun = 0x710812AA;
    EEPROM.put(1020, FirstRun);
  }

  //==============================================================================

  EEPROM.get(2, addr_vyroba); //adresa pro nacteni sumy vyroby
  EEPROM.get(6, addr_suma_write); //adresa pro nacteni sumy zapisu do EEPROM
  EEPROM.get(8, PreviousWrite); //predchozi hodnota poctu zapisu do EEPROM
  EEPROM.get(12, nodeID);// int
  EEPROM.get(14, teplotaMax);// int
  EEPROM.get(16, vykonMax);// int
  EEPROM.get(18, timeTestMPPT);// int
  EEPROM.get(20, onVA);// int
  EEPROM.get(22, maxV);// int
  EEPROM.get(24, maxA);// int
  EEPROM.get(26, maxW);// int
  EEPROM.get(28, kalibV);// int
  EEPROM.get(30, kalibA);// int
  EEPROM.get(32, KomTyp);// int
  EEPROM.get(34, LCDbacklight);// int
  EEPROM.get(36, veffMax);
  EEPROM.get(addr_vyroba, vyroba); //nacteni vyroby z aktualni adresy
  EEPROM.get(addr_suma_write, SumaWrite);// nacteni sumy zapisu z akt. adresy
  lastvyroba = vyroba;

  pinMode(ochrana_pin, INPUT); //vstup stavu nadproudove ochrany
  pinMode(TXenableRS485, OUTPUT);
  digitalWrite(TXenableRS485, LOW);
  pinMode(LEDpin, OUTPUT);
  pinMode(runLED, OUTPUT);
  digitalWrite(runLED, HIGH);
  pinMode(mppt_pin, INPUT_PULLUP);//jumper rezerva
  pinMode(enable_pin, INPUT_PULLUP);


  // softwareSerial pro RS485
  mySerial.begin(9600);
  ET.begin(details(emontx), &mySerial);// nastaveni EasyTransfer
  slave.start(); // Modbus
  slave.setID(nodeID);// nastavi Modbus slave adresu
  delay(60);

  if (lcdCheck(0x27)) 
    lcdInit(0x27); 
  else {
    lcdInit(0x3f);
    if (!lcdCheck(0x3f)) lcdAddr = -1;
  }

  lcd->begin(16, 2); // pocet znaku, pocet radku
  lcd->createChar(1, stupen); // ulozi do LCD symbol stupnu
  lcd->backlight(); // zapni podsvetleni (noBacklight -vypni)

  //----------------Test stisku tlacitka pro nastaveni vyroby------------
  klavesa = KeyScan();
  if (klavesa == KeyUp) {
    delay(2000);
    if (klavesa == KeyUp) {
      showStatus = true;
      nastaveni_kwh();
    }
  }
  //---------------------------------------------------------------------
  lcd->setCursor(0, 0);
  lcd->print(" SOLAR INVERTER ");
  lcd->setCursor(0, 1);
  lcd->print("    SITON 210   ");
  delay(2500);
  wdt_reset(); //reset watchdogu
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print(" Tomas Nevrela  ");
  lcd->setCursor(0, 1);
  lcd->print(" tnweb.tode.cz  ");
  delay(2000);
  wdt_reset(); //reset watchdogu
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print(" fork24 v5.8    ");
  lcd->setCursor(0, 1);
  lcd->print(" JS 07/2025     ");
  delay(2000);

  // nastavit offsetA
  mereni();
  offsetA = rawProud;

  //-----------------reset nadproudove ochrany----------------------------------
  digitalWrite(ochrana_reset_pin, HIGH);
  pinMode(ochrana_reset_pin, OUTPUT);// vystup na HIGH
  delay(10);
  pinMode(ochrana_reset_pin, INPUT);// HI Impedance
  //----------------------------------------------------------------------------

}

//==============================================================================
//==============================Hlavni smycka===================================
//==============================================================================
void loop() {

  mereni(); // mereni hodnot
  if ((digitalRead(enable_pin)) || (TeplBojl >= teplotaMax))
    strida = 0; // provoz menice neni povolen, nastav stridu na 0
  else {
    rizeni(); // MPPT rizeni
    if (!vykonMaxSts) testVA(); // test VA krivky, vynechat pokud jsme blizko max. vykonu

    //------------------pusobeni nadproudove ochrany---------------------------------

    if (ochrana) {
      lcd->clear();
      lcd->setCursor(0, 0);// sloupec, radek
      lcd->print("  NADPROUDOVA  ");
      lcd->setCursor(0, 1);// sloupec, radek
      lcd->print("   OCHRANA!!   ");

      if (restart <= 4) //reset ochrany povolen 5x
      {
        strida = 8;//
        smer = 1;
        set_PWM(strida);

        for (int i = 0; i <= 15; i++) // cekej 15s
        {
          delay(1000);
          wdt_reset();
        }
        // odblokuj nadproudovou ochranu a zkus spustit menic
        digitalWrite(ochrana_reset_pin, HIGH);
        pinMode(ochrana_reset_pin, OUTPUT);// vystup na HIGH
        delay(10);
        pinMode(ochrana_reset_pin, INPUT); // vystup HI impedance
        ochrana = false; //zrus priznak aktivace ochrany
        restart++;
      }
      // pokud byla naproudova ochrana aktivovana vice nez 5x od posledndiho resetu
      // zustane vykonovy stupen zablokovany a musi se provest vypnuti a zapnuti menice
      else
      {
        strida = 0;
        set_PWM(strida);
        wdt_disable();
        while (1);
      }
    }
  }
  //------------------------------------------------------------------------------
  set_PWM(strida);
  zobrazeni();
  komunikace();
  //------------------------------------------------------------------------------
  // zapis vyroby do EEprom po 60 min.
  timNow = millis();
  if (timNow - timWR >= INTERVAL_WR)
  {
    zapis = true; //
    timWR = timNow;
  }
  //-----------------------------------------------------------------------------
  // signalizace behu programu
  timNow = millis();
  if (timNow - timLed >= 1000)
  {
    digitalWrite(runLED, LOW);
    delay(30);
    digitalWrite(runLED, HIGH);

    timLed = timNow;
  }

  //--------------------------------Zapis do EEPROM------------------------------
  // Zapis do EEPROM se provede pokud se vyroba zvysi o 100Wh
  //  posun adresy po 90000 zapisech
  if (zapis)
  {
    EEPROM.put(addr_vyroba, vyroba); //uloz vyrobu
    SumaWrite++;
    zapis = false;
    EEPROM.put(addr_suma_write, SumaWrite); //uloz celkovy pocet zapisu
    // pokud pocet zapisu do jedne bunky presahl 90000 posun se na dalsi
    if ((SumaWrite - PreviousWrite) >= 90000)
    {
      addr_vyroba += 10; // posun adresy pro zapis o 10
      addr_suma_write += 10;
      // zapis aktualnich hodnot do EEPROM
      EEPROM.put(2, addr_vyroba); //uloz akt. adresu vyroby
      EEPROM.put(6, addr_suma_write); //uloz akt. adresu sumy zapisu
      PreviousWrite = SumaWrite;
      EEPROM.put(8, PreviousWrite);
      EEPROM.put(addr_vyroba, vyroba);
      EEPROM.put(addr_suma_write, SumaWrite);
    }
  }

  //=============================Obsluha tlacitek==================================

  klavesa = KeyScan();
  if (klavesa == KeySel) showStatus = true; offset = 0;
  if (showStatus)mainMenu();
  if (!digitalRead(enable_pin)) //proved pokud je povolen provoz menice na vstupu enable
  {

    if (klavesa == KeyUp)
    {
      strida += 5;
      rucne = true; //po stisku tlacitka rezim zmeny stridy rucne
      timRuc = millis();
    }
    if (strida > maxStrida) strida = maxStrida;

    if (klavesa == KeyDown)
    {
      strida -= 5;
      rucne = true; //po stisku tlacitka rezim zmeny stridy rucne
      timRuc = millis();
    }
    if (strida < 8) strida = 8;

  }
}
//==========================konec hlavni smycky=================================
//==============================================================================



//===============================Komunikace=====================================
void komunikace()
{
  if (KomTyp) {
    timNow = millis();
    if (timNow - timKom >= INTERVAL_KOM)
    {
      //EasyTransfer
      // naplneni struktury dat
      emontx.data1 = napeti;
      emontx.data2 = proud;
      emontx.data3 = vykon;
      emontx.data4 = TeplBojl;
      emontx.data9 = vyroba; //vyroba ve Wh
      emontx.nodeID = nodeID; // ID solar invertoru
      emontx.command = 5; // odeslani dat bez pozadavku
      emontx.address = 15; // adresa emonHUBu
      digitalWrite(LEDpin, HIGH);// activity LED
      delay(15);
      digitalWrite(TXenableRS485, HIGH);//prepni prevodnik RS485 na vysilani
      ET.sendData(); // odesli data na emonHUB
      digitalWrite(TXenableRS485, LOW);
      delay(20);
      digitalWrite(LEDpin, LOW); // activity LED
      timKom = timNow;
    }
  } else {
    //Modbus RTU
    unsigned int HighINT;
    unsigned int LowINT;
    holdingdata[0] = nodeID;
    holdingdata[1] = 0;//address
    holdingdata[2] = 0;//command
    holdingdata[3] = 0;//func
    holdingdata[4] = napeti;
    holdingdata[5] = proud;
    holdingdata[6] = vykon;
    holdingdata[7] = TeplBojl;
    holdingdata[8] = strida;
    holdingdata[9] = 0;
    holdingdata[10] = 0;
    holdingdata[11] = 0;
    //vyroba = 4567892;
    HighINT = vyroba >> 16 & 0x0000ffffl ;//vyroba;
    LowINT = vyroba & 0x0000ffffl;

    holdingdata[12] = LowINT; //vyroba LO
    holdingdata[13] = HighINT; //vyroba HI
    holdingdata[14] = 0;
    holdingdata[15] = 0;
    holdingdata[16] = 0;
    holdingdata[17] = 0;
    holdingdata[18] = vykonMaxSts;
    holdingdata[19] = offsetA;

    slave.poll( holdingdata, 200 );

    if (cnt_sl != slave.getOutCnt()) {
      digitalWrite(LEDpin, HIGH);
      delay(40);
      digitalWrite(LEDpin, LOW);
    }
    cnt_sl = slave.getOutCnt();

  }

}

//===============================Nastaveni PWM==================================
//==============================================================================
//Nastavi casovace PWM, max. hodnota OCR1A=1235, OCR1B=1265 pro dostatecny dead
//time (120us)
//vystup na pinech D9 a D10 casove posunute o 10ms
void set_PWM(int dutyCycle)
{
  if (nadproud) { // pri nadproudu sniz stridu
    dutyCycle = dutyCycle - 25;
    if (dutyCycle <= 5) dutyCycle = 5;
    strida = dutyCycle;

    smer = 2;//smer dolu
    lcd->clear();
    lcd->setCursor(0, 0);// sloupec, radek
    lcd->print("   NADPROUD!   ");
  }
  if (dutyCycle > maxStrida) dutyCycle = maxStrida;
  else if (dutyCycle < 0) dutyCycle = 0;
  cli();
  TCCR1B = _BV(WGM13) | _BV(CS11) | _BV(CS10);// rezim 8,clock/64

  int topv = (F_CPU / (frekvence * 2 * 64));//16000000/(50*2*64)=2500
  ICR1 = topv;
  OCR1A = (dutyCycle * 5); //od 0 do 1235
  OCR1B = ( topv - (dutyCycle * 5)); //od 2500 do 1265

  DDRB |= _BV(PORTB1) | (_BV(PORTB2)); //D9(PB1), D10(PB2) nastav na vystup
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0); //rezim vyvodu OC1A, OC1B
  sei();
}
//===============================Mereni=========================================
//==============================================================================
//mereni proudu,napeti a teploty, vypocet vykonu a vyroby,
//prevod potrebnych hodnot do int
void mereni()
{
  uint32_t sumV = 0;
  int32_t sumA = 0;
  uint32_t sumT = 0;


  wdt_reset();

  for (int i = 0; i < NSAMPLES; i++)
  {
    sumA = sumA + analogRead(pinA) - 512; //zmer proud
    sumV = sumV + analogRead(pinV); //zmer napeti
    sumT = sumT + analogRead(cidlo_B);//teplota
  }

  // vypocet proudu
  rawProud = koefProud * sumA;
  fProud = (rawProud - offsetA) * (1 + 0.001 * kalibA);
  proud = fProud;

  if (proud <= 9) proud = 0;//kalibrace 0
  nadproud = proud >= Maxproud;

  // vypocet napeti
  fNapeti = (koefNapeti * sumV) * (1 + 0.001 * kalibV);
  napeti = fNapeti;

  // vypocet teploty
  if (sumT > 1020L * NSAMPLES) // pokud je odpojené termočidlo nastav 0 oC
  {
    sumT = 437 * NSAMPLES;
  }
  TeplBojl = vypocet(sumT); //vypocet teploty

  // vypocet vykonu
  fVykon = fNapeti * fProud * 0.01; //okamzity vykon
  vykon = fVykon;

  // vypocet efektivni hodnoty napeti
  veff = sqrt(fNapeti * fNapeti * strida * 0.004);

  // maximalni hodnoty
  if (napeti > maxV) {
    maxV = napeti;
    EEPROM.put(22, maxV);//
  }
  if (proud > maxA) {
    maxA = proud;
    EEPROM.put(24, maxA);//
  }
  if (vykon > maxW) {
    maxW = vykon;
    EEPROM.put(26, maxW);//
  }

  //---------------------------------------------
  if (!digitalRead (ochrana_pin)) ochrana = true;// pri aktivni nadproudove ochrane nastav priznak

  //=====================Vypocet vyroby Wh====================================
  unsigned long ltimVyroba = timVyroba;
  timVyroba = millis();
  float whInc = fVykon * ((timVyroba - ltimVyroba) / 3600000.0);
  whInc2 = whInc2 + whInc;
  if (whInc2 > 1.0)
  {
    whInc2 = whInc2 - 1.0;
    vyroba++;
    if (vyroba >= 100000000) //maximalni rozsah 100000.00kWh
    {
      vyroba = 0;
      lastvyroba = vyroba;
    }
  }
  if ((vyroba - lastvyroba) >= 100) //zapis do eeprom pri zvyseni o 0.1kWh
  {
    zapis = true;
    lastvyroba = vyroba;
  }

}
//============================================================================
// vypocte z analogove hodnoty teplotu
int16_t vypocet(uint32_t value) {
  float fAdc = value / NSAMPLES;

  float a = 0.007874 * R25;
  float b = 0.00001874 * R25;
  float c = R25 - RDIV * fAdc / (ADCMAX - fAdc);
  float d = sqrt(a * a - 4 * b * c);
  return (int16_t) ((d - a) / (2 * b) + 25);
}

//==========================Rizeni vykonu MPPT==================================
//==============================================================================
//hledani max. vykonu fotovoltaickych panelu
void rizeni()
{
  timNow = millis();
  if (timNow - timRuc >= INTERVAL_RUC) rucne = false;


  if (rucne == false)
  {

    if (vykon == 0) {
      strida = 24;  //pri nulovem vykonu nastav zakladni stridu a smer
      smer = 1; //zvysovani
    }
    //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    // omezeni stridy pri max vykonu
    if (vykon >= vykonMax || veff > veffMax) {
      strida -= 4;
      if (strida < 5) {
        strida = 5;
        smer = 1;
      }
      smer = 2;
      vykonMaxSts = 10;
    } else if (timNow - timMppt > 500) {
      timMppt = timNow;
      //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
      //smer nahoru
      switch (smer) {
        case 1:
          {
            if (veff >= veff_prev || vykon >= vykon_prev) {
              strida++; //pokud se vykon zvysuje pri smeru nahoru zvys stridu
              if (strida > maxStrida) {
                strida = maxStrida;
                smer = 2;
              }
            }

            else smer = 2; //pokud je vykon nizsi zmen smer
            break;
          }
        //smer dolu
        case 2:
          {
            if ((veff >= veff_prev || vykon >= vykon_prev) && vykon > 0) {
              strida--; //pokud se vykon zvysuje pri smeru dolu sniz stridu
              if (strida < 5) {
                strida = 5;
                smer = 1;
              }
            }
            else smer = 1; //pokud je vykon nizsi zmen smer
            break;
          }
      }
      veff_prev = veff;
      vykon_prev = vykon;
      if (vykonMaxSts) vykonMaxSts--;
    }
  }
}

//==========================Test VA krivky =====================================
//==============================================================================
//nucene projede VA krivku FV z min. do max. stridy, pokud najde vyssi vykon
//nastavi stridu na tuto novou hodnotu
void testVA ()
{
  int regIdx;

  if (onVA)
  {
    // povolen test VA krivky
    timNow = millis();
    if ((timNow - timVA >= INTERVAL_VA * timeTestMPPT) &&
//      (strida > 50 && strida < 185) &&
      (!rucne))
    {
      //kontrola VA krivky pokud ubehl nastaveny interval
      //neni ruc. rezim a strida je mezi 50 a 185
      int vykonMpp = 0;
      int veffMpp = 0;
      int stridaMpp = 0;
      vykon = 0;
      lcd->clear();
      lcd->setCursor(0, 0);// sloupec, radek
      lcd->print("   Test MPPT    ");
      int m = 10;
      regIdx = 0;
      while (m < maxStrida && vykon < vykonMax && veff < veffMax)
      {
        strida = m;
        set_PWM(strida);
        lcd->setCursor(0, 1);// sloupec, radek
        if ((int)(strida / 2.5) < 10) lcd->print(" ");
        lcd->print((int)(strida / 2.5));
        lcd->print("% ");
        //lcd->print(m);
        if (strida == 10)
          delay(400); // cas na nabiti kondenzatoru
        else
          delay(100);
        mereni();
        if (vykon > vykonMpp && veff > veffMpp)
        {
          vykonMpp = vykon;
          veffMpp = veff;
          stridaMpp = strida;
        }
        if (regIdx < 48) {
          holdingdata[regIdx * 2 + 21] = napeti;
          holdingdata[regIdx * 2 + 22] = proud;
        }
        m += 5;
        regIdx++;
      }
      vykon = vykonMpp;
      veff = veffMpp;
      strida = stridaMpp;
      while (regIdx < 48) { // pokud byl prekrocen max. vykon, doplnit krivku nulami
          holdingdata[regIdx * 2 + 21] = 0;
          holdingdata[regIdx * 2 + 22] = 0;
          regIdx++;
      }
      if (holdingdata[20] < 100)
        holdingdata[20]++;
      else
        holdingdata[20] = 1;
      timVA = timNow;
    }
  }
}


//==============================Zobrazeni na LCD================================
//==============================================================================
void zobrazeni()
{
  //wdt_reset();
  unsigned long timNow = millis();
  if (timNow - timDisp >= INTERVAL_LCD) {
    if (!lcdCheck(lcdAddr)) {
      if (lcdCheck(0x27))
        lcdInit(0x27);
      else if (lcdCheck(0x3f))
        lcdInit(0x3f);
      else
        lcdAddr = -1;
    }

    lcd->setCursor(0, 0);// sloupec, radek
    if (napeti < 100 && napeti > 9) lcd->print(" ");
    if (napeti < 10) lcd->print("  ");
    lcd->print(napeti);
    lcd->print("V ");

    lcd->setCursor(5, 0);
    if (proud < 1000) lcd->print(" ");
    lcd->print(proud / 100);
    lcd->print(".");
    lcd->print((proud % 100) / 10);
    lcd->print("A ");

    lcd->setCursor(11, 0);
    if (vykon < 1000 && vykon > 99) lcd->print(" ");
    if (vykon < 100 && vykon > 9) lcd->print("  ");
    if (vykon < 10) lcd->print("   ");
    lcd->print(vykon);
    lcd->print("W");

    if (rucne)
    {
      lcd->setCursor(0, 1);
      lcd->print("strida:");
      if ((int)(strida / 2.5) < 10) lcd->print(" ");
      lcd->print((int)(strida / 2.5));
      lcd->print("%      ");
      setBacklight(LCDbacklight);
    } else {
      lcd->setCursor(0, 1);
      if (vyroba < 10000000) lcd->print(" ");
      if (vyroba < 1000000) lcd->print(" ");
      if (vyroba < 100000) lcd->print(" ");
      if (vyroba < 10000) lcd->print(" ");
      lcd->print(vyroba / 1000);
      lcd->print(".");
      if (((vyroba % 1000) / 10) < 10) lcd->print("0");
      lcd->print((vyroba % 1000) / 10);
      lcd->print("kWh   ");

      lcd->setCursor(11, 1);
      if (TeplBojl < 100 && TeplBojl > 9) lcd->print(" ");
      if (TeplBojl < 10 && TeplBojl >= 0) lcd->print("  ");

      if (TeplBojl <= 0 || TeplBojl > 120)// kdyz je teplota mimo rozsah
      {
        lcd->setCursor(11, 1);
        lcd->print(" --");
      }
      else {
        lcd->print(TeplBojl);
      }
      lcd->write(1);
      lcd->print("C");
      setBacklight(LCDbacklight);
    }
    timDisp = timNow;
  }
}

//================ Nastaveni vyroby kWh pri zapnuti===================
//====================================================================
void nastaveni_kwh() {

  LCDbacklightLast = LCDbacklight; // uloz puvodni hodnotu podsviceni
  LCDbacklight = 10; // nastav podsviceni na max.
  set_kwh = true;
  byte Pkwh [7];
  int hodnota = 0;
  int pozice = 6;
  hodnotaL = vyroba;
  // rozdeli jednotliva cisla do poli
  Pkwh[0] = hodnotaL /  10000000;
  Pkwh[1] = (hodnotaL % 10000000) / 1000000;
  Pkwh[2] = (hodnotaL % 1000000) / 100000;
  Pkwh[3] = (hodnotaL % 100000) / 10000;
  Pkwh[4] = (hodnotaL % 10000) / 1000;
  Pkwh[6] = (hodnotaL % 1000) / 100;

  zobraz_kwh();
  delay(1000);

  do {
    zobraz_kwh();
    lcd->setCursor(pozice, 1);
    whichkey = PollKey();
    switch (whichkey) {

      case KeyDown: // zmena pozice nastavovaneho cisla
        pozice--;
        if (pozice == 5) pozice = 4; //preskoc desetinnou tecku
        if (pozice < 0) { //prejdi zase vpravo
          pozice = 6;
        }
        break;

      case KeyUp: //nastaveni cisla
        hodnota = Pkwh[pozice];
        hodnota++;
        if (hodnota > 9) hodnota = 0;
        Pkwh[pozice] = hodnota;
        hodnotaL = (((long)Pkwh[0] * 100000) + ((long)Pkwh[1] * 10000) + ((long)Pkwh[2] * 1000) + ((long)Pkwh[3] * 100) + ((long)Pkwh[4] * 10) + ((long)Pkwh[6]));
        hodnotaL *= 100; // uloz pole do long cisla

        break;

      case KeySel: // uloz nastavenou vyrobu
        vyroba = hodnotaL;
        lcd->noBlink();
        EEPROM.put(addr_vyroba, vyroba); //uloz do eeprom
        ulozeno(); break;

    }
  } while (showStatus);
  lcd->noBlink();
  set_kwh = false;
  LCDbacklight = LCDbacklightLast; // obnov puvodni hodnotu podsviceni
}
// ==================Zobrazeni celeho cisla kWh s nulami======================
void zobraz_kwh()
{
  lcd->clear();

  PrintLCD_P(15); // zobr. textu Suma vyroby
  lcd->setCursor(0, 1);
  if (hodnotaL < 10000000) lcd->print("0");
  if (hodnotaL < 1000000) lcd->print("0");
  if (hodnotaL < 100000) lcd->print("0");
  if (hodnotaL < 10000) lcd->print("0");
  lcd->print(hodnotaL / 1000);
  lcd->print(".");
  lcd->print((hodnotaL % 1000) / 100);
  lcd->print("kWh");
  lcd->blink();
}
//=============================================================================





//================================== LCD MENU==================================
//=============================================================================

void mainMenu() {
  menuTime = millis();
  LCDbacklightLast = LCDbacklight; // uloz puvodni hodnotu podsviceni
  do {
    delay(50);
    lcd->clear();
    LCDbacklight = 10; // nastav podsviceni na max.
    PrintLCD_P(13); // zobr. text *NASTAVENI*
    PrintLCDAt_P(offset , 0, 1);
    delay(200);
    whichkey = PollKey();
    switch (whichkey) {

      case KeyUp:
        if (offset >= 0) offset++;
        if (offset > 11) offset = 0;
        break;

      case KeyDown:
        switch (offset) {
          case 0: //exit z menu
            menu_exit();
            break;
          case 1: //nastaveni max. teploty
            menu_Mteplota();
            break;
          case 2: //nastaveni max. vykonu
            menu_Mvykon();
            break;
          case 3: //nastaveni max. vykonu
            menu_Mveff();
            break;
          case 4: //nastaveni periody testu VA krivky
            menu_perVA();
            break;
          case 5: //nastaveni ID menice
            menu_zmenaID();
            break;
          case 6: //kalibrace napeti
            menu_kalibraceV();
            break;
          case 7: //kalibrace proudu
            menu_kalibraceA();
            break;
          case 8: //nastaveni komunikace
            menu_komunikace();
            break;
          case 9: //podsviceni LCD
            menu_LCDbacklight();
            break;
          case 10: //max hodnoty
            menu_maxHodnoty();
            break;
          case 11: //max hodnoty
            menu_TovReset();
            break;
        }
        break;

    }
    lcd->clear();
  } while (showStatus);
  LCDbacklight = LCDbacklightLast; // obnov puvodni hodnotu podsviceni
}

//===================================================================
// nastaveni ID menice
void menu_zmenaID() {
  int hodnota = nodeID;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(1, 1);
    lcd->print(hodnota);
    whichkey = PollKey();
    switch (whichkey) {
      case KeyUp:
        if (hodnota >= 10) hodnota++;
        if (hodnota > 20) hodnota = 10;
        break;
      case KeyDown:
        nodeID = hodnota;
        slave.setID(nodeID);// nastavi Modbus slave adresu
        EEPROM.put(12, nodeID); //uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);
}




//===================================================================
void menu_Mteplota() {
  int hodnota = teplotaMax;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(0, 1);
    lcd->print(hodnota);
    lcd->write(1);//znacka stupne
    lcd->print("C ");
    whichkey = PollKey();
    switch (whichkey) {
      case KeyUp:
        if (hodnota >= 40) hodnota += 1 ;
        if (hodnota > 90) hodnota = 40 ;
        break;
      case KeyDown:
        teplotaMax = hodnota;
        EEPROM.put(14, teplotaMax); //uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);
}

//===================================================================
void menu_Mvykon() {
  int hodnota = vykonMax;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(0, 1);
    lcd->print(hodnota);
    lcd->print("W ");
    whichkey = PollKey();
    switch (whichkey) {
      case KeyUp:
        if (hodnota >= 1000) hodnota += 100 ;
        if (hodnota > 4000) hodnota = 1000 ;
        break;
      case KeyDown:
        vykonMax = hodnota;
        EEPROM.put(16, vykonMax); //uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);
}

//===================================================================
void menu_Mveff() {
  int hodnota = veffMax;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(0, 1);
    lcd->print(hodnota);
    lcd->print("V ");
    whichkey = PollKey();
    switch (whichkey) {
      case KeyUp:
        if (hodnota >= 200) hodnota += 10;
        if (hodnota > 380) hodnota = 200;
        break;
      case KeyDown:
        veffMax = hodnota;
        EEPROM.put(36, veffMax); //uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);
}

//===================================================================

void menu_maxHodnoty() {

  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(0, 1);
    lcd->print(maxV);
    lcd->print("V ");
    lcd->print(maxA / 100);
    lcd->print(".");
    lcd->print((maxA % 100) / 10);
    lcd->print("A ");
    lcd->print(maxW);
    lcd->print("W");
    whichkey = PollKey();
    switch (whichkey) {
      case KeyUp:

        break;
      case KeySel:
        maxV = 0;
        maxA = 0;
        maxW = 0;
        EEPROM.put(22, maxV);// int
        EEPROM.put(24, maxA);// int
        EEPROM.put(26, maxW);// int
        break;
      case KeyDown:
        showStatus = false;
        break;

    }
  } while (showStatus);
}


//===================================================================

void menu_perVA() {
  int hodnota = timeTestMPPT;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(0, 1);
    if (hodnota >= 0 && hodnota < 10) lcd->print(" "); //zapise mezeru pred jednomistne cislo
    lcd->print(hodnota);
    lcd->print(" min.");
    whichkey = PollKey();
    switch (whichkey) {
      case KeyUp:
        if (hodnota >= 0) hodnota += 5;//+=5
        if (hodnota > 60) hodnota = 0;//60
        break;
      case KeyDown:
        timeTestMPPT = hodnota;
        if (timeTestMPPT == 0) onVA = 0;
        else onVA = 1;
        EEPROM.put(20, onVA); //uloz do eeprom

        EEPROM.put(18, timeTestMPPT);//uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);
}

//===================================================================
// kalibrace napeti
void menu_kalibraceV() {
  int hodnota = kalibV; //
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(1, 1);
    if (hodnota > 0) lcd->print("+");
    if (hodnota == 0) lcd->print(" ");
    if (hodnota == -5) lcd->print("-");
    lcd->print(hodnota / 10);
    lcd->print(".");
    lcd->print(abs(hodnota % 10));
    lcd->print("%  ");
    whichkey = PollKey();
    switch (whichkey) {

      case KeyUp:
        if (hodnota >= -50) hodnota += 5;
        if (hodnota > 50) hodnota = -50;
        break;
      case KeyDown:
        kalibV = hodnota;
        EEPROM.put(28, kalibV); //uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);
}

//===================================================================
// kalibrace proudu
void menu_kalibraceA() {
  int hodnota = kalibA;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(1, 1);
    if (hodnota > 0) lcd->print("+");
    if (hodnota == 0) lcd->print(" ");
    if (hodnota == -5) lcd->print("-");
    lcd->print(hodnota / 10);
    lcd->print(".");
    lcd->print(abs(hodnota % 10));
    lcd->print("%  ");
    whichkey = PollKey();
    switch (whichkey) {

      case KeyUp:
        if (hodnota >= -50) hodnota += 5;
        if (hodnota > 50) hodnota = -50;
        break;
      case KeyDown:
        kalibA = hodnota;
        EEPROM.put(30, kalibA); //uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);
}
//===================================================================
// nastaveni typu komunikace
void menu_komunikace() {
  int hodnota = KomTyp;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(0, 1);
    if (hodnota == 1)lcd->print("EasyTransfer");
    else lcd->print("Modbus RTU  ");
    whichkey = PollKey();
    switch (whichkey) {
      //case KeyDown:
      //  if (hodnota > 0) hodnota--; break;
      case KeyUp:
        if (hodnota >= 0) hodnota++;
        if (hodnota > 1) hodnota = 0;
        break;
      case KeyDown:
        KomTyp = hodnota;
        EEPROM.put(32, KomTyp); //uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);


}
//===================================================================
void menu_TovReset() {
  int hodnota = 1;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(0, 1);
    if (hodnota == 1)lcd->print("NE        ");
    else lcd->print("ANO        ");
    whichkey = PollKey();
    switch (whichkey) {
      case KeyUp:
        if (hodnota >= 0) hodnota++;
        if (hodnota > 1) hodnota = 0;
        break;
      case KeyDown:
        showStatus = false;
        if (hodnota == 0) {
          FirstRun = 0x00000000; //nastav odlisnou hodnotu
          EEPROM.put(1020, FirstRun);
          wdt_enable(WDTO_1S); //zkraceni periody watchdogu
          delay(2000);// cekej na reset
        }
        break;

    }
  } while (showStatus);


}
//===================================================================
// nastaveni podsviceni
void menu_LCDbacklight() {

  EEPROM.get(34, LCDbacklight);
  int hodnota = LCDbacklight;
  do {
    lcd->clear();
    PrintLCDAt_P(offset, 0, 0);
    lcd->setCursor(1, 1);
    lcd->print(hodnota);
    whichkey = PollKey();
    switch (whichkey) {
      case KeyUp:
        if (hodnota >= 0) hodnota++;
        if (hodnota > 10) hodnota = 0;
        LCDbacklight = hodnota;
        break;
      case KeyDown:
        LCDbacklight = hodnota;
        LCDbacklightLast = hodnota;
        EEPROM.put(34, LCDbacklight); //uloz do eeprom
        ulozeno(); break;
    }
  } while (showStatus);
}



void menu_exit() {

  showStatus = false;
}



//====================================================================
// vypis textu "ukladam..." na LCD

void ulozeno() {
  lcd->clear();
  PrintLCDAt_P(14, 0, 0);//zobrazi "ukladam..."
  delay(500);
  showStatus = false;
}

//===================================================================

void PrintLCDAt(char *inStr, char x, char y) {
  lcd->setCursor( x, y );
  delay(20);
  lcd->print(inStr);
  delay(40);
}
//===================================================================

void PrintLCDAt_P(int which, char x, char y) {
  lcd->setCursor( x, y );
  delay(20);
  PrintLCD_P(which);
}
//===================================================================

void PrintLCD_P(int which) {
  char buffer[21];
  strcpy_P(buffer, (char*)pgm_read_word(&(StringTable[which])));
  lcd->print(buffer);
  delay(40);
}
//===================================================================
//kontroluje analog. hodnotu stisknuteho tlacitka

char KeyScan() {
  wdt_reset();

  timNow = millis();
  unsigned int adc = analogRead(Buttons);

  keyNow = KeyInv;

  if (adc > 710 && adc < 800) keyNow = KeySel; // obe tlacitka
  if (adc > 600 && adc < 700) keyNow = KeyUp; // tlac. nahoru
  if (adc > 400 && adc < 550) keyNow = KeyDown; // tlac. dolu
  if (adc > 210 && adc < 300) keyNow = KeyExit; // nepouzito

  if (keyNow != keyPrev) {
    timKey = timNow;
    keyPrev = keyNow;
  } else if (keyNow == KeyInv && timNow - timKey > 200) { // neni stisknuto tlacitko - navrat do normalniho stavu
    keyLast = KeyInv;
    keyState = KEY_NONE;
    timKey = timNow;
  } else if (keyNow != KeyInv && keyState == KEY_NONE && timNow - timKey > 40) { // stisk tlacitka
    keyLast = keyNow;
    keyState = KEY_FIRST;
    timKey = timNow;
    return keyNow;
  } else if (keyNow == keyLast && keyState == KEY_FIRST && timNow - timKey > 1000) { // podrzeni tlacitka - prvotni prodleva
    keyState = KEY_REP;
    timKey = timNow;
    return keyNow;
  } else if (keyNow == keyLast && keyState == KEY_REP && timNow - timKey > 400) { // podrzeni tlacitka - interval opakovani
    timKey = timNow;
    return keyNow;
  } else {
    if (keyNow != keyLast && keyState != KEY_NONE) // zmena stisknutych tlacitek - ignorovat
      keyState = KEY_INVALID;
  }
  return KeyInv;
}

//==================================================================
//pri pohybu v Menu ceka na platnou hodnotu tlacitka,
//

char PollKey() {

  char Whichkey;
  do {
    Whichkey = KeyScan();
    delay(60);
    //cinosti vykonavane behem zobrazeni menu
    if (set_kwh) strida = 0; // provoz menice neni povolen, nastav stridu na 0
    else {
      mereni();
      rizeni();
    }
    set_PWM(strida);
    setBacklight(LCDbacklight);
    //pri necinnosti v menu zped na hlavni obrazovku
    if (millis() > menuTime + menuTimeExit) {
      showStatus = false;
    }

  } while ((Whichkey == KeyInv) && showStatus);
  menuTime = millis();
  delay(80);
  return Whichkey;
}