//Controlador de chocadeira PID
//Ryan Monteiro - 12/2022 - Versão 2.0

//Para alterar os paramtros de chocagem basta mudar as configs em setParametersByStage().

#include <DallasTemperature.h>
#include <OneWire.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v2.h>
#include <TimeLib.h>
#include <EEPROM.h>

//Porta do pino de sinal do DS18B20
#define ONE_WIRE_BUS 2
//Porta do pino de sinal e tipo do DHT
#define DHTPIN 3
#define DHTTYPE DHT11

//Definição dos pinos de saída
#define HEATER_PIN   6
#define RESET_PIN    7
#define ROTATOR_PIN  8

//Definição de parametros do PID
#define Kp 50
#define Ki 10
#define Kd  0

//Definição dos slots da memoria EEPROM
#define eepromTime    10
#define eepromStage   11
#define eepromRotTime 12

const int LCD_addr = 0x3F;  // Endereço LCD I2C
const int LCD_chars = 16;   // Numero de caracteres por linha
const int LCD_lines = 2;    // Numero de linhas

//Define uma instancia do oneWire para comunicar com o sensor
OneWire oneWire(ONE_WIRE_BUS);
//Define uma instancia do oneWire para comunicar com o sensor
DHT dht (DHTPIN, DHTTYPE);
//Define uma instancia do liquidCrystal para comunicar com o LCD
LiquidCrystal_I2C lcd(LCD_addr, LCD_chars, LCD_lines);
//Define uma instancia do PID_V2
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);
//Define uma instancia do tipo time para controle do tempo
time_t t, startTime, auxTime, rotTime;

//Definicao dos caracteres customizaveis do LCD
uint8_t tempSimbol[8]      = {0x04,0x0A,0x0A,0x0E,0x0E,0x1F,0x1F,0x0E};
uint8_t degreeSimbol[8]    = {0x08,0x14,0x08,0x00,0x00,0x00,0x00,0x00};
uint8_t clockSimbol[8]     = {0x00,0x0E,0x15,0x15,0x17,0x11,0x0E,0x00};
uint8_t humiditySimbol[8]  = {0x04,0x04,0x0E,0x0E,0x1F,0x1F,0x1F,0x0E};
uint8_t rotationSimbol[8]  = {0x07,0x06,0x15,0x11,0x11,0x15,0x0C,0x1C};

DallasTemperature sensors(&oneWire);
DeviceAddress sensor1;

//Definição das variaveis globais
float probeTemp;
float probeHumdt;
double pidSetpoint, pidInput, pidOutput;
int savedTime, savedStage, savedRotTime;
boolean rotation = false;
unsigned long targetInSeconds,targetDay, targetHour;
unsigned long lastSync, lastrotSync, leftTime, leftRotTime;


//Prototipo das funções
void getTemp(void);
void getHumdt(void);
void homeScreen(void);
void controlTemp(void);
void setParametersByStage(void);
void controlRoll(void);
void convertTimeToSec(void);
void controlTime(void);

void setup() {
    Serial.begin(115200);
    //Inicializa DS18B20
    sensors.begin();
    //Inicializa DHT
    dht.begin();
    //Inicializa e limpa LCD
    lcd.init();
    lcd.clear();
    lcd.backlight();

    //Cria os caracteres personalizados
    lcd.createChar(0, tempSimbol);
    lcd.createChar(1, degreeSimbol);
    lcd.createChar(2, clockSimbol);
    lcd.createChar(3, humiditySimbol);
    lcd.createChar(4, rotationSimbol);

    //Localiza e mostra o endereço dos sensores DS18B20
    Serial.println("Localizando sensores DS18B20...");
	Serial.print("Foram encontrados ");
	Serial.print(sensors.getDeviceCount(), DEC);
	Serial.println(" Sensor(es)");
	sensors.getAddress(sensor1, 5);

    //Carrega da EEPROM os valores salvos de datas e estagio.
    EEPROM.get(eepromTime, savedTime);
    EEPROM.get(eepromStage, savedStage);
    EEPROM.get(eepromRotTime, savedRotTime);
    //savedStage =  2; //Para simular outras funcoes
    
    //Chama a função para setar os parametros
    setParametersByStage();

    //Seta as informações de saida do rotacionador
    pinMode(ROTATOR_PIN, OUTPUT);
    pinMode(RESET_PIN, INPUT_PULLUP);
    digitalWrite(ROTATOR_PIN, HIGH);

    //Captura e armazena o tempo ao iniciar;
    startTime = now();

    //define os parametros de sincronização para 0, reeiniciando contador de sincronização a cada boot do arduino
    lastSync = 0;
    lastrotSync = 0;

    //Define os limites da saida PID
    myPID.SetOutputLimits(0, 255);
    //Inicializa o controlador PID passando a leitura do sensor de temperatura e o alvo
    myPID.Start(pidInput, pidOutput, pidSetpoint);
}

void loop() {
    //Captura as informações dos sensores
    getTemp();
    getHumdt();
    //Gera a tela de inicio
    homeScreen();
    //Chama a função de controlar a temperatura
    controlTemp();
    //Chama a função de girar os ovos na incubadora
    if (rotation == true){
        controlRoll();   
    }
    //Calcula o tempo restante e qual o estagio por input automaticamente
    controlTime();

    if (digitalRead(RESET_PIN) == LOW)
    {
        savedStage = 2;
        savedTime = 0;
        savedRotTime = 0;
        setParametersByStage();
        EEPROM.put(eepromStage, savedStage);
        Serial.println("Estagio salvo na eeprom");
    }
    
}

//Função para ler e tratar a temperatura do sensor
void getTemp(){
    sensors.requestTemperatures();
    probeTemp = sensors.getTempC(sensor1);
    //Serial.print("Temperatura:");
    //Serial.println(probeTemp);
}

//Função para exibição da temperatura no LCD
void showTemp(){
    lcd.home();
	lcd.write(0);
	lcd.print(probeTemp);
    lcd.setCursor(5, 0);
	lcd.write(1);
}

//Função para ler e tratar a umidade do sensor
void getHumdt(){
    probeHumdt = dht.readHumidity();
    //Serial.print("Umidade:");
    //Serial.println(probeHumdt);
}

//Função para exibição da umidade no LCD
void showHumdt(){
    lcd.setCursor(0,1);
	lcd.write(3);
	lcd.print(probeHumdt,1);
    lcd.print("%");
}

//Função para gerenciar a temperatura interna com o resultado do calculo PID
void controlTemp(){
    pidInput = probeTemp;
    pidOutput = myPID.Run(pidInput);
    analogWrite(HEATER_PIN, pidOutput);
    Serial.println(pidOutput);
}

// Define os parametros de acordo com o estagio de chocagem
void setParametersByStage(){
// implementar os parametros e a definicão do estagio
    switch (savedStage) {
    case 1: //Pré Aquecimento
        pidSetpoint = 27.0;
        rotation = false;
        targetDay = 00;
        targetHour = 12;
        break;
    case 2: //Incubação
        pidSetpoint = 37.7;
        rotation = true;
        targetDay = 18;
        targetHour = 00;
        break;
    case 3:
        pidSetpoint = 36.8;
        rotation = false;
        targetDay = 05;
        targetHour = 00;
        break;
    default:
        pidSetpoint = 0;
        rotation = false;
        targetDay = 00;
        targetHour = 00;
        break;
    }
    
    //Chama a função para convertert o tempo para segundos;
    convertTimeToSec();
}

//Função para exibir o estagio na tela inicial
void showStage(){
    //Serial.println(leftTime);
    unsigned long day;
    unsigned long hour;
    unsigned long min;
    day = (leftTime / 86400);
    hour = ((leftTime % 86400)/3600);
    min = (((leftTime % 86400) % 3600)/60);

    switch (savedStage) {
    case 1:
        lcd.setCursor(7, 0);
	    lcd.print("AQUECENDO");
        lcd.setCursor(7,1);
        if (day < 10){
            lcd.print("0");
        }
        lcd.print(day);
        lcd.print("D");
        if (hour < 10){
            lcd.print("0");
        }
        lcd.print(hour);
        lcd.print("H");
        if (min < 10){
            lcd.print("0");
        }
        lcd.print(min);
        lcd.print("M");    
        break;
    case 2:
        lcd.setCursor(7, 0);
	    lcd.print("INCUBANDO");
        lcd.setCursor(7,1);
        if (day < 10){
            lcd.print("0");
        }
        lcd.print(day);
        lcd.print("D");
        if (hour < 10){
            lcd.print("0");
        }
        lcd.print(hour);
        lcd.print("H");
        if (min < 10){
            lcd.print("0");
        }
        lcd.print(min);
        lcd.print("M"); 
        break;
    case 3:
        lcd.setCursor(7, 0);
	    lcd.print("NASCENDO");
        lcd.setCursor(7,1);
        if (day < 10){
            lcd.print("0");
        }
        lcd.print(day);
        lcd.print("D");
        if (hour < 10){
            lcd.print("0");
        }
        lcd.print(hour);
        lcd.print("H");
        if (min < 10){
            lcd.print("0");
        }
        lcd.print(min);
        lcd.print("M"); 
        break;
    default:
        lcd.setCursor(7, 0);
	    lcd.print("DESLIGADO");
        break;
    }
}

void showRollTime(){
    //Serial.println(leftRotTime);
    unsigned long min;
    min = (((leftRotTime % 86400) % 3600)/60);

    lcd.setCursor(7, 0);
	lcd.print("VIRANDO");
    lcd.setCursor(7,1);
    lcd.print("EM: ");
    lcd.print("0");
    lcd.print(min);
    lcd.print("Min");
}

//Função para girar os ovos na incubadora
void controlRoll(){
    t = now(); //captura os dados de data e armazena em t

    //Função para diferenciar se houve sincronia antes ou não e realizar a conta de tempo decorrido
    if (lastrotSync == 0)//Primeira sincronia
    {
        auxTime = (t - startTime);   
    }
    else{
        auxTime = (t- lastrotSync);
    }
    
    lastrotSync = t;
    savedRotTime = savedRotTime + auxTime;

    if (savedRotTime % 600 == 0) //Grava na EEPROM a cada 10 minutos
    {
        Serial.println("Gravando tempo de rotação na EEPROM");
        EEPROM.put(eepromRotTime, savedRotTime);
    }

    leftRotTime = (3600 - savedRotTime);

    //Valida que esgotou o tempo de rotação e aciona o rele para rotacionar
    if (leftRotTime <= 0)
    {
        digitalWrite(ROTATOR_PIN, LOW);
        //delay(1000);
        digitalWrite(ROTATOR_PIN, HIGH);
        savedRotTime = 0;
    }
}

//Função para converter o tempo do objetivo em segundos
void convertTimeToSec(){
    targetInSeconds = ((targetDay * 86400) + (targetHour * 3600));
    //Serial.println(targetInSeconds);
}

//Função para controlar o tempo e mudança de estagio
void controlTime(){    
    t = now(); //captura os dados de data e armazena em t

    //Função para diferenciar se houve sincronia antes ou não e realizar a conta de tempo decorrido
    if (lastSync == 0)//Primeira sincronia
    {
        auxTime = (t - startTime);  
    }
    else{
        auxTime = (t - lastSync);   
    }

    lastSync = t;
    savedTime = savedTime + auxTime;

    if (savedTime >= targetInSeconds) // Verfica se decorreu o tempo naquele estágio e faz a troca para o proximo e salva na eeprom
    {
        switch (savedStage)
        {
        case 1:
            savedStage = 2;
            savedTime = 0;
            EEPROM.put(eepromStage, savedStage);
            EEPROM.put(eepromTime, savedTime);
            break;
        case 2:
            savedStage = 3;
            savedTime = 0;
            EEPROM.put(eepromStage, savedStage);
            EEPROM.put(eepromTime, savedTime);
            break;
        case 3:
            savedStage = 0;
            savedTime = 0;
            EEPROM.put(eepromStage, savedStage);
            EEPROM.put(eepromTime, savedTime);
            break;
        default:
            break;
        }
        //Define os parametros de controle de acordo com o estagio
        setParametersByStage();
    }

    if ((savedTime % 600) == 0) //Grava na EEPROM a cada 10 minutos
    {
        Serial.println("Gravando tempo na EEPROM");
        EEPROM.put(eepromTime, savedTime);
    }

    leftTime = (targetInSeconds - savedTime);
}

//Função para criar a tela inicial do sistema
void homeScreen(){
    lcd.clear();
    showTemp();
    showHumdt();
    if ((rotation == true) && (leftRotTime < 120)){ // IMPLEMENTAR ALTERNAR ENTRE O ESTAGIO E O ROTACIONAR
        showRollTime();
    }
    else{
        showStage();
    }
}
