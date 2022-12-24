//Controlador de chocadeira PID
//Ryan Monteiro - 12/2022 - Versão 2.0

#include <DallasTemperature.h>
#include <OneWire.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v2.h>

//Porta do pino de sinal do DS18B20
#define ONE_WIRE_BUS 2
//Porta do pino de sinal e tipo do DHT
#define DHTPIN 4
#define DHTTYPE DHT11

//Definição dos pinos de saída
#define HEATER_PIN       6
#define ROTATOR_KEY_PIN  8
#define ROTATOR_OUT_PIN  9
#define COOLER_PIN      10

//Definição de parametros do PID
#define consKp 50
#define consKi 10
#define consKd  0
#define aggrKp 50
#define aggrKi 10
#define aggrKd  0

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
PID_v2 myPID(consKp, consKi, consKd, PID::Direct);

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
double pidSetpoint = 33.0, pidInput, pidOutput;

//Prototipo das funções
void getTemp(void);
void getHumdt(void);
void homeScreen(void);
void controlTemp(void);
void setPidParameters(void);

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

    //Define os limites da saida PID
    myPID.SetOutputLimits(0, 255);
    //Inicializa o controlador PID passando a leitura do sensor de temperatura e o alvo
    myPID.Start(pidInput, pidOutput, pidSetpoint);

}

void loop() {
    //Captura as informações dos sensores
    getTemp();
    getHumdt();
    // Gera a tela de inicio
    homeScreen();
    // Chama a função de controlar a temperatura
    controlTemp();

}

//Função para ler e tratar a temperatura do sensor
void getTemp(){
    sensors.requestTemperatures();
    probeTemp = sensors.getTempC(sensor1);
    Serial.print("Temperatura:");
    Serial.println(probeTemp);
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
    Serial.print("Umidade:");
    Serial.println(probeHumdt);
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

//PARA IMPLEMENTAÇÃO - DEFINIR OS PARAMETROS DE ACORDO COM O ESTAGIO DE CHOCAGEM
void setParametersByStage(){
// implementar os parametros e a definicão do estagio
    switch (stage) {
    case 1: //Pré Aquecimento
        pidSetpoint = 27.0;
        rotation = false;
        break;
    case 2: //Incubação
        pidSetpoint = 37.7;
        rotation = true;
        break;
    case 3:
        pidSetpoint = 36.8;
        rotation = false;
        break;
    default:
        pidSetpoint = 0;
        rotation = false;
        break;
    }
}

//Função para ajustar os parametros de acordo com gap para o alvo
void setPidParameters(){
    double gap = abs(myPID.GetSetpoint() - pidInput);  // distance away from setpoint
    if (gap < 10) {
        // we're close to setpoint, use conservative tuning parameters
        myPID.SetTunings(consKp, consKi, consKd);
    }
    else {
        // we're far from setpoint, use aggressive tuning parameters
        myPID.SetTunings(aggrKp, aggrKi, aggrKd);
    }
}

//Função para exibir o estagio na tela inicial
void showStage(){
    switch (stage) {
    case 1:
        lcd.setCursor(7, 0);
	    lcd.print("AQUECENDO");
        lcd.setCursor(7,1);
        lcd.print("TEMPO");
        break;
    case 2:
        lcd.setCursor(7, 0);
	    lcd.print("INCUBANDO");
        lcd.setCursor(7,1);
        lcd.print("TEMPO");
        break;
    case 3:
        lcd.setCursor(7, 0);
	    lcd.print("NASCENDO");
        lcd.setCursor(7,1);
        lcd.print("TEMPO");
        break;
    default:
        lcd.setCursor(7, 0);
	    lcd.print("DESLIGADO");
        break;
    }
}

//Função para girar os ovos na incubadora
void controlRoll(){
    if(rotation == true){
        unsigned long actTime = (millis() / 1000);
        unsigned long auxTime;
        //Serial.println(actTime);
        if ((actTime - lastTime) > 3600){
            auxTime = (millis() / 1000);
            digitalWrite(ROTATOR_PIN, LOW);
            lastTime = (millis() / 1000);
        }
        if(((millis() / 1000) - auxTime) > 10) {
            digitalWrite(ROTATOROUTPIN, HIGH);
        }
        remainingTime = (lastTime + 3600) - actTime;
        //Serial.println(remainingTime);
        //Serial.println(lastTime); 
  }
}

//Função para definir o estagio de acordo com o tempo desde o inicio
void setStage(){
    //IMPLEMETAR FUNCAO PARA DEFINIR ESTAGIO DE ACORDO COM O TEMPO
}

//Função para criar a tela inicial do sistema
void homeScreen(){
    lcd.clear();
    showTemp();
    showHumdt();
    if (1 == 1){ // IMPLEMENTAR ALTERNAR ENTRE O ESTAGIO E O ROTACIONAR
        showStage();
    }

}
