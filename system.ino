#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
 
#define I2C_ADDR    0x3F
#define LCD_COLUMNS 16
#define LCD_LINES   2
#define DHTPIN 2
#define DHTTYPE DHT22
#define PHOTORESISTOR_PIN A0
 
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
 
void setup() {
  Serial.begin(9600);
 
  lcd.init();
  lcd.backlight();
  dht.begin();
 
  pinMode(PHOTORESISTOR_PIN, INPUT);
 
  lcd.setCursor(0, 0);
  lcd.print("Marius' project!");
 
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
 
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
 
  bmp_temp->printSensorDetails();
}
 
 
void loop() {
  // Wait a few seconds between measurements.
  delay(2500);
 

  // FOTOREZISTOR PT LUMINA
  Serial.println(analogRead(PHOTORESISTOR_PIN));

 int ldrValue = analogRead(PHOTORESISTOR_PIN);
  
  // Conversia valorii brute într-o valoare de tensiune (0 - 5V)
  float voltage = ldrValue * (5.0 / 1023.0);

  // Conversia tensiunii în lux
  float lux = voltageToLux(voltage); 

  // DHT22 PT TEMP&UMIDITATE
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
 
 
  // BMP280 PT TEMP&PRESIUNE
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
 
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
 
  //AFISARE LDC:
  //Tura 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Temp: ");
  lcd.print(t);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("  Hum: ");
  lcd.print(h);
  lcd.print("%");


  //Tura 2
  delay(3000);
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Pressure:");
  lcd.setCursor(2, 1);
  lcd.print(pressure_event.pressure);
  lcd.print("hPa");

  //Tura 3
  delay(2500);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Outside: ");
  if (lux > 50) {
    lcd.print("Light!");
  } else {
    lcd.print("Dark!");
  }

  lcd.setCursor(2, 1);
  lcd.print(" Lux: ");
  lcd.print(lux);
 


  //Afisare Monitor
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F("°C "));
 
  Serial.print(F("T = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");
 
  Serial.print(F("P = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");
  Serial.print("Lux: ");
  Serial.print(lux);
  Serial.println();
}

float voltageToLux(float voltage) {
  // Formula de conversie depinde de caracteristicile LDR și configurația circuitului
  // Exemplu simplificat: Lux = (voltage * 100)
  return voltage * 100.0;
}