
/*
 * 	STM32F103C8 platformio
 * 	The circuit:
 * 	RFM69 attached to SPI_2 Softeare SPI bo HW SPI nie działa jescze
 * 	CS    <-->  PB12 <-->  BOARD_SPI2_NSS_PIN
 * 	SCK   <-->  PB13 <-->  BOARD_SPI2_SCK_PIN
 * 	MISO  <-->  PB14 <-->  BOARD_SPI2_MISO_PIN
 * 	MOSI  <-->  PB15 <-->  BOARD_SPI2_MOSI_PIN
 * 	DIO0  <-->  PA8  <-->  Interrupt on receiving message
 * 	RESET <-->  PA9  <-->  Reset pin of RF69
 *
 * 	RadioHead w wersji 1.112
 * 	SPI w wersji 1.0
 * 	QuickMedianLib w wersji 1.1.1
 * 	OneWire w wersji 2.3.5
 * 	DallasTemperature w wersji 3.8.0
 *
 */

/* Included librarys*/
#include <Arduino.h>
#include <RHMesh.h>
#include <RH_RF69.h>
#include <RHSoftwareSPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avdweb_VirtualDelay.h>
#include <libmaple/iwdg.h>
#include <ledIndicator.h>

/*
 *
 * Pinout Config
 *
 */

/*miso, mosi, sck, nss, irq, reset*/
const uint8_t radioPins[] = {PB14, PB15, PB13, PB12, PA8, PA9};

/*OneWire pins*/
const uint8_t oneWirePins[] = {PA10, PB9, PB8, PB7, PB6, PB11};

/*ADC pins HP, LP, dla pomiaru ciśnienia*/
const uint8_t adcPinsPress[] = {PB1, PB0};

/*ADC pins  L1, L2, L3 dla pomiaru prądu*/
const uint8_t adcPinsCurr[] = {PA7, PA6, PA5};

/*
 *
 *  RFM69HCW Config
 *
 */

/*adres tego nodea*/
#define CLIENT_ADDRESS 102
/*adres gatewaya*/
#define SERVER_ADDRESS 001
/*częstotoliwośc pracy yyy.yyy Mhz*/
#define RF_FREQUENCY 868.150
/*moc nadawania w dBm od -2 do +20. (tylko dla RF69HCW)*/
#define TX_POWER -2
/*softwarespi uzywam bo HW_SPI2 dla STM32 nie jest jeszcze zaimplementowan*/
RHSoftwareSPI softwarespi;
/*piny (nss, irq) dla radia*/
RH_RF69 rf69(radioPins[3], radioPins[4], softwarespi);
/*mesh manager*/
RHMesh manager(rf69, CLIENT_ADDRESS);
/*Własne ID sieci, muisi być takie samo dla wszystkich urzadzeń*/
const uint8_t syncwords[] = {0x2d, 0xd5};
/*bufor dla widomości odebranych*/
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
/* watchdog dl RFM69 jeżeli send message nie ustawi true wtedy funcka radioModuleWatchdog wzwoli reset radia*/
bool rfWd = true;
uint16_t previousMillis = 0;
/*zmienne dodatkowe*/
int8_t rssi;
uint8_t radioModuleTemp;
uint8_t message[26];

/*
 *
 * OneWire Config
 *
 */

/*utworzenie  instancji OneWire do komunikaci*/
OneWire *oneWire[sizeof(oneWirePins) / sizeof(uint8_t)];
/*przekzanie referencji OneWire do DallasTemperaturte*/
DallasTemperature *sensors[sizeof(oneWirePins) / sizeof(uint8_t)];
/*tablica do obsługi wartości  adresów DS18b20*/
uint8_t dsAddress[sizeof(oneWirePins) / sizeof(uint8_t)][8];
/*tablica dla obsługi watrość odczytów temperatur*/
int16_t tempReadings[sizeof(oneWirePins) / sizeof(uint8_t)];
/*rozdzielczość dla DS18b20 12 bit*/
uint8_t tempRes = 12;

/*
 *
 * HP and LP ADC Config
 *
 */

uint16_t tmpPressValues[2][19]; // !!! nieparzyste wartości kolumnach !!! = ilość odczytów podczas cyklu
uint16_t pressureValues[2];
uint32_t delayTimer = 20;
uint8_t counterReadPressure = 0;
uint32_t presentTime;

/*L1, L2, L3 ADC Config*/

/*tablica z wynikami odczytów dla L1 L2 L3*/
int16_t irmsMedianValues[3];

/*
 *
 * Virtual Delay & Timers Config
 *
 */

/*utworzenie  3 instancji VirtualDelay*/
VirtualDelay delay1, delay2, delay3;
uint16_t startTime = 0;
uint16_t endTime = 0;

/*
 *
 * Funkcje
 *
 */

/*Reset modułu radiowego*/
void radioModuleReset(uint8_t pinName)
{

	Serial.print(">>> Radio Module Reset... ");
	// Reset the RFM69 Radio
	pinMode(pinName, OUTPUT);
	digitalWrite(pinName, HIGH);
	delayMicroseconds(100);
	digitalWrite(pinName, LOW);
	// wait until chip ready
	delay(5);
	Serial.println("DONE");
}

/*Konfiguracja modułu radiowego*/
void radioModuleStart()
{
	radioModuleReset(PA9);
	Serial.print(">>> Radio Module Starting... ");
	if (!manager.init())
		Serial.println("RF module init failed");

	if (!rf69.setFrequency(RF_FREQUENCY))
		Serial.println("setFrequency failed");

	rf69.setSyncWords(syncwords, sizeof(syncwords));

	/*true tylko  dla RFM69HCW*/
	rf69.setTxPower(TX_POWER, true);
	rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250); /*transmition time ~5ms*/
	radioModuleTemp = rf69.temperatureRead();

	Serial.println("DONE");
}

/*Konfiguracja modułu odczytu temperatury*/
void tempModuleStart()
{

	Serial.print(">>> Temperature Module Starting... ");
	///search each bus one by one
	for (uint8_t i = 0; i < sizeof(oneWirePins) / sizeof(uint8_t); i++)
	{
		oneWire[i] = new OneWire(oneWirePins[i]);
		sensors[i] = new DallasTemperature(oneWire[i]);
		sensors[i]->begin();
		sensors[i]->setResolution(tempRes);
		sensors[i]->setWaitForConversion(false);
		sensors[i]->getAddress(dsAddress[i], 0);
		sensors[i]->requestTemperatures();
	}
	Serial.println("DONE");
}

/*Wyszukiwanie numeru pozycji w tabeli po wartości*/
uint8_t findIndex(const uint8_t a[], uint8_t size, uint8_t value)
{

	uint8_t index = 0;
	while (index < size && a[index] != value)
		++index;
	return (index == size ? -1 : index);
}

/*Funcka zwraca wartość odcztyu ADC z CT  delay(1) dla  freq=50hz*/
uint16_t getIrms(uint8_t cTpin)
{

	uint16_t b;
	uint16_t readings[100];
	for (int i = 0; i < 100; ++i)
	{
		readings[i] = analogRead(cTpin);
		delay(1);

		uint16_t first, second, third;
		third = first = second = 0;

		for (int i = 0; i < 100; i++)
		{
			/* If current element is greater than first*/
			if (readings[i] > first)
			{
				third = second;
				second = first;
				first = readings[i];
			}

			/* If arr[i] is in between first and second then update second  */
			else if (readings[i] > second)
			{
				third = second;
				second = readings[i];
			}

			else if (readings[i] > third)
				third = readings[i];
		}
		b = (first + second + third) / 3;

		/*	if (b < 50) {
		 b = 0;
		 }*/
	}
	return (b);
}

/*Funcja zapisule wartości odcztów z ADC dla faz L1 L2 L3*/
void getIrmsValues()
{

	Serial.print(">>> Requesting L1 L2 L3 I values... ");

	irmsMedianValues[0] = (getIrms(PA7));
	irmsMedianValues[1] = (getIrms(PA6));
	irmsMedianValues[2] = (getIrms(PA5));

	Serial.println("DONE");
}

void reqTempConv()
{

	Serial.print(">>> Requesting temperature conversion... ");
	/// poll devices connect to the bus
	for (uint8_t i = 0; i < sizeof(oneWirePins) / sizeof(uint8_t); i++)
	{
		sensors[i]->requestTemperatures();
	}
	Serial.println("DONE");
}

void getTempValues()
{

	Serial.print(">>> Requesting temperature values... ");
	for (uint8_t i = 0; i < sizeof(oneWirePins) / sizeof(uint8_t); i++)
	{
		tempReadings[i] = (sensors[i]->getTempC(dsAddress[i])) * 100;
	}
	Serial.println("DONE");
}

void printReadings()
{

	Serial.println();
	for (uint8_t i = 0; i < sizeof(oneWirePins) / sizeof(uint8_t); i++)
	{
		Serial.print(">>> Sensor ");
		Serial.print(i + 1);
		Serial.print(" Temp: ");
		Serial.print((float)tempReadings[i] / 100, 2);
		Serial.println(" *C");
	}

	Serial.print(">>> HP");
	Serial.print(" ADC Value: ");
	Serial.println(pressureValues[0]);
	Serial.print(">>> LP");
	Serial.print(" ADC Value: ");
	Serial.println(pressureValues[1]);

	for (uint8_t i = 0;
		 i < sizeof(irmsMedianValues) / sizeof(irmsMedianValues[0]); i++)
	{
		Serial.print(">>> L");
		Serial.print(i + 1);
		Serial.print(" ADC Value: ");
		Serial.println(irmsMedianValues[i]);
	}
	Serial.println();
}

void clearArrays()
{

	memset(tempReadings, -127, sizeof(tempReadings));
	memset(irmsMedianValues, -1, sizeof(irmsMedianValues));
}

void meshRepeterFeture()
{

	uint8_t len = sizeof(buf);
	uint8_t from;
	manager.recvfromAck(buf, &len, &from);
}

void sendMessage()
{

	rfWd = false;
	if (manager.sendtoWait((uint8_t *)message, sizeof(message),
						   SERVER_ADDRESS) == RH_ROUTER_ERROR_NONE)
	{
		Serial.print(">>> Sending data via RF Link... ");
		uint8_t len = sizeof(buf);
		uint8_t from;
		Serial.println("DONE");
		Serial.print(">>> Waiting for ACK from Gatewy... ");
		if (manager.recvfromAckTimeout(buf, &len, 3000, &from))
		{
			blink(30, 150); // wyzwolenie blinkania ledem
			rfWd = true;	// reset Watchdoga radia
			Serial.println("ACK OK");
			Serial.print(">>> GW ID: ");
			Serial.print(from, DEC);
			Serial.print("  Response  >>> ");
			Serial.println((char *)buf);
			Serial.println();
			radioModuleTemp = rf69.temperatureRead();
			rssi = rf69.lastRssi();
		}
		else
		{
			Serial.print("No reply :( ");
		}
	}
	else
	{
		Serial.print("No reply, is Server: ");
		Serial.print(SERVER_ADDRESS);
		Serial.println(" Online?");
	}
}

void messageConstructor()
{

	int16_t messagePrototype[] = {
		rssi,
		radioModuleTemp,
		pressureValues[0],
		pressureValues[1],
		tempReadings[0],
		tempReadings[1],
		tempReadings[2],
		tempReadings[3],
		tempReadings[4],
		tempReadings[5],
		irmsMedianValues[0],
		irmsMedianValues[1],
		irmsMedianValues[2],

	};

	uint8_t *ptr = (uint8_t *)&messagePrototype; //cast the 16bit pointer to an 8bit pointer
	for (uint8_t i = 0; i < (sizeof(messagePrototype)); i++)
	{
		message[i] = *ptr; //pass data to other array
		ptr++;			   //move your pointer
	}
}

/* funkcja wyzwala reset modułu radiowego*/
void radioModuleWatchdog(void)
{

	uint16_t currentMillis = millis();
	if (currentMillis - previousMillis > 10000)
	{
		previousMillis = currentMillis;
		if (rfWd == false)
		{
			radioModuleReset(PA6);
			rfWd = true;
		}
	}
}
/*
Wyliczenie mediany z odczytów z ADC dla ciśnień HP i LP
*/
void readPressure(uint8_t pinHP, uint8_t pinLP, uint32_t readInterval)
{
	uint8_t readCount = sizeof(tmpPressValues[0]) / sizeof(tmpPressValues[0][0]);
	uint16_t first, second, third;
	presentTime = millis();
	if ((millis() - delayTimer > readInterval) && counterReadPressure < readCount)
	{
		delayTimer = millis();
		tmpPressValues[0][counterReadPressure] = analogRead(pinHP);
		tmpPressValues[1][counterReadPressure] = analogRead(pinLP);
		++counterReadPressure;

		if (counterReadPressure == readCount)
		{
			uint8_t i, j;
			uint16_t a;
			for (i = 0; i < readCount; ++i)
			{
				for (j = i + 1; j < readCount; ++j)
				{
					if (tmpPressValues[0][i] > tmpPressValues[0][j])
					{
						a = tmpPressValues[0][i];
						tmpPressValues[0][i] = tmpPressValues[0][j];
						tmpPressValues[0][j] = a;
					}
				}
			}
			for (i = 0; i < readCount; ++i)
			{
				for (j = i + 1; j < readCount; ++j)
				{
					if (tmpPressValues[1][i] > tmpPressValues[1][j])
					{
						a = tmpPressValues[1][i];
						tmpPressValues[1][i] = tmpPressValues[1][j];
						tmpPressValues[1][j] = a;
					}
				}
			}
			first = (readCount + 1) / 2 - 1;
			second = first + 1;
			third = second + 1;
			pressureValues[0] = (tmpPressValues[0][first] + tmpPressValues[0][second] + tmpPressValues[0][third]) / 3;
			pressureValues[1] = (tmpPressValues[1][first] + tmpPressValues[1][second] + tmpPressValues[1][third]) / 3;

			/* 	for (i = 0; i < readCount; ++i)
			{
				Serial.print(tmpPressValues[0][i]);
				Serial.print(" | ");
				Serial.println(tmpPressValues[1][i]);
			}

			Serial.print(">>> HP");
			Serial.print(" Value: ");
			Serial.println(pressureValues[0]);
			Serial.print(">>> LP");
			Serial.print(" Value: ");
			Serial.println(pressureValues[1]);
		} */
		}
	}
}

/*
 *
 * Setup
 *
 */
void setup()
{
	/* IWDG init  t_IWDG = 19.2s */
	iwdg_init(IWDG_PRE_256, 3000);

	/* 	czekaj na serial lub timeout 5s*/
	Serial.begin(115200);
	bool beginTimeout = false;
	do
	{
		if (millis() > 5000)
		{
			beginTimeout = true;
		}
	} while (!beginTimeout && !Serial);

	/* Sets the pins used by this SoftwareSPIClass instance.
	 * miso, mosi ,sck  */
	softwarespi.setPins(radioPins[0], radioPins[1], radioPins[2]);
	Serial.print(">>> Initialazing ZEN NODE #");
	Serial.println(CLIENT_ADDRESS);

	/*Start modułu radiowego*/
	radioModuleStart();

	/*Start modułu OneWire*/
	tempModuleStart();

	/*Sets pinMode as a analog input*/
	pinMode(PA7, INPUT_ANALOG);
	pinMode(PA6, INPUT_ANALOG);
	pinMode(PA5, INPUT_ANALOG);
	pinMode(PB1, INPUT_ANALOG);
	pinMode(PB0, INPUT_ANALOG);
	pinMode(PC13, OUTPUT);

	/*ustawienie zawartości tablicy*/
	memset(pressureValues, 0, sizeof(pressureValues));
	memset(irmsMedianValues, -1, sizeof(irmsMedianValues));
	Serial.println(">>> Falling into main loop >>>");
	Serial.println();
}
/*
 *
 * Loop
 *
 */
void loop()
{
	/*Funckej w  CONT.LOOP!*/
	meshRepeterFeture();
	led();
	readPressure(PB0, PB1, 10);

	/*MAIN FUNCIONS TIMERS*/
	/*first timer delay  musi być, breaks the deadlock*/
	DO_ONCE(delay1.start(500));

	if (delay1.elapsed())
	{
		reqTempConv();
		counterReadPressure = 0;
		delay2.start(500);
	}

	if (delay2.elapsed())
	{
		getTempValues();
		getIrmsValues();
		delay3.start(1000);
	}

	if (delay3.elapsed())
	{
		printReadings();
		messageConstructor();
		sendMessage();
		clearArrays();
		delay1.start(1000);
	}

	radioModuleWatchdog();
	iwdg_feed();
}
