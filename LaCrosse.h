/* LaCrosse - Header */

#ifndef LaCrosse_h
	#define LaCrosse_h

		#include <Arduino.h>
		#include <inttypes.h>
		#include <stdbool.h>

	class LaCrosseClass
	{
	  public:
		  void setTxPinMode(byte value);
		  void setSensorId(byte SensorId); 
		  void sendTemperature();
		  void sendHumidity();
		  void sleep(int sek);
		  //--- Temperature and Hunidity
		  float t, h;
		  byte bSensorId; 
	  private:    	
		void O(); 
		void I();
		void CalcNibles();
		void SendNibble(int i);
		void sendNibles();
		void CalcTemp();
		void CalcHum();	
		int freeRam();
	};

	extern LaCrosseClass LaCrosse;

#endif
