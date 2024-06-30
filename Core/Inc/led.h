/*
 * led.h
 *
 *  Created on: Jun 23, 2024
 *      Author: matthias
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32l4xx_hal.h"

class LED{

public:
	enum state { OFF, BLINKING, ON };

	LED();
	LED(GPIO_TypeDef *port, uint16_t pin);
	void setState(LED::state state);
	void setBlinkTime(uint32_t time);
	void doBlink();
	void toggle();
	void update();



private:
	state _state = state::OFF;
	uint32_t _blinkTime = 1000;
	uint32_t _lastBlink = 0;
	GPIO_TypeDef *_port;
	uint16_t _pin;


};




#endif /* INC_LED_H_ */
