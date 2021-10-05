/***************************************************
Copyright (c) 2019 Luis Llamas
(www.luisllamas.es)
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

#include "BTS7960.h"

const int freq = 5000;
const int R_PWM_CHN = 0;
const int L_PWM_CHN = 1;
const int resolution = 8;

BTS7960::BTS7960(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM)
{
	_R_PWM = R_PWM;
	_L_PWM = L_PWM;
	_L_EN = L_EN;
	_R_EN = R_EN;
	pinMode(_R_PWM, OUTPUT);
	pinMode(_L_PWM, OUTPUT);
	pinMode(_L_EN, OUTPUT);
	pinMode(_R_EN, OUTPUT);
	ledcSetup(R_PWM_CHN, freq, resolution);
	ledcSetup(L_PWM_CHN, freq, resolution);

	ledcAttachPin(_R_PWM, R_PWM_CHN);
	ledcAttachPin(_L_PWM, L_PWM_CHN);
}

void BTS7960::TurnRight(uint8_t pwm)
{
	ledcWrite(L_PWM_CHN, 0);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	ledcWrite(R_PWM_CHN, pwm);
}

void BTS7960::TurnLeft(uint8_t pwm)
{
	ledcWrite(R_PWM_CHN, 0);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	ledcWrite(L_PWM_CHN, pwm);
}

void BTS7960::Enable()
{
	digitalWrite(_L_EN, HIGH);
	if (_R_EN != 0)
		digitalWrite(_R_EN, HIGH);
}

void BTS7960::Disable()
{
	digitalWrite(_L_EN, LOW);
	if (_R_EN != 0)
		digitalWrite(_R_EN, LOW);
}

void BTS7960::Stop()
{
	ledcWrite(L_PWM_CHN, LOW);
	ledcWrite(R_PWM_CHN, HIGH);
}
