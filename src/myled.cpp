#include "myled.h"

/// Class to blink led
MyLed::MyLed(const uint8_t led)
    : _led(led),
    _ticker(std::bind(&MyLed::Toggle, this), 500)
{
  _initialise();
}

/// Turn led off
void MyLed::Off()
{
  _ticker.stop();
  _off();
};

/// Turn led on
void MyLed::On()
{
  _ticker.stop();
  _on();
};

/// Toggle state of led
void MyLed::Toggle()
{
  // _ticker.stop();
  digitalWrite(_led, !digitalRead(_led));
};

/// Blink led
void MyLed::Blink(uint32_t milliseconds)
{
  _ticker.interval(milliseconds);
  _on();
}

/// Turn led off
void MyLed::loop()
{
  _ticker.update();
};

// * Initialise led
void MyLed::_initialise()
{
  pinMode(_led, OUTPUT);
  _off();
};

void MyLed::_on()
{
  digitalWrite(_led, _LEDON);
}

void MyLed::_off()
{
  digitalWrite(_led, _LEDOFF);
}
