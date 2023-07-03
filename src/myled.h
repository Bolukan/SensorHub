#ifndef MY_LED_H
#define MY_LED_H

#include <Arduino.h>
#include <TickTwo.h>

class MyLed
{
public:
  /// Class to blink led
  MyLed(const uint8_t led = LED_BUILTIN);

  /// Turn led off
  void Off();

  /// Turn led on
  void On();

  /// Toggle state of led
  void Toggle();

  /// Blink led
  void Blink(uint32_t milliseconds);

  void loop();

private:
  // * led pin
  const uint8_t _led;
  const uint8_t _LEDOFF = LOW;
  const uint8_t _LEDON = HIGH;

  /// blink timer
  TickTwo _ticker;

  /// Initialise led
  void _initialise();

  /// Led On
  void _on();

  // Led Off
  void _off();
};
#endif