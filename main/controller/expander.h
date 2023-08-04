#ifndef EXPANDER_H_INCLUDED
#define EXPANDER_H_INCLUDED

#include <stdint.h>

void expander_init();
void expander_test();
void expander_out_set_gpio_pin(uint8_t pin);
void expander_out_unset_gpio_pin(uint8_t pin);
_Bool expander_in_get_gpio_pin(uint8_t pin);

#endif
