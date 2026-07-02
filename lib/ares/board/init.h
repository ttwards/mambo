#ifndef ARES_BOARD_INIT_H_
#define ARES_BOARD_INIT_H_

#include <stdint.h>

struct ares_led_rgb {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

void ares_board_power_init(void);
void ares_board_status_led_init(void);
void ares_board_status_led_service_start(void);
int ares_board_status_led_set_rgb(const struct ares_led_rgb *color);
uint8_t ares_board_status_led_max_channel(void);

#endif /* ARES_BOARD_INIT_H_ */
