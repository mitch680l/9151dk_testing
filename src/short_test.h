#ifndef SHORT_TEST_H
#define SHORT_TEST_H

#include <stdint.h>

int short_test_read_pins(const char *context);
int short_test_drive_pin(int port, int pin);
int short_test_drive_all_local(void);

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
int short_test_request_remote_pins(const char *context);
int short_test_read_all_pins(void);
int short_test_request_remote_drive(int port, int pin);
int short_test_drive_all_with_remote(void);
#endif

#endif
