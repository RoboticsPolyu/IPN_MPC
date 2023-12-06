#include <termios.h>
#include <unistd.h>
#include <stdbool.h>

#ifndef SRC_UART_H_
#define SRC_UART_H_

#define DEBUG

struct UartDevice {
	char* filename;
	int rate;

	int fd;
	struct termios *tty;
};

int uart_start(struct UartDevice* dev, bool canonic);
int uart_writen(struct UartDevice* dev, char *buf, size_t buf_len);
int uart_writes(struct UartDevice* dev, char *string);
int uart_reads(struct UartDevice* dev, char *buf, size_t buf_len);
void uart_stop(struct UartDevice* dev);

#endif /* SRC_UART_H_ */