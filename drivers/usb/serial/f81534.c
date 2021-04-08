/*
 * F81532/F81534 USB to Serial Ports Bridge
 *
 * F81532 => 2 Serial Ports
 * F81534 => 4 Serial Ports
 *
 * Copyright (C) 2014 Tom Tsai (Tom_Tsai@fintek.com.tw)
 *
 * The F81532/F81534 had 1 control endpoint for setting,
 * 1 endpoint bulk-out for all serial port write out and
 * 1 endpoint bulk-in for all serial port read in.
 *
 * write URB is fixed with 512bytes, per serial port used 128Bytes.
 * is can be described by f81534_prepare_write_buffer()
 *
 * read URB is 512Bytes max. per serial port used 128Bytes.
 * is can be described by f81534_process_read_urb(), it' maybe
 * received with 128x1,2,3,4 bytes.
 *
 * We can control M0(SD)/M1/M2 per ports by gpiolib, This IC contains a
 * internal flash to save configuration. Due to reduce erase/write operation,
 * it's recommend sequence to request 3 pins, change value and release 3 gpio
 * pins. We'll really save configurations when M0(SD)/M1/M2 pin all released
 * for a port.
 *
 * Features:
 * 1. F81534 is 1-to-4 & F81532 is 1-to-2 serial ports IC
 * 2. Support Baudrate from B50 to B1500000 (excluding B1000000).
 * 3. The RTS signal can be transformed their behavior with
 *    configuration by default ioctl TIOCGRS485/TIOCSRS485
 *    (for RS232/RS485/RS422 with transceiver)
 *
 *    If the driver setting with SER_RS485_ENABLED, the RTS signal will
 *    high with not in TX and low with in TX.
 *
 *    If the driver setting with SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND,
 *    the RTS signal will low with not in TX and high with in TX.
 *
 * 4. There are 4x3 output-only ic pins to control transceiver mode with our
 *    EVB Board. It's can be controlled via gpiolib. We could found the gpio
 *    number from /sys/class/tty/ttyUSB[x]/device/gpiochip[yyy] where
 *    x is F81532/534 serial port and yyy is gpiochip number.
 *
 *    After we found chip number, we can export 3 gpios(M0(SD)/M1/M2) per
 *    serial port by
 *       echo yyy > /sys/class/gpio/export
 *       echo yyy+1 > /sys/class/gpio/export
 *       echo yyy+2 > /sys/class/gpio/export
 *
 *    then we can control it with
 *       echo [M2 value] > /sys/class/gpio/gpio[yyy]/value
 *       echo [M1 value] > /sys/class/gpio/gpio[yyy+1]/value
 *       echo [M0(SD) value] > /sys/class/gpio/gpio[yyy+2]/value
 *    which M0(SD)/M1/M2 as your desired value, value is only 0 or 1.
 *
 *    When configure complete, It's a must to free all gpio by
 *       echo yyy > /sys/class/gpio/unexport
 *       echo yyy+1 > /sys/class/gpio/unexport
 *       echo yyy+2 > /sys/class/gpio/unexport
 *
 *    The driver will "save" gpio configure after we release
 *    all gpio of a serial port.
 *
 *    For examples to change mode & gpio with F81532/534
 *    Evalaution Board.
 *
 *    F81532 EVB
 *       port0: F81437 (RS232 only)
 *       port1: F81439 (RS232/RS485/RS422 ... etc.)
 *    F81534 EVB
 *       port0/1: F81437 (RS232 only)
 *       port2/3: F81439 (RS232/RS485/RS422 ... etc.)
 *
 *       1. RS232 Mode (Default IC Mode)
 *          1. Set struct serial_rs485 flags "without" SER_RS485_ENABLED
 *             (control F81532/534 RTS control)
 *          2. Set M0(SD)/M1/M2 as 0/0/1
 *             (control F81532/534 output pin to control transceiver mode)
 *
 *       2. RS485 Mode (RTS Low when TX Mode)
 *          1. Set struct serial_rs485 flags with SER_RS485_ENABLED
 *          2. Set M0(SD)/M1/M2 as 0/1/0
 *
 *       3. RS485 Mode (RTS High when TX Mode)
 *          1. Set struct serial_rs485 flags with SER_RS485_ENABLED and
 *             SER_RS485_RTS_ON_SEND
 *          2. Set M0(SD)/M1/M2 as 0/1/1
 *
 *       4. RS422 Mode
 *          1. The RTS mode is dont care.
 *          2. Set M0(SD)/M1/M2 as 0/0/0
 *
 *    Please reference https://bitbucket.org/hpeter/fintek-general/src/
 *    with f81534/tools to get set_gpio.c & set_mode.c. Please use it
 *    carefully.
 */
//#undef CONFIG_GPIOLIB
//#define CONFIG_GPIOLIB

#define ENABLE_CUSTOM_IOCTLS
#define DEBUG_INTERNAL_SYSFS
//#define DEBUG_TX_EMPTY
//#define WRITER_WQ
#define LSR_ISSUE
#define HW_DISABLE
#define USER_MODE

#ifdef ENABLE_CUSTOM_IOCTLS
#define MULTIDROP_ENABLE
#define FLASH_RW_DRIVER
#define SAVE_CONTROL
#endif

#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/serial_reg.h>
#ifdef ENABLE_CUSTOM_IOCTLS
#include <linux/version.h>
#endif
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0)
#define __dynamic_dev_dbg(...)

#undef dev_err
#define dev_err(dev, text, arg...) \
	printk(KERN_INFO "%s %s: "#text, \
			dev_driver_string(dev), \
			__func__, \
			## arg)

#undef dev_warn
#define dev_warn(dev, text, arg...) \
	printk(KERN_INFO "%s %s: "#text, \
			dev_driver_string(dev), \
			__func__, \
			## arg)

#undef dev_info
#define dev_info(dev, text, arg...) \
	printk(KERN_INFO "%s %s: "#text, \
			dev_driver_string(dev), \
			__func__, \
			## arg)
#endif

#define VERSION "$Rev: 2138 $"

/* Serial Port register Address */
#define SERIAL_BASE_ADDRESS		0x1200
#define RECEIVE_BUFFER_REGISTER		(0x00 + SERIAL_BASE_ADDRESS)
#define TRANSMIT_HOLDING_REGISTER	(0x00 + SERIAL_BASE_ADDRESS)
#define DIVISOR_LATCH_LSB		(0x00 + SERIAL_BASE_ADDRESS)
#define INTERRUPT_ENABLE_REGISTER	(0x01 + SERIAL_BASE_ADDRESS)
#define DIVISOR_LATCH_MSB		(0x01 + SERIAL_BASE_ADDRESS)
#define INTERRUPT_IDENT_REGISTER	(0x02 + SERIAL_BASE_ADDRESS)
#define FIFO_CONTROL_REGISTER		(0x02 + SERIAL_BASE_ADDRESS)
#define LINE_CONTROL_REGISTER		(0x03 + SERIAL_BASE_ADDRESS)
#define MODEM_CONTROL_REGISTER		(0x04 + SERIAL_BASE_ADDRESS)
#define LINE_STATUS_REGISTER		(0x05 + SERIAL_BASE_ADDRESS)
#define MODEM_STATUS_REGISTER		(0x06 + SERIAL_BASE_ADDRESS)
#define CLK_SEL_REGISTER		(0x08 + SERIAL_BASE_ADDRESS)
#define CONFIG1_REGISTER		(0x09 + SERIAL_BASE_ADDRESS)
#define SADDRESS_REGISTER		(0x0a + SERIAL_BASE_ADDRESS)
#define SADEN_REGISTER			(0x0b + SERIAL_BASE_ADDRESS)

#define IER_DMA_TX_EN			BIT(7)
#define IER_DMA_RX_EN			BIT(6)

#define F81534_DEF_CONF_ADDRESS_START	0x3000
#define F81534_DEF_CONF_SIZE		12

#define F81534_CUSTOM_ADDRESS_START	0x2f00
#define F81534_CUSTOM_TOTAL_SIZE	0x10
#define F81534_CUSTOM_DATA_SIZE		0x10
#define F81534_CUSTOM_MAX_IDX \
		(F81534_CUSTOM_TOTAL_SIZE/F81534_CUSTOM_DATA_SIZE)
#define F81534_CUSTOM_NO_CUSTOM_DATA	(-1)
#define F81534_CUSTOM_VALID_TOKEN	0xf0
#define F81534_CONF_OFFSET		1
#define F81534_CONF_SIZE		8

#define F81534_MAX_DATA_BLOCK		64
#define F81534_MAX_BUS_RETRY		2000

/* default URB timeout for usb operations */
#define F81534_USB_MAX_RETRY		10
#define F81534_USB_TIMEOUT		1000
#define F81534_CONTROL_BYTE		0x1B
#define F81534_SET_GET_REGISTER		0xA0

#define F81534_NUM_PORT			4
#define F81534_UNUSED_PORT		0xff
#define F81534_WRITE_BUFFER_SIZE	512
#define F81534_MAX_RECEIVE_BLOCK_SIZE	512

#define IC_NAME				"f81534"
#define DRIVER_DESC \
	"Fintek USB to Serial Ports Driver (F81532/F81534-Evaluation Board)"
#define FINTEK_VENDOR_ID_1		0x1934
#define FINTEK_VENDOR_ID_2		0x2C42
#define FINTEK_DEVICE_ID		0x1202	/* RS232 four port */
#define F81534_MAX_TX_SIZE		100
#define F81534_FIFO_SIZE		128
#define F81534_RECEIVE_BLOCK_SIZE	128

#define F81534_TOKEN_RECEIVE		0x01
#define F81534_TOKEN_WRITE		0x02
#define F81534_TOKEN_TX_EMPTY		0x03
#define F81534_TOKEN_MSR_CHANGE		0x04

#define F81534_BUS_BUSY			0x03
#define F81534_BUS_IDLE			0x04
#define F81534_BUS_READ_DATA		0x1004
#define F81534_BUS_REG_STATUS		0x1003
#define F81534_BUS_REG_START		0x1002
#define F81534_BUS_REG_END		0x1001

#define F81534_CMD_READ			0x03
#define F81534_CMD_ENABLE_WR		0x06
#define F81534_CMD_PROGRAM		0x02
#define F81534_CMD_ERASE		0x20
#define F81534_CMD_READ_STATUS		0x05

#define F81534_MEDIA_BUSY_STATUS	0x03

#define F81534_1X_RXTRIGGER		0xc3
#define F81534_8X_RXTRIGGER		0xcf

#define F81534_DEFAULT_BAUD_RATE	9600
#define F81534_MAX_BAUDRATE		1500000

#define F81534_DELAY_READ_MSR		10

#ifdef SAVE_CONTROL
#define PORT_NOSAVE			BIT(0)
#endif

#ifdef ENABLE_CUSTOM_IOCTLS
#define FINTEK_MAGIC	'F'

#ifdef MULTIDROP_ENABLE
#define FINTEK_SET_MULTI_DROP_MODE	_IOW(FINTEK_MAGIC, 0, int)
#define FINTEK_GET_MULTI_DROP_MODE	_IOR(FINTEK_MAGIC, 1, int)
#define FINTEK_GET_SM2_STATE		_IOR(FINTEK_MAGIC, 2, int)
#endif

#ifdef FLASH_RW_DRIVER
#define FINTEK_GET_DATA		_IOR(FINTEK_MAGIC, 8, struct internal_data)
#define FINTEK_SET_DATA		_IOW(FINTEK_MAGIC, 9, struct internal_data)
#define FINTEK_ERASE_DATA_PAGE	_IOW(FINTEK_MAGIC, 10, struct internal_data)
#endif
#endif

#define F81534_RS232_FLAG		0x00
#define F81534_RS485_FLAG		0x03
#define F81534_RS485_1_FLAG		0x01
#define F81534_MODE_MASK		0x03
#define F81534_PORT_CONF_RS485		BIT(0)
#define F81534_PORT_CONF_RS485_INVERT	BIT(1)
#define F81534_PORT_CONF_DISABLE_PORT	BIT(3)
#define F81534_PORT_CONF_NOT_EXIST_PORT	BIT(7)
#define F81534_PORT_UNAVAILABLE		\
	(F81534_PORT_CONF_DISABLE_PORT | F81534_PORT_CONF_NOT_EXIST_PORT)

#define F81534_RS485_MODE	BIT(4)
#define F81534_RS485_INVERT	BIT(5)

#define F81534_PIN_SET_DEFAULT	0x01
#define F81534_PIN_SET_MAX	0x07
#define F81534_PIN_SET_MIN	0x00

/*
 * Clock rate selector, always or-ed with CLKSEL_ENABLE_UART to enable
 * UART functional.
 */
#define CLKSEL_ENABLE_UART	BIT(0)
#define CLKSEL_1DOT846_MHZ	CLKSEL_ENABLE_UART
#define CLKSEL_18DOT46_MHZ	(BIT(1) | CLKSEL_ENABLE_UART)
#define CLKSEL_24_MHZ		(BIT(2) | CLKSEL_ENABLE_UART)
#define CLKSEL_14DOT77_MHZ	(BIT(1) | BIT(2) | CLKSEL_ENABLE_UART)
#define CLKSEL_TX_DELAY_1BIT	BIT(3)

#ifndef C_CMSPAR
#define C_CMSPAR(tty)   _C_FLAG((tty), CMSPAR)
#endif

enum uart_mode {
	uart_mode_rs422,
	uart_mode_rs232,
	uart_mode_rs485,
	uart_mode_rs485_1,
	uart_mode_rs422_term,
	uart_mode_rs232_coexist,
	uart_mode_rs485_1_term,
	uart_mode_shutdown,
	uart_mode_invalid,
};

#ifdef FLASH_RW_DRIVER
struct internal_data {
	unsigned int address;
	unsigned int size;
	unsigned char buf[F81534_MAX_DATA_BLOCK];
};
#endif

struct f81534_pin_config_data {
	enum uart_mode force_uart_mode;
	u8 gpio_mode;
	const int address[9];
	const int offset[9];
};

/* Save for a control register and bit offset */
struct reg_value {
	const u16 reg_address;
	const u16 reg_offset;
	const u16 reg_bit;
};

/* 3 control register to configuration a output pin mode and value */
struct pin_data {
	struct reg_value port_mode_1;
	struct reg_value port_mode_0;
	struct reg_value port_io;
};

/* 3 output pins to control transceiver mode */
struct out_pin {
	struct pin_data m1;
	struct pin_data m2;
	struct pin_data m0_sd;
};

struct io_map_value {
	int product_id;
	int max_port;
	enum uart_mode mode;
	struct out_pin port[MAX_NUM_PORTS + 1];
};

/*
 * The following magic numbers is F81532/534 output pin
 * register maps
 */
static const struct io_map_value f81534_rs232_control = {
	FINTEK_DEVICE_ID, F81534_NUM_PORT, uart_mode_rs232,
	{
	 /* please reference f81439 io port */
	 {
	  {{0x2ad5, 4, 1}, {0x2ad4, 4, 1}, {0x2a90, 4, 0},},
	  {{0x2ad5, 5, 1}, {0x2ad4, 5, 1}, {0x2a90, 5, 0},},
	  {{0x2add, 7, 1}, {0x2adc, 7, 1}, {0x2ae8, 7, 1},},
	  },
	 {
	  {{0x2add, 3, 1}, {0x2adc, 3, 1}, {0x2ae8, 3, 0},},
	  {{0x2add, 0, 1}, {0x2adc, 0, 1}, {0x2ae8, 0, 0},},
	  {{0x2add, 6, 1}, {0x2adc, 6, 1}, {0x2ae8, 6, 1},},
	  },
	 {
	  {{0x2ad3, 6, 1}, {0x2ad2, 6, 1}, {0x2a80, 6, 0},},
	  {{0x2add, 2, 1}, {0x2adc, 2, 1}, {0x2ae8, 2, 0},},
	  {{0x2ad5, 0, 1}, {0x2ad4, 0, 1}, {0x2a90, 0, 1},},
	  },
	 {
	  {{0x2ad5, 1, 1}, {0x2ad4, 1, 1}, {0x2a90, 1, 0},},
	  {{0x2ad5, 2, 1}, {0x2ad4, 2, 1}, {0x2a90, 2, 0},},
	  {{0x2ad5, 3, 1}, {0x2ad4, 3, 1}, {0x2a90, 3, 1},},
	  },
	 },
};

static const struct io_map_value f81534_rs485_control = {
	FINTEK_DEVICE_ID, F81534_NUM_PORT, uart_mode_rs485,
	{
	 /* please reference f81439 io port */
	 {
	  {{0x2ad5, 4, 1}, {0x2ad4, 4, 1}, {0x2a90, 4, 0},},
	  {{0x2ad5, 5, 1}, {0x2ad4, 5, 1}, {0x2a90, 5, 1},},
	  {{0x2add, 7, 1}, {0x2adc, 7, 1}, {0x2ae8, 7, 0},},
	  },
	 {
	  {{0x2add, 3, 1}, {0x2adc, 3, 1}, {0x2ae8, 3, 0},},
	  {{0x2add, 0, 1}, {0x2adc, 0, 1}, {0x2ae8, 0, 1},},
	  {{0x2add, 6, 1}, {0x2adc, 6, 1}, {0x2ae8, 6, 0},},
	  },
	 {
	  {{0x2ad3, 6, 1}, {0x2ad2, 6, 1}, {0x2a80, 6, 0},},
	  {{0x2add, 2, 1}, {0x2adc, 2, 1}, {0x2ae8, 2, 1},},
	  {{0x2ad5, 0, 1}, {0x2ad4, 0, 1}, {0x2a90, 0, 0},},
	  },
	 {
	  {{0x2ad5, 1, 1}, {0x2ad4, 1, 1}, {0x2a90, 1, 0},},
	  {{0x2ad5, 2, 1}, {0x2ad4, 2, 1}, {0x2a90, 2, 1},},
	  {{0x2ad5, 3, 1}, {0x2ad4, 3, 1}, {0x2a90, 3, 0},},
	  },
	 },
};

static const struct io_map_value f81534_rs485_1_control = {
	FINTEK_DEVICE_ID, F81534_NUM_PORT, uart_mode_rs485_1,
	{
	 /* please reference f81439 io port */
	 {
	  {{0x2ad5, 4, 1}, {0x2ad4, 4, 1}, {0x2a90, 4, 0},},
	  {{0x2ad5, 5, 1}, {0x2ad4, 5, 1}, {0x2a90, 5, 1},},
	  {{0x2add, 7, 1}, {0x2adc, 7, 1}, {0x2ae8, 7, 1},},
	  },
	 {
	  {{0x2add, 3, 1}, {0x2adc, 3, 1}, {0x2ae8, 3, 0},},
	  {{0x2add, 0, 1}, {0x2adc, 0, 1}, {0x2ae8, 0, 1},},
	  {{0x2add, 6, 1}, {0x2adc, 6, 1}, {0x2ae8, 6, 1},},
	  },
	 {
	  {{0x2ad3, 6, 1}, {0x2ad2, 6, 1}, {0x2a80, 6, 0},},
	  {{0x2add, 2, 1}, {0x2adc, 2, 1}, {0x2ae8, 2, 1},},
	  {{0x2ad5, 0, 1}, {0x2ad4, 0, 1}, {0x2a90, 0, 1},},
	  },
	 {
	  {{0x2ad5, 1, 1}, {0x2ad4, 1, 1}, {0x2a90, 1, 0},},
	  {{0x2ad5, 2, 1}, {0x2ad4, 2, 1}, {0x2a90, 2, 1},},
	  {{0x2ad5, 3, 1}, {0x2ad4, 3, 1}, {0x2a90, 3, 1},},
	  },
	 },
};

static const struct io_map_value f81534_rs422_control = {
	FINTEK_DEVICE_ID, F81534_NUM_PORT, uart_mode_rs422,
	{
	 /* please reference f81439 io port */
	 {
	  {{0x2ad5, 4, 1}, {0x2ad4, 4, 1}, {0x2a90, 4, 0},},
	  {{0x2ad5, 5, 1}, {0x2ad4, 5, 1}, {0x2a90, 5, 0},},
	  {{0x2add, 7, 1}, {0x2adc, 7, 1}, {0x2ae8, 7, 0},},
	  },
	 {
	  {{0x2add, 3, 1}, {0x2adc, 3, 1}, {0x2ae8, 3, 0},},
	  {{0x2add, 0, 1}, {0x2adc, 0, 1}, {0x2ae8, 0, 0},},
	  {{0x2add, 6, 1}, {0x2adc, 6, 1}, {0x2ae8, 6, 0},},
	  },
	 {
	  {{0x2ad3, 6, 1}, {0x2ad2, 6, 1}, {0x2a80, 6, 0},},
	  {{0x2add, 2, 1}, {0x2adc, 2, 1}, {0x2ae8, 2, 0},},
	  {{0x2ad5, 0, 1}, {0x2ad4, 0, 1}, {0x2a90, 0, 0},},
	  },
	 {
	  {{0x2ad5, 1, 1}, {0x2ad4, 1, 1}, {0x2a90, 1, 0},},
	  {{0x2ad5, 2, 1}, {0x2ad4, 2, 1}, {0x2a90, 2, 0},},
	  {{0x2ad5, 3, 1}, {0x2ad4, 3, 1}, {0x2a90, 3, 0},},
	  },
	 },
};

static const struct io_map_value f81534_shutdown_control = {
	FINTEK_DEVICE_ID, F81534_NUM_PORT, uart_mode_shutdown,
	{
	 /* please reference f81439 io port */
	 {
	  {{0x2ad5, 4, 0}, {0x2ad4, 4, 0}, {0x2a90, 4, 1},},
	  {{0x2ad5, 5, 0}, {0x2ad4, 5, 0}, {0x2a90, 5, 1},},
	  {{0x2add, 7, 0}, {0x2adc, 7, 0}, {0x2ae8, 7, 1},},
	  },
	 {
	  {{0x2add, 3, 0}, {0x2adc, 3, 0}, {0x2ae8, 3, 1},},
	  {{0x2add, 0, 0}, {0x2adc, 0, 0}, {0x2ae8, 0, 1},},
	  {{0x2add, 6, 0}, {0x2adc, 6, 0}, {0x2ae8, 6, 1},},
	  },
	 {
	  {{0x2ad3, 6, 0}, {0x2ad2, 6, 0}, {0x2a80, 6, 1},},
	  {{0x2add, 2, 0}, {0x2adc, 2, 0}, {0x2ae8, 2, 1},},
	  {{0x2ad5, 0, 0}, {0x2ad4, 0, 0}, {0x2a90, 0, 1},},
	  },
	 {
	  {{0x2ad5, 1, 0}, {0x2ad4, 1, 0}, {0x2a90, 1, 1},},
	  {{0x2ad5, 2, 0}, {0x2ad4, 2, 0}, {0x2a90, 2, 1},},
	  {{0x2ad5, 3, 0}, {0x2ad4, 3, 0}, {0x2a90, 3, 1},},
	  },
	 },
};

static const struct io_map_value f81534_rs422_term_control = {
	FINTEK_DEVICE_ID, F81534_NUM_PORT, uart_mode_shutdown,
	{
	 /* please reference f81439 io port */
	 {
	  {{0x2ad5, 4, 1}, {0x2ad4, 4, 1}, {0x2a90, 4, 1},},
	  {{0x2ad5, 5, 1}, {0x2ad4, 5, 1}, {0x2a90, 5, 0},},
	  {{0x2add, 7, 1}, {0x2adc, 7, 1}, {0x2ae8, 7, 0},},
	  },
	 {
	  {{0x2add, 3, 1}, {0x2adc, 3, 1}, {0x2ae8, 3, 1},},
	  {{0x2add, 0, 1}, {0x2adc, 0, 1}, {0x2ae8, 0, 0},},
	  {{0x2add, 6, 1}, {0x2adc, 6, 1}, {0x2ae8, 6, 0},},
	  },
	 {
	  {{0x2ad3, 6, 1}, {0x2ad2, 6, 1}, {0x2a80, 6, 1},},
	  {{0x2add, 2, 1}, {0x2adc, 2, 1}, {0x2ae8, 2, 0},},
	  {{0x2ad5, 0, 1}, {0x2ad4, 0, 1}, {0x2a90, 0, 0},},
	  },
	 {
	  {{0x2ad5, 1, 1}, {0x2ad4, 1, 1}, {0x2a90, 1, 1},},
	  {{0x2ad5, 2, 1}, {0x2ad4, 2, 1}, {0x2a90, 2, 0},},
	  {{0x2ad5, 3, 1}, {0x2ad4, 3, 1}, {0x2a90, 3, 0},},
	  },
	 },
};

static const struct io_map_value f81534_rs232_coexist_control = {
	FINTEK_DEVICE_ID, F81534_NUM_PORT, uart_mode_shutdown,
	{
	 /* please reference f81439 io port */
	 {
	  {{0x2ad5, 4, 1}, {0x2ad4, 4, 1}, {0x2a90, 4, 1},},
	  {{0x2ad5, 5, 1}, {0x2ad4, 5, 1}, {0x2a90, 5, 0},},
	  {{0x2add, 7, 1}, {0x2adc, 7, 1}, {0x2ae8, 7, 1},},
	  },
	 {
	  {{0x2add, 3, 1}, {0x2adc, 3, 1}, {0x2ae8, 3, 1},},
	  {{0x2add, 0, 1}, {0x2adc, 0, 1}, {0x2ae8, 0, 0},},
	  {{0x2add, 6, 1}, {0x2adc, 6, 1}, {0x2ae8, 6, 1},},
	  },
	 {
	  {{0x2ad3, 6, 1}, {0x2ad2, 6, 1}, {0x2a80, 6, 1},},
	  {{0x2add, 2, 1}, {0x2adc, 2, 1}, {0x2ae8, 2, 0},},
	  {{0x2ad5, 0, 1}, {0x2ad4, 0, 1}, {0x2a90, 0, 1},},
	  },
	 {
	  {{0x2ad5, 1, 1}, {0x2ad4, 1, 1}, {0x2a90, 1, 1},},
	  {{0x2ad5, 2, 1}, {0x2ad4, 2, 1}, {0x2a90, 2, 0},},
	  {{0x2ad5, 3, 1}, {0x2ad4, 3, 1}, {0x2a90, 3, 1},},
	  },
	 },
};

static const struct io_map_value f81534_rs485_1_term_control = {
	FINTEK_DEVICE_ID, F81534_NUM_PORT, uart_mode_shutdown,
	{
	 /* please reference f81439 io port */
	 {
	  {{0x2ad5, 4, 1}, {0x2ad4, 4, 1}, {0x2a90, 4, 1},},
	  {{0x2ad5, 5, 1}, {0x2ad4, 5, 1}, {0x2a90, 5, 1},},
	  {{0x2add, 7, 1}, {0x2adc, 7, 1}, {0x2ae8, 7, 0},},
	  },
	 {
	  {{0x2add, 3, 1}, {0x2adc, 3, 1}, {0x2ae8, 3, 1},},
	  {{0x2add, 0, 1}, {0x2adc, 0, 1}, {0x2ae8, 0, 1},},
	  {{0x2add, 6, 1}, {0x2adc, 6, 1}, {0x2ae8, 6, 0},},
	  },
	 {
	  {{0x2ad3, 6, 1}, {0x2ad2, 6, 1}, {0x2a80, 6, 1},},
	  {{0x2add, 2, 1}, {0x2adc, 2, 1}, {0x2ae8, 2, 1},},
	  {{0x2ad5, 0, 1}, {0x2ad4, 0, 1}, {0x2a90, 0, 0},},
	  },
	 {
	  {{0x2ad5, 1, 1}, {0x2ad4, 1, 1}, {0x2a90, 1, 1},},
	  {{0x2ad5, 2, 1}, {0x2ad4, 2, 1}, {0x2a90, 2, 1},},
	  {{0x2ad5, 3, 1}, {0x2ad4, 3, 1}, {0x2a90, 3, 0},},
	  },
	 },
};

static const struct io_map_value *f81534_mode_control[uart_mode_invalid] = {
	&f81534_rs422_control,
	&f81534_rs232_control,
	&f81534_rs485_control,
	&f81534_rs485_1_control,
	&f81534_rs422_term_control,
	&f81534_rs232_coexist_control,
	&f81534_rs485_1_term_control,
	&f81534_shutdown_control,
};

static const struct usb_device_id id_table[] = {
	{USB_DEVICE(FINTEK_VENDOR_ID_1, FINTEK_DEVICE_ID)},
	{USB_DEVICE(FINTEK_VENDOR_ID_2, FINTEK_DEVICE_ID)},
	{}			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, id_table);

struct f81534_serial_private {
	bool is_phy_port_not_empty[F81534_NUM_PORT];
	spinlock_t tx_empty_lock;
	struct mutex change_mode_mutex;
	u8 default_conf_data[F81534_DEF_CONF_SIZE];
	u32 setting_idx;
	atomic_t port_active[F81534_NUM_PORT];
	struct mutex user_mutex;
};

struct f81534_port_private {
	u8 phy;
	u8 shadow_mcr;
	u8 shadow_lcr;
	u32 current_baud_rate;
	u32 current_baud_base;
#ifdef MULTIDROP_ENABLE
	u16 mode_9bit;
	u16 bitmask_9bit;
	u16 addr_9bit;
#endif
	struct f81534_pin_config_data port_pin_data;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip f81534_gpio_chip;
#endif
	atomic_t gpio_active;
#ifdef SAVE_CONTROL
	int port_flag;
#endif
	spinlock_t msr_lock;
	struct mutex msr_mutex;
	struct mutex lcr_mutex;
	u8 shadow_msr;
#ifdef LSR_ISSUE
	struct work_struct lsr_work;
#endif

#ifdef WRITER_WQ
	struct work_struct writer_work;
#endif
	struct usb_serial_port *port;
};

/*
 * Get the current port index of this device. e.g., 0 is the start index of
 * this device.
 */
static int f81534_port_index(struct usb_serial_port *port)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	int logic = port->port_number;
#else
	int logic = port->number - port->serial->minor;
#endif
	return logic;
}

/*
 * Find logic serial port index with H/W phy index mapping
 */
static int f81534_phy_to_logic_port(struct usb_serial *serial, int phy)
{
	int count = 0, i;
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);

	for (i = 0; i < phy; ++i) {
		if (serial_priv->default_conf_data[i] &
				F81534_PORT_UNAVAILABLE)
			continue;

		++count;
	}

	dev_dbg(&serial->dev->dev, "%s: phy:%d count:%d\n", __func__, phy,
			count);
	return count;
}

static int f81534_set_normal_register(struct usb_device *dev, u16 reg, u8 data)
{
	int count = F81534_USB_MAX_RETRY;
	int status = 0;
	u8 *tmp;

	tmp = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	*tmp = data;

	/*
	 * Our device maybe not reply when heavily loading,
	 * We'll retry for F81534_USB_MAX_RETRY times
	 */
	while (count--) {
		status = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
					 F81534_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_OUT,
					 reg, 0, tmp, sizeof(u8),
					 F81534_USB_TIMEOUT);
		if (status > 0)
			break;

		if (status == 0)
			status = -EIO;
	}

	if (status < 0) {
		dev_err(&dev->dev,
				"%s ERROR reg:%x data:0x%x status:%i failed\n",
				__func__, reg, data, status);
		kfree(tmp);
		return status;
	}

	kfree(tmp);
	return 0;
}

static int f81534_get_normal_register(struct usb_device *dev, u16 reg,
					u8 *data)
{
	int count = F81534_USB_MAX_RETRY;
	int status;
	u8 *tmp;

	tmp = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/*
	 * Our device maybe not reply when heavily loading,
	 * We'll retry for F81534_USB_MAX_RETRY times
	 */
	while (count--) {
		status = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
					 F81534_SET_GET_REGISTER,
					 USB_TYPE_VENDOR | USB_DIR_IN,
					 reg, 0, tmp, sizeof(u8),
					 F81534_USB_TIMEOUT);
		if (status > 0)
			break;

		if (status == 0)
			status = -EIO;
	}

	if (status < 0) {
		dev_err(&dev->dev, "%s ERROR reg:%x status:%i failed\n",
				__func__, reg, status);
		kfree(tmp);
		return status;
	}

	*data = *tmp;
	kfree(tmp);
	return 0;
}

static int f81534_set_mask_normal_register(struct usb_device *dev, u16 reg,
						u8 mask, u8 data)
{
	int status;
	u8 tmp;

	status = f81534_get_normal_register(dev, reg, &tmp);
	if (status)
		return status;

	tmp = (tmp & ~mask) | (mask & data);

	status = f81534_set_normal_register(dev, reg, tmp);
	if (status)
		return status;

	return 0;
}

static int f81534_command_delay(struct usb_serial *usbserial)
{
	unsigned int count = F81534_MAX_BUS_RETRY;
	unsigned char tmp;
	int status;
	struct usb_device *dev = usbserial->dev;

	do {
		status = f81534_get_normal_register(dev, F81534_BUS_REG_STATUS,
							&tmp);
		if (status)
			return status;

		if (tmp & F81534_BUS_BUSY)
			continue;

		if (tmp & F81534_BUS_IDLE)
			break;

	} while (--count);

	if (!count)
		return -EIO;

	status = f81534_set_normal_register(dev, F81534_BUS_REG_STATUS,
				tmp & ~F81534_BUS_IDLE);
	if (status)
		return status;

	return 0;
}

static int f81534_get_normal_register_with_delay(struct usb_serial *usbserial,
							u16 reg, u8 *data)
{
	int status;
	struct usb_device *dev = usbserial->dev;

	status = f81534_get_normal_register(dev, reg, data);
	if (status)
		return status;

	status = f81534_command_delay(usbserial);
	if (status)
		return status;

	return 0;
}

static int f81534_set_normal_register_with_delay(struct usb_serial *usbserial,
							u16 reg, u8 data)
{
	int status;
	struct usb_device *dev = usbserial->dev;

	status = f81534_set_normal_register(dev, reg, data);
	if (status)
		return status;

	status = f81534_command_delay(usbserial);
	if (status)
		return status;

	return 0;
}

static int f81534_read_data(struct usb_serial *usbserial, u32 address,
				u32 size, unsigned char *buf)
{
	u32 read_size, count;
	u32 block = 0;
	u16 reg_tmp;
	u8 tmp_buf[F81534_MAX_DATA_BLOCK];
	int status, offset;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_START, F81534_CMD_READ);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_START, (address >> 16) & 0xff);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_START, (address >> 8) & 0xff);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_START, (address >> 0) & 0xff);
	if (status)
		return status;

	/* continuous read mode */
	do {
		read_size = min_t(u32, F81534_MAX_DATA_BLOCK, size);

		for (count = 0; count < read_size; ++count) {
			/* We need write F81534_BUS_REG_END when final byte */
			if ((size <= F81534_MAX_DATA_BLOCK) &&
					(read_size == (count + 1)))
				reg_tmp = F81534_BUS_REG_END;
			else
				reg_tmp = F81534_BUS_REG_START;

			/*
			 * dummy code, force IC to generate a read
			 * pulse, the set of value 0xf1 is dont care
			 * (any value is ok)
			 */
			status = f81534_set_normal_register_with_delay(
					usbserial, reg_tmp, 0xf1);
			if (status)
				return status;

			status = f81534_get_normal_register_with_delay(
						usbserial,
						F81534_BUS_READ_DATA,
						&tmp_buf[count]);
			if (status)
				return status;

			offset = count + block * F81534_MAX_DATA_BLOCK;
			buf[offset] = tmp_buf[count];
		}

		size -= read_size;
		++block;
	} while (size);

	return 0;
}

/*
 * This function maybe cause IC no workable, Please take this carefully.
 *
 * The function is used to modify the configuration area of this device
 * (F81534_CUSTOM_ADDRESS_START). If wrong operation with this function, it'll
 * make the device malfuncional.
 */
static int f81534_write_data(struct usb_serial *usbserial, u32 address,
			     u32 size, unsigned char *buf)
{
	u32 count, write_size;
	u32 block = 0;
	u16 reg_tmp;
	int offset, status;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_END, F81534_CMD_ENABLE_WR);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_START, F81534_CMD_PROGRAM);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_START, (address >> 16) & 0xff);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_START, (address >> 8) & 0xff);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_START, (address >> 0) & 0xff);
	if (status)
		return status;

	do {
		write_size = min_t(u32, F81534_MAX_DATA_BLOCK, size);

		for (count = 0; count < write_size; ++count) {
			offset = count + block * F81534_MAX_DATA_BLOCK;

			if ((size <= F81534_MAX_DATA_BLOCK)
					&& (write_size == (count + 1)))
				reg_tmp = F81534_BUS_REG_END;
			else
				reg_tmp = F81534_BUS_REG_START;

			status = f81534_set_normal_register_with_delay(
						usbserial, reg_tmp,
						buf[offset]);
			if (status)
				return status;
		}

		size -= write_size;
		++block;
	} while (size);

	return 0;
}

/*
 * This function maybe cause IC no workable, Please take this carefully.
 *
 * The function is used to clear the configuration area of this device
 * (F81534_CUSTOM_ADDRESS_START). If wrong operation with this function, it'll
 * make the device malfuncional.
 */
static int f81534_erase_sector(struct usb_serial *usbserial, int address)
{
	u8 current_status = 0;
	int status;
	unsigned int count = F81534_MAX_BUS_RETRY;

	status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_END, F81534_CMD_ENABLE_WR);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
			F81534_BUS_REG_START, F81534_CMD_ERASE);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
			F81534_BUS_REG_START, (address >> 16) & 0xff);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
			F81534_BUS_REG_START, (address >> 8) & 0xff);
	if (status)
		return status;

	status = f81534_set_normal_register_with_delay(usbserial,
			F81534_BUS_REG_END, (address >> 0) & 0xff);
	if (status)
		return status;

	while (--count) {
		status = f81534_set_normal_register_with_delay(usbserial,
						F81534_BUS_REG_START,
						F81534_CMD_READ_STATUS);
		if (status)
			return status;

		/* dummy write, any value is acceptable */
		status = f81534_set_normal_register_with_delay(usbserial,
				F81534_BUS_REG_END, 0xff);
		if (status)
			return status;

		status = f81534_get_normal_register_with_delay(usbserial,
					F81534_BUS_READ_DATA, &current_status);
		if (status)
			return status;

		if (!(F81534_MEDIA_BUSY_STATUS & current_status)) {
			dev_dbg(&usbserial->dev->dev,
					"%s: data:%x, count:%d, ok\n",
					__func__, current_status, count);
			break;
		}
	}

	return 0;
}

static int f81534_prepare_write_buffer(struct usb_serial_port *port,
					void *dest, size_t size)
{
	unsigned char *ptr = (unsigned char *) dest;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	int port_num = port_priv->phy;
	struct usb_serial *serial = port->serial;

	WARN_ON(size != serial->port[0]->bulk_out_size);

	/*
	 * The block layout is fixed with 4x128 Bytes, per 128 Bytes
	 * for a port.
	 * index 0: port phy idx (e.g., 0,1,2,3)
	 * index 1: only F81534_TOKEN_WRITE
	 * index 2: serial out size
	 * index 3: fix to 0
	 * index 4~127: serial out data block
	 */
	ptr[F81534_RECEIVE_BLOCK_SIZE * 0] = 0;
	ptr[F81534_RECEIVE_BLOCK_SIZE * 1] = 1;
	ptr[F81534_RECEIVE_BLOCK_SIZE * 2] = 2;
	ptr[F81534_RECEIVE_BLOCK_SIZE * 3] = 3;
	ptr[F81534_RECEIVE_BLOCK_SIZE * port_num + 1] = F81534_TOKEN_WRITE;
	ptr[F81534_RECEIVE_BLOCK_SIZE * port_num + 3] = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	ptr[F81534_RECEIVE_BLOCK_SIZE * port_num + 2] =
		kfifo_out_locked(&port->write_fifo,
				&ptr[F81534_RECEIVE_BLOCK_SIZE * port_num + 4],
				F81534_MAX_TX_SIZE, &port->lock);
#else
	ptr[F81534_RECEIVE_BLOCK_SIZE * port_num + 2] =
		kfifo_get(port->write_fifo,
				&ptr[F81534_RECEIVE_BLOCK_SIZE * port_num + 4],
				F81534_MAX_TX_SIZE);
#endif

	return F81534_WRITE_BUFFER_SIZE;
}

static int f81534_submit_writer(struct usb_serial_port *port, gfp_t mem_flags)
{
	struct usb_serial *serial = port->serial;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);
	struct tty_struct *tty;
	struct urb *urb;
	bool cts_status = true;
	unsigned long flags;
	int updating_data, result;

	tty = tty_port_tty_get(&port->port);
	if (tty) {
		/* check H/W Flow status */
		if (C_CRTSCTS(tty)) {
			spin_lock_irqsave(&port_priv->msr_lock, flags);
			cts_status = !!(port_priv->shadow_msr & UART_MSR_CTS);
			spin_unlock_irqrestore(&port_priv->msr_lock, flags);
		}

		tty_kref_put(tty);
	}

	dev_dbg(&port->dev, "%s: check CTS: %d\n", __func__, cts_status);

	if (!cts_status)
		return 0;

	/* someone is changing setting, pause TX */
	updating_data = mutex_is_locked(&serial_priv->change_mode_mutex);
	if (updating_data)
		return 0;

	dev_dbg(&port->dev, "%s: check FIFO\n", __func__);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	/* check is any data in write_fifo */
	spin_lock_irqsave(&port->lock, flags);

	if (kfifo_is_empty(&port->write_fifo)) {
		spin_unlock_irqrestore(&port->lock, flags);
		return 0;
	}

	spin_unlock_irqrestore(&port->lock, flags);
#else
	if (!kfifo_len(port->write_fifo))
		return 0;
#endif

	dev_dbg(&port->dev, "%s: check TX_EMPTY\n", __func__);

	/* check H/W is TXEMPTY */
	spin_lock_irqsave(&serial_priv->tx_empty_lock, flags);

	if (serial_priv->is_phy_port_not_empty[port_priv->phy]) {
		spin_unlock_irqrestore(&serial_priv->tx_empty_lock, flags);
		return 0;
	}

	serial_priv->is_phy_port_not_empty[port_priv->phy] = true;
	spin_unlock_irqrestore(&serial_priv->tx_empty_lock, flags);

	dev_dbg(&port->dev, "%s: Ready to send\n", __func__);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	urb = port->write_urbs[0];
	f81534_prepare_write_buffer(port, port->bulk_out_buffers[0],
					port->bulk_out_size);
#else
	urb = port->write_urb;
	f81534_prepare_write_buffer(port, port->bulk_out_buffer,
					port->bulk_out_size);
#endif
	urb->transfer_buffer_length = F81534_WRITE_BUFFER_SIZE;

	result = usb_submit_urb(urb, mem_flags);
	if (result) {
		dev_err(&port->dev, "%s: submit error, result:%d\n", __func__,
				result);
		return result;
	}

	return 0;
}

static void f81534_wakeup_all_port(struct usb_serial *serial)
{
#ifdef WRITER_WQ
	int i;
	struct f81534_port_private *port_priv;
#else
	int i, status;
#endif

	for (i = 0; i < serial->num_ports; ++i) {
		if (!serial->port[i])
			continue;

#ifdef WRITER_WQ
		port_priv = usb_get_serial_port_data(serial->port[i]);
		schedule_work(&port_priv->writer_work);
#else
		status = f81534_submit_writer(serial->port[i], GFP_KERNEL);
		if (status) {
			dev_err(&serial->port[i]->dev, "%s: submit failed\n",
					__func__);
		}
#endif
	}
}

static int f81534_switch_gpio_mode(struct usb_serial_port *port, u8 mode)
{
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	int x = port_priv->phy;
	int y;
	int val;
	int status;
	int idx = (mode > F81534_PIN_SET_MAX) ? F81534_PIN_SET_DEFAULT : mode;
	struct usb_device *dev = port->serial->dev;
	const struct io_map_value *request_mode = f81534_mode_control[idx];
	/* our EVB m0 sometime will print as SD(Shutdown) */
	const struct pin_data *pins[3] = {&request_mode->port[x].m1,
						&request_mode->port[x].m2,
						&request_mode->port[x].m0_sd};

	if (mode > F81534_PIN_SET_MAX)
		return -EINVAL;

	for (y = 0; y < ARRAY_SIZE(pins); ++y) {
#if 1
		status = f81534_set_mask_normal_register(dev,
						     pins[y]->
						     port_mode_0.reg_address,
						     BIT(pins[y]->
						     port_mode_0.reg_offset),
						     pins[y]->port_mode_0.
						     reg_bit ? 0xff : 0x00);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
					__LINE__);
			return status;
		}

		status = f81534_set_mask_normal_register(dev,
						     pins[y]->
						     port_mode_1.reg_address,
						     BIT(pins[y]->
						     port_mode_1.reg_offset),
						     pins[y]->port_mode_1.
						     reg_bit ? 0xff : 0x00);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
					__LINE__);
			return status;
		}
#endif
		val = pins[y]->port_io.reg_bit ? 0xff : 0x00;
		status = f81534_set_mask_normal_register(dev,
					pins[y]->port_io.reg_address,
					BIT(pins[y]->port_io.reg_offset), val);
		if (status) {
			dev_err(&port->dev, "%s: failed, index:%d\n", __func__,
					y);
			return status;
		}
	}

	return 0;
}

/*
 * This function could be executed when
 *	1. Port configuration change. (e.g., UART/GPIO Mode changed)
 *	2. Old IC or configuration detected.
 *         During the port probe(), We'll check the current port is final port.
 *	   If we found a old style configuration value, the
 *	   f81534_load_configure_data() will transform old to new default
 *	   setting to RAM, then f81534_save_configure_data() will compare the
 *	   flash & RAM setting, If not the same, write it with new data with
 *	   final port probe().
 */
static int f81534_save_configure_data(struct usb_serial_port *port)
{
	int status;
	int count;
	int phy;
	int gpio_address, uart_address;
	int offset;
	bool reConfigure = false;
	u8 uart_mode, gpio_mode;
	u8 data[F81534_DEF_CONF_SIZE + 1];
	u8 tmp[F81534_DEF_CONF_SIZE];
	enum uart_mode current_mode;
	struct usb_serial *serial = port->serial;
	struct f81534_port_private *port_priv;
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);

	/* compare memory with ic data */
	for (count = 0; count < serial->num_ports; ++count) {
		port_priv = usb_get_serial_port_data(serial->port[count]);

		if (!port_priv) {
			dev_err(&port->dev, "%s: port_priv == NULL\n",
					__func__);
			continue;
		}

		phy = port_priv->phy;

#ifdef SAVE_CONTROL
		if (port_priv->port_flag & PORT_NOSAVE) {
			dev_info(&serial->port[count]->dev,
					"%s: skipping check configuration\n",
					__func__);
			continue;
		}
#endif

		if (serial_priv->setting_idx == F81534_CUSTOM_NO_CUSTOM_DATA) {
			uart_address = F81534_DEF_CONF_ADDRESS_START + phy;
			gpio_address = F81534_DEF_CONF_ADDRESS_START + phy +
							F81534_CONF_SIZE;
		} else {
			/*
			 * if had custom setting, override
			 * 1st byte is a indicator, 0xff is empty,
			 * 0xf0 is had data. Skip with 1st data
			 */

			uart_address = F81534_CUSTOM_ADDRESS_START +
				serial_priv->setting_idx *
				F81534_CUSTOM_DATA_SIZE + phy +
				F81534_CONF_OFFSET;

			gpio_address = F81534_CUSTOM_ADDRESS_START +
				serial_priv->setting_idx *
				F81534_CUSTOM_DATA_SIZE + phy +
				F81534_CONF_SIZE + F81534_CONF_OFFSET;
		}

		status = f81534_read_data(port->serial, uart_address, 1,
						&uart_mode);
		if (status) {
			dev_err(&port->dev,
					"%s: read uart data fail. status:%d\n",
					__func__, status);
			return status;
		}

		status = f81534_read_data(port->serial, gpio_address, 1,
						&gpio_mode);
		if (status) {
			dev_err(&port->dev,
					"%s: read gpio data fail. status:%d\n",
					__func__, status);
			return status;
		}

		if (port_priv->port_pin_data.gpio_mode != gpio_mode)
			reConfigure = true;

		/* check uart flag */
		if (port_priv->port_pin_data.force_uart_mode ==
				uart_mode_rs232) {
			if ((uart_mode & F81534_MODE_MASK) !=
					F81534_RS232_FLAG)
				reConfigure = true;
		} else if (port_priv->port_pin_data.force_uart_mode ==
					uart_mode_rs485_1) {
			if ((uart_mode & F81534_MODE_MASK) !=
					F81534_RS485_1_FLAG)
				reConfigure = true;
		} else if (port_priv->port_pin_data.force_uart_mode ==
			   uart_mode_rs485) {
			if ((uart_mode & F81534_MODE_MASK) !=
					F81534_RS485_FLAG)
				reConfigure = true;
		} else {
			reConfigure = true;
		}

		if (reConfigure)
			break;
	}

	if (serial_priv->setting_idx == F81534_CUSTOM_NO_CUSTOM_DATA) {
		dev_info(&serial->dev->dev, "%s: force to reconfigure\n",
					__func__);
	} else if (!reConfigure) {
		dev_dbg(&serial->dev->dev, "%s: update-to-date\n", __func__);
		return 0;
	}

	dev_info(&serial->dev->dev, "%s: updating\n", __func__);

	/* next setting block */
	serial_priv->setting_idx =
			(serial_priv->setting_idx + 1) % F81534_CUSTOM_MAX_IDX;
	dev_info(&serial->dev->dev, "%s: saving to block index:%d\n", __func__,
			serial_priv->setting_idx);

	/* erase when start block is 0 */
	if (!serial_priv->setting_idx) {
		dev_dbg(&serial->dev->dev, "%s: need erase\n", __func__);

		/* erase */
		status = f81534_erase_sector(serial,
					F81534_CUSTOM_ADDRESS_START);
		if (status) {
			dev_err(&port->dev,
					"%s: erase sector failed! status:%d\n",
					__func__, status);
			return status;
		}
	} else {
		dev_dbg(&serial->dev->dev,
				"%s: dont need erase\n", __func__);
	}

	/* reprogram */
	for (count = 0; count < serial->num_ports; ++count) {
		port_priv = usb_get_serial_port_data(serial->port[count]);
		phy = port_priv->phy;
		current_mode = port_priv->port_pin_data.force_uart_mode;
		gpio_mode = port_priv->port_pin_data.gpio_mode;

#ifdef SAVE_CONTROL
		if (port_priv->port_flag & PORT_NOSAVE) {
			dev_info(&serial->port[count]->dev,
					"%s: skipping change configure\n",
					__func__);
			continue;
		}
#endif

		serial_priv->default_conf_data[phy + F81534_CONF_SIZE] =
								gpio_mode;
		serial_priv->default_conf_data[phy] &= ~(F81534_MODE_MASK);

		/* check uart flag */
		if (current_mode == uart_mode_rs232) {
			serial_priv->default_conf_data[phy] |=
					F81534_RS232_FLAG;
		} else if (current_mode == uart_mode_rs485_1) {
			serial_priv->default_conf_data[phy] |=
					F81534_RS485_1_FLAG;
		} else if (current_mode == uart_mode_rs485) {
			serial_priv->default_conf_data[phy] |=
					F81534_RS485_FLAG;
		} else {
			dev_err(&serial->dev->dev,
					"%s: current_mode error, value:%d\n",
					__func__, current_mode);
		}

		dev_info(&serial->dev->dev,
				"%s: port:%d uart_mode:%x, gpio_mode:%x\n",
				__func__, count,
				serial_priv->default_conf_data[phy + 0],
				gpio_mode);
	}

	/*
	 * 1st byte is a indicator, 0xff is empty, 0xf0 is had data
	 * only write 8 bytes of total 4 port uart & gpio mode
	 * so we need write 1+8 data
	 */

	/* token of data exist */
	data[0] = F81534_CUSTOM_VALID_TOKEN;
	memcpy(&data[1], serial_priv->default_conf_data, F81534_DEF_CONF_SIZE);

	offset = F81534_CUSTOM_ADDRESS_START +
			F81534_CUSTOM_DATA_SIZE * serial_priv->setting_idx;

	status = f81534_write_data(serial, offset, sizeof(data), data);
	if (status) {
		dev_err(&port->dev,
				"%s: f81534_write_data failed!! status:%d\n",
				__func__, status);
		return status;
	}

	/* recheck save & memory data */
	memset(tmp, 0, sizeof(tmp));

	status = f81534_read_data(serial,
				  F81534_CUSTOM_ADDRESS_START +
				  F81534_CUSTOM_DATA_SIZE *
				  serial_priv->setting_idx + 1, sizeof(tmp),
				  tmp);
	if (status) {
		dev_err(&port->dev,
				"%s: f81534_read_data failed!! status:%d\n",
				__func__, status);
		return status;
	}

	for (count = 0; count < F81534_DEF_CONF_SIZE; ++count) {
		if (tmp[count] == serial_priv->default_conf_data[count])
			continue;

		dev_err(&port->dev,
				"%s:read data error, count:%d, data:%x %x\n",
				__func__, count, tmp[count],
				serial_priv->default_conf_data[count]);
	}

	dev_dbg(&serial->dev->dev, "%s: complete\n", __func__);

	return 0;
}

#ifdef CONFIG_GPIOLIB
static int f81534_gpio_get(struct gpio_chip *chip, unsigned gpio_num)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_serial_port *port = container_of(chip->dev,
						struct usb_serial_port, dev);
#else
	struct usb_serial_port *port = container_of(chip->parent,
					struct usb_serial_port, dev);
#endif
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(port->serial);
	int current_mode;
	int status;

	status = mutex_lock_killable(&serial_priv->change_mode_mutex);
	if (status) {
		dev_err(&port->dev, "%s: interrupted!\n", __func__);
		return status;
	}

	current_mode = port_priv->port_pin_data.gpio_mode & BIT(gpio_num);

	mutex_unlock(&serial_priv->change_mode_mutex);
	f81534_wakeup_all_port(port->serial);

	return !!current_mode;
}

static int f81534_gpio_direction_in(struct gpio_chip *chip, unsigned gpio_num)
{
	/* always failed */
	return -EINVAL;
}

static int f81534_gpio_direction_out(struct gpio_chip *chip,
				     unsigned gpio_num, int val)
{
	/* always successful */
	return 0;
}

static void f81534_gpio_set(struct gpio_chip *chip, unsigned gpio_num, int val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_serial_port *port = container_of(chip->dev,
						struct usb_serial_port, dev);
#else
	struct usb_serial_port *port = container_of(chip->parent,
					struct usb_serial_port, dev);
#endif
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(port->serial);
	int current_mode;
	int status;

	status = mutex_lock_killable(&serial_priv->change_mode_mutex);
	if (status) {
		dev_err(&port->dev, "%s: interrupted!\n", __func__);
		return;
	}

	current_mode = port_priv->port_pin_data.gpio_mode;
	current_mode &= ~BIT(gpio_num);
	current_mode |= val ? BIT(gpio_num) : 0;

	status = f81534_switch_gpio_mode(port, current_mode);
	if (status) {
		dev_err(&port->dev, "%s: set gpio error!!\n", __func__);
		goto out;
	}

	dev_dbg(&port->dev, "%s: num: %d, val:%d\n", __func__, gpio_num, val);
	port_priv->port_pin_data.gpio_mode = current_mode;

out:
	mutex_unlock(&serial_priv->change_mode_mutex);
	f81534_wakeup_all_port(port->serial);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int f81534_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	/* F81532/534 provide output only output port */
	return GPIOF_DIR_OUT;
}
#endif

static int f81534_gpio_request(struct gpio_chip *chip, unsigned offset)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_serial_port *port = container_of(chip->dev,
						struct usb_serial_port, dev);
#else
	struct usb_serial_port *port = container_of(chip->parent,
					struct usb_serial_port, dev);
#endif
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);

	/* add current actives gpio */
	atomic_inc(&port_priv->gpio_active);
	return 0;
}

static void f81534_gpio_free(struct gpio_chip *chip, unsigned offset)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct usb_serial_port *port = container_of(chip->dev,
						struct usb_serial_port, dev);
#else
	struct usb_serial_port *port = container_of(chip->parent,
					struct usb_serial_port, dev);
#endif
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(port->serial);
	int status;

	/* if no current actives gpio, save to IC */
	if (atomic_dec_return(&port_priv->gpio_active) != 0)
		return;

	status = mutex_lock_killable(&serial_priv->change_mode_mutex);
	if (status) {
		dev_err(&port->dev, "%s: interrupted!\n", __func__);
		return;
	}

	f81534_save_configure_data(port);

	mutex_unlock(&serial_priv->change_mode_mutex);
	f81534_wakeup_all_port(port->serial);
}

static struct gpio_chip f81534_gpio_chip_templete = {
	.owner = THIS_MODULE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	.get_direction = f81534_gpio_get_direction,
#endif
	.get = f81534_gpio_get,
	.direction_input = f81534_gpio_direction_in,
	.set = f81534_gpio_set,
	.direction_output = f81534_gpio_direction_out,
	.request = f81534_gpio_request,
	.free = f81534_gpio_free,
	.ngpio = 3, /* M0(SD)/M1/M2 */
	.base = -1,
};

static int f81534_prepare_gpio(struct usb_serial_port *port)
{
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	int max_name = 32;
	char *name = NULL;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	int idx = port->minor;
#else
	int idx = port->number;
#endif
	int rc;

	name = devm_kzalloc(&port->dev, max_name, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	memcpy(&port_priv->f81534_gpio_chip, &f81534_gpio_chip_templete,
			sizeof(f81534_gpio_chip_templete));

	snprintf(name, max_name - 1, "%s-%d", IC_NAME, idx);

	port_priv->f81534_gpio_chip.label = name;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	port_priv->f81534_gpio_chip.dev = &port->dev;
#else
	port_priv->f81534_gpio_chip.parent = &port->dev;
#endif

	rc = gpiochip_add(&port_priv->f81534_gpio_chip);
	if (rc) {
		dev_err(&port->dev, "%s: f81534_prepare_gpio failed:%d\n",
				__func__, rc);
		return rc;
	}

	return 0;
}

static int f81534_release_gpio(struct usb_serial_port *port)
{
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);

	gpiochip_remove(&port_priv->f81534_gpio_chip);
	return 0;
}
#else
static int f81534_prepare_gpio(struct usb_serial_port *port)
{
	dev_info(&port->dev, "CONFIG_GPIOLIB is not enabled\n");
	dev_info(&port->dev, "The GPIOLIB interface will not register\n");
	return 0;
}

static int f81534_release_gpio(struct usb_serial_port *port)
{
	return 0;
}
#endif

static int f81534_calc_baud_divisor(u32 baudrate, u32 clockrate, u32 *remain)
{
	u32 divisor, rem;

	if (!baudrate)
		return 0;

	rem = clockrate % baudrate;

	if (remain)
		*remain = rem;

	/* Round to nearest divisor */
	divisor = DIV_ROUND_CLOSEST(clockrate, baudrate);

	return divisor;
}

static int f81534_setregister(struct usb_device *dev, u8 uart, u16 reg,
				u8 data)
{
	return f81534_set_normal_register(dev, reg + uart * 0x10, data);
}

static int f81534_getregister(struct usb_device *dev, u8 uart, u16 reg,
				u8 *data)
{
	return f81534_get_normal_register(dev, reg + uart * 0x10, data);
}

static int f81534_set_port_config(struct usb_device *dev, u8 port_number,
					 struct usb_serial_port *port,
					 u32 baudrate, u16 lcr)
{
	struct usb_serial *serial = port->serial;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	u16 device_port = port_priv->phy;
	u32 divisor = 0;
	u32 rem, baud_base;
	int status, count;
	u8 value;
	bool is_485_mode = false;
	bool is_need_invert = false;
	static u32 const baudrate_table[3] = { 1500000, 1152000, 921600};
	static u8 const clock_table[3] = { CLKSEL_24_MHZ, CLKSEL_18DOT46_MHZ,
						CLKSEL_14DOT77_MHZ};

#ifdef MULTIDROP_ENABLE
	bool is_9bits = port_priv->mode_9bit;
#endif

	switch (port_priv->port_pin_data.force_uart_mode) {
	case uart_mode_rs232:
	case uart_mode_shutdown:
	case uart_mode_rs232_coexist:
	case uart_mode_invalid:
		break;
	case uart_mode_rs485:
		is_need_invert = true;
	default:
		is_485_mode = true;
		break;
	}

	/*
	 * This device clock rate is configured by CLK_SEL_REGISTER bit 2~1.
	 * The following list is the list of clock rate.
	 *
	 * bit 2~1 = 00: 1.8461 MHz
	 * bit 2~1 = 01: 18.461 MHz
	 * bit 2~1 = 10: 24 MHz
	 * bit 2~1 = 11: 14.77 MHz
	 */
	if (!baudrate) {
		switch (port_priv->current_baud_base) {
		case 1500000: value = CLKSEL_24_MHZ; break;
		case 1152000: value = CLKSEL_18DOT46_MHZ; break;
		case 921600: value = CLKSEL_14DOT77_MHZ; break;
		default:
		case 115200: value = CLKSEL_1DOT846_MHZ; break;
		}
	} else if (baudrate <= 115200) {
		/* clock rate fixed 1.8461 MHz when below or equal 115200bps */
		value = CLKSEL_1DOT846_MHZ;
		divisor = f81534_calc_baud_divisor(baudrate, 115200, NULL);
		port_priv->current_baud_base = 115200;
	} else {
		/* Higher than 115200bps need calculate suitable clock rate */
		for (count = 0; count < ARRAY_SIZE(baudrate_table) ; ++count) {
			baud_base = baudrate_table[count];
			divisor = f81534_calc_baud_divisor(baudrate, baud_base,
								&rem);
			if (!rem) {
				dev_dbg(&port->dev, "%s: found clockbase %d\n",
						__func__,
						baudrate_table[count]);
				value = clock_table[count];
				port_priv->current_baud_base = baud_base;
				break;
			}
		}

		if (count >= ARRAY_SIZE(baudrate_table)) {
			dev_err(&port->dev,
					"%s: cant find suitable clockbase\n",
					__func__);
			return -EINVAL;
		}
	}

	value &= ~(F81534_RS485_MODE | F81534_RS485_INVERT);
	value |= is_485_mode ? F81534_RS485_MODE : 0;
	value |= is_need_invert ? F81534_RS485_INVERT : 0;
	value |= CLKSEL_TX_DELAY_1BIT;

#ifdef MULTIDROP_ENABLE
	if (is_9bits)
		value |= 1L << 6;	/* enable 9bits */
#endif

	status = f81534_setregister(serial->dev, device_port, CLK_SEL_REGISTER,
					value);
	if (status) {
		dev_err(&port->dev, "%s: CLK REG setting failed\n", __func__);
		return status;
	}

	if (port_priv->current_baud_rate <= 1200)
		value = F81534_1X_RXTRIGGER;	/* 128 FIFO & TL: 1x */
	else
		value = F81534_8X_RXTRIGGER;	/* 128 FIFO & TL: 8x */

#ifdef MULTIDROP_ENABLE
	if (is_9bits)
		value |= 1L << 4;	/* enable 9bits auto addr */
#endif

	status = f81534_setregister(serial->dev, device_port, CONFIG1_REGISTER,
					value);
	if (status) {
		dev_err(&port->dev, "%s: CONFIG1 setting failed\n", __func__);
		return status;
	}

	if (port_priv->current_baud_rate <= 1200)
		value = UART_FCR_TRIGGER_1 | UART_FCR_ENABLE_FIFO; /* TL: 1 */
	else if (port_priv->current_baud_rate >= 1152000)
		value = UART_FCR_R_TRIG_10 | UART_FCR_ENABLE_FIFO; /* TL: 8 */
	else
		value = UART_FCR_R_TRIG_11 | UART_FCR_ENABLE_FIFO; /* TL: 14 */

	status = f81534_setregister(serial->dev, device_port,
					    FIFO_CONTROL_REGISTER, value);
	if (status) {
		dev_err(&port->dev, "%s: FCR setting failed\n", __func__);
		return status;
	}

	if (baudrate) {
		value = UART_LCR_DLAB;
		status = f81534_setregister(serial->dev, device_port,
							LINE_CONTROL_REGISTER, value);
		if (status) {
			dev_err(&port->dev, "%s: set LCR failed, %d\n", __func__,
					status);
			return status;
		}

		value = divisor & 0xFF;
		status = f81534_setregister(serial->dev, device_port,
						DIVISOR_LATCH_LSB, value);
		if (status) {
			dev_err(&port->dev, "%s: set DLAB LSB failed, %d\n", __func__,
					status);
			return status;
		}

		value = (divisor >> 8) & 0xFF;
		status = f81534_setregister(serial->dev, device_port,
						DIVISOR_LATCH_MSB, value);
		if (status) {
			dev_err(&port->dev, "%s: set DLAB MSB failed, %d\n", __func__,
					status);
			return status;
		}
	}

	status = f81534_setregister(serial->dev, device_port,
						LINE_CONTROL_REGISTER, lcr);
	if (status) {
		dev_err(&port->dev, "%s: set LCR failed, %d\n", __func__,
				status);
		return status;
	}

#ifdef MULTIDROP_ENABLE
	if (is_9bits) {
		status = f81534_setregister(dev, port_number,
				SADDRESS_REGISTER, port_priv->addr_9bit);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
					__LINE__);
			return status;
		}

		status = f81534_setregister(dev, port_number, SADEN_REGISTER,
						port_priv->bitmask_9bit);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
					__LINE__);
			return status;
		}

	}
#endif
	return 0;
}

static int f81534_update_mctrl(struct usb_serial_port *port, unsigned int set,
				unsigned int clear)
{
	struct usb_device *dev = port->serial->dev;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	u8 tmp;
	int status;

	status = mutex_lock_killable(&port_priv->msr_mutex);
	if (status) {
		dev_info(&port->dev, "%s: interrupted!\n", __func__);
		return status;
	}

	if (((set | clear) & (TIOCM_DTR | TIOCM_RTS | TIOCM_LOOP)) == 0) {
		dev_dbg(&dev->dev, "%s -DTR|RTS not being set|cleared\n",
				__func__);
		mutex_unlock(&port_priv->msr_mutex);
		return 0;	/* no change */
	}

	/* 'set' takes precedence over 'clear' */
	clear &= ~set;

	/* always enable UART_MCR_OUT2 */
	tmp = UART_MCR_OUT2 | port_priv->shadow_mcr;

	if (clear & TIOCM_DTR) {
		tmp &= ~UART_MCR_DTR;
		dev_dbg(&dev->dev, "%s: port:%d clear DTR\n", __func__,
				port_priv->phy);
	}

	if (clear & TIOCM_RTS) {
		tmp &= ~UART_MCR_RTS;
		dev_dbg(&dev->dev, "%s: port:%d clear RTS\n", __func__,
				port_priv->phy);

	}

	if (clear & TIOCM_LOOP) {
		tmp &= ~UART_MCR_LOOP;
		dev_dbg(&dev->dev, "%s: port:%d clear LOOP\n", __func__,
				port_priv->phy);
	}

	if (set & TIOCM_DTR) {
		tmp |= UART_MCR_DTR;
		dev_dbg(&dev->dev, "%s: port:%d set DTR\n", __func__,
				port_priv->phy);

	}

	if (set & TIOCM_RTS) {
		tmp |= UART_MCR_RTS;
		dev_dbg(&dev->dev, "%s: port:%d set RTS\n", __func__,
				port_priv->phy);
	}

	if (set & TIOCM_LOOP) {
		tmp |= UART_MCR_LOOP;
		dev_dbg(&dev->dev, "%s: port:%d set LOOP\n", __func__,
				port_priv->phy);
	}


	status = f81534_setregister(dev, port_priv->phy,
					MODEM_CONTROL_REGISTER, tmp);
	if (status < 0) {
		dev_err(&port->dev, "%s- Error from MODEM_CTRL URB: %i\n",
				__func__, status);
		mutex_unlock(&port_priv->msr_mutex);
		return status;
	}

	port_priv->shadow_mcr = tmp;
	mutex_unlock(&port_priv->msr_mutex);
	return 0;
}

/*
 * This function will search the data area with token F81534_CUSTOM_VALID_TOKEN
 * for latest configuration index. If nothing found (*index = -1), the caller
 * will load default configure in F81534_DEF_CONF_ADDRESS_START section
 */
static int f81534_find_config_idx(struct usb_serial *serial, uintptr_t *index)
{
	int idx, status;
	u8 custom_data;
	int offset;

	for (idx = F81534_CUSTOM_MAX_IDX - 1; idx >= 0; --idx) {
		offset = F81534_CUSTOM_ADDRESS_START +
					F81534_CUSTOM_DATA_SIZE * idx;
		status = f81534_read_data(serial, offset, 1, &custom_data);
		if (status) {
			dev_err(&serial->dev->dev,
					"%s: read error, idx:%d, status:%d\n",
					__func__, idx, status);
			return status;
		}

		/*
		 * if had custom setting, override
		 * 1st byte is a indicator, 0xff is empty, 0xf0 is had data
		 */

		/* found */
		if (custom_data == F81534_CUSTOM_VALID_TOKEN)
			break;
	}

	*index = idx;
	return 0;
}

#ifdef HW_DISABLE
static int f81534_check_port_hw_disbled(struct usb_serial *serial,
					int port_index)
{
	int status;
	u8 old_mcr, msr, lsr;
	u8 msr_mask = UART_MSR_DCD | UART_MSR_RI | UART_MSR_DSR | UART_MSR_CTS;

	status = f81534_getregister(serial->dev, port_index,
				MODEM_STATUS_REGISTER, &msr);
	if (status)
		return status;

	dev_dbg(&serial->interface->dev, "MSR: %x\n", msr);

	if ((msr & msr_mask) != msr_mask)
		return 0;

	status = f81534_setregister(serial->dev, port_index,
				FIFO_CONTROL_REGISTER, 0xc7);
	if (status)
		return status;

	status = f81534_getregister(serial->dev, port_index,
				MODEM_CONTROL_REGISTER, &old_mcr);
	if (status)
		return status;

	status = f81534_setregister(serial->dev, port_index,
				MODEM_CONTROL_REGISTER, 0x10);
	if (status)
		return status;

	status = f81534_setregister(serial->dev, port_index,
				MODEM_CONTROL_REGISTER, 0x0);
	if (status)
		return status;

	msleep(60);
	status = f81534_getregister(serial->dev, port_index,
				LINE_STATUS_REGISTER, &lsr);
	if (status)
		return status;

	dev_dbg(&serial->interface->dev, "LSR: %x\n", lsr);

	status = f81534_setregister(serial->dev, port_index,
				MODEM_CONTROL_REGISTER, old_mcr);
	if (status)
		return status;

	if ((lsr & UART_LSR_BI) == UART_LSR_BI)
		return -ENODEV;

	return 0;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
static int f81534_calc_num_ports(struct usb_serial *serial,
		struct usb_serial_endpoints *epds)
#else
static int f81534_calc_num_ports(struct usb_serial *serial)
#endif
{
	uintptr_t setting_idx;
	int i;
	u8 verify_flag, num_port = 0;
	int status;
	unsigned char setting[F81534_CUSTOM_DATA_SIZE + 1];
	bool reset = false;

	/* check had custom setting */
	status = f81534_find_config_idx(serial, &setting_idx);
	if (status) {
		dev_err(&serial->dev->dev,
				"%s: f81534_find_config_idx read failed!!\n",
				__func__);
		return 0;
	}

	/* Save the configuration area idx as private data for attach() */
	usb_set_serial_data(serial, (void *) setting_idx);

	/*
	 * if had custom setting, override it.
	 * 1st byte is a indicator, 0xff is empty, F81534_CUSTOM_VALID_TOKEN
	 * is had data, then skip with 1st data
	 */
	if (setting_idx != F81534_CUSTOM_NO_CUSTOM_DATA) {
		status = f81534_read_data(serial,
					  F81534_CUSTOM_ADDRESS_START +
					  F81534_CUSTOM_DATA_SIZE *
					  setting_idx + 1, sizeof(setting),
					  setting);
		if (status) {
			dev_err(&serial->dev->dev,
					"%s: get custom data failed!!\n",
					__func__);
			return 0;
		}

		dev_info(&serial->dev->dev,
				"%s: read configure from block:%d\n", __func__,
				(int) setting_idx);

		verify_flag = F81534_MODE_MASK | F81534_PORT_CONF_DISABLE_PORT |
				F81534_PORT_CONF_NOT_EXIST_PORT;

		for (i = 0; i < F81534_NUM_PORT; ++i) {
			if (setting[i] != 0xff && setting[i] & ~verify_flag) {
				reset = true;
				break;
			} else if (!(setting[i] & F81534_PORT_UNAVAILABLE)) {
				if (setting[i + F81534_CONF_SIZE] >= 8) {
					reset = true;
					break;
				}
			}
		}
	} else {
		reset = true;
		dev_info(&serial->dev->dev, "%s: read configure default\n",
				__func__);
	}

	if (reset) {
		/* read default board setting */
		status = f81534_read_data(serial, F81534_DEF_CONF_ADDRESS_START,
					  F81534_CUSTOM_DATA_SIZE, setting);
		if (status) {
			dev_err(&serial->dev->dev,
					"%s: f81534_read_data read failed!!\n",
					__func__);
			return 0;
		}
	}

	/* new style, find all possible ports */
	for (i = 0; i < F81534_NUM_PORT; ++i) {
		if (f81534_check_port_hw_disbled(serial, i))
			continue;

		if (setting[i] & F81534_PORT_UNAVAILABLE)
			continue;

		++num_port;
	}

	if (num_port)
		return num_port;

	dev_err(&serial->dev->dev, "Read Failed!!, default 4 ports\n");
	return 4;		/* nothing found, oldest version IC */
}

static void f81534_set_termios(struct tty_struct *tty,
				struct usb_serial_port *port,
				struct ktermios *old_termios)
{
	struct usb_device *dev = port->serial->dev;
	struct f81534_port_private *port_priv;
	u32 baud, old_baud = 0;
	u16 new_lcr = 0;
	int status;

	port_priv = usb_get_serial_port_data(port);

	if (C_BAUD(tty) == B0)
		f81534_update_mctrl(port, 0, TIOCM_DTR | TIOCM_RTS);
	else if (old_termios && (old_termios->c_cflag & CBAUD) == B0)
		f81534_update_mctrl(port, TIOCM_DTR | TIOCM_RTS, 0);

	if (C_PARENB(tty)) {
		new_lcr |= UART_LCR_PARITY;

		if (!C_PARODD(tty))
			new_lcr |= UART_LCR_EPAR;

		if (C_CMSPAR(tty))
			new_lcr |= UART_LCR_SPAR;
	}

	if (C_CSTOPB(tty))
		new_lcr |= UART_LCR_STOP;

	switch (C_CSIZE(tty)) {
	case CS5:
		new_lcr |= UART_LCR_WLEN5;
		break;
	case CS6:
		new_lcr |= UART_LCR_WLEN6;
		break;
	case CS7:
		new_lcr |= UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		new_lcr |= UART_LCR_WLEN8;
		break;
	}

	if (old_termios)
		old_baud = old_termios->c_ospeed;

	baud = tty_get_baud_rate(tty);
	if (baud) {
		/* Our device is not support for 1Mbps baudrate */
		if ((baud == 1000000) || (baud > F81534_MAX_BAUDRATE)) {
			if (old_termios)
				old_baud = baud = old_termios->c_ospeed;
			else
				baud = F81534_DEFAULT_BAUD_RATE;
		}

		dev_dbg(&dev->dev, "%s-baud: %d\n", __func__, baud);
		tty_encode_baud_rate(tty, baud, baud);

		port_priv->current_baud_rate = baud;
	}

	mutex_lock(&port_priv->lcr_mutex);

	new_lcr |= (port_priv->shadow_lcr & UART_LCR_SBC);
	port_priv->shadow_lcr = new_lcr;

	if (old_baud == baud)
		baud = 0;

	status = f81534_set_port_config(dev, port_priv->phy, port, baud,
					new_lcr);
	if (status < 0)
		dev_err(&port->dev, "%s - f81534_set_port_config failed: %i\n",
				__func__, status);
	mutex_unlock(&port_priv->lcr_mutex);

	/* Re-Enable writer for to check H/W flow Control */
	status = f81534_submit_writer(port, GFP_KERNEL);
	if (status)
		dev_err(&port->dev, "%s: submit failed\n", __func__);
}

#ifdef USER_MODE
static int f81534_detect_mode(struct usb_serial_port *port)
{
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
					usb_get_serial_data(port->serial);
	int x = port_priv->phy;
	const struct io_map_value *request_mode =
					f81534_mode_control[uart_mode_shutdown];
	const struct pin_data *pins[3] = {&request_mode->port[x].m0_sd,
						&request_mode->port[x].m2,
						&request_mode->port[x].m1};
	struct usb_device *dev = port->serial->dev;
	int i;
	int status;
	u8 val;
	int mode = 0;

	status = mutex_lock_killable(&serial_priv->user_mutex);
	if (status)
		return status;

	status = f81534_switch_gpio_mode(port, F81534_PIN_SET_MAX);
	if (status) {
		dev_err(&port->dev, "%s: switch gpio mode failed!! status:%d\n",
				__func__, status);
		goto end;
	}

	for (i = 0; i < ARRAY_SIZE(pins); ++i) {
		status = f81534_get_normal_register(dev,
					pins[i]->port_io.reg_address, &val);
		if (status) {
			dev_err(&port->dev, "%s: failed, index:%d\n", __func__,
					i);
			goto end;
		}

		mode |= (!!(BIT(pins[i]->port_io.reg_offset) & val)) << i;
	}

	if (mode == F81534_PIN_SET_MAX) {
		mode = port_priv->port_pin_data.gpio_mode;
		dev_dbg(&port->dev, "%s: pin out mode: %x\n", __func__, mode);

		status = f81534_switch_gpio_mode(port,
					port_priv->port_pin_data.gpio_mode);
		if (status) {
			dev_err(&port->dev,
					"%s: switch gpio mode failed!! status:%d\n",
					__func__, status);
			goto end;
		}
	} else {
		port_priv->port_pin_data.force_uart_mode = mode;
		port_priv->port_pin_data.gpio_mode = mode;
		dev_dbg(&port->dev, "%s: user mode: %x\n", __func__, mode);
	}

end:
	mutex_unlock(&serial_priv->user_mutex);
	return status;
}
#endif

static int f81534_init_uart(struct usb_serial_port *port)
{
	struct f81534_port_private *port_priv;
	struct f81534_serial_private *serial_priv;
	int status;

	port_priv = usb_get_serial_port_data(port);
	serial_priv = usb_get_serial_data(port->serial);

#ifdef USER_MODE
	status = f81534_detect_mode(port);
	if (status) {
		dev_err(&port->dev,
				"%s: f81534_detect_mode failed!! status:%d\n",
				__func__, status);
		return status;
	}
#else
	status =
	    f81534_switch_gpio_mode(port, port_priv->port_pin_data.gpio_mode);
	if (status) {
		dev_err(&port->dev,
				"%s: switch gpio mode failed!! status:%d\n",
				__func__, status);
		return status;
	}
#endif

	status = f81534_setregister(port->serial->dev, port_priv->phy,
			INTERRUPT_ENABLE_REGISTER,
			UART_IER_MSI | UART_IER_THRI | UART_IER_RDI);

	return status;
}

static int f81534_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(port->serial);
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	int phy = port_priv->phy;
	int status;

	status = f81534_init_uart(port);
	if (status)
		return status;

	if (tty)
		f81534_set_termios(tty, port, NULL);

	atomic_inc(&serial_priv->port_active[phy]);
	return 0;
}

static void f81534_close(struct usb_serial_port *port)
{
	int i;
	unsigned long flags;
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(port->serial);
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	int phy = port_priv->phy;

	atomic_dec(&serial_priv->port_active[phy]);

#ifdef WRITER_WQ
	cancel_work_sync(&port_priv->writer_work);
	flush_scheduled_work();
#endif

	/* referenced from usb_serial_generic_close() */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	for (i = 0; i < ARRAY_SIZE(port->write_urbs); ++i)
		usb_kill_urb(port->write_urbs[i]);
#else
	usb_kill_urb(port->write_urb);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	spin_lock_irqsave(&port->lock, flags);
	kfifo_reset_out(&port->write_fifo);
	spin_unlock_irqrestore(&port->lock, flags);
#endif

	dev_dbg(&port->dev, "%s\n", __func__);

#ifdef DEBUG_TX_EMPTY
	/* check tx empty */
	i = 50;

	while (--i) {
		if (!serial_priv->is_phy_port_not_empty[phy])
			break;

		if (schedule_timeout_killable(msecs_to_jiffies(10))) {
			dev_info(&port->dev, "%s: breaked !!\n", __func__);
			break;
		}
	}

	if (i == 0) {
		dev_warn(&port->dev, "%s: force clear tx_empty\n", __func__);
		serial_priv->is_phy_port_not_empty[phy] = false;
	}
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
static void f81534_remove_urbs(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port = NULL;

	for (i = 1; i < serial->num_ports; ++i) {
		port = serial->port[i];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
		for (j = 0; j < ARRAY_SIZE(port->write_urbs); ++j) {
			usb_kill_urb(port->write_urbs[j]);
			usb_free_urb(port->write_urbs[j]);
			kfree(port->bulk_out_buffers[j]);
			clear_bit(j, &port->write_urbs_free);

			port->write_urbs[j] = NULL;
			port->bulk_out_buffers[j] = NULL;
		}
#else
		usb_kill_urb(port->write_urb);
		usb_free_urb(port->write_urb);
		kfree(port->bulk_out_buffer);

		if (!IS_ERR(port->write_fifo) && port->write_fifo) {
			kfifo_free(port->write_fifo);
			port->write_fifo = NULL;
		}
#endif
		port->write_urb = NULL;
		port->bulk_out_buffer = NULL;
		port->bulk_out_size = 0;
		port->bulk_out_endpointAddress = 0;
	}
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
static void f81534_disconnect(struct usb_serial *serial)
{
	f81534_remove_urbs(serial);
}
#endif

static void f81534_release(struct usb_serial *serial)
{
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);

	kfree(serial_priv);

	device_set_wakeup_enable(&serial->dev->dev, false);
}

static int f81534_get_serial_info(struct usb_serial_port *port,
				  struct serial_struct __user *retinfo)
{
	struct serial_struct tmp;
	struct f81534_port_private *port_priv;

	port_priv = usb_get_serial_port_data(port);
	if (!port_priv)
		return -EFAULT;

	if (!retinfo)
		return -EFAULT;

	memset(&tmp, 0, sizeof(tmp));

	tmp.type = PORT_16550A;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	tmp.port = port->port_number;
	tmp.line = port->minor;
#else
	tmp.port = port->number - port->serial->minor;
	tmp.line = port->number;
#endif

	tmp.baud_base = port_priv->current_baud_base;

	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;

	return 0;
}

#ifdef MULTIDROP_ENABLE
int f81534_mask_setregister(struct usb_device *dev, u8 uart, u16 reg, u8 mask,
				u8 data)
{
	int status;
	u8 tmp;

	status = f81534_getregister(dev, uart, reg, &tmp);
	if (status)
		return status;

	tmp &= ~mask;
	tmp |= (mask & data);

	status = f81534_setregister(dev, uart, reg, tmp);
	if (status)
		return status;

	return 0;
}
#endif

#ifdef MULTIDROP_ENABLE
static int f81534_set_9Bits(struct usb_serial_port *port, int __user *arg)
{
	struct usb_device *dev = port->serial->dev;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	int status;
	int port_num = port_priv->phy;
	int mode = 0;

	if (copy_from_user(&mode, (int *)arg, sizeof(int)))
		return -EFAULT;

	if (mode & (1L << 17)) {
		port_priv->mode_9bit = true;
		port_priv->addr_9bit = (mode >> 0) & 0xFF;
		port_priv->bitmask_9bit = (mode >> 8) & 0xFF;
	} else {
		port_priv->mode_9bit = port_priv->bitmask_9bit =
		    port_priv->addr_9bit = 0;
	}

	/* bit4 9bit mode */
	status = f81534_mask_setregister(dev, port_num,
				CONFIG1_REGISTER, (1 << 4),
				port_priv->mode_9bit << 4);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	/* bit6 auto addr */
	status = f81534_mask_setregister(dev, port_num,
				CLK_SEL_REGISTER, (1 << 6),
				port_priv->mode_9bit << 6);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	/* saddress */
	status = f81534_setregister(dev, port_num,
				SADDRESS_REGISTER, port_priv->addr_9bit);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	/* saden */
	status = f81534_setregister(dev, port_num,
				SADEN_REGISTER, port_priv->bitmask_9bit);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	/* bit4 sm2 */
	status = f81534_mask_setregister(dev, port_num,
				INTERRUPT_ENABLE_REGISTER, (1 << 4),
				port_priv->mode_9bit << 4);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	return 0;
}

static int f81534_get_9Bits(struct usb_serial_port *port, int __user *arg)
{
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);

	int data = 0;

	if (port_priv->mode_9bit != false)
		data = (port_priv->bitmask_9bit << 8) | (port_priv->addr_9bit);

	if (copy_to_user((int __user *)arg, &data, sizeof(int)))
		return -EFAULT;

	return 0;
}

static int f81534_get_sm2(struct usb_serial_port *port, int __user *arg)
{
	struct usb_device *dev = port->serial->dev;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	unsigned int data = 0;
	u8 tmp = 0;
	int status;
	int port_num = port_priv->phy;

	status = f81534_getregister(dev, port_num,
					INTERRUPT_ENABLE_REGISTER,
					&tmp);
	if (status) {
		dev_err(&port->dev, "%s: failed, %d\n", __func__, __LINE__);
		return status;
	}

	data = (tmp & 0x10) ? 1 : 0;

	if (copy_to_user((int __user *)arg, &data, sizeof(int)))
		return -EFAULT;

	return 0;
}
#endif

static int f81534_set_port_mode(struct usb_serial_port *port,
		enum uart_mode eMode)
{
	int status;
	u8 tmp;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);

	if (eMode > uart_mode_invalid)
		return -EINVAL;

	if (eMode != uart_mode_invalid) {
		status = f81534_getregister(port->serial->dev, port_priv->phy,
						CLK_SEL_REGISTER, &tmp);
		if (status)
			return status;

		tmp &= ~(F81534_RS485_MODE | F81534_RS485_INVERT);

		switch (port_priv->port_pin_data.force_uart_mode) {
		case uart_mode_rs232:
		case uart_mode_shutdown:
		case uart_mode_rs232_coexist:
			break;

		case uart_mode_rs485:
			tmp |= (F81534_RS485_MODE | F81534_RS485_INVERT);
			dev_dbg(&port->dev, "%s: uart_mode_rs485 URB:%x\n",
					__func__, tmp);
			break;

		default:
			tmp |= F81534_RS485_MODE;
			dev_dbg(&port->dev, "%s others URB:%x\n",
					__func__, tmp);
			break;

		}

		status = f81534_setregister(port->serial->dev, port_priv->phy,
						CLK_SEL_REGISTER, tmp);
		if (status)
			return status;
	}

	port_priv->port_pin_data.force_uart_mode = eMode;
	return 0;
}

#ifdef FLASH_RW_DRIVER
static int f81534_get_configure_data(struct usb_serial_port *port,
				     struct internal_data __user *arg)
{
	struct usb_serial *serial = port->serial;
	struct internal_data data;
	int nRet = 0;
	unsigned int max_block = F81534_MAX_DATA_BLOCK;

	memset(&data, 0, sizeof(data));

	if (copy_from_user
	    (&data, (struct internal_data __user *)arg, sizeof(data)))
		return -EFAULT;

	data.size = min(data.size, max_block);

	nRet = f81534_read_data(serial, data.address, data.size, data.buf);
	if (nRet)
		return nRet;

	if (copy_to_user
	    ((struct internal_data __user *)arg, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static int f81534_set_configure_data(struct usb_serial_port *port,
				     struct internal_data __user *arg)
{
	struct usb_serial *serial = port->serial;
	struct internal_data data;
	int nRet = 0;
	unsigned int max_block = F81534_MAX_DATA_BLOCK;

	memset(&data, 0, sizeof(data));

	if (copy_from_user
	    (&data, (struct internal_data __user *)arg, sizeof(data)))
		return -EFAULT;

	data.size = min(data.size, max_block);

	nRet = f81534_write_data(serial, data.address, data.size, data.buf);
	if (nRet)
		return nRet;

	if (copy_to_user
	    ((struct internal_data __user *)arg, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static int f81534_erase_configure_data(struct usb_serial_port *port,
				       struct internal_data __user *arg)
{
	struct usb_serial *serial = port->serial;
	struct internal_data data;
	int nRet = 0;

	memset(&data, 0, sizeof(data));

	if (copy_from_user
	    (&data, (struct internal_data __user *)arg, sizeof(data)))
		return -EFAULT;

	nRet = f81534_erase_sector(serial, data.address);
	if (nRet)
		return nRet;

	return 0;
}
#endif

static int f81534_ioctl_set_rs485(struct usb_serial_port *port,
					struct serial_rs485 __user *arg)
{

	struct serial_rs485 data;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(port->serial);
	struct usb_device *usb_dev = port->serial->dev;
	u16 device_port = port_priv->phy;
	int status;

	status = mutex_lock_killable(&serial_priv->change_mode_mutex);
	if (status) {
		dev_info(&port->dev, "%s: interrupted!\n", __func__);
		return status;
	}

	status = copy_from_user(&data, (struct serial_rs485 __user *)arg,
				sizeof(data));
	if (status) {
		status = -EFAULT;
		goto finish;
	}

	if (data.flags & SER_RS485_ENABLED) {
		if (data.flags & SER_RS485_RTS_ON_SEND) {
			dev_dbg(&port->dev, "%s: uart_mode_rs485_1\n",
					__func__);
			port_priv->port_pin_data.force_uart_mode =
					uart_mode_rs485_1;
		} else {
			dev_dbg(&port->dev, "%s: uart_mode_rs485\n", __func__);
			port_priv->port_pin_data.force_uart_mode =
					uart_mode_rs485;
		}
	} else {
		dev_dbg(&port->dev, "%s: uart_mode_rs232\n", __func__);
		port_priv->port_pin_data.force_uart_mode = uart_mode_rs232;
	}

	mutex_lock(&port_priv->lcr_mutex);
	status = f81534_set_port_config(usb_dev, device_port, port,
					port_priv->current_baud_rate,
					port_priv->shadow_lcr);
	if (status) {
		dev_err(&usb_dev->dev, "%s: set port error!!\n", __func__);
		goto finish;
	}

	status = f81534_save_configure_data(port);

finish:
	mutex_unlock(&port_priv->lcr_mutex);
	mutex_unlock(&serial_priv->change_mode_mutex);
	f81534_wakeup_all_port(port->serial);

	return status;
}

static int f81534_ioctl_get_rs485(struct usb_serial_port *port,
					struct serial_rs485 __user *arg)
{
	int status;
	struct serial_rs485 data;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(port->serial);

	status = mutex_lock_killable(&serial_priv->change_mode_mutex);
	if (status) {
		dev_info(&port->dev, "%s: interrupted!\n", __func__);
		return status;
	}

	memset(&data, 0, sizeof(data));

	switch (port_priv->port_pin_data.force_uart_mode) {
	case uart_mode_rs485:
		dev_dbg(&port->dev, "%s: uart_mode_rs485\n", __func__);
		data.flags = SER_RS485_ENABLED;
		break;
	case uart_mode_rs485_1:
		dev_dbg(&port->dev, "%s: uart_mode_rs485_1\n", __func__);
		data.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
		break;
	default:
		dev_dbg(&port->dev, "%s: uart_mode_rs232\n", __func__);
		break;
	}

	if (copy_to_user((struct serial_rs485 *)arg, &data,
			sizeof(struct serial_rs485)))
		status = -EFAULT;

	mutex_unlock(&serial_priv->change_mode_mutex);
	f81534_wakeup_all_port(port->serial);

	return status;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static int f81534_ioctl(struct tty_struct *tty, unsigned int cmd,
			unsigned long arg)
#else
static int f81534_ioctl(struct tty_struct *tty, struct file *file,
			unsigned int cmd, unsigned long arg)
#endif
{
	struct usb_serial_port *port = tty->driver_data;

	switch (cmd) {
	case TIOCGRS485:
		return f81534_ioctl_get_rs485(port,
						(struct serial_rs485 __user *)
						arg);

	case TIOCSRS485:
		return f81534_ioctl_set_rs485(port,
						(struct serial_rs485 __user *)
						arg);

	case TIOCGSERIAL:
		return f81534_get_serial_info(port,
						(struct serial_struct __user *)
						arg);
#ifdef MULTIDROP_ENABLE
	case FINTEK_SET_MULTI_DROP_MODE:
		return f81534_set_9Bits(port, (int __user *)arg);
	case FINTEK_GET_MULTI_DROP_MODE:
		return f81534_get_9Bits(port, (int __user *)arg);
	case FINTEK_GET_SM2_STATE:
		return f81534_get_sm2(port, (int __user *)arg);
#endif
#ifdef FLASH_RW_DRIVER
	case FINTEK_ERASE_DATA_PAGE:
		return f81534_erase_configure_data(port,
						(struct internal_data __user *)
						arg);

	case FINTEK_GET_DATA:
		return f81534_get_configure_data(port,
						(struct internal_data __user *)
						arg);

	case FINTEK_SET_DATA:
		return f81534_set_configure_data(port,
						(struct internal_data __user *)
						arg);
#endif
	default:
		break;
	}

	return -ENOIOCTLCMD;
}

#ifdef WRITER_WQ
static void f81534_writer_worker(struct work_struct *work)
{
	struct f81534_port_private *port_priv =
			container_of(work, struct f81534_port_private,
					writer_work);
	struct usb_serial_port *port = port_priv->port;
	int status;

	status = f81534_submit_writer(port, GFP_KERNEL);
	if (status) {
		dev_err(&port->dev, "%s: submit failed\n", __func__);
	}
}
#endif

#ifdef LSR_ISSUE
static void f81534_lsr_worker(struct work_struct *work)
{
	struct f81534_port_private *port_priv =
			container_of(work, struct f81534_port_private,
					lsr_work);
	struct usb_serial_port *port = port_priv->port;
	int status;
	u8 tmp;

	status = f81534_getregister(port->serial->dev, port_priv->phy,
				LINE_STATUS_REGISTER, &tmp);
	if (status) {
		dev_err(&port->dev, "%s: read failed: %x\n",__func__, status);
	}
}
#endif

static void f81534_compare_msr(struct usb_serial_port *port, u8 msr,
				bool is_port_open)
{
	u8 old_msr;
	struct tty_struct *tty = NULL;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	unsigned long flags;
#ifndef WRITER_WQ
	int status;
#endif

	if (!(msr & UART_MSR_ANY_DELTA))
		return;

	spin_lock_irqsave(&port_priv->msr_lock, flags);
	old_msr = port_priv->shadow_msr;
	port_priv->shadow_msr = msr;
	spin_unlock_irqrestore(&port_priv->msr_lock, flags);

	if ((msr & (UART_MSR_CTS | UART_MSR_DCTS)) ==
			(UART_MSR_CTS | UART_MSR_DCTS)) {
		/* CTS changed, wakeup writer to re-check flow control */
		if (is_port_open) {
#ifdef WRITER_WQ
			schedule_work(&port_priv->writer_work);
#else
			status = f81534_submit_writer(port, GFP_ATOMIC);
			if (status) {
				dev_err(&port->dev, "%s: submit failed\n",
						__func__);
			}
#endif
		}
		dev_dbg(&port->dev, "%s: CTS Flag changed, value: %x\n",
				__func__, !!(msr & UART_MSR_CTS));
	}

	dev_dbg(&port->dev, "%s: MSR from %02x to %02x\n", __func__, old_msr,
			msr);

	if (!is_port_open)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	/* update input line counters */
	if (msr & UART_MSR_DCTS)
		port->icount.cts++;
	if (msr & UART_MSR_DDSR)
		port->icount.dsr++;
	if (msr & UART_MSR_DDCD)
		port->icount.dcd++;
	if (msr & UART_MSR_TERI)
		port->icount.rng++;
#endif
	wake_up_interruptible(&port->port.delta_msr_wait);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
	if (!(msr & UART_MSR_DDCD))
		return;

	dev_dbg(&port->dev, "%s: DCD Changed: port %d from %x to %x.\n",
			__func__, port_priv->phy, old_msr, msr);

	tty = tty_port_tty_get(&port->port);
	if (!tty)
		return;

	usb_serial_handle_dcd_change(port, tty, msr & UART_MSR_DCD);
	tty_kref_put(tty);
#endif
}

static void f81534_process_per_serial_block(struct usb_serial_port *port,
		unsigned char *data)
{
	u8 lsr, lsr_total = 0;
	char tty_flag;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
	struct tty_struct *tty;
#endif
	struct usb_serial *serial = port->serial;
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);
#ifdef LSR_ISSUE
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
#endif
#ifdef WRITER_WQ
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
#else
	int status;
#endif
	int phy_port_num = data[0];
	int read_size = 0;
	int i;
	unsigned long flags;
	bool available = !!atomic_read(
				&serial_priv->port_active[phy_port_num]);

	/*
	 * The block layout is 128 Bytes
	 * index 0: port phy idx (e.g., 0,1,2,3),
	 * index 1: It's could be
	 *			F81534_TOKEN_RECEIVE
	 *			F81534_TOKEN_TX_EMPTY
	 *			F81534_TOKEN_MSR_CHANGE
	 * index 2: serial in size (data+lsr, must be even)
	 *			meaningful for F81534_TOKEN_RECEIVE only
	 * index 3: current MSR with device read
	 * index 4~127: serial in data block (data+lsr, must be even)
	 */
	switch (data[1]) {
	case F81534_TOKEN_TX_EMPTY:
		/*
		 * We should record TX_EMPTY flag even the port is not opened
		 */
		spin_lock_irqsave(&serial_priv->tx_empty_lock, flags);
		serial_priv->is_phy_port_not_empty[phy_port_num] = false;
		spin_unlock_irqrestore(&serial_priv->tx_empty_lock, flags);
		usb_serial_port_softint(port);
		dev_dbg(&port->dev, "%s: F81534_TOKEN_TX_EMPTY\n", __func__);
		break;

	case F81534_TOKEN_MSR_CHANGE:
		/*
		 * We'll save MSR value when device reported even when port
		 * is not opened. If the port is not opened, the MSR will only
		 * recorded without any future process.
		 */
		f81534_compare_msr(port, data[3], available);
		dev_dbg(&port->dev, "%s: F81534_TOKEN_MSR_CHANGE\n", __func__);
		break;

	case F81534_TOKEN_RECEIVE:
		read_size = data[2];
		dev_dbg(&port->dev, "%s: F81534_TOKEN_RECEIVE read_size:%d\n",
				__func__, read_size);
		break;

	default:
		dev_warn(&port->dev, "%s: unknown token:%02x\n", __func__,
				data[1]);
		return;
	}

	/* if the port not had open, dont do future process */
	if (!available)
		return;

	/* Wakeup writer workqueue only when port is opened */
	if (data[1] == F81534_TOKEN_TX_EMPTY) {
#ifdef WRITER_WQ
		schedule_work(&port_priv->writer_work);
#else
		status = f81534_submit_writer(port, GFP_ATOMIC);
		if (status)
			dev_err(&port->dev, "%s: submit failed\n", __func__);
#endif
	}

	if (data[1] != F81534_TOKEN_RECEIVE)
		return;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
	tty = tty_port_tty_get(&port->port);
	if (!tty)
		return;
#endif

	for (i = 4; i < 4 + read_size; i += 2) {
		tty_flag = TTY_NORMAL;
		lsr = data[i + 1];

		lsr_total |= lsr;
		
		if (lsr & UART_LSR_BRK_ERROR_BITS) {
			//dev_warn(&port->dev, "lsr : %x\n", lsr);

			if (lsr & UART_LSR_BI) {
				tty_flag = TTY_BREAK;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				port->icount.brk++;
#endif
				usb_serial_handle_break(port);
			} else if (lsr & UART_LSR_PE) {
				tty_flag = TTY_PARITY;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				port->icount.parity++;
#endif
			} else if (lsr & UART_LSR_FE) {
				tty_flag = TTY_FRAME;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				port->icount.frame++;
#endif
			}

			if (lsr & UART_LSR_OE) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
				port->icount.overrun++;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
				tty_insert_flip_char(&port->port, 0,
						TTY_OVERRUN);
#else
				tty_insert_flip_char(tty, 0, TTY_OVERRUN);
#endif
			}
		}


		if (
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
			port->port.console &&
#endif
			port->sysrq) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
			if (usb_serial_handle_sysrq_char(port, data[i]))
#else
			if (usb_serial_handle_sysrq_char(tty, port, data[i]))
#endif
				continue;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		tty_insert_flip_char(&port->port, data[i], tty_flag);
#else
		tty_insert_flip_char(tty, data[i], tty_flag);
#endif
	}

	if (lsr_total & UART_LSR_BRK_ERROR_BITS) {
		//if (phy_port_num != 3)
		dev_dbg(&port->dev, "lsr : %x\n", lsr_total);
#ifdef LSR_ISSUE
		schedule_work(&port_priv->lsr_work);
#endif
	}
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	tty_flip_buffer_push(&port->port);
#else
	tty_flip_buffer_push(tty);
	tty_kref_put(tty);
#endif
}

static void f81534_process_read_urb(struct urb *urb)
{
	int i;
	int phy_port_num;
	int tty_port_num;
	unsigned char *ch;
	struct usb_serial *serial;
 	struct f81534_serial_private *serial_priv = NULL;
	struct usb_serial_port *port = NULL;
	struct f81534_port_private *port_priv = NULL;

	if (!urb->actual_length)
		return;

	port = urb->context;
	serial = port->serial;
	ch = urb->transfer_buffer;
	serial_priv = usb_get_serial_data(serial);

	for (i = 0; i < urb->actual_length; i += F81534_RECEIVE_BLOCK_SIZE) {
		phy_port_num = ch[i];
		if (phy_port_num >= F81534_NUM_PORT) {
			dev_err(&serial->dev->dev,
					"phy_port_num >= F81534_NUM_PORT: %d\n",
					phy_port_num);
			continue;
		}

		if(serial_priv->default_conf_data[phy_port_num] &
				F81534_PORT_UNAVAILABLE) {
			dev_dbg(&serial->dev->dev, 
					"phy_port_num: %d, skipped\n",
					phy_port_num);
			continue;
		}

		tty_port_num = f81534_phy_to_logic_port(serial, phy_port_num);
		port = serial->port[tty_port_num];

		/*
		 * The device will send back all information when we submitted
		 * a read URB (MSR/DATA/TX_EMPTY). But it maybe get callback
		 * before port_probe() or after port_remove().
		 *
		 * So we'll verify the pointer. If the pointer is NULL, it's
		 * mean the port not init complete and the block will skip.
		 */
		port_priv = usb_get_serial_port_data(port);
		if (!port_priv) {
			dev_dbg(&serial->dev->dev,
					"%s: phy: %d not ready!\n", __func__,
					phy_port_num);
			continue;
		}

		f81534_process_per_serial_block(port, &ch[i]);
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	i = usb_submit_urb(urb, GFP_ATOMIC);
	if (i)
		dev_err(&port->dev,
				"%s: resubmitting read URB, failed %d\n",
				__func__, i);
#endif
}

static void f81534_write_usb_callback(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	int status = urb->status;

	if (status) {
		dev_warn(&port->dev, "%s - non-zero URB status: %d\n",
				__func__, status);
	} else {
		usb_serial_port_softint(port);
	}
}

static int f81534_setup_urbs(struct usb_serial *serial)
{
	int i = 0;
	u8 port0_out_address;
	int j;
	int buffer_size;
	struct usb_serial_port *port = NULL;

	/*
	 * In our system architecture, we had 4 or 2 serial ports,
	 * but only get 1 set of bulk in/out endpoints.
	 *
	 * The usb-serial subsystem will generate port 0 data,
	 * but port 1/2/3 will not. It's will generate write URB and buffer
	 * by following code
	 */
	for (i = 1; i < serial->num_ports; ++i) {
		port0_out_address = serial->port[0]->bulk_out_endpointAddress;
		buffer_size = serial->port[0]->bulk_out_size;
		port = serial->port[i];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
		if (kfifo_alloc(&port->write_fifo, PAGE_SIZE, GFP_KERNEL))
			goto failed;
#else
		port->write_fifo = kfifo_alloc(PAGE_SIZE, GFP_KERNEL,
								&port->lock);
		if (IS_ERR(port->write_fifo))
			goto failed;
#endif

		port->bulk_out_size = buffer_size;
		port->bulk_out_endpointAddress = port0_out_address;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
		for (j = 0; j < ARRAY_SIZE(port->write_urbs); ++j) {
			set_bit(j, &port->write_urbs_free);

			port->write_urbs[j] = usb_alloc_urb(0, GFP_KERNEL);
			if (!port->write_urbs[j])
				goto failed;

			port->bulk_out_buffers[j] = kmalloc(buffer_size,
								GFP_KERNEL);
			if (!port->bulk_out_buffers[j])
				goto failed;

			usb_fill_bulk_urb(port->write_urbs[j], serial->dev,
					usb_sndbulkpipe(serial->dev,
						port0_out_address),
					port->bulk_out_buffers[j], buffer_size,
					serial->type->write_bulk_callback,
					port);
		}

		port->write_urb = port->write_urbs[0];
		port->bulk_out_buffer = port->bulk_out_buffers[0];
#else
		port->write_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!port->write_urb)
			goto failed;

		port->bulk_out_buffer = kmalloc(buffer_size, GFP_KERNEL);
		if (!port->bulk_out_buffer)
			goto failed;

		usb_fill_bulk_urb(port->write_urb, serial->dev,
			usb_sndbulkpipe(serial->dev,
				port0_out_address), port->bulk_out_buffer,
			buffer_size, serial->type->write_bulk_callback, port);
#endif

	}
	return 0;

failed:
	return -ENOMEM;
}

static int f81534_submit_read_urb(struct usb_serial *serial, gfp_t mem_flags)
{
	int status;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	status = usb_serial_generic_submit_read_urbs(serial->port[0],
				mem_flags);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	status = usb_serial_generic_submit_read_urb(serial->port[0],
				mem_flags);
#else
	/* Continue reading from device */
	usb_fill_bulk_urb(serial->port[0]->read_urb, serial->dev,
			usb_rcvbulkpipe(serial->dev,
				serial->port[0]->bulk_in_endpointAddress),
			serial->port[0]->read_urb->transfer_buffer,
			serial->port[0]->read_urb->transfer_buffer_length,
			f81534_process_read_urb, serial->port[0]);
	status = usb_submit_urb(serial->port[0]->read_urb, mem_flags);
	if (status)
		dev_err(&serial->dev->dev,
				"%s: submitting read URB, fail %d\n",
				__func__, status);
#endif
	if (status) {
		dev_err(&serial->dev->dev,
				"%s: submit read URB failed!! status:%d!!\n",
				__func__, status);
		return status;
	}

	return 0;
}

static int f81534_load_configure_data(struct usb_serial_port *port)
{
	int status;
	u8 uart_flag, gpio_mode;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(port->serial);
	int device_port = port_priv->phy;

	uart_flag = serial_priv->default_conf_data[device_port];
	gpio_mode = serial_priv->default_conf_data[device_port +
							F81534_CONF_SIZE];

	if (uart_flag & F81534_PORT_CONF_RS485) {
		if (uart_flag & F81534_PORT_CONF_RS485_INVERT)
			port_priv->port_pin_data.force_uart_mode =
					uart_mode_rs485;
		else
			port_priv->port_pin_data.force_uart_mode =
					uart_mode_rs485_1;
	} else {
		port_priv->port_pin_data.force_uart_mode =
				uart_mode_rs232;
	}

	if ((gpio_mode >= F81534_PIN_SET_MIN) &&
			(gpio_mode <= F81534_PIN_SET_MAX)) {
		port_priv->port_pin_data.gpio_mode = gpio_mode;
		dev_dbg(&port->dev, "gpio set to %d\n", gpio_mode);
	} else {
		port_priv->port_pin_data.gpio_mode = F81534_PIN_SET_DEFAULT;
		dev_info(&port->dev, "unknown gpio %d, setting to %d\n",
				gpio_mode, F81534_PIN_SET_DEFAULT);
	}

#if 0 //USER_MODE
	status = mutex_lock_killable(&serial_priv->user_mutex);
	if (status)
		return status;

	status = f81534_switch_gpio_mode(port, F81534_PIN_SET_MAX);
	if (status) {
		dev_err(&port->dev,
				"%s: switch gpio mode failed!! status:%d\n",
				__func__, status);
		mutex_unlock(&serial_priv->user_mutex);
		return status;
	}

	mutex_unlock(&serial_priv->user_mutex);
#else
	status =
	    f81534_switch_gpio_mode(port, port_priv->port_pin_data.gpio_mode);
	if (status) {
		dev_err(&port->dev,
				"%s: switch gpio mode failed!! status:%d\n",
				__func__, status);
		return status;
	}
#endif

	return 0;
}

static void dump_configure(struct usb_serial *serial)
{
	unsigned char transceiver, mode;
	int count;
	int index;
	int gpio_address, uart_address;
	struct f81534_serial_private *serial_priv = usb_get_serial_data(serial);

	index = serial_priv->setting_idx;

	for (count = 0; count < 4; ++count) {
		if (index == F81534_CUSTOM_NO_CUSTOM_DATA) {
			uart_address = F81534_DEF_CONF_ADDRESS_START + count;
			gpio_address = F81534_DEF_CONF_ADDRESS_START + count +
						F81534_CONF_SIZE;
		} else {

			/*
			 * if had custom setting, override
			 * 1st byte is a indicator.
			 * 0xff is empty, 0xf0 is had data.
			 * read and skip with 1st data.
			 */

			uart_address = F81534_CUSTOM_ADDRESS_START +
					F81534_CUSTOM_DATA_SIZE * index +
					count + F81534_CONF_OFFSET;

			gpio_address = F81534_CUSTOM_ADDRESS_START +
					F81534_CUSTOM_DATA_SIZE * index +
					count + F81534_CONF_SIZE +
					F81534_CONF_OFFSET;
		}

		f81534_read_data(serial, uart_address, 1, &transceiver);
		f81534_read_data(serial, gpio_address, 1, &mode);

		dev_info(&serial->dev->dev,
				"%s: port:%d uart_flag:%x gpio:%x\n", __func__,
				count, transceiver, mode);
	}
#if 0
	u8 buf[13];

	f81534_read_data(serial, F81534_CUSTOM_ADDRESS_START, sizeof(buf), buf);
	dev_info(&serial->dev->dev, "%x %x %x %x %x %x %x %x %x %x %x %x %x",
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
			buf[7], buf[8], buf[9], buf[10], buf[11], buf[12]);
#endif
}

static int f81534_detect_port_count(struct usb_serial *serial)
{
	int status;
	u8 buf[4];

	status = f81534_read_data(serial, 0x3022, 1, buf);
	if (status)
		return status;

	return buf[0] - '0';
}

static int f81534_fixup_aa66(struct usb_serial *serial)
{
	int status;
	int addr;
	int i;
	int port;
	bool flag = false;
	u8 buf[16];
	u8 config_port2[] = {0xf0, 0x00, 0x80, 0x80, 0x00, 0x07, 0x80, 0x80,
				0x07, 0x01, 0x01, 0x01, 0x01};
	u8 config_port4[] = {0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07,
				0x07, 0x01, 0x01, 0x01, 0x01};
	u8 *config;

	port = f81534_detect_port_count(serial);
	dev_info(&serial->dev->dev, "%s: %d port detected\n", __func__, port);

	switch (port) {
	case 2:
		addr = 0x0B5B;
		break;
	case 4:
		addr = 0x0C63;
		break;
	default:
		return -EINVAL;
	}

	status = f81534_read_data(serial, addr, 1, buf);
	if (status)
		return status;
	
	if (buf[0] != 0x00) {
		buf[0] = 0;
		dev_info(&serial->dev->dev, "%s: patch 1 Patching: %x\n", __func__,
				addr);

		status = f81534_write_data(serial, addr, 1, buf);
		if (status)
			return status;
	} else {
		dev_info(&serial->dev->dev, "%s: patch 1 Patched\n", __func__);
	}

	switch (port) {
	case 2:
		addr = 0x0AA4;
		break;
	case 4:
		addr = 0x0D26;
		break;
	default:
		return -EINVAL;
	}

	status = f81534_read_data(serial, addr, 1, buf);
	if (status)
		return status;
	
	if (buf[0] != 0x00) {
		dev_info(&serial->dev->dev, "%s: patch 2 Patching: %x %x\n", __func__,
				addr, buf[0]);

		buf[0] = 0;
		status = f81534_write_data(serial, addr, 1, buf);
		if (status)
			return status;
	} else {
		dev_info(&serial->dev->dev, "%s: patch 2 Patched\n", __func__);
	}

	status = f81534_read_data(serial, F81534_CUSTOM_ADDRESS_START,
					sizeof(buf), buf);
	if (status)
		return status;

	for (i = 5; i < 9; ++i) {
		if (buf[i] != 7 && buf[i] != 0x80) {
			dev_info(&serial->dev->dev, "%s: %d need reset\n",
					__func__, i);
			flag = true;
			break;
		}
	}

	if (flag) {
		switch (port) {
		case 2: config = config_port2; break;
		case 4: config = config_port4; break;
		default:
			return -EINVAL;
		}

		status = f81534_erase_sector(serial,
						F81534_CUSTOM_ADDRESS_START);
		if (status)
			return status;

		status = f81534_write_data(serial, F81534_CUSTOM_ADDRESS_START,
						13, config);
		if (status)
			return status;
	}

	return 0;
}

static int f81534_fixup(struct usb_serial *serial)
{
	int status;
	int ver = 0;
	int i;
	u8 buf[4];

	/* Get H/W version */
	status = f81534_read_data(serial, 0x1ffc, 4, buf);
	if (status)
		return status;

	for (i = 0; i < 4; ++i)
		ver |= buf[i] << (3 - i) * 4;

	dev_info(&serial->dev->dev, "%s: H/W version: %x\n", __func__, ver);

	switch (ver) {
	case 0xaa66:
		return f81534_fixup_aa66(serial);
	default:
		dev_err(&serial->dev->dev, "%s: ver: %x dont need patch\n",
			__func__, ver);
	}

	return 0;
}

static int f81534_attach(struct usb_serial *serial)
{
	struct f81534_serial_private *serial_priv = NULL;
	int status;
	int i;
	int offset;
	uintptr_t setting_idx = (uintptr_t) usb_get_serial_data(serial);
	bool reset = false;
	u8 verify_flag;

	status = f81534_fixup(serial);
	if (status)
		return status;

	serial_priv = kzalloc(sizeof(*serial_priv), GFP_KERNEL);
	if (!serial_priv)
		return -ENOMEM;

	usb_set_serial_data(serial, serial_priv);
	serial_priv->setting_idx = setting_idx;

	for (i = 0; i < F81534_NUM_PORT; ++i)
		atomic_set(&serial_priv->port_active[i], 0);

	spin_lock_init(&serial_priv->tx_empty_lock);
	mutex_init(&serial_priv->change_mode_mutex);
	mutex_init(&serial_priv->user_mutex);

	status = f81534_setup_urbs(serial);
	if (status)
		goto failed;

	pr_info(KBUILD_MODNAME ": " DRIVER_DESC ": " VERSION "\n");

	/*
	 * if had custom setting, override
	 * 1st byte is a indicator, 0xff is empty, 0xf0 is had data
	 * skip with 1st data
	 *
	 * if serial_priv->setting_idx == F81534_CUSTOM_NO_CUSTOM_DATA
	 * it's mean for no configuration is custom section, so we'll use
	 * default config read from F81534_DEF_CONF_ADDRESS_START
	 */
	if (serial_priv->setting_idx == F81534_CUSTOM_NO_CUSTOM_DATA) {
		reset = true;
	} else {
		offset = F81534_CUSTOM_ADDRESS_START +
				F81534_CUSTOM_DATA_SIZE * serial_priv->setting_idx + 1;
		/* only read 8 bytes for mode & GPIO */
		status = f81534_read_data(serial, offset,
						sizeof(serial_priv->default_conf_data),
						serial_priv->default_conf_data);
		if (status) {
			dev_err(&serial->dev->dev,
					"%s: get data failed, idx:%d, status:%d!!\n",
					__func__, serial_priv->setting_idx, status);
			goto failed;
		}

		verify_flag = F81534_MODE_MASK | F81534_PORT_CONF_DISABLE_PORT |
				F81534_PORT_CONF_NOT_EXIST_PORT;

		for (i = 0; i < F81534_NUM_PORT; ++i) {
			if (serial_priv->default_conf_data[i] != 0xff &&
					serial_priv->default_conf_data[i] & ~verify_flag) {
				reset = true;
				break;
			} else if (!(serial_priv->default_conf_data[i] &
					F81534_PORT_UNAVAILABLE)) {
				if (serial_priv->default_conf_data[i + F81534_CONF_SIZE] >= 8) {
					reset = true;
					break;
				}
			}
		}
	}

	if (reset) {
		dev_warn(&serial->dev->dev, "configure setting error\n");
		dump_configure(serial);
		dev_warn(&serial->dev->dev, "reset to default\n");

		/* read default board setting */
		status = f81534_read_data(serial, F81534_DEF_CONF_ADDRESS_START,
					  F81534_CUSTOM_DATA_SIZE,
					  serial_priv->default_conf_data);
		if (status) {
			dev_err(&serial->dev->dev,
					"%s: f81534_read_data read failed!!\n",
					__func__);
			goto failed;
		}

	}

	for (i = 0; i < F81534_NUM_PORT; ++i) {
		if (!f81534_check_port_hw_disbled(serial, i))
			continue;

		serial_priv->default_conf_data[i] |= F81534_PORT_UNAVAILABLE;		
	}

	/*
	 * We'll register port 0 bulkin only once, It'll take all port received
	 * data, MSR register change and TX_EMPTY information.
	 */
	status = f81534_submit_read_urb(serial, GFP_KERNEL);
	if (status) {
		dev_err(&serial->dev->dev,
				"%s: submit read URB failed!! status:%d!!\n",
				__func__, status);
		goto failed;
	}

	device_set_wakeup_enable(&serial->dev->dev, true);
	return 0;

failed:
	kfree(serial_priv);
	return status;
}

#ifdef SAVE_CONTROL
static ssize_t no_save_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);

	return sprintf(buf, "%d\n", !!(port_priv->port_flag & PORT_NOSAVE));
}

static ssize_t no_save_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);

	if (!count)
		return -EINVAL;

	if (buf[0] != '0')
		port_priv->port_flag |= PORT_NOSAVE;
	else
		port_priv->port_flag &= ~PORT_NOSAVE;

	return count;
}
#endif

#ifdef DEBUG_INTERNAL_SYSFS
static ssize_t internal_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	int fifo_size = kfifo_len(&port->write_fifo);
#else
	int fifo_size = kfifo_len(port->write_fifo);
#endif

	return sprintf(buf, "is using(txempty):%d\nfifo size:%d\n",
			serial_priv->is_phy_port_not_empty[port_priv->phy],
			fifo_size);
}

static ssize_t internal_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);

	if (!count)
		return -EINVAL;

	if (buf[0] == '0')
		serial_priv->is_phy_port_not_empty[port_priv->phy] = false;
	else
		serial_priv->is_phy_port_not_empty[port_priv->phy] = true;

	return count;
}

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	u8 tmp;
	int i, status, len = 0;

	len += sprintf(&buf[len], "Port Phy: %d\n", port_priv->phy);
	for (i = 0; i < 12; ++i) {
		status = f81534_getregister(serial->dev, port_priv->phy,
						SERIAL_BASE_ADDRESS + i, &tmp);
		if (status) {
			dev_err(&port->dev, "%s: failed, %d\n", __func__,
					__LINE__);
			return status;
		}
		len += sprintf(&buf[len], "Reg: %04x, value:%02x\n",
					SERIAL_BASE_ADDRESS + i, tmp);
	}

	return len;
}

static ssize_t reg_dump_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	//struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	int status;

	status = f81534_set_normal_register(serial->dev, 0x2007, 0x01);

	return count;
}
#endif

#ifndef CONFIG_GPIO_SYSFS
static ssize_t gpio_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);

	return sprintf(buf, "gpio mode: %d\n",
			port_priv->port_pin_data.gpio_mode);
}

static ssize_t gpio_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct usb_serial *serial = port->serial;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);
	int status;
	u8 tmp;

	if (count <= 0)
		return -EINVAL;

	tmp = buf[0] - '0';

	status = mutex_lock_killable(&serial_priv->change_mode_mutex);
	if (status) {
		dev_err(&port->dev, "%s: interrupted!\n", __func__);
		return -EINTR;
	}

	status = f81534_switch_gpio_mode(port, tmp);
	if (!status) {
		port_priv->port_pin_data.gpio_mode = tmp;
		f81534_save_configure_data(port);
	}

	mutex_unlock(&serial_priv->change_mode_mutex);
	f81534_wakeup_all_port(port->serial);

	return status ? -EINVAL : count;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
#ifdef SAVE_CONTROL
static DEVICE_ATTR_RW(no_save);
#endif

#ifdef DEBUG_INTERNAL_SYSFS
static DEVICE_ATTR_RW(internal);
static DEVICE_ATTR_RW(reg_dump);
#endif

#ifndef CONFIG_GPIO_SYSFS
static DEVICE_ATTR_RW(gpio);
#endif

#else

#ifdef SAVE_CONTROL
static DEVICE_ATTR(no_save, S_IRUGO | S_IWUSR, no_save_show, no_save_store);
#endif

#ifdef DEBUG_INTERNAL_SYSFS
static DEVICE_ATTR(internal, S_IRUGO | S_IWUSR, internal_show, internal_store);
static DEVICE_ATTR(reg_dump, S_IRUGO | S_IWUSR, reg_dump_show, reg_dump_store);
#endif

#ifndef CONFIG_GPIO_SYSFS
static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show, gpio_store);
#endif

#endif

static int f81534_init_msr(struct usb_serial_port *port)
{
	int status;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;
	unsigned long flags;
	int phy = port_priv->phy;
	u8 msr;

	/* Get MSR initial value*/
	status = f81534_getregister(serial->dev, phy, MODEM_STATUS_REGISTER,
					&msr);
	if (status)
		return status;

	spin_lock_irqsave(&port_priv->msr_lock, flags);
	port_priv->shadow_msr = msr;
	spin_unlock_irqrestore(&port_priv->msr_lock, flags);
	return 0;
}

static int f81534_port_probe(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	struct f81534_serial_private *serial_priv =
			usb_get_serial_data(serial);
	struct f81534_port_private *port_priv = NULL;
	int status, i, count = 0;
	int port_index = f81534_port_index(port);

#ifdef SAVE_CONTROL
	status = device_create_file(&port->dev, &dev_attr_no_save);
	if (status)
		return -EPERM;
#endif

#ifdef DEBUG_INTERNAL_SYSFS
	status = device_create_file(&port->dev, &dev_attr_internal);
	if (status)
		return -EPERM;

	status = device_create_file(&port->dev, &dev_attr_reg_dump);
	if (status)
		return -EPERM;
#endif

#ifndef CONFIG_GPIO_SYSFS
	status = device_create_file(&port->dev, &dev_attr_gpio);
	if (status)
		return -EPERM;
#endif

	port_priv = kzalloc(sizeof(*port_priv), GFP_KERNEL);
	if (!port_priv)
		return -ENOMEM;

	atomic_set(&port_priv->gpio_active, 0);
	spin_lock_init(&port_priv->msr_lock);
	mutex_init(&port_priv->msr_mutex);
	mutex_init(&port_priv->lcr_mutex);
#ifdef WRITER_WQ
	INIT_WORK(&port_priv->writer_work, f81534_writer_worker);
	port_priv->port = port;
#endif

#ifdef LSR_ISSUE
	INIT_WORK(&port_priv->lsr_work, f81534_lsr_worker);
	port_priv->port = port;
#endif

	/* assign logic-to-phy mapping */
	port_priv->phy = F81534_UNUSED_PORT;

	for (i = 0; i < F81534_NUM_PORT; ++i) {
		if (serial_priv->default_conf_data[i] &
				F81534_PORT_UNAVAILABLE)
			continue;

		if (port_index == count) {
			port_priv->phy = i;
			break;
		}

		++count;
	}

	if (port_priv->phy == F81534_UNUSED_PORT) {
		status = -ENODEV;
		goto port_fail;
	}

	usb_set_serial_port_data(port, port_priv);
	dev_info(&port->dev, "%s: mapping to phy: %d\n", __func__,
			port_priv->phy);

	/*
	 * We'll read MSR reg only with port_porbe() for initial once, then
	 * the MSR will received from read URB with token
	 * F81534_TOKEN_MSR_CHANGE when MSR is changed.
	 */
	status = f81534_init_msr(port);
	if (status)
		goto port_fail;

	status = f81534_load_configure_data(port);
	if (status)
		goto port_fail;

	/*
	 * Driver will compare memory & flash configure. If it not the same,
	 * We'll save it when final port probed.
	 */
	if ((serial->num_ports - 1) == f81534_port_index(port)) {
		f81534_save_configure_data(port);
		dump_configure(serial);
	}

	status = f81534_set_port_mode(port,
				port_priv->port_pin_data.force_uart_mode);
	if (status < 0) {
		dev_err(&port->dev, "%s: initial setup failed phy: (%i)\n",
				__func__, port_priv->phy);
		goto port_fail;
	}

	status = f81534_prepare_gpio(port);
	if (status)
		goto port_fail;

	return 0;
port_fail:
	dev_err(&port->dev, "%s: failed, %d\n", __func__, status);
	kfree(port_priv);
	return status;
}

static int f81534_port_remove(struct usb_serial_port *port)
{
	int i;
	struct f81534_port_private *port_priv;
	struct usb_serial *serial = port->serial;
	struct usb_serial_port *port0 = serial->port[0];

#ifdef SAVE_CONTROL
	device_remove_file(&port->dev, &dev_attr_no_save);
#endif

#ifdef DEBUG_INTERNAL_SYSFS
	device_remove_file(&port->dev, &dev_attr_internal);
	device_remove_file(&port->dev, &dev_attr_reg_dump);
#endif

#ifndef CONFIG_GPIO_SYSFS
	device_remove_file(&port->dev, &dev_attr_gpio);
#endif

	/*
	 * Cancel "port0" read URB to avoid reference to a freed pointer on
	 * f81534_process_read_urb()
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	for (i = 0; i < ARRAY_SIZE(port0->read_urbs); ++i)
		usb_kill_urb(port0->read_urbs[i]);
#else
	usb_kill_urb(port0->read_urb);
#endif

	f81534_release_gpio(port);
	port_priv = usb_get_serial_port_data(port);
	kfree(port_priv);

	return 0;
}

static void f81534_break_ctl(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
	struct usb_device *dev = port->serial->dev;
	struct f81534_serial_private *serial_priv;
	struct f81534_port_private *port_priv;
	enum uart_mode old_uart_mode;
	int status;
	int phy;
	u8 old_gpio_mode, data;
	bool is_alternate_mode = false;

	serial_priv = usb_get_serial_data(port->serial);
	port_priv = usb_get_serial_port_data(port);
	phy = port_priv->phy;

	/* set/unset break mode */
	mutex_lock(&port_priv->lcr_mutex);

	if (break_state)
		port_priv->shadow_lcr |= UART_LCR_SBC;
	else
		port_priv->shadow_lcr &= ~UART_LCR_SBC;

	status = f81534_setregister(dev, port_priv->phy, LINE_CONTROL_REGISTER,
					port_priv->shadow_lcr);
	if (status) {
		dev_err(&port->dev, "set break failed: %d\n", status);
		return;
	}

	mutex_unlock(&port_priv->lcr_mutex);

	/* check is RS485 & need break mode */
	status = mutex_lock_killable(&serial_priv->change_mode_mutex);
	if (status) {
		dev_err(&port->dev, "change_mode_mutex busy: %d\n", status);
		return;
	}

	old_uart_mode = port_priv->port_pin_data.force_uart_mode;
	old_gpio_mode = port_priv->port_pin_data.gpio_mode;

	if (old_uart_mode == uart_mode_rs485 &&
			old_gpio_mode == uart_mode_rs485) {
		is_alternate_mode = true;
		data = 0;
	} else	if (old_uart_mode == uart_mode_rs485_1 &&
			old_gpio_mode == uart_mode_rs485_1) {
		is_alternate_mode = true;
		data = F81534_RS485_INVERT;
	}

	if (is_alternate_mode) {
		status = f81534_mask_setregister(dev, phy, CLK_SEL_REGISTER,
						F81534_RS485_INVERT, data);
		if (status) {
			dev_err(&port->dev, "set break alter mode failed: %d\n",
					status);
		}
	}

	mutex_unlock(&serial_priv->change_mode_mutex);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static int f81534_tiocmget(struct tty_struct *tty)
#else
static int f81534_tiocmget(struct tty_struct *tty, struct file *file)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	unsigned long flags;
	int r;
	u8 msr, mcr;

	/*
	 * We'll avoid to direct read MSR register. The IC will read the MSR
	 * changed and report it f81534_process_per_serial_block() by
	 * F81534_TOKEN_MSR_CHANGE.
	 *
	 * When this device in heavy loading (e.g., BurnInTest Loopback Test)
	 * The report of MSR register will delay received a bit. It's due to
	 * MSR interrupt is lowest priority in 16550A. So we decide to sleep
	 * a little time to pass the test.
	 */
	if (schedule_timeout_killable(
			msecs_to_jiffies(F81534_DELAY_READ_MSR))) {
		dev_info(&port->dev, "%s: breaked !!\n", __func__);
		return -EINTR;
	}

	mutex_lock(&port_priv->msr_mutex);
	spin_lock_irqsave(&port_priv->msr_lock, flags);

	msr = port_priv->shadow_msr;
	mcr = port_priv->shadow_mcr;

	spin_unlock_irqrestore(&port_priv->msr_lock, flags);
	mutex_unlock(&port_priv->msr_mutex);

	r = (mcr & UART_MCR_DTR ? TIOCM_DTR : 0) |
	    (mcr & UART_MCR_RTS ? TIOCM_RTS : 0) |
	    (msr & UART_MSR_CTS ? TIOCM_CTS : 0) |
	    (msr & UART_MSR_DCD ? TIOCM_CAR : 0) |
	    (msr & UART_MSR_RI ? TIOCM_RI : 0) |
	    (msr & UART_MSR_DSR ? TIOCM_DSR : 0);

	return r;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static int f81534_tiocmset(struct tty_struct *tty,
			   unsigned int set, unsigned int clear)
#else
static int f81534_tiocmset(struct tty_struct *tty,
			   struct file *file, unsigned int set,
			   unsigned int clear)
#endif
{
	struct usb_serial_port *port = tty->driver_data;

	return f81534_update_mctrl(port, set, clear);
}

static void f81534_dtr_rts(struct usb_serial_port *port, int on)
{
	if (on)
		f81534_update_mctrl(port, TIOCM_DTR | TIOCM_RTS, 0);
	else
		f81534_update_mctrl(port, 0, TIOCM_DTR | TIOCM_RTS);

	//dev_info(&port->dev, "%s: %d\n", __func__, on);
}

static int f81534_write(struct tty_struct *tty,
			struct usb_serial_port *port,
			const unsigned char *buf, int count)
{
#ifdef WRITER_WQ
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	int bytes_out;
#else
	int bytes_out, status;
#endif

	if (!count)
		return 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
	bytes_out = kfifo_in_locked(&port->write_fifo, buf, count,
					&port->lock);
#else
	bytes_out = kfifo_put(port->write_fifo, buf, count);
#endif
#ifdef WRITER_WQ
	schedule_work(&port_priv->writer_work);
#else
	status = f81534_submit_writer(port, GFP_ATOMIC);
	if (status) {
		dev_err(&port->dev, "%s: submit failed\n", __func__);
		return status;
	}

#endif
	return bytes_out;
}

static int f81534_resume(struct usb_serial *serial)
{
	struct usb_serial_port *port;
#ifdef WRITER_WQ
	int status;
	struct f81534_port_private *port_priv;
#else
	int status, error = 0;
#endif
	int i;

	status = f81534_submit_read_urb(serial, GFP_NOIO);
	if (status) {
		dev_err(&serial->dev->dev,
				"%s: submit read URB failed!! status:%d!!\n",
				__func__, status);
		return status;
	}

	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		if (!test_bit(ASYNCB_INITIALIZED, &port->port.flags))
			continue;

		status = f81534_init_uart(port);
		if (status) {
			dev_err(&port->dev, "%s: submit failed\n", __func__);
			++error;
		}

#ifdef WRITER_WQ
		port_priv = usb_get_serial_port_data(port);
		schedule_work(&port_priv->writer_work);
#else
		status = f81534_submit_writer(port, GFP_NOIO);
		if (status) {
			dev_err(&port->dev, "%s: submit failed\n", __func__);
			++error;
		}
#endif
	}

#ifdef WRITER_WQ
	return 0;
#else
	return error ? -EIO : 0;
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
static inline int usb_endpoint_maxp(const struct usb_endpoint_descriptor *epd)
{
	return __le16_to_cpu(epd->wMaxPacketSize);
}
#endif

static int f81534_probe(struct usb_serial *serial,
					const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *endpoint;
	struct usb_host_interface *iface_desc;
	struct device *dev;
	int num_bulk_in = 0;
	int num_bulk_out = 0;
	int size_bulk_in = 0;
	int size_bulk_out = 0;
	int i;

	dev = &serial->interface->dev;
	iface_desc = serial->interface->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			++num_bulk_in;
			size_bulk_in = usb_endpoint_maxp(endpoint);
		}

		if (usb_endpoint_is_bulk_out(endpoint)) {
			++num_bulk_out;
			size_bulk_out = usb_endpoint_maxp(endpoint);
		}
	}

	if (num_bulk_in != 1 || num_bulk_out != 1) {
		dev_err(dev, "expected endpoints not found\n");
		return -ENODEV;
	}

	if (size_bulk_out != F81534_WRITE_BUFFER_SIZE ||
			size_bulk_in != F81534_MAX_RECEIVE_BLOCK_SIZE) {
		dev_err(dev, "unsupported endpoint max packet size\n");
		return -ENODEV;
	}

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
static bool f81534_tx_empty(struct usb_serial_port *port)
{
	struct f81534_port_private *port_priv = usb_get_serial_port_data(port);
	struct f81534_serial_private *serial_priv =
				usb_get_serial_data(port->serial);
	unsigned long flags;
	bool status;

	spin_lock_irqsave(&serial_priv->tx_empty_lock, flags);
	status = serial_priv->is_phy_port_not_empty[port_priv->phy];
	spin_unlock_irqrestore(&serial_priv->tx_empty_lock, flags);

	return !status;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 5, 0)
static struct usb_driver f81534_driver = {
	.name = "F81534",
	.probe = usb_serial_probe,
	.disconnect = usb_serial_disconnect,
	.id_table = id_table,
	.suspend = usb_serial_suspend,
	.resume = usb_serial_resume,
	.no_dynamic_id = 1,
	.supports_autosuspend = 1,
};
#endif

static struct usb_serial_driver f81534_device = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = IC_NAME,
		   },
	.description = DRIVER_DESC,
	.id_table = id_table,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
	.usb_driver = &f81534_driver,
#endif
	.probe = f81534_probe,
	.open = f81534_open,
	.close = f81534_close,
	.write = f81534_write,
	.calc_num_ports = f81534_calc_num_ports,
	.attach = f81534_attach,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
	.disconnect = f81534_disconnect,
#endif
	.release = f81534_release,
	.port_probe = f81534_port_probe,
	.port_remove = f81534_port_remove,
	.break_ctl = f81534_break_ctl,
	.dtr_rts = f81534_dtr_rts,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)	
	.tx_empty = f81534_tx_empty,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	.process_read_urb = f81534_process_read_urb,
#endif
	.ioctl = f81534_ioctl,
	.tiocmget = f81534_tiocmget,
	.tiocmset = f81534_tiocmset,
	.write_bulk_callback = f81534_write_usb_callback,
	.set_termios = f81534_set_termios,
	.resume = f81534_resume,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
/* need to be check with version change 3.4.0 / 3.11.0 @@ */
static struct usb_serial_driver *const serial_drivers[] = {
	&f81534_device, NULL
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 5, 0)
	/* 3.4 ver */
module_usb_serial_driver(f81534_driver, serial_drivers);
#else

	/* 3.5 up */
module_usb_serial_driver(serial_drivers, id_table);
#endif

#else
static int __init f81534_init(void)
{
	int retval;

	retval = usb_serial_register(&f81534_device);
	if (retval)
		goto failed_usb_serial_register;

	retval = usb_register(&f81534_driver);
	if (retval)
		goto failed_usb_register;

	return 0;

failed_usb_register:
	usb_serial_deregister(&f81534_device);

failed_usb_serial_register:
	return retval;
}

static void __exit f81534_exit(void)
{
	usb_deregister(&f81534_driver);
	usb_serial_deregister(&f81534_device);
}

module_init(f81534_init);
module_exit(f81534_exit);
#endif

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Peter Hong <Peter_Hong@fintek.com.tw>");
MODULE_AUTHOR("Tom Tsai <Tom_Tsai@fintek.com.tw>");
MODULE_LICENSE("GPL");
