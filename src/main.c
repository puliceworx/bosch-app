/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// system includes ------------------------------------------------------------
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/logging/log.h>

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

// local includes -------------------------------------------------------------

// library includes -----------------------------------------------------------

// Macros and Defines ---------------------------------------------------------
LOG_MODULE_REGISTER(APPMAIN, CONFIG_APPMAIN_LOG_LEVEL);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LEDR_NODE DT_ALIAS(led0)
#define LEDG_NODE DT_ALIAS(led1)
#define LEDB_NODE DT_ALIAS(led2)

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

#define SW1_NODE	DT_ALIAS(sw1)
#if !DT_NODE_HAS_STATUS(SW1_NODE, okay)
#error "Unsupported board: sw1 devicetree alias is not defined"
#endif

#define RING_BUF_SIZE 1024

// enumerations ---------------------------------------------------------------

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
enum eled {RED_LED, GREEN_LED, BLUE_LED, END_LED};

// structures -----------------------------------------------------------------

/// define the gpio callback
static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;

// global parameter declarations ----------------------------------------------

// local parameter declarations -----------------------------------------------
uint8_t ring_buffer[RING_BUF_SIZE];
struct ring_buf ringbuf;

// local function prototypes --------------------------------------------------
static int configure_leds(void);
static int configure_btns(void);
static int configure_usb(void);

// constant parameter initializations -----------------------------------------
static const struct gpio_dt_spec leds[] = 
{
  GPIO_DT_SPEC_GET(LEDR_NODE, gpios),
  GPIO_DT_SPEC_GET(LEDG_NODE, gpios),
  GPIO_DT_SPEC_GET(LEDB_NODE, gpios)
};

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios,
							      {0});

const struct device *dev_usb = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

void button0_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
  LOG_DBG("Button 0 pressed at %" PRIu32 "", k_cycle_get_32());
}

void button1_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
  LOG_DBG("Button 1 pressed at %" PRIu32 "", k_cycle_get_32());
}

int main(void)
{
	int ret;
  uint8_t active_led = 0;

  LOG_DBG("Let's go!");

  if (!configure_leds())
  {
    LOG_ERR("leds not ready");
    return 0;
  }

  if (!configure_btns())
  {
    LOG_ERR("buttons not ready");
    return 0;
  }

  if (!configure_usb())
  {
    LOG_ERR("usb not ready");
    return 0;
  }

  while (1) 
  {
    for (uint8_t x = 0; x < 3; x++)
    {
      if (x == active_led)
      {
        ret = gpio_pin_set_dt(&leds[x], 1);
      }
      else
      {
        ret = gpio_pin_set_dt(&leds[x], 0);
      }

      if (ret < 0) 
      {
        LOG_ERR("led set fail");
        return 0;
      }
    }

    active_led++;
    if (active_led == END_LED)
    {
      active_led = RED_LED;
    }

		k_msleep(SLEEP_TIME_MS);
	}

  return 0;
}

static int configure_leds(void)
{
  int ret;

  if (!gpio_is_ready_dt(&leds[0]))
  {
    LOG_ERR("gpio not ready");
    return 0;
  }

  for (uint8_t x = 0; x < END_LED; x++)
  {
    ret = gpio_pin_configure_dt(&leds[x], GPIO_OUTPUT_INACTIVE);
    if (ret < 0) 
    {
      LOG_ERR("gpio red configure fail");
      return 0;
    }
  }

  // Turn on the red led solid until usb port is opened
  gpio_pin_configure_dt(&leds[RED_LED], GPIO_OUTPUT_ACTIVE);

  return 1;
}

static int configure_btns(void)
{
  int ret;

  if (!gpio_is_ready_dt(&button0)) 
  {
		LOG_ERR("Error: button device %s is not ready",
		       button0.port->name);
		return 0;
	}

  if (!gpio_is_ready_dt(&button1)) 
  {
    LOG_ERR("Error: button device %s is not ready",
           button1.port->name);
    return 0;
  }

	ret = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (ret != 0) 
  {
		LOG_ERR("Error %d: failed to configure %s pin %d",
		       ret, button0.port->name, button0.pin);
		return 0;
	}

  ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
  if (ret != 0) 
  {
    LOG_ERR("Error %d: failed to configure %s pin %d",
           ret, button1.port->name, button1.pin);
    return 0;
  }

  ret = gpio_pin_interrupt_configure_dt(&button0,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
			ret, button0.port->name, button0.pin);
		return 0;
	}

  ret = gpio_pin_interrupt_configure_dt(&button1,
                GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
      ret, button1.port->name, button1.pin);
    return 0;
  }

  gpio_init_callback(&button0_cb_data, button0_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button0_cb_data);

  gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1.pin));
  gpio_add_callback(button1.port, &button1_cb_data);

  LOG_ERR("Set up button0 at %s pin %d", button0.port->name, button0.pin);
  LOG_ERR("Set up button1 at %s pin %d", button1.port->name, button1.pin);

  return 1;
}

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) 
  {
		if (uart_irq_rx_ready(dev)) 
    {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) 
      {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) 
      {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) 
      {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) 
    {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) 
      {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) 
      {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

static int configure_usb(void)
{
  int ret;
  uint32_t baudrate, dtr = 0U;

  if (!device_is_ready(dev_usb)) 
  {
    LOG_ERR("CDC ACM device not ready");
    return 0;
  }

  ret = usb_enable(NULL);
  if (ret != 0) 
  {
    LOG_ERR("Failed to enable USB");
    return 0;
  }

  ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

  LOG_INF("Wait for DTR");
  while (true) 
  {
    uart_line_ctrl_get(dev_usb, UART_LINE_CTRL_DTR, &dtr);
    if (dtr) 
    {
      break;
    }
    else 
    {
      /* Give CPU resources to low priority threads. */
      k_sleep(K_MSEC(100));
    }
  }

  LOG_INF("DTR set");

  /* They are optional, we use them to test the interrupt endpoint */
  ret = uart_line_ctrl_set(dev_usb, UART_LINE_CTRL_DCD, 1);
  if (ret) 
  {
    LOG_WRN("Failed to set DCD, ret code %d", ret);
  }

  ret = uart_line_ctrl_set(dev_usb, UART_LINE_CTRL_DSR, 1);
  if (ret) 
  {
    LOG_WRN("Failed to set DSR, ret code %d", ret);
  }

  /* Wait 100ms for the host to do all settings */
  k_msleep(100);

  ret = uart_line_ctrl_get(dev_usb, UART_LINE_CTRL_BAUD_RATE, &baudrate);
  if (ret) 
  {
    LOG_WRN("Failed to get baudrate, ret code %d", ret);
  }
  else 
  {
    LOG_INF("Baudrate detected: %d", baudrate);
  }

  uart_irq_callback_set(dev_usb, interrupt_handler);

  /* Enable rx interrupts */
  uart_irq_rx_enable(dev_usb);

  return 1;
}

