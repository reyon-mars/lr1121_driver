#pragma once
#include <cstdint>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

struct lr1121_version
{
  uint8_t hw_version{};
  uint8_t use_case{};
  uint8_t fw_major{};
  uint8_t fw_minor{};
};

enum class lr1121_err : int32_t
{
  OK = ESP_OK,
  TimeoutBusy = 0x10001,
  SpiTransfer = 0x10002,
  InvalidArg = 0x10003,
};

struct lr1121_pins
{
  gpio_num_t nss;
  gpio_num_t busy;
  gpio_num_t rst;
};

class lr1121
{
public:

  lr1121(const lr1121 &) = delete;
  lr1121 &operator=(const lr1121 &) = delete;

  lr1121(spi_host_device_t host, const lr1121_pins &pins, int spi_clock_hz = 8'000'000) noexcept;

  lr1121_err init();

  lr1121_err reset(uint32_t reset_low_us = 1000, uint32_t post_reset_wait_ms = 3);

  lr1121_err get_version(lr1121_version *out);

  lr1121_err wait_while_busy(uint32_t timeout_us = 20000);

private:
  lr1121_err write_cmd(uint8_t op_hi, uint8_t op_lo);
  lr1121_err read_response(uint8_t *rx, size_t len);

  spi_host_device_t host_;
  lr1121_pins pins_;
  spi_device_handle_t dev_{};
  int clk_hz_{};
};
