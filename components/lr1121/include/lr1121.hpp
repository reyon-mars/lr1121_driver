#pragma once
#include <cstdint>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

struct Lr1121Version
{
  uint8_t hw_version{};
  uint8_t use_case{};
  uint8_t fw_major{};
  uint8_t fw_minor{};
};

enum class Lr1121Err : int32_t
{
  OK = ESP_OK,
  TimeoutBusy = 0x10001,
  SpiTransfer = 0x10002,
  InvalidArg = 0x10003,
};

struct Lr1121Pins
{
  gpio_num_t nss;
  gpio_num_t busy;
  gpio_num_t rst;
};

class Lr1121
{
public:
  Lr1121(const Lr1121 &) = delete;
  Lr1121 &operator=(const Lr1121 &) = delete;

  Lr1121(spi_host_device_t host, const Lr1121Pins &pins, int spi_clock_hz = 8'000'000) noexcept;

  Lr1121Err init();

  Lr1121Err reset(uint32_t reset_low_us = 1000, uint32_t post_reset_wait_ms = 3);

  Lr1121Err get_version(Lr1121Version *out);

  Lr1121Err wait_while_busy(uint32_t timeout_us = 20000);

private:
  Lr1121Err write_cmd(uint8_t op_hi, uint8_t op_lo);
  Lr1121Err read_response(uint8_t *rx, size_t len);

  spi_host_device_t host_;
  Lr1121Pins pins_;
  spi_device_handle_t dev_{};
  int clk_hz_{};
};
