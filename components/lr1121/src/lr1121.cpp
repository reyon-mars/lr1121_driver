#include "lr1121.hpp"
#include <cstdint>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "LR1121";

namespace
{
  constexpr uint8_t NOP = 0x00;

  constexpr uint8_t OP_GET_STATUS_H = 0x01, OP_GET_STATUS_L = 0x00;
  constexpr uint8_t OP_GET_VERSION_H = 0x01, OP_GET_VERSION_L = 0x01;

  constexpr int kSpiMode = 0;
}

Lr1121::Lr1121(spi_host_device_t host, const Lr1121Pins &pins, int spi_clock_hz) noexcept
    : host_(host), pins_(pins), clk_hz_(spi_clock_hz) {}

Lr1121Err Lr1121::init()
{

  gpio_config_t io{};

  io.intr_type = GPIO_INTR_DISABLE;
  io.mode = GPIO_MODE_INPUT;
  io.pin_bit_mask = 1ULL << pins_.busy;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  if (gpio_config(&io) != ESP_OK)
    return Lr1121Err::InvalidArg;

  io = {};
  io.intr_type = GPIO_INTR_DISABLE;
  io.mode = GPIO_MODE_OUTPUT;
  io.pin_bit_mask = 1ULL << pins_.rst;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  if (gpio_config(&io) != ESP_OK)
    return Lr1121Err::InvalidArg;

  spi_device_interface_config_t devcfg{};
  devcfg.clock_speed_hz = clk_hz_;
  devcfg.mode = kSpiMode;
  devcfg.spics_io_num = pins_.nss;
  devcfg.queue_size = 4;
  devcfg.flags = 0;
  devcfg.cs_ena_pretrans = 0;
  devcfg.cs_ena_posttrans = 0;

  auto err = spi_bus_add_device(host_, &devcfg, &dev_);
  if (err != ESP_OK)
    return Lr1121Err::SpiTransfer;

  return Lr1121Err::OK;
}

Lr1121Err Lr1121::reset(uint32_t reset_low_us, uint32_t post_reset_wait_ms)
{
  gpio_set_level(pins_.rst, 0);
  esp_rom_delay_us(reset_low_us);
  gpio_set_level(pins_.rst, 1);
  vTaskDelay(pdMS_TO_TICKS(post_reset_wait_ms));
  return wait_while_busy(250000);
}

Lr1121Err Lr1121::wait_while_busy(uint32_t timeout_us)
{
  const int busy_pin = pins_.busy;
  const int64_t start = esp_timer_get_time();
  while (gpio_get_level((gpio_num_t)busy_pin) != 0)
  {
    if ((uint32_t)(esp_timer_get_time() - start) > timeout_us)
    {
      ESP_LOGE(TAG, "BUSY timeout");
      return Lr1121Err::TimeoutBusy;
    }
  }
  return Lr1121Err::OK;
}

Lr1121Err Lr1121::write_cmd(uint8_t op_hi, uint8_t op_lo)
{
  auto e = wait_while_busy(20000);
  if (e != Lr1121Err::OK)
    return e;

  uint8_t tx[2] = {op_hi, op_lo};
  spi_transaction_t t{};
  t.length = sizeof(tx) * 8;
  t.tx_buffer = tx;

  auto err = spi_device_transmit(dev_, &t);
  return (err == ESP_OK) ? Lr1121Err::OK : Lr1121Err::SpiTransfer;
}

Lr1121Err Lr1121::read_response(uint8_t *rx, size_t len)
{
  if (!rx || len == 0)
    return Lr1121Err::InvalidArg;

  auto e = wait_while_busy(20000);
  if (e != Lr1121Err::OK)
    return e;

  uint8_t tx_dummy[32];
  size_t remaining = len;
  size_t offset = 0;

  while (remaining)
  {
    const size_t chunk = (remaining > sizeof(tx_dummy)) ? sizeof(tx_dummy) : remaining;
    std::memset(tx_dummy, NOP, chunk);

    spi_transaction_t t{};
    t.length = chunk * 8;
    t.tx_buffer = tx_dummy;
    t.rxlength = chunk * 8;
    t.rx_buffer = rx + offset;

    auto err = spi_device_transmit(dev_, &t);
    if (err != ESP_OK)
      return Lr1121Err::SpiTransfer;

    offset += chunk;
    remaining -= chunk;
  }
  return Lr1121Err::OK;
}

Lr1121Err Lr1121::get_version(Lr1121Version *out)
{
  if (!out)
    return Lr1121Err::InvalidArg;

  auto e = write_cmd(OP_GET_VERSION_H, OP_GET_VERSION_L);
  if (e != Lr1121Err::OK)
    return e;

  uint8_t buf[5] = {};
  e = read_response(buf, sizeof(buf));
  if (e != Lr1121Err::OK)
    return e;

  out->hw_version = buf[1];
  out->use_case = buf[2];
  out->fw_major = buf[3];
  out->fw_minor = buf[4];

  return Lr1121Err::OK;
}
