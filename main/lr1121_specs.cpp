#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lr1121.hpp"

static constexpr gpio_num_t PIN_MOSI = GPIO_NUM_5;
static constexpr gpio_num_t PIN_MISO = GPIO_NUM_6;
static constexpr gpio_num_t PIN_SCLK = GPIO_NUM_4;

static constexpr Lr1121Pins LR_PINS{
    .nss = GPIO_NUM_7,
    .busy = GPIO_NUM_15,
    .rst = GPIO_NUM_16};

static const char *TAG = "APP";

extern "C" void app_main(void)
{
  spi_bus_config_t buscfg{};
  buscfg.mosi_io_num = PIN_MOSI;
  buscfg.miso_io_num = PIN_MISO;
  buscfg.sclk_io_num = PIN_SCLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 64;

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  Lr1121 lr(SPI2_HOST, LR_PINS, 8'000'000);
  auto err = lr.init();
  if (err != Lr1121Err::OK)
  {
    ESP_LOGE(TAG, "LR1121 init failed: 0x%x", (int)err);
    return;
  }

  err = lr.reset();
  if (err != Lr1121Err::OK)
  {
    ESP_LOGE(TAG, "LR1121 reset failed: 0x%x", (int)err);
    return;
  }

  Lr1121Version ver{};
  err = lr.get_version(&ver);
  if (err != Lr1121Err::OK)
  {
    ESP_LOGE(TAG, "GetVersion failed: 0x%x", (int)err);
    return;
  }

  ESP_LOGI(TAG, "LR1121 Version -> HardWare:0x%02X  UseCase:0x%02X  FirmWare:%u.%u",
           ver.hw_version, ver.use_case, ver.fw_major, ver.fw_minor);

  switch (ver.use_case)
  {
  case 0x03:
    ESP_LOGI(TAG, "Use Case: LR1121");
    break;
  case 0xDF:
    ESP_LOGW(TAG, "Use Case: Bootloader mode!");
    break;
  default:
    ESP_LOGW(TAG, "Use Case: 0x%02X (unexpected for LR1121)", ver.use_case);
    break;
  }

  while (true)
    vTaskDelay(pdMS_TO_TICKS(1000));
}
