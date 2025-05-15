/* main/main.c */

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "esp_log.h"

#ifdef CONFIG_PSTAR_KCONFIG_DHT22_ENABLED

#include "pstar_dht22_hal.h"

#include "driver/gpio.h"

#define NUM_DHT22_SENSORS 4

static const gpio_num_t DHT22_PINS[NUM_DHT22_SENSORS] = {
  GPIO_NUM_5,  // judas - bottom server
  GPIO_NUM_18, // shinji - top server
  GPIO_NUM_19, // back of rack
  GPIO_NUM_21, // front of rack
};

// Array for sensor names
static const char* DHT22_NAMES[NUM_DHT22_SENSORS] = {"judas - bottom server",
                                                     "shinji - top server",
                                                     "back of rack",
                                                     "front of rack"};

static pstar_dht22_hal_handle_t dht22_handlers[NUM_DHT22_SENSORS];

#endif /* CONFIG_PSTAR_KCONFIG_DHT22_ENABLED */

/* --- Conditional Includes --- */

#ifndef CONFIG_PSTAR_EXAMPLE_NONE /* If building the main application */
#include "pstar_examples.h"
#endif /* CONFIG_PSTAR_EXAMPLE_NONE */

static const char* TAG = "Project Star";

/* --- MAIN APPLICATION LOGIC (Only compiled if selected in Kconfig) --- */
#ifdef CONFIG_PSTAR_EXAMPLE_NONE

void run_main_application(void)
{
  ESP_LOGI(TAG, "Running: Main Application");
#ifndef CONFIG_PSTAR_KCONFIG_DHT22_ENABLED
  /* I have 4 DHT22's and I am putting them in my server rack to monitor temperatures */
  while (1) {
    ESP_LOGE(TAG, "Recompile with DHT22 Enabled");
    vTaskDelay(portMAX_DELAY);
  }
#endif /* CONFIG_PSTAR_KCONFIG_DHT22_ENABLED */

  ESP_LOGI(TAG, "Initializing %d DHT22 Sensors", NUM_DHT22_SENSORS);
  for (uint8_t i = 0; i < NUM_DHT22_SENSORS; i++) {
    esp_err_t ret = pstar_dht22_hal_create_custom(DHT22_PINS[i], &dht22_handlers[i]);

    if (ret != ESP_OK) {
      ESP_LOGE(TAG,
               "Failed to initialize DHT22 Sensor '%s' on GPIO %d: %s",
               DHT22_NAMES[i], // Use sensor name
               DHT22_PINS[i],
               esp_err_to_name(ret));
      dht22_handlers[i] = NULL;
    } else {
      ESP_LOGI(TAG,
               "DHT22 Sensor '%s' on GPIO %d initialized successfully",
               DHT22_NAMES[i],
               DHT22_PINS[i]); // Use sensor name
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGI(TAG, "Starting periodic temperature monitoring.");

  float temperature, humidity;

  while (1) {
    ESP_LOGI(TAG, "--- Reading sensor data cycle ---");

    for (uint8_t i = 0; i < NUM_DHT22_SENSORS; i++) {
      if (!dht22_handlers[i]) {
        ESP_LOGW(TAG,
                 "Skipping uninitialized DHT22 Sensor '%s' on GPIO %d",
                 DHT22_NAMES[i],
                 DHT22_PINS[i]); // Use sensor name
        continue;
      }

      esp_err_t read_ret = pstar_dht22_hal_read_data(dht22_handlers[i], &humidity, &temperature);
      if (read_ret == ESP_OK) {
        ESP_LOGI(TAG,
                 "Sensor '%s' (GPIO %d): Temperature = %.1f C",
                 DHT22_NAMES[i], // Use sensor name
                 DHT22_PINS[i],
                 temperature);
      } else if (read_ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG,
                 "Sensor '%s' on GPIO %d: Read attempt too soon. Last read time not yet elapsed.",
                 DHT22_NAMES[i], // Use sensor name
                 DHT22_PINS[i]);
      } else {
        ESP_LOGE(TAG,
                 "Sensor '%s' on GPIO %d: Failed to read data. Error: %s",
                 DHT22_NAMES[i], // Use sensor name
                 DHT22_PINS[i],
                 esp_err_to_name(read_ret));
      }
    }

    ESP_LOGI(TAG, "--- End of cycle. Waiting for 2 seconds ---");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

#endif /* CONFIG_PSTAR_EXAMPLE_NONE */

/* --- PRIMARY ENTRY POINT --- */
void app_main(void)
{
  ESP_LOGI(TAG, "*** Project Star Firmware Starting ***");

#if CONFIG_PSTAR_EXAMPLE_NONE
  /* Run the actual main application */
  run_main_application();
#else
  /* Run the selected example via the dispatcher */
  run_selected_example();
#endif

  /* This point should ideally not be reached */
  ESP_LOGW(TAG, "app_main has unexpectedly finished.");
  /* Halt */
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}