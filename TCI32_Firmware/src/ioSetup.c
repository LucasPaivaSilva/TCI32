#include "driver/gpio.h"

void setupGPIO() {
    // This funcition confiure the GPIO 
    gpio_config_t io_conf;

    //----------------------------------//
    // Enable GPIO2 as OUTPUT
    // This is the GPIO pin that is connected to the LED on the ESP32 DEV Board
    // This is also the pin used to control the Tesla Coil Driver
    // Disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // Set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Selects GPIO 2
    io_conf.pin_bit_mask = (1ULL<<((int) GPIO_NUM_2));
    // Disable pull-down mode
    io_conf.pull_down_en = 0;
    // Disable pull-up mode
    io_conf.pull_up_en = 0;
    // Configure GPIO with the given settings
    gpio_config(&io_conf);
    // Makes sure that GPIO2 is on Low (important to not fire the TC)
    gpio_set_level(GPIO_NUM_2, 0);
    //----------------------------------// 

    //----------------------------------//
    // Enable GPI32 as INPUT
    // Interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    // Set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<32);
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    // Configure GPIO with the given settings
    gpio_config(&io_conf); 
    //----------------------------------// 
}