#ifndef dis
#define dis

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

void draw_thing(String msg, String pos); // Draw something idk
#define SCREEN_ADDRESS 0x3c              // See datasheet for Address

// in order of appearance, Width, Height, SPI pin, Reset pint
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void init_DIS() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 type display not present"));
  }
  display.display(); // You must call .display() after draw command to apply
}

// It take string in, and it puts shit onto display
void draw_thing(String msg, String pos) {
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);                 // Use full 256 char 'Code Page 437' font

  if (pos == "top") {
    display.setCursor(0, 0);
    display.print(msg);
  } else if (pos == "bot") {
    display.setTextSize(2);
    display.setCursor(0, 32);
    display.print(msg);
  } else {
    return;
  }

  display.display();
}

#endif
