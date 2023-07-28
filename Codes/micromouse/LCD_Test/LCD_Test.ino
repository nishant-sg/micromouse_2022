#include <LiquidCrystal.h>

#include <Wire.h>

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

void setup()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();


}

void loop()
{
    lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("Micromouse");        // print message at (0, 0)
  lcd.setCursor(2, 1);         // move cursor to   (2, 1)
  lcd.print("- MNG"); // print message at (2, 1)
}

