
#include "main.h"
#include "ste2007.h"

uint8_t v_buffer[2*14];  // шрифт 14 ширина и 16 высота
extern SPI_HandleTypeDef hspi1;

extern uint8_t BACK_COLOR, MENU_COLOR;

/*
Обмен по SPI1
*/


void WR_SPI (uint16_t data) {
  
  HAL_SPI_Transmit(&hspi1,(uint8_t *)(&data),1,10000);
}

void ste2007_sleep (void){
  LCD_command(0xA5); // Display All Points ON
  LCD_command(0xAE); // Display OFF
}

void ste2007_wakeup (void){
  LCD_command(0xA4); //  Normal Display Mode
  LCD_command(0xAF); // Display ON
}

void ste2007_init(void)
{
    // http://tuxotronic.org/wiki/component/lcd/ste2007
  
    LCD_command(0xE2); // Reset
    _delay_ms(10);
    // LCD_command(0x3D);  // Charge pump
    // LCD_command(0x01);  // Charge pump = 4 (default 5 is too hight for 3.0 volt)
    // LCD_command(0xE1);  // Additional VOP for contrast increase
    // LCD_command(0x16);  // from -127 to +127
    LCD_command(0xA4); // Power saver off
    LCD_command(0x2F); // Booster ON Voltage regulator ON Voltage follover ON
    LCD_command(0xAF); // LCD display on

    _delay_ms(10);
    
    //LCD_command(0xA7);    // инверсия
    LCD_command(0xA6);    // норм
    LCD_command(0xA1);      // слева направо  // не поддерживается
    //LCD_command(0xA0);      // справа налево    // не поддерживается
    //LCD_command(0xc8);        // сверху вниз  // поддерживается
    LCD_command(0xc0);        // снизу вверх    // поддерживается
}

void set_pixel(uint8_t x, uint8_t y, uint8_t color, uint8_t width){
  uint8_t byte=0,bite=0,temp=1;
  
  byte=y/8;     // номер байта 
  bite=y%8;     // номер бита в байте
  
  for(uint8_t i=0;i<bite;i++){
    temp<<=1;
  }
  
  if(color){                    // установить бит
    v_buffer[byte*width+x]|=temp;
  }
  else{                         // снять бит
    v_buffer[byte*width+x]&=~temp;
  }
}

void send_v_buff(uint8_t x, uint8_t y, uint8_t width){
  
  uint16_t i=0;
  
  lcd_set_row(y);             // верхняя половина
  lcd_set_col(x);

  for( i=0; i<width; i++)
    {
        LCD_data(v_buffer[i]);
    }
  
  lcd_set_row(y+1);             // нижняя половина
  lcd_set_col(x);
  
  for( i=0; i<width; i++)
    {
        LCD_data(v_buffer[width+i]);
    }
}

void send_3_buff(uint8_t x, uint8_t y){
  
    lcd_set_row(y);             // верх
    lcd_set_col(x);

    LCD_data(v_buffer[0]);
  
    lcd_set_row(y+1);             // середина
    lcd_set_col(x);

    LCD_data(v_buffer[1]);

  
    lcd_set_row(y+2);             // низ
    lcd_set_col(x);

    LCD_data(v_buffer[2]);
    
}

void clear_screen(void){
  lcd_set_row(0);
  lcd_set_col(0);
  
  uint8_t color=0;
  
  if(BACK_COLOR){
    color=0xff;
  }

  for(uint16_t i=0; i<MAX_X*9; i++)
    {
        LCD_data(color);
    }
}