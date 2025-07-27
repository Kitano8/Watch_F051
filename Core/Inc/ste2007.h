
#define LCD_send(x)	        WR_SPI(x)
#define LCD_command(cmd)	LCD_send(cmd)
#define LCD_data(data)          LCD_send(0x0100|(uint8_t)(data))
/* Page address set */
#define lcd_set_row(row) LCD_command(0xB0 | ((row) & 0x0F)) 

#define lcd_set_col(col) do { \
        LCD_command(0x10 | ((col)>>4)); /* Sets the DDRAM column address - upper 3-bit */ \
        LCD_command(0x00 | ((col) & 0x0F)); /* lower 4-bit */ \
    } while(0)
      
#define MAX_X 96
#define MAX_Y 68
      
#define _delay_ms HAL_Delay

void WR_SPI (uint16_t data);
void ste2007_init(void); 
void ste2007_sleep (void);
void ste2007_wakeup (void);
void set_pixel(uint8_t x, uint8_t y, uint8_t color, uint8_t width);
void clear_screen(void);
void send_v_buff(uint8_t x, uint8_t y, uint8_t width);
void send_3_buff(uint8_t x, uint8_t y);