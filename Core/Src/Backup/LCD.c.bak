// code from https://code.google.com/p/lcd-image-converter/wiki/ExamplesSources
#include "stm32f0xx_hal.h"
#include <string.h> 
#include "LCD.h"
#include "ste2007.h"

//Вывод изображения
extern uint8_t BACK_COLOR, MENU_COLOR;
extern uint16_t Max_X;
extern uint16_t Max_Y;


/*Режим отрисовки*/

void draw_bitmap_mono(uint16_t x, uint16_t y, const tImage *image, uint8_t color)
{
    uint8_t value = 0;
    uint16_t x0 = 0, y0 = 0;
    int counter = 0;
    const uint8_t *pdata = (const uint8_t *) image->data;
    // rows
    
    for (y0 = 0; y0 < image->height; y0++)
    {
       // columns
        for (x0 = 0; x0 < image->width; x0++)
        {
            // load new data
            if (counter == 0)
            {
                value = *pdata++;
                counter = image->dataSize;
            }
            counter--;

            // set pixel
            if ((value & 0x80) != 0)
              
              {
              set_pixel(x0, y0, color,image->width);
              }
            else
            {
              set_pixel(x0, y0, BACK_COLOR,image->width);
            }
            value = value << 1;
        }
    }
    send_v_buff(x,y,image->width);
}

void draw_bitmap_mono_inv(uint16_t x, uint16_t y, const tImage *image, uint8_t color)
{
    uint8_t value = 0;
    uint16_t x0 = 0, y0 = 0;
    int counter = 0;
    const uint8_t *pdata = (const uint8_t *) image->data;
    // rows
    
    for (y0 = 0; y0 < image->height; y0++)
    {
       // columns
        for (x0 = 0; x0 < image->width; x0++)
        {
            // load new data
            if (counter == 0)
            {
                value = *pdata++;
                counter = image->dataSize;
            }
            counter--;

            // set pixel
            if ((value & 0x80) != 0)
              
              {
              set_pixel(x0, y0,BACK_COLOR,image->width);
              }
            else
            {
              set_pixel(x0, y0, color,image->width);
            }
            value = value << 1;
        }
    }
    send_v_buff(x,y,image->width);
}

//  Вывод изображения (RLE)

void draw_bitmap_mono_rle(uint16_t x, uint16_t y, const tImage *image , uint16_t color)
{
    char value = 0;
    uint16_t x0 = 0, y0 = 0;
    int counter = 0;
    int8_t sequence = 0;
    int8_t nonsequence = 0;
    const uint8_t *pdata = (const uint8_t *) image->data;
    // rows
    for (y0 = 0; y0 < image->height; y0++)
    {
#ifdef FAST      
	set_area(x,(x+image->width),(y+y0),(y+y0));
#endif
        // columns
        for (x0 = 0; x0 < image->width; x0++)
        {
            // load new data
            if (counter == 0)
            {
                if ((sequence == 0) && (nonsequence == 0))
                {
                    sequence = *pdata++;
                    if (sequence < 0)
                    {
                        nonsequence = -sequence;
                        sequence = 0;
                    }
                }
                if (sequence > 0)
                {
                    value = *pdata;

                    sequence--;

                    if (sequence == 0)
                        pdata++;
                }
                if (nonsequence > 0)
                {
                    value = *pdata++;

                    nonsequence--;
                }
                counter = image->dataSize;
            }
            counter--;

            // set pixel
            if ((value & 0x80) != 0)
#ifndef FAST             
            {
              //Lcd_SPI_SetPixel(x + x0, y + y0, color);
            }
#endif
#ifdef FAST            
            {
              LCD_color(color);
            }
#endif            
            else 
#ifndef FAST              
            {
              //Lcd_SPI_SetPixel(x + x0, y + y0, BACK_COLOR);
            }
#endif
#ifdef FAST            
            {
              LCD_color(BACK_COLOR);
            }
#endif

            value = value << 1;
        }
    }
}

// Поиск символа в шрифте по коду

const tChar *find_char_by_code(int code, const tFont *font)
{
    int count = 0;
    int first = 0;
    int last = 0;
    int mid = 0;
    
    count = font->length;
    last = count - 1;

    if (count > 0)
    {
        if ((code >= font->chars[0].code) && (code <= font->chars[count - 1].code))
        {
            while (last >= first)
            {
                mid = first + ((last - first) / 2);

                if (font->chars[mid].code < code)
                    first = mid + 1;
                else
                    if (font->chars[mid].code > code)
                        last = mid - 1;
                    else
                        break;
            }

            if (font->chars[mid].code == code)
                return (&font->chars[mid]);
        }
    }

    return (0);
}

// Получение следующего кода символа в строке UTF-8

int utf8_next_char(const char *str, int start, int *resultCode, int *nextIndex)
{
    int len = 0;
    int index = 0;
    char c = 0;
    int code = 0;
    int result = 0;
    int skip = 0;
    
    *resultCode = 0;

    while (*(str + index) != 0)
    {
        len++;
        index++;
    }

   

    *resultCode = 0;
    *nextIndex = -1;

    if (start >= 0 && start < len)
    {
        index = start;

        while (index < len)
        {
            c = *(str + index);
            index++;

            // msb
            if (skip == 0)
            {
                // if range 0x00010000-0x001fffff
                if ((c & 0xf8) == 0xf0)
                {
                    skip = 0; // 3
                    code = c;
                }
                // if range 0x00000800-0x0000ffff
                else if ((c & 0xf0) == 0xe0)
                {
                    skip = 0; // 2
                    code = c;
                }
                // if range 0x00000080-0x000007ff
                else if ((c & 0xe0) == 0xc0)
                {
                    skip = 0; // 1
                    code = c;
                }
                // if range 0x00-0x7f
                else //if ((c & 0x80) == 0x00)
                {
                    skip = 0;
                    code = c;
                }
            }
            else // not msb
            {
                code = code << 8;
                code |= c;
                skip--;
            }
            if (skip == 0)
            {
                // completed
                *resultCode = code;
                *nextIndex = index;
                result = 1;
                break;
            }
        }
    }
    return (result);
}

// Вывод строки


void draw_string(const char *str, uint16_t x, uint16_t y, const tFont *font, uint8_t color)
{  
    char len = 0;
    int index = 0;
    int code = 0;
    //uint16_t x1 = 0, y1 = 0;
    int nextIndex = 0;
    len =  strlen(str);

    while (index < len)
    {
        if (1)
        {
        utf8_next_char(str, index, &code, &nextIndex);
            const tChar *ch = find_char_by_code(code, font);
            if (ch != 0)
            {
             
                draw_bitmap_mono(x, y, ch->image,color);
                x += ch->image->width;
            }
        }
        index = nextIndex;
        if (nextIndex < 0)
            break;
    }

}

void draw_string_inv(const char *str, uint16_t x, uint16_t y, const tFont *font, uint8_t color,uint8_t pos)
{  
    char len = 0;
    int index = 0;
    int code = 0;
    //uint16_t x1 = 0, y1 = 0;
    int nextIndex = 0;
    len =  strlen(str);
    uint8_t count=0;

    while (index < len)
    {
        if (1)
        {
        utf8_next_char(str, index, &code, &nextIndex);
            const tChar *ch = find_char_by_code(code, font);
            if (ch != 0)
            {
              if(count==pos || count==pos+1){
                draw_bitmap_mono_inv(x, y, ch->image,color);
              }
              else{
                draw_bitmap_mono(x, y, ch->image,color);
              }
                x += ch->image->width;
                count++;
            }
        }
        index = nextIndex;
        if (nextIndex < 0)
            break;
    }

}