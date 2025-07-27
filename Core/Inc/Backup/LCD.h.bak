
/*------Определения структур-------------*/


typedef struct
{
    //const uint16_t *data;
    const  char *data;
    int width;
    int height;
    char  dataSize;
} tImage;

typedef struct
{
    long int code;
    const tImage *image;
} tChar;

typedef struct
{
    int length;
    const tChar *chars;
} tFont;

#define ADDR_LINE_LCD(a)     (2*a)

void draw_bitmap_mono(uint16_t x, uint16_t y, const tImage *image, uint8_t color);
void draw_bitmap_mono_inv(uint16_t x, uint16_t y, const tImage *image, uint8_t color);
void draw_bitmap_mono_rle(uint16_t x, uint16_t y, const tImage *image , uint16_t color);
const tChar *find_char_by_code(int code, const tFont *font);
int utf8_next_char(const char *str, int start, int *resultCode, int *nextIndex);
void draw_string(const char *str, uint16_t x, uint16_t y, const tFont *fonts, uint8_t color);
void draw_string_inv(const char *str, uint16_t x, uint16_t y, const tFont *font, uint8_t color,uint8_t pos);
