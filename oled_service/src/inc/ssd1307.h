#ifndef SSD1307_IF_H
#define SSD1307_SERVICE_IF_H

#include <vector>
#include <cmath>
#include <common.h>
extern "C"
{
    #include <pthread.h>
    #include <time.h>
    #include <zlib.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <stdint.h>
    #include <stdio.h>
    #include <string.h>
}

/*1 pixel = 1 bit*/

/*Offset define*/
#ifdef SSD1307_X_OFFSET
#define SSD1307_X_OFFSET_LOWER (SSD1307_X_OFFSET & 0x0F)
#define SSD1307_X_OFFSET_UPPER ((SSD1307_X_OFFSET >> 4) & 0x07)
#else
#define SSD1307_X_OFFSET_LOWER 0
#define SSD1307_X_OFFSET_UPPER 0
#endif

/* SSD1307 OLED height in pixels  */
#ifndef SSD1307_HEIGHT
#define SSD1307_HEIGHT          32
#endif

/* SSD1307 width in pixels	  */
#ifndef SSD1307_WIDTH
#define SSD1307_WIDTH           128
#endif

#ifndef SSD1307_BUFFER_SIZE
#define SSD1307_BUFFER_SIZE   SSD1307_WIDTH * SSD1307_HEIGHT / 8
#endif

typedef enum 
{
    BLACK = 0x00,
    WHITE = 0x01,
} SSD1307_COLOR;

class ssd1307 {
private:
    const char *oled_device;
    int fbfd;
    char *font;
    int charsize;

    void ssd1307_open();
    void ssd1307_write(uint8_t* buf, uint32_t len, uint32_t offset);
    void ssd1307_read(char* buf, uint32_t len, uint32_t offset);
    void ssd1307_close();

public:
    uint8_t *buffer;

    ssd1307(const char* device);
    ~ssd1307();

    void ssd1307_fill(SSD1307_COLOR color);
    void ssd1307_refresh();
    void ssd1307_draw_pixel(uint8_t x, uint8_t y, SSD1307_COLOR color);
    void ssd1307_update_buffer_pixel(uint8_t x, uint8_t y, SSD1307_COLOR color);
    void ssd1307_write_string(unsigned char *str, int row, int line, SSD1307_COLOR color);
    void ssd1307_write_char(char str, int row, int line, SSD1307_COLOR color);
    void ssd1307_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1307_COLOR color);
    void ssd1307_draw_circle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1307_COLOR color);
    void ssd1307_fill_circle(uint8_t par_x,uint8_t par_y,uint8_t par_r,SSD1307_COLOR par_color);
    void ssd1307_draw_arc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1307_COLOR color);
    void ssd1307_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1307_COLOR color);
    void ssd1307_fill_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1307_COLOR color);
    void ssd1307_draw_bitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1307_COLOR color);
    void reverse_byte_in_bitmap(uint8_t *buffer, uint32_t len);
    uint16_t ssd1307_normalize_to_0_360(uint16_t par_deg);
    float ssd1307_deg_to_rad(float par_deg);
};

#endif