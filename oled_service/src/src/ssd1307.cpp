#include "ssd1307.h"

ssd1307::ssd1307(const char* device):buffer(new uint8_t[SSD1307_BUFFER_SIZE]), font(new char[5000*sizeof(char)]), oled_device(device)
{
    /*Open file and get font's character bitmap value*/
    gzFile fontfd;
    fontfd = gzopen("/lib/banvien/Lat2-VGA8.psf.gz", "rb");
    gzread(fontfd, font, 4);
    this->charsize = font[3];
    gzread(fontfd, font, charsize * 256);
    gzclose(fontfd);
}

ssd1307::~ssd1307() 
{
    this->fbfd = 0;
    delete(this->buffer);
    delete(this->font);
}

void ssd1307::ssd1307_open()
{
    this->fbfd = open(this->oled_device, O_RDWR);

    if (this->fbfd == -1) {
        ERROR("Error opening frambuffer, fb: %d !", this->fbfd);
        ssd1307_close();
        return;
    }

    INFO("Open OLED successfully, fd: %d .", this->fbfd);
}

void ssd1307::ssd1307_write(uint8_t* buf, uint32_t len, uint32_t offset)
{
    ssd1307_open();

    uint8_t buf_reverse[SSD1307_BUFFER_SIZE];
    memcpy(buf_reverse, buf, len);
    reverse_byte_in_bitmap(buf_reverse, len);

    ssize_t num_of_byte = pwrite(this->fbfd, buf_reverse, len, offset);
    
    if (num_of_byte == -1) {
        ERROR("Write to %s failed", "OLED");
        ssd1307_close();
        return;
    }
    
    INFO("Write into OLED with %ld byte success. ", num_of_byte);
    
    ssd1307_close();
}

void ssd1307::ssd1307_read(char* buf, uint32_t len, uint32_t offset)
{
    ssd1307_open();
    ssize_t num_of_byte = pread(this->fbfd, buf, len, offset);
    if (num_of_byte == -1) {
        ERROR("Read to %s failed", "OLED");
        return;
    }
    ssd1307_close();
}

void ssd1307::ssd1307_close() 
{
    if (close(this->fbfd) == -1) {
        ERROR("Close OLED %d failed ! ", this->fbfd);
        return;
    }

    INFO("Close OLED %d success. ", this->fbfd);
}

void ssd1307::ssd1307_fill(SSD1307_COLOR color) 
{
    for (int i=0; i<SSD1307_BUFFER_SIZE; i++) {
        this->buffer[i] = (color == WHITE)?0xFF:0x00;
    }
}

void ssd1307::ssd1307_refresh() {
    ssd1307_write(&this->buffer[0], SSD1307_BUFFER_SIZE, 0);
}

void ssd1307::ssd1307_draw_pixel(uint8_t x, uint8_t y, SSD1307_COLOR color) 
{
    if ((x >= SSD1307_WIDTH) || (y >= SSD1307_HEIGHT)) {
        return;
    }
    
    if (color == WHITE) {
        this->buffer[(x + y * SSD1307_WIDTH)/8] |= 0x80 >> (x % 8);

        ssd1307_write(&this->buffer[(x + y * SSD1307_WIDTH)/8], 1, (x + y * SSD1307_WIDTH)/8);
    } 
    else { 
        this->buffer[(x + y * SSD1307_WIDTH)/8] &= ~(0x80 >> (x % 8));
        ssd1307_write(&this->buffer[(x + y * SSD1307_WIDTH)/8], 1, (x + y * SSD1307_WIDTH)/8);
    }
}

void ssd1307::ssd1307_update_buffer_pixel(uint8_t x, uint8_t y, SSD1307_COLOR color)
{
    if ((x >= SSD1307_WIDTH) || (y >= SSD1307_HEIGHT)) {
        return;
    }
    
    if (color == WHITE) {
        this->buffer[(x + y * SSD1307_WIDTH)/8] |= 0x80 >> (x % 8);
    } 
    else { 
        this->buffer[(x + y * SSD1307_WIDTH)/8] &= ~(0x80 >> (x % 8));
    }
}

void ssd1307::ssd1307_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1307_COLOR color) 
{
    int32_t deltaX = abs(x2 - x1);
    int32_t deltaY = abs(y2 - y1);
    int32_t signX = ((x1 < x2) ? 1 : -1);
    int32_t signY = ((y1 < y2) ? 1 : -1);
    int32_t error = deltaX - deltaY;
    int32_t error2;

    ssd1307_update_buffer_pixel(x2, y2, color);
    while ((x1 != x2) || (y1 != y2))
    {
        ssd1307_update_buffer_pixel(x1, y1, color);
        error2 = error * 2;

        if (error2 > -deltaY) {
            error -= deltaY;
            x1 += signX;
        }

        if (error2 < deltaX) {
            error += deltaX;
            y1 += signY;
        }
    }

    return;
}

void ssd1307::ssd1307_draw_circle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1307_COLOR color) 
{
    int32_t x = -par_r;
    int32_t y = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= SSD1307_WIDTH || par_y >= SSD1307_HEIGHT) {
        return;
    }

    do {
        ssd1307_update_buffer_pixel(par_x - x, par_y + y, color);
        ssd1307_update_buffer_pixel(par_x + x, par_y + y, color);
        ssd1307_update_buffer_pixel(par_x + x, par_y - y, color);
        ssd1307_update_buffer_pixel(par_x - x, par_y - y, color);
        e2 = err;

        if (e2 <= y) {
            y++;
            err = err + (y * 2 + 1);

            if (-x == y && e2 <= x) {
                e2 = 0;
            }
        }

        if (e2 > x) {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);

    return;
}

void ssd1307::ssd1307_fill_circle(uint8_t par_x,uint8_t par_y,uint8_t par_r,SSD1307_COLOR par_color)
{
    int32_t x = -par_r;
    int32_t y = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= SSD1307_WIDTH || par_y >= SSD1307_HEIGHT) {
        return;
    }

    do {
        for (uint8_t _y = (par_y + y); _y >= (par_y - y); _y--) {
            for (uint8_t _x = (par_x - x); _x >= (par_x + x); _x--) {
                ssd1307_update_buffer_pixel(_x, _y, par_color);
            }
        }

        e2 = err;
        if (e2 <= y) {
            y++;
            err = err + (y * 2 + 1);
            if (-x == y && e2 <= x) {
                e2 = 0;
            }
        }

        if (e2 > x) {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);

    return;
}

uint16_t ssd1307::ssd1307_normalize_to_0_360(uint16_t par_deg)
{
    uint16_t loc_angle;
    if (par_deg <= 360) {
        loc_angle = par_deg;
    } else {
        loc_angle = par_deg % 360;
        loc_angle = ((par_deg != 0)?par_deg:360);
    }
    return loc_angle;
}

float ssd1307::ssd1307_deg_to_rad(float par_deg)
{
    return par_deg * 3.14 / 180.0;
}

void ssd1307::ssd1307_draw_arc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1307_COLOR color)
{
    static const uint8_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1,xp2;
    uint8_t yp1,yp2;
    uint32_t count = 0;
    uint32_t loc_sweep = 0;
    float rad;
    
    loc_sweep = ssd1307_normalize_to_0_360(sweep);
    
    count = (ssd1307_normalize_to_0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;
    while (count < approx_segments)
    {
        rad = ssd1307_deg_to_rad(count*approx_degree);
        xp1 = x + (int8_t)(sin(rad)*radius);
        yp1 = y + (int8_t)(cos(rad)*radius);    
        count++;
        if(count != approx_segments) {
            rad = ssd1307_deg_to_rad(count*approx_degree);
        } else {
            rad = ssd1307_deg_to_rad(loc_sweep);
        }
        xp2 = x + (int8_t)(sin(rad)*radius);
        yp2 = y + (int8_t)(cos(rad)*radius);    
        ssd1307_draw_line(xp1,yp1,xp2,yp2,color);
    }
    
    return;
}

void ssd1307::ssd1307_write_char(char ch, int row, int line, SSD1307_COLOR color)
{
    int get, put;

    for (int i = 0; i < this->charsize; i++) {
        get = (ch*this->charsize) + i;
        put = (i*16) + (line*this->charsize /* *8 */ * 16) + row;
        if(put < (16*64)) this->buffer[put] = (color==WHITE)?font[get] >> 1:~(font[get] >> 1);
    }

    return;
}

void ssd1307::ssd1307_write_string(unsigned char *str, int row, int line, SSD1307_COLOR color) 
{
    char buf;

    /*Convert given string to font value and insert it to oled buffer*/
    while (*str) 
    {
        buf = *str++;

        ssd1307_write_char(buf, row, line, color);

        row++;

        if (row == 15) 
        {
            line++;
            row = 0;
        }
    }

    return;
}

void ssd1307::ssd1307_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1307_COLOR color)
{
    ssd1307_draw_line(x1,y1,x2,y1,color);
    ssd1307_draw_line(x2,y1,x2,y2,color);
    ssd1307_draw_line(x2,y2,x1,y2,color);
    ssd1307_draw_line(x1,y2,x1,y1,color);

    return;
}

void ssd1307::ssd1307_fill_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1307_COLOR color)
{
    uint8_t x_start = ((x1<=x2) ? x1 : x2);
    uint8_t x_end   = ((x1<=x2) ? x2 : x1);
    uint8_t y_start = ((y1<=y2) ? y1 : y2);
    uint8_t y_end   = ((y1<=y2) ? y2 : y1);

    for (uint8_t y= y_start; (y<= y_end)&&(y<SSD1307_HEIGHT); y++) {
        for (uint8_t x= x_start; (x<= x_end)&&(x<SSD1307_WIDTH); x++) {
            this->buffer[(x + y * SSD1307_WIDTH)/8] = (color == WHITE)?0xFF:0x00;
        }
    }

    return;
}

void ssd1307::ssd1307_draw_bitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1307_COLOR color)
{
    int16_t byteWidth = (w + 7) / 8;
    uint8_t byte = 0;

    if (x >= SSD1307_WIDTH || y >= SSD1307_HEIGHT) {
        return;
    }

    for (uint8_t j = 0; j < h; j++, y++) {
        for (uint8_t i = 0; i < w; i++) {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }

            if (byte & 0x80) {
                ssd1307_update_buffer_pixel(x + i, y, color);
            }
        }
    }

    return;
}

void ssd1307::reverse_byte_in_bitmap(uint8_t *buffer, uint32_t len)
{
    int get, put; 
    for (int i = 0; i < len; i++) {
        get = buffer[i];
        put = 0;

        for(int j = 0; j < 8; j++) 
        {
            if ((get & (1 << j)) > 0) put |= (0x80 >> j);
        }
        buffer[i] = put;
    }
}