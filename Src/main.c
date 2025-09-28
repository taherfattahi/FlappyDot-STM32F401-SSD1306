#include <stdint.h>
/* ============================================================
   SSD1306 Flappy Dot (STM32F401CCU6, bare-metal, I2C bit-bang)
   Pins:
     - OLED I2C: PA9=SCL, PA10=SDA  (addr 0x3C)
     - Button:   PA0 = input with pull-up (to GND)
   ============================================================ */

/* ------------------ Basic chip addresses ------------------ */
#define PERIPH_BASE        0x40000000UL
#define AHB1PERIPH_BASE    0x40020000UL
#define APB2PERIPH_BASE    0x40010000UL

#define GPIOA_BASE         (AHB1PERIPH_BASE + 0x0000)
#define RCC_BASE           0x40023800UL

/* ------------------ GPIOA registers ------------------ */
#define GPIOA_MODER        (*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER       (*(volatile uint32_t*)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR      (*(volatile uint32_t*)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR        (*(volatile uint32_t*)(GPIOA_BASE + 0x0C))
#define GPIOA_IDR          (*(volatile uint32_t*)(GPIOA_BASE + 0x10))
#define GPIOA_ODR          (*(volatile uint32_t*)(GPIOA_BASE + 0x14))
#define GPIOA_BSRR         (*(volatile uint32_t*)(GPIOA_BASE + 0x18))

/* ------------------ RCC registers ------------------ */
#define RCC_AHB1ENR        (*(volatile uint32_t*)(RCC_BASE + 0x30))

/* ------------------ SysTick registers ------------------ */
#define SYST_CSR           (*(volatile uint32_t*)0xE000E010UL)
#define SYST_RVR           (*(volatile uint32_t*)0xE000E014UL)
#define SYST_CVR           (*(volatile uint32_t*)0xE000E018UL)
/* SysTick bits */
#define SYST_CSR_CLKSOURCE (1U<<2)
#define SYST_CSR_TICKINT   (1U<<1)
#define SYST_CSR_ENABLE    (1U<<0)

/* ------------------ Handy defines ------------------ */
#define F_CPU              16000000UL

/* I2C pins (bit-banged) */
#define I2C_SCL_PIN        9   /* PA9  */
#define I2C_SDA_PIN        10  /* PA10 */

/* Button on PA0, active low to GND */
#define BTN_PIN            0

/* GPIO helpers */
static inline void pin_mode_input_pullup(uint32_t pin)
{
    GPIOA_MODER &= ~(3U<<(pin*2));            /* 00 = input */
    GPIOA_PUPDR &= ~(3U<<(pin*2));
    GPIOA_PUPDR |=  (1U<<(pin*2));            /* 01 = pull-up */
}

static inline void pin_mode_output_od(uint32_t pin) /* open-drain */
{
    GPIOA_MODER  &= ~(3U<<(pin*2));
    GPIOA_MODER  |=  (1U<<(pin*2));           /* 01 = output */
    GPIOA_OTYPER |=  (1U<<pin);               /* open-drain */
    GPIOA_OSPEEDR|=  (2U<<(pin*2));           /* high speed */
}

static inline void pin_mode_output_pp(uint32_t pin) /* push-pull */
{
    GPIOA_MODER  &= ~(3U<<(pin*2));
    GPIOA_MODER  |=  (1U<<(pin*2));
    GPIOA_OTYPER &= ~(1U<<pin);
    GPIOA_OSPEEDR|=  (2U<<(pin*2));
}

/* Drive OD line low (write 0) or release (write 1) */
static inline void gpio_write_od(uint32_t pin, int level)
{
    if (level) {
        GPIOA_BSRR = (1U<<pin); /* set bit = 1 (released) */
    } else {
        GPIOA_BSRR = (1U<<(pin+16)); /* reset bit = 0 (pull low) */
    }
}
static inline int gpio_read(uint32_t pin)
{
    return (GPIOA_IDR >> pin) & 1U;
}

/* --------------- Very small busy-waits --------------- */
static inline void delay_cycles(volatile uint32_t n) { while (n--) { __asm__ volatile ("nop"); } }

/* I2C bit-bang timing: ~100kHz on 16MHz CPU with tiny waits */
static inline void i2c_half_delay(void) { delay_cycles(1); }

/* --------------- Bit-banged I2C (PA9=SCL, PA10=SDA) --------------- */
static void i2c_init_pins(void)
{
    pin_mode_output_od(I2C_SCL_PIN);
    pin_mode_output_od(I2C_SDA_PIN);
    /* release both lines (high via pull-ups) */
    gpio_write_od(I2C_SCL_PIN, 1);
    gpio_write_od(I2C_SDA_PIN, 1);
}

static void i2c_start(void)
{
    gpio_write_od(I2C_SDA_PIN, 1);
    gpio_write_od(I2C_SCL_PIN, 1);
    i2c_half_delay();
    gpio_write_od(I2C_SDA_PIN, 0);  /* SDA falling while SCL high = START */
    i2c_half_delay();
    gpio_write_od(I2C_SCL_PIN, 0);
}

static void i2c_stop(void)
{
    gpio_write_od(I2C_SDA_PIN, 0);
    i2c_half_delay();
    gpio_write_od(I2C_SCL_PIN, 1);
    i2c_half_delay();
    gpio_write_od(I2C_SDA_PIN, 1);  /* SDA rising while SCL high = STOP */
    i2c_half_delay();
}

static int i2c_write_byte(uint8_t b)
{
    for (int i=7; i>=0; --i) {
        gpio_write_od(I2C_SDA_PIN, (b>>i)&1);
        i2c_half_delay();
        gpio_write_od(I2C_SCL_PIN, 1);
        i2c_half_delay();
        gpio_write_od(I2C_SCL_PIN, 0);
    }
    /* ACK bit */
    gpio_write_od(I2C_SDA_PIN, 1); /* release */
    i2c_half_delay();
    gpio_write_od(I2C_SCL_PIN, 1);
    int ack = (gpio_read(I2C_SDA_PIN) == 0); /* 0 = ACK */
    i2c_half_delay();
    gpio_write_od(I2C_SCL_PIN, 0);
    return ack;
}

/* --------------- SSD1306 128x64 (I2C) --------------- */
#define SSD1306_I2C_ADDR  0x3C
static void ssd1306_cmd(uint8_t c)
{
    i2c_start();
    i2c_write_byte((SSD1306_I2C_ADDR<<1) | 0); /* write */
    i2c_write_byte(0x00); /* control: Co=0, D/C#=0 (command) */
    i2c_write_byte(c);
    i2c_stop();
}
static void ssd1306_cmd2(uint8_t c1, uint8_t c2)
{
    i2c_start();
    i2c_write_byte((SSD1306_I2C_ADDR<<1) | 0);
    i2c_write_byte(0x00);
    i2c_write_byte(c1);
    i2c_write_byte(c2);
    i2c_stop();
}

static void ssd1306_init(void)
{
    /* Typical init (internal charge pump, horizontal addressing) */
    ssd1306_cmd(0xAE);           /* display off */
    ssd1306_cmd2(0xD5, 0x80);    /* clock div */
    ssd1306_cmd2(0xA8, 0x3F);    /* multiplex 0x3F for 64 */
    ssd1306_cmd2(0xD3, 0x00);    /* display offset */
    ssd1306_cmd(0x40 | 0x00);    /* start line = 0 */
    ssd1306_cmd2(0x8D, 0x14);    /* charge pump on */
    ssd1306_cmd2(0x20, 0x00);    /* memory mode: horizontal */
    ssd1306_cmd(0xA1);           /* segment remap */
    ssd1306_cmd(0xC8);           /* COM scan dec */
    ssd1306_cmd2(0xDA, 0x12);    /* COM pins */
    ssd1306_cmd2(0x81, 0x7F);    /* contrast */
    ssd1306_cmd(0xA4);           /* display from RAM */
    ssd1306_cmd(0xA6);           /* normal display (not inverted) */
    ssd1306_cmd2(0xD9, 0xF1);    /* pre-charge */
    ssd1306_cmd2(0xDB, 0x40);    /* VCOM detect */
    ssd1306_cmd(0x2E);           /* deactivate scroll */
    ssd1306_cmd(0xAF);           /* display on */
}

/* Framebuffer: 128x64 / 8 = 1024 bytes */
static uint8_t fb[128*64/8];

static void fb_clear(void)
{
    for (uint32_t i=0;i<sizeof(fb);++i) fb[i]=0;
}
static void fb_set_pixel(int x,int y, int on)
{
    if (x<0||x>=128||y<0||y>=64) return;
    uint32_t idx = x + (y/8)*128;
    uint8_t  mask = 1U<<(y&7);
    if (on) fb[idx] |= mask; else fb[idx] &= ~mask;
}

static void fb_fill_rect(int x,int y,int w,int h,int on)
{
    for (int yy=y; yy<y+h; ++yy)
        for (int xx=x; xx<x+w; ++xx)
            fb_set_pixel(xx,yy,on);
}

/* Simple 5x7 font for score (ASCII 32..127, only digits & colon used to keep it short) */
static const uint8_t font5x7_digits[][5] = {
    /* '0'..'9' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
};
static void fb_draw_digit(int x,int y,int d)
{
    if (d<0||d>9) return;
    for (int col=0; col<5; ++col) {
        uint8_t bits = font5x7_digits[d][col];
        for (int row=0; row<7; ++row) {
            fb_set_pixel(x+col, y+row, (bits>>row)&1);
        }
    }
}
static void fb_draw_number(int x,int y,int n)
{
    int digits[6]; int len=0;
    if (n==0) { fb_draw_digit(x,y,0); return; }
    while (n>0 && len<6) { digits[len++]=n%10; n/=10; }
    for (int i=0;i<len;++i) fb_draw_digit(x + (len-1-i)*6, y, digits[len-1-i]);
}

/* Push whole framebuffer to OLED using page writes */
static void ssd1306_flush_fb(void)
{
    for (uint8_t page=0; page<8; ++page) {
        ssd1306_cmd(0xB0 | page);         /* set page */
        ssd1306_cmd(0x00);                /* low col = 0 */
        ssd1306_cmd(0x10);                /* high col = 0 */

        i2c_start();
        i2c_write_byte((SSD1306_I2C_ADDR<<1) | 0);
        i2c_write_byte(0x40);             /* control: Co=0, D/C#=1 (data) */
        /* Write 128 bytes for this page */
        for (int x=0;x<128;++x) {
            i2c_write_byte(fb[page*128 + x]);
        }
        i2c_stop();
    }
}

/* ------------------ Button & timing ------------------ */
static inline int button_pressed(void) /* active-low */
{
    return ((GPIOA_IDR & (1U<<BTN_PIN)) == 0);
}

/* SysTick ~1ms ticks */
static volatile uint32_t ms_ticks = 0;
void SysTick_Handler(void) { ms_ticks++; }
static void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms) { /* spin */ }
}

/* ------------------ Tiny PRNG for gaps ------------------ */
static uint32_t rng_state = 1;
static uint32_t xorshift32(void)
{
    uint32_t x = rng_state;
    x ^= x<<13; x ^= x>>17; x ^= x<<5;
    rng_state = x ? x : 1;
    return x;
}

/* ------------------ Game logic ------------------ */
typedef struct {
    int x;         /* pipe x position (left edge) */
    int gap_y;     /* top of gap */
    int gap_h;     /* gap height */
} Pipe;

static void pipe_reset(Pipe* p)
{
    p->x = 128;                        /* start off the right edge */
    p->gap_h = 20;                     /* gap size */
    int min_top = 8;
    int max_top = 64 - p->gap_h - 8;
    p->gap_y = min_top + (int)(xorshift32() % (uint32_t)(max_top - min_top + 1));
}

static int pipe_collide(const Pipe* p, int dot_x, int dot_y)
{
    /* Pipe occupies all columns except the gap */
    if (dot_x >= p->x && dot_x < p->x + 10) { /* pipe width 10 px */
        if (dot_y < p->gap_y || dot_y >= p->gap_y + p->gap_h) return 1;
    }
    return 0;
}

/* ------------------ Draw scene ------------------ */
static void draw_pipe(const Pipe* p)
{
    /* top pipe: from y=0 to gap_y-1; bottom: from gap_y+gap_h to 63 */
    if (p->x >= 128 || p->x + 10 <= 0) return;
    /* top */
    fb_fill_rect(p->x, 0, 10, p->gap_y, 1);
    /* bottom */
    int bh = 64 - (p->gap_y + p->gap_h);
    fb_fill_rect(p->x, p->gap_y + p->gap_h, 10, bh, 1);
}

static void draw_dot(int x,int y)
{
    fb_fill_rect(x-1, y-1, 3, 3, 1); /* 3x3 dot */
}

/* ------------------ Main ------------------ */
int main(void)
{
    /* Enable GPIOA clock */
    RCC_AHB1ENR |= (1U<<0);

    /* Configure I2C lines (open-drain outputs) and button */
    i2c_init_pins();
    pin_mode_input_pullup(BTN_PIN);

    /* SysTick 1ms: reload = F_CPU/1000 - 1 */
    SYST_RVR = (F_CPU/1000) - 1;
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;

    /* Init display */
    ssd1306_init();
    fb_clear();
    ssd1306_flush_fb();

    /* Game state */
    int dot_x = 20;
    int dot_y = 32;
    int vel_y = 0;
    const int gravity = 1;     /* per frame */
    const int flap_impulse = -6;
    int score = 0;
    Pipe pipe; pipe_reset(&pipe);

    /* Debounce/bookkeeping */
    int btn_last = 0;

    /* Simple title screen */
    fb_clear();
    fb_draw_number(55, 20, 0);
    fb_fill_rect(20-1, 32-1, 3, 3, 1);
    ssd1306_flush_fb();
    delay_ms(600);

    /* Main loop @ ~30 FPS: 33ms/frame */
    uint32_t next = ms_ticks + 33;
    while (1) {
        /* --- Input (one button) --- */
        int btn = button_pressed();
        if (btn && !btn_last) {
            vel_y = flap_impulse;
        }
        btn_last = btn;

        /* --- Physics --- */
        vel_y += gravity;
        if (vel_y > 4) vel_y = 4;
        if (vel_y < -6) vel_y = -6;
        dot_y += vel_y;

        if (dot_y < 1)  { dot_y = 1;  vel_y = 0; }
        if (dot_y > 62) { dot_y = 62; vel_y = 0; }

        /* --- Move pipe --- */
        pipe.x -= 2;
        if (pipe.x + 10 < 0) {
            /* passed a pipe -> score++ and respawn */
            score++;
            pipe_reset(&pipe);
        }

        /* --- Collision --- */
        if (pipe_collide(&pipe, dot_x, dot_y)) {
            /* flash and reset */
            for (int k=0;k<3;++k) {
                fb_clear(); ssd1306_flush_fb(); delay_ms(120);
                draw_pipe(&pipe); draw_dot(dot_x,dot_y); ssd1306_flush_fb(); delay_ms(120);
            }
            /* reset everything */
            score = 0;
            dot_x = 20; dot_y = 32; vel_y = 0;
            pipe_reset(&pipe);
        }

        /* --- Render --- */
        fb_clear();
        draw_pipe(&pipe);
        draw_dot(dot_x, dot_y);
        fb_draw_number(0,0,score);
        ssd1306_flush_fb();

        /* --- Frame pacing (~33 ms) --- */
        uint32_t now = ms_ticks;
        if ((int32_t)(next - now) > 0) delay_ms(next - now);
        next += 33;
    }
}
