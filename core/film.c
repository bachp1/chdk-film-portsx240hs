/* DR+ aplicado */
#include "platform.h"
#include "camera_info.h"
#include "conf.h"
#include "raw.h"
#include "properties.h"
#include "gui.h"
#include "gui_draw.h"
#include "keyboard.h"
#include "gui_batt.h"
#include "sd_card.h"
#include "debug_led.h"
#include "film.h"
#ifndef FILM_HOST_PREVIEW
#include "gui_osd.h"
#endif

// Speed matters more than size here: the raw loop runs over millions of pixels
#pragma GCC optimize("O3")
#pragma GCC target("arm")

#ifndef FILM_PROFILE
#define FILM_PROFILE 0      // 1 = TEST BUILD: only unpack and repack, no effect
#endif

/*
    Film simulation

    Runs in the raw hook after any RAW/DNG has been written. Each 2x2 Bayer
    quad is white balanced with a grey-world estimate, then:

        vignette -> saturation -> grain -> per-channel tone curve

    and un-balanced again so the camera's own JPEG pipeline develops the result.
    Tone curves are built in a perceptual (gamma 2) space so contrast, lift and
    tints behave like they would on a print. All maths is integer, Q12 (4096 = 1.0).
*/

#define Q(x)        ((short)((x)*4096))
#define ONE         4096
#define LUT_SIZE    4096        // balanced linear input range 0..4095
#define PIVOT       1740        // perceptual mid grey (~18% linear)

// My Colors propcase values (propset 2)
#define MC_OFF      0
#define MC_VIVID    1
#define MC_BW       3
#define MC_POSITIVE 5

static void film_cc_suspend(void);

const film_stock_t film_stocks[] =
{
    //  name               tag        description                                 my_colors    sat       contrast  lift      white      shadow tint R,G,B                highlight tint R,G,B              grain size vignette
    { "Off",              "",        "No film look, camera colour only",         -1,          Q(1.00),  Q(0.00),  Q(0.000), Q(1.000), { 0, 0, 0 },                    { 0, 0, 0 },                     0, 0, 0 },
    { "Portra 400", "PORTRA", "Soft contrast, warm skin, lifted blacks", MC_OFF, Q(0.900), Q(0.080), Q(0.042), Q(0.963), { Q(0.010), Q(0.004), Q(-0.004) }, { Q(0.016), Q(0.006), Q(-0.018) }, 3, 0, 0 },
    { "Ektar 100", "EKTAR", "Punchy colour, fine grain", MC_VIVID, Q(1.150), Q(0.180), Q(0.012), Q(0.988), { Q(-0.004), Q(0.000), Q(0.010) }, { Q(0.010), Q(0.004), Q(-0.008) }, 1, 0, 0 },
    { "Gold 200", "GOLD", "Warm golden highlights", MC_OFF, Q(1.050), Q(0.120), Q(0.032), Q(0.973), { Q(0.012), Q(0.006), Q(-0.006) }, { Q(0.026), Q(0.014), Q(-0.030) }, 3, 0, 0 },
    { "Superia 400", "SUPERIA", "Green shadows, cool everyday colour", MC_OFF, Q(1.000), Q(0.120), Q(0.037), Q(0.968), { Q(-0.010), Q(0.012), Q(0.006) }, { Q(0.000), Q(0.004), Q(0.006) }, 4, 0, 0 },
    { "Velvia 50", "VELVIA", "Vivid slide film, deep contrast", MC_POSITIVE, Q(1.300), Q(0.280), Q(0.012), Q(0.988), { Q(-0.004), Q(-0.006), Q(0.012) }, { Q(0.006), Q(0.000), Q(-0.006) }, 1, 0, Q(0.10) },
    { "Ektachrome E100", "E100", "Clean slide film, cool blues", MC_OFF, Q(1.120), Q(0.200), Q(0.012), Q(0.988), { Q(-0.006), Q(0.000), Q(0.016) }, { Q(-0.004), Q(0.002), Q(0.012) }, 1, 0, 0 },
    { "Cinestill 800T", "800T", "Tungsten night film, teal shadows", MC_OFF, Q(1.000), Q(0.140), Q(0.032), Q(0.973), { Q(-0.012), Q(0.010), Q(0.030) }, { Q(0.012), Q(0.000), Q(-0.004) }, 6, 1, 0 },
    { "Tri-X 400",        "TRI-X",   "Contrasty black & white, classic grain",   MC_BW,       Q(0.00),  Q(0.25),  Q(0.010), Q(1.000), { 0, 0, 0 },                    { 0, 0, 0 },                     6, 1, 0 },
    { "HP5 Plus",         "HP5",     "Soft black & white, coarse grain",         MC_BW,       Q(0.00),  Q(0.12),  Q(0.030), Q(0.980), { 0, 0, 0 },                    { 0, 0, 0 },                     7, 2, 0 },
    { "T-Max 100",        "T-MAX",   "Smooth, fine black & white",               MC_BW,       Q(0.00),  Q(0.15),  Q(0.000), Q(1.000), { 0, 0, 0 },                    { 0, 0, 0 },                     2, 0, 0 },
    { "Expired Lomo", "LOMO", "Faded, cross-processed, dark corners", MC_OFF, Q(1.200), Q(0.250), Q(0.060), Q(0.938), { Q(-0.016), Q(0.010), Q(0.020) }, { Q(0.024), Q(0.016), Q(-0.024) }, 7, 2, Q(0.45) },
    { "Kodachrome 64", "K64", "Warm, rich reds, strong contrast", MC_OFF, Q(1.150), Q(0.340), Q(0.017), Q(0.978), { Q(-0.012), Q(0.006), Q(0.026) }, { Q(0.044), Q(0.014), Q(-0.044) }, 2, 0, Q(0.06) },
    { "Fuji Pro 400H", "PRO400H", "Soft pastel, airy cyan-green tint", MC_OFF, Q(0.920), Q(0.260), Q(0.047), Q(0.958), { Q(-0.016), Q(0.018), Q(0.024) }, { Q(0.022), Q(0.020), Q(-0.014) }, 3, 0, 0 },
    { "Polaroid", "POLA", "Faded instant film, blue shadows", MC_OFF, Q(0.950), Q(0.240), Q(0.060), Q(0.928), { Q(0.008), Q(0.010), Q(0.022) }, { Q(0.020), Q(0.012), Q(-0.016) }, 4, 1, Q(0.25) },
    { "Portra 800 v3",    "P800V3",  "Warm chrome, soft highlights, big grain",  MC_OFF,      Q(0.95),  Q(0.16),  Q(0.015), Q(0.965), { Q(0.006),  Q(0.002), Q(0.010) },  { Q(0.024),  Q(0.010), Q(-0.026) }, 6, 2, 0 },
    { "Last Summer Roll", "SUMMER",  "Warm summer negative, green-tinged shadows", MC_OFF,    Q(1.10),  Q(0.20),  Q(0.030), Q(0.975), { Q(-0.004), Q(0.010), Q(0.004) },  { Q(0.028),  Q(0.014), Q(-0.024) }, 4, 0, 0 },
    { "Classic Neg DR400","CNEG400", "Rich colour negative, green shadows",      MC_OFF,      Q(1.22),  Q(0.18),  Q(0.020), Q(0.970), { Q(-0.010), Q(0.016), Q(0.004) },  { Q(0.006),  Q(0.008), Q(-0.004) }, 3, 0, 0 },
};

const int film_stock_count = sizeof(film_stocks)/sizeof(film_stocks[0]);

static const char *size_names[]  = { "Auto", "Fine", "Medium", "Coarse" };

//-------------------------------------------------------------------
// Settings helpers

#ifndef FILM_STRENGTH
#define FILM_STRENGTH   7373    // Q12 multiplier on contrast and tints of the original stocks (4096 = as designed)
#endif
#define FILM_SCALED_UPTO 12     // stocks with index below this (the originals) are scaled; newer ones are used as written

static int sclamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static const film_stock_t* current_stock()
{
    static film_stock_t scaled;
    int i = conf.film_stock;
    if (i < 0 || i >= film_stock_count) i = 0;
    const film_stock_t *s = &film_stocks[i];
    if (i == 0 || i >= FILM_SCALED_UPTO) return s;

    int k = FILM_STRENGTH, c;
    scaled = *s;
    scaled.contrast = sclamp((s->contrast * k) >> 12, 0, Q(0.40));
    for (c = 0; c < 3; c++)
    {
        scaled.shadow[c]    = sclamp((s->shadow[c] * k) >> 12, Q(-0.08), Q(0.08));
        scaled.highlight[c] = sclamp((s->highlight[c] * k) >> 12, Q(-0.08), Q(0.08));
    }
    return &scaled;
}

static int grain_amount()
{
    if (conf.film_grain == FILM_GRAIN_STOCK) return current_stock()->grain;
    return conf.film_grain - 1;     // 1 = off -> 0
}

static int grain_size()
{
    if (conf.film_grain_size == FILM_SIZE_STOCK) return current_stock()->grain_size;
    return conf.film_grain_size - 1;
}

// Each film brings its own grain, so changing stock goes back to the stock's grain
static void set_stock(int i)
{
    if (i < 0) i = film_stock_count - 1;
    if (i >= film_stock_count) i = 0;
    conf.film_stock = i;
    conf.film_grain = FILM_GRAIN_STOCK;
    conf.film_grain_size = FILM_SIZE_STOCK;
    film_settings_changed();
}

static void set_grain(int amount)
{
    if (amount < 0) amount = 0;
    if (amount > 10) amount = 10;
    conf.film_grain = amount + 1;
}

static const char* grain_text()
{
    static char buf[16];
    int a = grain_amount();
    if (a == 0) strcpy(buf, "Off");
    else sprintf(buf, "%d", a);
    if (conf.film_grain == FILM_GRAIN_STOCK) strcat(buf, " (auto)");
    return buf;
}

static const char* raw_text()
{
    return !conf.save_raw ? "Off" : conf.dng_raw ? "DNG" : "CRW";
}

//-------------------------------------------------------------------
// Curves

static unsigned short *lut = 0;     // 3 * LUT_SIZE: R, G, B
static int lut_stock = -1;
static int lut_white = -1;

static unsigned isqrt(unsigned v)
{
    unsigned r = 0, b = 1 << 30;
    while (b > v) b >>= 2;
    while (b)
    {
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }
        else r >>= 1;
        b >>= 2;
    }
    return r;
}

static int clamp(int v, int lo, int hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static int build_luts(int range)
{
    const film_stock_t *s = current_stock();

    if (lut && lut_stock == conf.film_stock && lut_white == range)
        return 1;

    if (!lut)
        lut = malloc(3 * LUT_SIZE * sizeof(unsigned short));
    if (!lut)
        return 0;

    int x, c;
    for (x = 0; x < LUT_SIZE; x++)
    {
        // linear -> perceptual
        int p = isqrt(((x << 12) / range) << 12);
        if (p > ONE) p = ONE;

        // S-curve around mid grey; derivative stays positive for contrast <= 0.4
        int t = (p * (ONE - p)) >> 12;
        int f = ((p - PIVOT) * t) >> 10;
        int v = p + ((s->contrast * f) >> 12);
        v = clamp(v, 0, ONE);

        // faded blacks / muted whites
        v = s->lift + ((v * (s->white - s->lift)) >> 12);

        int inv = ONE - v;
        int sh = (inv * inv) >> 12;
        int hi = (v * v) >> 12;

        for (c = 0; c < 3; c++)
        {
            int vc = v + ((s->shadow[c] * sh) >> 12) + ((s->highlight[c] * hi) >> 12);
            vc = clamp(vc, 0, ONE);
            // perceptual -> linear
            lut[c*LUT_SIZE + x] = (((vc * vc) >> 12) * range) >> 12;
        }
    }

    lut_stock = conf.film_stock;
    lut_white = range;
    return 1;
}

//-------------------------------------------------------------------
// Grain

static unsigned rng_state = 0x9E3779B9;

static inline unsigned xorshift()
{
    unsigned x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return rng_state = x;
}

// Approximate gaussian, mean 0, sd ~147
static inline int noise()
{
    unsigned r = xorshift();
    return (int)((r & 0xFF) + ((r >> 8) & 0xFF) + ((r >> 16) & 0xFF) + (r >> 24)) - 510;
}

// Grain amplitude by luminance (L >> 4), in linear units scaled so that (noise * amp) >> 12
// gives the target standard deviation. Strongest in the mid tones like real film.
static void build_grain_amp(int *amp, int amount, int range)
{
    int i;
    for (i = 0; i < 256; i++)
    {
        int x = (i << 4) + 8;
        int p = isqrt(((x << 12) / range) << 12);
        if (p > ONE) p = ONE;
        int shape = 1434 + ((10650 * ((p * (ONE - p)) >> 12)) >> 12);   // 0.35 .. 1.0
        int dp = (amount * 184 * shape) / (10 * ONE);                   // Q12, 0.45% per step at peak
        int pd = (p * dp) >> 12;
        amp[i] = (2 * range * pd) / 147;
    }
}

//-------------------------------------------------------------------
// Raw buffer access (10 bit packed, 8 pixels per 10 bytes)

#if CAM_SENSOR_BITS_PER_PIXEL == 10

static inline void unpack8(const unsigned char *b, int *p)
{
    p[0] = (b[1] << 2) | (b[0] >> 6);
    p[1] = ((b[0] & 0x3F) << 4) | (b[3] >> 4);
    p[2] = ((b[3] & 0x0F) << 6) | (b[2] >> 2);
    p[3] = ((b[2] & 0x03) << 8) | b[5];
    p[4] = (b[4] << 2) | (b[7] >> 6);
    p[5] = ((b[7] & 0x3F) << 4) | (b[6] >> 4);
    p[6] = ((b[6] & 0x0F) << 6) | (b[9] >> 2);
    p[7] = ((b[9] & 0x03) << 8) | b[8];
}

static inline void pack8(unsigned char *b, const int *p)
{
    b[1] = p[0] >> 2;
    b[0] = (p[0] << 6) | (p[1] >> 4);
    b[3] = (p[1] << 4) | (p[2] >> 6);
    b[2] = (p[2] << 2) | (p[3] >> 8);
    b[5] = p[3];
    b[4] = p[4] >> 2;
    b[7] = (p[4] << 6) | (p[5] >> 4);
    b[6] = (p[5] << 4) | (p[6] >> 6);
    b[9] = (p[6] << 2) | (p[7] >> 8);
    b[8] = p[7];
}

#define GROUP_PIX   8
#define GROUP_BYTES 10

#endif

// Raw buffer access (12 bit packed: 4 pixels per 6 bytes, same layout as core/raw.c; two groups = 8 pixels per 12 bytes)
#if CAM_SENSOR_BITS_PER_PIXEL == 12
static inline void unpack8(const unsigned char *b, int *p) {
    p[0] = (b[1] << 4) | (b[0] >> 4);
    p[1] = ((b[0] & 0x0F) << 8) | b[3];
    p[2] = (b[2] << 4) | (b[5] >> 4);
    p[3] = ((b[5] & 0x0F) << 8) | b[4];
    p[4] = (b[7] << 4) | (b[6] >> 4);
    p[5] = ((b[6] & 0x0F) << 8) | b[9];
    p[6] = (b[8] << 4) | (b[11] >> 4);
    p[7] = ((b[11] & 0x0F) << 8) | b[10];
}
static inline void pack8(unsigned char *b, const int *p) {
    b[1] = p[0] >> 4;
    b[0] = (p[0] << 4) | (p[1] >> 8);
    b[3] = p[1];
    b[2] = p[2] >> 4;
    b[5] = (p[2] << 4) | (p[3] >> 8);
    b[4] = p[3];
    b[7] = p[4] >> 4;
    b[6] = (p[4] << 4) | (p[5] >> 8);
    b[9] = p[5];
    b[8] = p[6] >> 4;
    b[11] = (p[6] << 4) | (p[7] >> 8);
    b[10] = p[7];
}
#define GROUP_PIX 8
#define GROUP_BYTES 12
#endif

//-------------------------------------------------------------------
// Developing light: the LCD is dark while the camera captures, so the blue
// Print LED blinks instead, speeding up as the shot nears done.

#if defined(CAMERA_a570)
#define DEV_LED_REG     0xc02200C4      // Print button LED, same one debug_led() drives
#define DEV_LED_ON      0x46
#endif

#if defined(CAMERA_sx240hs)
static void dev_led_set(int on) { camera_set_led(0, on, 0); }    // power LED
#else
static void dev_led_set(int on) { debug_led(on); }
#endif

static int dev_led_was_on, dev_led_state;
static long dev_led_next;

static void dev_light_begin()
{
#ifdef DEV_LED_REG
    dev_led_was_on = (*(volatile long*)DEV_LED_REG == DEV_LED_ON);
#elif defined(CAMERA_sx240hs)
    dev_led_was_on = 1;        // power LED is normally lit
#elif defined(CAMERA_sx240hs)
    dev_led_was_on = 1;        // power LED is normally lit
#else
    dev_led_was_on = 0;
#endif
    dev_led_state = 1;
    dev_led_set(1);
    dev_led_next = get_tick_count() + 250;
}

// Half period goes from 250ms at the start to 50ms at the end
static void dev_light_progress(int done, int total)
{
    long now = get_tick_count();
    if (now >= dev_led_next)
    {
        dev_led_state = !dev_led_state;
        dev_led_set(dev_led_state);
        dev_led_next = now + 250 - 200 * done / total;
    }
}

static void dev_light_end()
{
    dev_led_set(dev_led_was_on);
}

//-------------------------------------------------------------------

static unsigned film_dbg_ms, film_dbg_wb_ms, film_dbg_raw;

// One line per shot in CHDK/FILM.LOG: which film the picture was taken with
static void film_log_shot(void)
{
#ifndef FILM_HOST_PREVIEW
    static char line[64];           // static: raw hook runs on a small task stack
    int n = sprintf(line, "IMG_%04d  %s\n", (int)get_target_file_num(), current_stock()->name);
    int fd = open("A/CHDK/FILM.LOG", O_WRONLY|O_CREAT, 0777);
    if (fd >= 0)
    {
        lseek(fd, 0, SEEK_END);
        write(fd, line, n);
        close(fd);
    }
#endif
}

void film_apply(void)
{
    film_cc_suspend();
    film_log_shot();
#if CAM_SENSOR_BITS_PER_PIXEL == 10 || CAM_SENSOR_BITS_PER_PIXEL == 12
    const film_stock_t *s = current_stock();
    if (conf.film_stock == 0)
        return;

    unsigned char *raw = (unsigned char*)get_raw_image_addr();
    film_dbg_raw = (unsigned)raw;
    unsigned t_start = get_tick_count();
    int black = camera_sensor.black_level;
    int white = camera_sensor.white_level;
    int range = white - black;
    int rowlen = camera_sensor.raw_rowlen;

    if (!build_luts(range))
        return;

    // Bayer layout of a 2x2 quad: index 0=(0,0) 1=(0,1) 2=(1,0) 3=(1,1), colour 0=R 1=G 2=B
    int iR = 0, iB = 3, iG1 = 1, iG2 = 2, k;
    {
        int n = 0, g[2] = { 1, 2 };
        for (k = 0; k < 4; k++)
        {
            int col = (camera_sensor.cfa_pattern >> (k*8)) & 0xFF;
            if (col == 0) iR = k;
            else if (col == 2) iB = k;
            else if (n < 2) g[n++] = k;
        }
        iG1 = g[0]; iG2 = g[1];
    }

    // Active area, aligned so groups start on whole 8 pixel blocks and quads on even rows
    int x0 = (camera_sensor.active_area.x1 + GROUP_PIX - 1) / GROUP_PIX;
    int x1 = camera_sensor.active_area.x2 / GROUP_PIX;
    int y0 = (camera_sensor.active_area.y1 + 1) & ~1;
    int y1 = camera_sensor.active_area.y2 & ~1;
    int groups = x1 - x0;
    if (groups <= 0 || y1 <= y0)
        return;

    int x, y, q[4], a[8], b[8];

    // Grey world white balance estimate on a sparse grid, skipping clipped and very dark quads
    unsigned sr = 0, sg = 0, sb = 0;
    for (y = y0; y < y1; y += 32)
    {
        unsigned char *ra = raw + y*rowlen;
        unsigned char *rb = ra + rowlen;
        for (x = x0; x < x1; x += 4)
        {
            unpack8(ra + x*GROUP_BYTES, a);
            unpack8(rb + x*GROUP_BYTES, b);
            q[0] = a[0]; q[1] = a[1]; q[2] = b[0]; q[3] = b[1];
            int r = q[iR] - black, g = ((q[iG1] + q[iG2]) >> 1) - black, bl = q[iB] - black;
            if (r < 16 || g < 16 || bl < 16) continue;
            if (q[iR] >= white - 8 || q[iG1] >= white - 8 || q[iB] >= white - 8) continue;
            sr += r; sg += g; sb += bl;
        }
    }
    int gR = ONE, gB = ONE;
    if (sr > 1000 && sb > 1000)
    {
        gR = clamp((int)(((unsigned long long)sg << 12) / sr), ONE/2, ONE*4);
        gB = clamp((int)(((unsigned long long)sg << 12) / sb), ONE/2, ONE*4);
    }
    int igR = (ONE << 12) / gR;
    int igB = (ONE << 12) / gB;
    film_dbg_wb_ms = get_tick_count() - t_start;

    // Grain setup
    int amount = grain_amount();
    int gsize = grain_size();
    static int gamp[256];   // static: raw hook runs on a small task stack
    int quads = groups * 4;
    int *nrow = 0;
    if (amount > 0)
    {
        build_grain_amp(gamp, amount, range);
        if (gsize > 0)
        {
            nrow = malloc(quads * sizeof(int));
            if (nrow) memset(nrow, 0, quads * sizeof(int));
            else gsize = 0;
        }
        rng_state ^= get_tick_count() * 2654435761u;
        if (!rng_state) rng_state = 0x9E3779B9;
    }

    int show_dev = conf.film_dev_screen;
    if (show_dev)
        dev_light_begin();

    // Vignette: separable falloff, per quad column and per row pair
    int vig = s->vignette;
    if (conf.film_vignette == 0) vig = 0;
    int *vx = 0;
    int qcx = quads / 2, qcy = (y1 - y0) / 4;
    if (vig)
    {
        vx = malloc(quads * sizeof(int));
        if (vx)
        {
            for (x = 0; x < quads; x++)
            {
                int d = ((x - qcx) << 12) / qcx;
                vx[x] = ONE - ((vig * ((d * d) >> 12)) >> 12);
            }
        }
        else vig = 0;
    }

    unsigned short *lR = lut, *lG = lut + LUT_SIZE, *lB = lut + 2*LUT_SIZE;
    int sat = s->saturation;

    for (y = y0; y < y1; y += 2)
    {
        if (show_dev && ((y - y0) & 15) == 0)
            dev_light_progress(y - y0, y1 - y0);

        unsigned char *ra = raw + y*rowlen + x0*GROUP_BYTES;
        unsigned char *rb = ra + rowlen;
        int vy = ONE;
        if (vig)
        {
            int d = (((y - y0) / 2 - qcy) << 12) / qcy;
            vy = ONE - ((vig * ((d * d) >> 12)) >> 12);
        }
        int nleft = 0;
        int qx = 0;

        for (x = 0; x < groups; x++, ra += GROUP_BYTES, rb += GROUP_BYTES)
        {
            unpack8(ra, a);
            unpack8(rb, b);
#if FILM_PROFILE == 1
            pack8(ra, a);
            pack8(rb, b);
            continue;
#endif

            for (k = 0; k < 8; k += 2, qx++)
            {
                q[0] = a[k]; q[1] = a[k+1]; q[2] = b[k]; q[3] = b[k+1];

                int R  = q[iR]  - black; if (R  < 0) R  = 0;
                int G1 = q[iG1] - black; if (G1 < 0) G1 = 0;
                int G2 = q[iG2] - black; if (G2 < 0) G2 = 0;
                int B  = q[iB]  - black; if (B  < 0) B  = 0;

                R = (R * gR) >> 12;
                B = (B * gB) >> 12;

                if (vig)
                {
                    int v = (vx[qx] * vy) >> 12;
                    R = (R * v) >> 12; G1 = (G1 * v) >> 12; G2 = (G2 * v) >> 12; B = (B * v) >> 12;
                }

                int L = (77*R + 76*G1 + 75*G2 + 28*B) >> 8;

                if (sat != ONE)
                {
                    R  = L + (((R  - L) * sat) >> 12);
                    G1 = L + (((G1 - L) * sat) >> 12);
                    G2 = L + (((G2 - L) * sat) >> 12);
                    B  = L + (((B  - L) * sat) >> 12);
                }

                if (amount > 0)
                {
                    int n = noise();
                    if (gsize == 1)
                    {
                        n = (2*n + nleft + nrow[qx]) >> 2;
                        nleft = nrow[qx] = n;
                        n = (n * 13) >> 3;
                    }
                    else if (gsize == 2)
                    {
                        n = ((n + 2*nleft + 2*nrow[qx]) * 13) >> 6;
                        nleft = nrow[qx] = n;
                        n = (n * 19) >> 3;
                    }
                    int g = (n * gamp[clamp(L >> 4, 0, 255)]) >> 12;
                    R += g; G1 += g; G2 += g; B += g;
                }

                R  = lR[clamp(R,  0, LUT_SIZE-1)];
                G1 = lG[clamp(G1, 0, LUT_SIZE-1)];
                G2 = lG[clamp(G2, 0, LUT_SIZE-1)];
                B  = lB[clamp(B,  0, LUT_SIZE-1)];

                R = (R * igR) >> 12;
                B = (B * igB) >> 12;

                q[iR]  = clamp(R  + black, 0, white);
                q[iG1] = clamp(G1 + black, 0, white);
                q[iG2] = clamp(G2 + black, 0, white);
                q[iB]  = clamp(B  + black, 0, white);

                a[k] = q[0]; a[k+1] = q[1]; b[k] = q[2]; b[k+1] = q[3];
            }

            pack8(ra, a);
            pack8(rb, b);
        }
    }

    if (nrow) free(nrow);
    if (vx) free(vx);

    if (show_dev)
        dev_light_end();
    film_dbg_ms = get_tick_count() - t_start;
#endif
}

//-------------------------------------------------------------------
// Camera colour mode follows the stock (only when changed, so manual
// Canon menu changes are respected afterwards)

#ifndef FILM_PREVIEW_COLORS
#define FILM_PREVIEW_COLORS 0   // stocks without a Canon colour mode use Custom Color so the live view approximates the look
#endif
#define MC_CUSTOM 11

#ifdef FILM_HOST_PREVIEW
#define cc_now() 0u
#else
#define cc_now() ((unsigned int)get_tick_count())
#endif

static int applied_stock = -1;
static int cc_calls = 0;
static int cc_active = 0;           // Custom Color preview currently set on the camera
static unsigned int cc_resume = 0;  // tick at which the preview is put back after a shot

static int step_from(int v, int unit)   // nearest step, limited to -2..+2
{
    int st = (v >= 0) ? (v + unit / 2) / unit : -((-v + unit / 2) / unit);
    return sclamp(st, -2, 2);
}

#if FILM_PREVIEW_COLORS && defined(PROPCASE_CUSTOM_RED) && defined(PROPCASE_CUSTOM_CONTRAST)
static void apply_custom_color(const film_stock_t *s)
{
    int c = s->contrast, sat = s->saturation;
    int cstep = c < Q(0.08) ? -1 : (c < Q(0.15) ? 0 : (c < Q(0.25) ? 1 : 2));
    int sstep = sat < Q(0.88) ? -2 : (sat < Q(0.97) ? -1 : (sat < Q(1.08) ? 0 : (sat < Q(1.20) ? 1 : 2)));
    int t[3], k, avg = 0;
    for (k = 0; k < 3; k++) { t[k] = (s->highlight[k] * 6 + s->shadow[k] * 4) / 10; avg += t[k]; }
    avg /= 3;
    // Canon sliders: 254 = -2, 255 = -1, 0, 1, 2
    shooting_set_prop(PROPCASE_CUSTOM_CONTRAST,   cstep & 0xFF);
    shooting_set_prop(PROPCASE_CUSTOM_SATURATION, sstep & 0xFF);
    shooting_set_prop(PROPCASE_CUSTOM_RED,   step_from(t[0] - avg, Q(0.012)) & 0xFF);
    shooting_set_prop(PROPCASE_CUSTOM_GREEN, step_from(t[1] - avg, Q(0.012)) & 0xFF);
    shooting_set_prop(PROPCASE_CUSTOM_BLUE,  step_from(t[2] - avg, Q(0.012)) & 0xFF);
    // toggle the mode so the camera refreshes the live view even if it was already in Custom Color
    shooting_set_prop(PROPCASE_MY_COLORS, MC_OFF);
#ifndef FILM_HOST_PREVIEW
    msleep(100);
#endif
    shooting_set_prop(PROPCASE_MY_COLORS, MC_CUSTOM);
    cc_active = 1;
}

static void cc_reset(void)
{
    shooting_set_prop(PROPCASE_CUSTOM_CONTRAST, 0);
    shooting_set_prop(PROPCASE_CUSTOM_SATURATION, 0);
    shooting_set_prop(PROPCASE_CUSTOM_RED, 0);
    shooting_set_prop(PROPCASE_CUSTOM_GREEN, 0);
    shooting_set_prop(PROPCASE_CUSTOM_BLUE, 0);
    shooting_set_prop(PROPCASE_MY_COLORS, MC_OFF);
    cc_active = 0;
}
#endif

static void apply_camera_colors()
{
    cc_calls++;
    if (!conf.film_camera_color || !camera_info.state.mode_rec)
        return;
    if (cc_resume && (int)(cc_now() - cc_resume) >= 0) { cc_resume = 0; applied_stock = -1; }
    if (cc_resume)
        return;                     // wait until the shot has been developed
    if (applied_stock == conf.film_stock)
        return;
#if FILM_PREVIEW_COLORS && defined(PROPCASE_CUSTOM_RED) && defined(PROPCASE_CUSTOM_CONTRAST)
    if (conf.film_stock == 0 && shooting_get_prop(PROPCASE_MY_COLORS) == MC_CUSTOM)
    {
        cc_reset();
        applied_stock = 0;
        return;
    }
#endif
    const film_stock_t *s = current_stock();
    int mc = s->my_colors;
#if FILM_PREVIEW_COLORS && defined(PROPCASE_CUSTOM_RED) && defined(PROPCASE_CUSTOM_CONTRAST)
    if (mc == MC_OFF && conf.film_stock > 0)
    {
        apply_custom_color(s);
        applied_stock = conf.film_stock;
        return;
    }
#endif
    cc_active = 0;
    if (mc >= 0)
        shooting_set_prop(PROPCASE_MY_COLORS, mc);
    applied_stock = conf.film_stock;
}

// Called from the raw hook: put the camera back to plain colour so Canon does not add
// its own contrast/saturation on top of the film look, then restore the preview later.
static void film_cc_suspend(void)
{
#if FILM_PREVIEW_COLORS
    if (cc_active)
    {
        shooting_set_prop(PROPCASE_MY_COLORS, MC_OFF);
        cc_active = 0;
        cc_resume = cc_now() + 3000;
        if (!cc_resume) cc_resume = 1;
    }
#endif
}

void film_settings_changed(void)
{
    lut_stock = -1;
    if (conf.film_stock == 0 && lut)
    {
        free(lut);
        lut = 0;
    }
    applied_stock = -1;
    apply_camera_colors();
}

//-------------------------------------------------------------------
// Status pill at the top of the shooting screen, replacing CHDK's separate
// battery / card space / RAW indicators:  ● PORTRA G3 DNG | 87% 1.2G

static int pill_text(int x, int y, const char *t, color fg)
{
    draw_string(x, y, t, MAKE_COLOR(COLOR_BLACK, fg));
    return x + strlen(t) * FONT_WIDTH;
}

#define FILM_LIVE_PREVIEW 0     // software live view preview is too slow on the SX240 (about 4 fps), kept disabled

#ifndef FILM_HOST_PREVIEW
//-------------------------------------------------------------------
// Approximate live view preview: contrast curve on luma, tints and saturation on chroma.
// Works on the UYVY viewport buffer (U Y1 V Y2 per pair of pixels).

static unsigned char lv_y[256], lv_u[256], lv_v[256];   // luma curve, chroma tint (+128 bias)
static int lv_stock = -1;
static unsigned char *lv_last_buf = 0;
static unsigned int lv_last_hash = 0;

static void lv_build(void)
{
    const film_stock_t *s = current_stock();
    int y;
    for (y = 0; y < 256; y++)
    {
        int p = (y << 12) / 255;
        int t = (p * (ONE - p)) >> 12;
        int f = ((p - PIVOT) * t) >> 10;
        int v = p + ((s->contrast * f) >> 12);
        v = sclamp(v, 0, ONE);
        v = s->lift + ((v * (s->white - s->lift)) >> 12);
        v = sclamp(v, 0, ONE);

        int inv = ONE - v;
        int sh = (inv * inv) >> 12;
        int hi = (v * v) >> 12;
        int dr = ((s->shadow[0] * sh) >> 12) + ((s->highlight[0] * hi) >> 12);
        int dg = ((s->shadow[1] * sh) >> 12) + ((s->highlight[1] * hi) >> 12);
        int db = ((s->shadow[2] * sh) >> 12) + ((s->highlight[2] * hi) >> 12);
        int du = ((-43 * dr - 85 * dg + 128 * db) * 255) >> 20;
        int dv = ((128 * dr - 107 * dg - 21 * db) * 255) >> 20;

        lv_y[y] = (v * 255) >> 12;
        lv_u[y] = sclamp(du, -127, 127) + 128;
        lv_v[y] = sclamp(dv, -127, 127) + 128;
    }
    lv_stock = conf.film_stock;
}

static unsigned int lv_hash(unsigned char *buf, int n6, int rows, int stride)
{
    unsigned int h = 0;
    int r, g;
    for (r = rows / 16; r < rows; r += rows / 8)
        for (g = n6 / 8; g < n6; g += n6 / 4)
            h = h * 31 + buf[r * stride + g * 6 + 1];
    return h;
}

static int lv_calls, lv_done, lv_why;
static int lv_g, lv_r, lv_st;
static unsigned int lv_buf;

extern char active_viewport_buffer;
extern void* viewport_buffers[];

static unsigned int lv_hashes[4];
static unsigned char lv_valid[4];
static int lv_ms, lv_fps;
static unsigned int lv_t0, lv_d0;

static void lv_process(unsigned char *buf, int n6, int rows, int stride, int sat)
{
    int r, g;
    for (r = 0; r < rows; r++)
    {
        unsigned char *b = buf + r * stride;
        for (g = 0; g < n6; g++, b += 6)
        {
            int y1 = b[1], y2 = b[2], y3 = b[4], y4 = b[5];
            int ya = (y1 + y2 + y3 + y4) >> 2;
            int u = (((signed char)b[0] * sat) >> 12) + (lv_u[ya] - 128);
            int v = (((signed char)b[3] * sat) >> 12) + (lv_v[ya] - 128);
            b[0] = (unsigned char)sclamp(u, -128, 127);
            b[3] = (unsigned char)sclamp(v, -128, 127);
            b[1] = lv_y[y1];
            b[2] = lv_y[y2];
            b[4] = lv_y[y3];
            b[5] = lv_y[y4];
        }
    }
}

static void film_live_apply(void)
{
    lv_calls++;
    if (conf.film_stock <= 0 || conf.film_stock >= film_stock_count) { lv_why = 1; return; }
    if ((mode_get() & MODE_MASK) != MODE_REC) { lv_why = 2; return; }
    if (MODE_IS_VIDEO(mode_get()) || is_video_recording()) { lv_why = 3; return; }

    // viewport is YUV 4:1:1: 6 bytes per 4 pixels, order U Y1 Y2 V Y3 Y4 (U,V signed)
    int groups = vid_get_viewport_width();
    int rows   = vid_get_viewport_height() * vid_get_viewport_yscale();
    int stride = vid_get_viewport_byte_width();
    int n6 = groups / 2;
    if (n6 <= 0 || rows <= 0 || stride < n6 * 6) { lv_why = 5; return; }

    int i, did = 0;
    unsigned int t0 = get_tick_count();

    if (lv_stock != conf.film_stock)
    {
        lv_build();
        for (i = 0; i < 4; i++) lv_valid[i] = 0;
    }

    int sat = current_stock()->saturation;
    for (i = 0; i < 4; i++)
    {
        unsigned char *vb = (unsigned char*)viewport_buffers[i];
        if (!vb) continue;
        if (lv_valid[i] && lv_hash(vb, n6, rows, stride) == lv_hashes[i]) continue;   // not redrawn by Canon yet
        lv_process(vb, n6, rows, stride, sat);
        lv_hashes[i] = lv_hash(vb, n6, rows, stride);
        lv_valid[i] = 1;
        did++;
    }
    lv_done += did;
    lv_why = 0;
    if (did) lv_ms = get_tick_count() - t0;
    if (get_tick_count() - lv_t0 >= 1000)
    {
        lv_fps = (lv_done - lv_d0) * 1000 / (get_tick_count() - lv_t0);
        lv_t0 = get_tick_count();
        lv_d0 = lv_done;
    }
}
#else
static void film_live_apply(void) {}
#endif

static int canon_menu_open(void);
static int pill_valid;

void film_draw_osd(int is_osd_edit)
{
    #if 0
    {
        char fdbg[64];
        sprintf(fdbg, "F tot%u wb%u raw%08x", film_dbg_ms, film_dbg_wb_ms, film_dbg_raw);
        draw_string(8, 56, fdbg, MAKE_COLOR(COLOR_BLACK, COLOR_WHITE));
    }
#endif
#if 0
    {
        char dbg[96];
        sprintf(dbg, "MC%d C%d S%d R%d G%d B%d ap%d n%d",
            (int)shooting_get_prop(PROPCASE_MY_COLORS),
            (int)(signed char)shooting_get_prop(PROPCASE_CUSTOM_CONTRAST),
            (int)(signed char)shooting_get_prop(PROPCASE_CUSTOM_SATURATION),
            (int)(signed char)shooting_get_prop(PROPCASE_CUSTOM_RED),
            (int)(signed char)shooting_get_prop(PROPCASE_CUSTOM_GREEN),
            (int)(signed char)shooting_get_prop(PROPCASE_CUSTOM_BLUE),
            applied_stock, cc_calls);
        draw_string(8, 40, dbg, MAKE_COLOR(COLOR_BLACK, COLOR_WHITE));
    }
#endif
#if FILM_LIVE_PREVIEW
    film_live_apply();
#ifndef FILM_HOST_PREVIEW
    {
        char dbg[72];
        sprintf(dbg, "LV fps%d ms%d w%d a%d", lv_fps, lv_ms, lv_why, (int)active_viewport_buffer);
        draw_string(8, 40, dbg, MAKE_COLOR(COLOR_BLACK, COLOR_WHITE));
    }
#endif
#endif
    apply_camera_colors();

    if (!conf.film_osd || canon_menu_open())
    {
        pill_valid = 0;
        return;
    }
    if (!is_osd_edit && (!camera_info.state.mode_rec || camera_info.state.mode_video || camera_info.state.is_shutter_half_press))
    {
        pill_valid = 0;
        return;
    }

    static char grain[8], batt[8], space[8];
    const char *tag = conf.film_stock ? current_stock()->tag : "NO FILM";
    int amount = grain_amount();
    grain[0] = 0;
    if (conf.film_stock && amount > 0) sprintf(grain, " G%d", amount);
    const char *raw = conf.save_raw ? (conf.dng_raw ? " DNG" : " CRW") : "";

    int bp = get_batt_perc();
    sprintf(batt, "%d%%", bp);
    unsigned mb = GetFreeCardSpaceKb() >> 10;
    if (mb >= 1024) sprintf(space, " %d.%dG", mb >> 10, ((mb & 1023) * 10) >> 10);
    else sprintf(space, " %dM", mb);

    int chars = strlen(tag) + strlen(grain) + strlen(raw) + strlen(batt) + strlen(space);
    int w = chars * FONT_WIDTH + 13 + 12 + 8;       // dot + separator gap + padding
    int h = FONT_HEIGHT;
    int x = camera_screen.disp_left + (camera_screen.disp_width - w) / 2;
    int y = 4;

    // Repainting the pill background every cycle makes the bar flicker. Repaint it only when
    // something changed (or now and then in case Canon drew over it); text alone repaints in place.
    static char last_key[48];
    static int redraw_ticks;
    char key[48];
    sprintf(key, "%s|%s|%s|%s|%s|%d", tag, grain, raw, batt, space, is_osd_edit);
    int full = !pill_valid || strcmp(key, last_key) || !((++redraw_ticks) & 63);
    if (full)
    {
        draw_rectangle(x, y, x + w, y + h + 3, MAKE_COLOR(COLOR_BLACK, COLOR_GREY_DK), RECT_BORDER1|DRAW_FILLED|RECT_ROUND_CORNERS);
        draw_rectangle(x + 5, y + h/2 - 1, x + 8, y + h/2 + 2, MAKE_COLOR(COLOR_RED, COLOR_RED), RECT_BORDER0|DRAW_FILLED);
        strcpy(last_key, key);
        pill_valid = 1;
    }
    int tx = pill_text(x + 13, y + 2, tag, conf.film_stock ? COLOR_WHITE : COLOR_GREY);
    tx = pill_text(tx, y + 2, grain, COLOR_GREY_LT);
    tx = pill_text(tx, y + 2, raw, COLOR_GREY_LT);
    draw_vline(tx + 5, y + 4, h - 4, COLOR_GREY_DK);
    tx = pill_text(tx + 12, y + 2, batt, bp <= 20 ? COLOR_RED : COLOR_GREY);
    pill_text(tx, y + 2, space, (mb < 100) ? COLOR_RED : COLOR_GREY);
}

//-------------------------------------------------------------------
// Film picker: what the ALT button shows in record mode.
//
//   FILM                                4 / 11
//   <        Superia 400                     >
//        Green shadows, cool everyday colour
//   ------------------------------------------
//   GRAIN  [##########]  4 (auto)       Medium
//   RAW    DNG
//   <> film   ^v grain   SET size   DISP raw

#define PK_H        140
#define PK_PAD      12

// True while a Canon menu (MENU or FUNC.SET) is on screen: CHDK must leave the screen to Canon then
#ifdef FILM_HOST_PREVIEW
static int canon_menu_open(void) { return 0; }
#else
static int canon_menu_open(void)
{
    return (canon_menu_active != (int)&canon_menu_active - 4) || canon_shoot_menu_active != 0;
}
#endif

int film_picker_active(void)
{
    int open = canon_menu_open();
#ifndef FILM_HOST_PREVIEW
    static int was_open = 0;
    if (open && !was_open)
        gui_set_need_restore();     // erase what CHDK drew; Canon's menu takes the screen
    was_open = open;
#endif
    return camera_info.state.mode_rec && !camera_info.state.mode_video && !camera_info.state.state_kbd_script_run && !open;
}

static int raw_key_free()
{
    return conf.alt_mode_button != KEY_DISPLAY;
}

void film_picker_draw(int force_redraw)
{
    static int last_sig = -1, ticks = 0;
    int sig = (conf.film_stock << 16) | (conf.film_grain << 8) | (conf.film_grain_size << 4) | (conf.save_raw << 1) | conf.dng_raw;

    // Redraw on change, on request, and now and then in case Canon's OSD drew over us
    if (!force_redraw && sig == last_sig && (++ticks & 31))
        return;
    last_sig = sig;

    static char buf[48];
    const film_stock_t *s = current_stock();
    int x = camera_screen.disp_left + 8;
    int w = camera_screen.disp_width - 16;
    int y = camera_screen.height - FONT_HEIGHT - PK_H - 4;
    int ix = x + PK_PAD, iw = w - 2*PK_PAD;
    twoColors label = MAKE_COLOR(COLOR_BLACK, COLOR_GREY);
    twoColors value = MAKE_COLOR(COLOR_BLACK, COLOR_WHITE);
    twoColors accent = MAKE_COLOR(COLOR_BLACK, COLOR_RED);

    draw_rectangle(x, y, x+w-1, y+PK_H-1, MAKE_COLOR(COLOR_BLACK, COLOR_GREY_DK), RECT_BORDER1|DRAW_FILLED|RECT_ROUND_CORNERS);

    // header
    draw_string(ix, y+6, "FILM", label);
    sprintf(buf, "%d / %d", conf.film_stock, film_stock_count-1);
    draw_string_justified(ix, y+6, conf.film_stock ? buf : "OFF", label, 0, iw, TEXT_RIGHT);
    draw_string_justified(ix, y+6, "PortHack by Bastian", accent, 0, iw, TEXT_CENTER);

    // stock name, double size, with arrows
    int nl = strlen(s->name);
    int sc = (nl * FONT_WIDTH * 2 <= iw - 2*FONT_WIDTH*2) ? 2 : 1;
    int nw = nl * FONT_WIDTH * sc;
    int ny = y + 24 + (sc == 1 ? FONT_HEIGHT/2 : 0);
    draw_string_scaled(ix, y+24+FONT_HEIGHT/2, "<", accent, 1, 1);
    draw_string_scaled(ix+iw-FONT_WIDTH, y+24+FONT_HEIGHT/2, ">", accent, 1, 1);
    draw_string_scaled(ix + (iw - nw)/2, ny, s->name, value, sc, sc);

    // description
    draw_string_justified(ix, y+60, s->desc, label, 0, iw, TEXT_CENTER);

    draw_hline(ix, y+80, iw, COLOR_GREY_DK);

    // grain: segmented meter + amount + size
    int amount = grain_amount();
    draw_string(ix, y+86, "GRAIN", label);
    int bx = ix + 7*FONT_WIDTH, i;
    for (i = 0; i < 10; i++)
    {
        color c = (i < amount) ? COLOR_RED : COLOR_GREY_DK;
        draw_rectangle(bx + i*9, y+90, bx + i*9 + 6, y+97, MAKE_COLOR(c, c), RECT_BORDER0|DRAW_FILLED);
    }
    draw_string(bx + 96, y+86, grain_text(), value);
    draw_string_justified(ix, y+86, amount ? size_names[grain_size()+1] : "", label, 0, iw, TEXT_RIGHT);

    // raw
    draw_string(ix, y+104, "RAW", label);
    draw_string(bx, y+104, raw_text(), value);

    // key hints
    sprintf(buf, "<> film  ^v grain  SET size%s", raw_key_free() ? "  DISP raw" : "");
    draw_string_justified(ix, y+122, buf, label, 0, iw, TEXT_CENTER);
}

int film_picker_kbd(void)
{
    if (!film_picker_active())
        return 0;

    switch (kbd_get_autoclicked_key())
    {
    case KEY_RIGHT: set_stock(conf.film_stock + 1); return 1;
    case KEY_LEFT:  set_stock(conf.film_stock - 1); return 1;
    case KEY_UP:    set_grain(grain_amount() + 1);  return 1;
    case KEY_DOWN:  set_grain(grain_amount() - 1);  return 1;
    }

    if (kbd_is_key_clicked(KEY_SET))
    {
        conf.film_grain_size = (grain_size() + 1) % 3 + 1;
        return 1;
    }
    if (raw_key_free() && kbd_is_key_clicked(KEY_DISPLAY))
    {
        gui_film_raw_enum(1, 0);
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------
// Menu

const char* gui_film_stock_enum(int change, __attribute__ ((unused))int arg)
{
    if (change)
        set_stock(conf.film_stock + change);
    return current_stock()->name;
}

// Auto (stock default), Off, 1..10
const char* gui_film_grain_enum(int change, __attribute__ ((unused))int arg)
{
    gui_enum_value_change(&conf.film_grain, change, 12);
    return grain_text();
}

const char* gui_film_grain_size_enum(int change, __attribute__ ((unused))int arg)
{
    static char buf[20];
    gui_enum_value_change(&conf.film_grain_size, change, 4);
    if (conf.film_grain_size == FILM_SIZE_STOCK)
    {
        sprintf(buf, "%s (auto)", size_names[grain_size()+1]);
        return buf;
    }
    return size_names[conf.film_grain_size];
}

const char* gui_film_vignette_enum(int change, __attribute__ ((unused))int arg)
{
    static const char *modes[] = { "Off", "Auto" };
    gui_enum_value_change(&conf.film_vignette, change, 2);
    if (conf.film_vignette && !current_stock()->vignette)
        return "Auto (none)";
    return modes[conf.film_vignette];
}

// RAW: Off / DNG / CRW maps onto the existing save_raw + dng_raw settings
const char* gui_film_raw_enum(int change, __attribute__ ((unused))int arg)
{
    int v = !conf.save_raw ? 0 : conf.dng_raw ? 1 : 2;
    gui_enum_value_change(&v, change, 3);
    if (change)
    {
        conf.save_raw = (v != 0);
        if (v) conf.dng_raw = (v == 1);
        conf_change_dng();
    }
    return raw_text();
}
