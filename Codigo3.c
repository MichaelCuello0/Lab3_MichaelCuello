#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define ADDR_LCD        0x27
#define ADDR_DS1307     0x68

#define SDA_DISP        13
#define SCL_DISP        16
#define I2C_BUS_DISP    I2C_NUM_0

#define SDA_CLOCK       21
#define SCL_CLOCK       22
#define I2C_BUS_CLOCK   I2C_NUM_1

#define BACKLIGHT_BIT   0x08
#define ENABLE_BIT      0x04
#define RS_BIT          0x01

#define SPI_MISO        27
#define SPI_MOSI        26
#define SPI_SCK         32
#define SPI_SS          12
#define RFID_RST        14

#define SPI_BUS         SPI2_HOST

static spi_device_handle_t handle_rfid;

#define SALIDA_VERDE    GPIO_NUM_5
#define SALIDA_ROJA     GPIO_NUM_4
#define SALIDA_AZUL     GPIO_NUM_15
#define SALIDA_BUZZER   GPIO_NUM_18

volatile int estado_panel = 0;
volatile int flag_lcd_bt  = 0;

char msg_display[21]  = "Sin mensajes";
char str_hora[9]      = "00:00:00";

int64_t ts_ultima_rfid = 0;
int64_t ts_ultima_hora = 0;

#define COOLDOWN_RFID_MS 4000

uint8_t uid_permitido[4] = {0x5A, 0xD5, 0xFA, 0x03};

#define R_CMD           0x01
#define R_IRQ           0x04
#define R_ERR           0x06
#define R_FIFO_DATA     0x09
#define R_FIFO_LVL      0x0A
#define R_CTRL          0x0C
#define R_BIT_FRAME     0x0D
#define R_MODE          0x11
#define R_TX_CTRL       0x14
#define R_TX_ASK        0x15
#define R_TMODE         0x2A
#define R_TPRESCALER    0x2B
#define R_TRELOAD_H     0x2C
#define R_TRELOAD_L     0x2D
#define R_VER           0x37

#define OP_IDLE         0x00
#define OP_TRANSCEIVE   0x0C
#define OP_SOFTRESET    0x0F

#define TAG_REQALL      0x52
#define TAG_ANTICOLL    0x93

#define BT_NOMBRE       "PanelHMI"

static uint8_t bt_tipo_addr;

static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

static const ble_uuid128_t chr_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

static const ble_uuid128_t chr_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static int gap_cb(struct ble_gap_event *ev, void *arg);

void setup_i2c(void)
{
    i2c_config_t disp_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_DISP,
        .scl_io_num = SCL_DISP,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_BUS_DISP, &disp_cfg);
    i2c_driver_install(I2C_BUS_DISP, disp_cfg.mode, 0, 0, 0);

    i2c_config_t clk_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_CLOCK,
        .scl_io_num = SCL_CLOCK,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_BUS_CLOCK, &clk_cfg);
    i2c_driver_install(I2C_BUS_CLOCK, clk_cfg.mode, 0, 0, 0);
}

void disp_raw(uint8_t val)
{
    i2c_master_write_to_device(I2C_BUS_DISP, ADDR_LCD, &val, 1, pdMS_TO_TICKS(100));
}

void disp_strobe(uint8_t val)
{
    disp_raw(val | ENABLE_BIT | BACKLIGHT_BIT);
    esp_rom_delay_us(1000);
    disp_raw((val & ~ENABLE_BIT) | BACKLIGHT_BIT);
    esp_rom_delay_us(1000);
}

void disp_nibble(uint8_t n, uint8_t modo)
{
    disp_strobe((n & 0xF0) | modo | BACKLIGHT_BIT);
}

void disp_byte(uint8_t b, uint8_t modo)
{
    disp_nibble(b & 0xF0, modo);
    disp_nibble((b << 4) & 0xF0, modo);
}

void disp_cmd(uint8_t cmd)
{
    disp_byte(cmd, 0);
}

void disp_chr(uint8_t c)
{
    disp_byte(c, RS_BIT);
}

void disp_setup(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    disp_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(10));
    disp_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(10));
    disp_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(10));
    disp_nibble(0x20, 0); vTaskDelay(pdMS_TO_TICKS(10));
    disp_cmd(0x28);       vTaskDelay(pdMS_TO_TICKS(5));
    disp_cmd(0x08);       vTaskDelay(pdMS_TO_TICKS(5));
    disp_cmd(0x01);       vTaskDelay(pdMS_TO_TICKS(5));
    disp_cmd(0x06);       vTaskDelay(pdMS_TO_TICKS(5));
    disp_cmd(0x0C);       vTaskDelay(pdMS_TO_TICKS(5));
}

void disp_limpiar(void)
{
    disp_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void disp_pos(uint8_t fila, uint8_t col)
{
    uint8_t base;
    switch (fila)
    {
        case 0:  base = 0x80 + col; break;
        case 1:  base = 0xC0 + col; break;
        case 2:  base = 0x94 + col; break;
        case 3:  base = 0xD4 + col; break;
        default: base = 0x80 + col; break;
    }
    disp_cmd(base);
}

void disp_str(const char *s)
{
    while (*s) disp_chr(*s++);
}

void disp_str16(const char *s)
{
    char tmp[17];
    for (int i = 0; i < 16; i++)
        tmp[i] = s[i] ? s[i] : ' ';
    tmp[16] = '\0';
    disp_str(tmp);
}

int bcd2dec(uint8_t v)  { return ((v >> 4) * 10) + (v & 0x0F); }
uint8_t dec2bcd(int v)  { return ((v / 10) << 4) | (v % 10); }

void hora_cero(char *b)
{
    b[0]='0'; b[1]='0'; b[2]=':';
    b[3]='0'; b[4]='0'; b[5]=':';
    b[6]='0'; b[7]='0'; b[8]='\0';
}

void rtc_set(int h, int m, int s)
{
    uint8_t d[4] = { 0x00, dec2bcd(s) & 0x7F, dec2bcd(m), dec2bcd(h) };
    i2c_master_write_to_device(I2C_BUS_CLOCK, ADDR_DS1307, d, 4, pdMS_TO_TICKS(100));
}

void rtc_init(void)
{
    uint8_t d[2] = { 0x00, 0x00 };
    i2c_master_write_to_device(I2C_BUS_CLOCK, ADDR_DS1307, d, 2, pdMS_TO_TICKS(100));
}

void rtc_get(char *b)
{
    uint8_t reg = 0x00;
    uint8_t d[3] = {0};

    esp_err_t r = i2c_master_write_read_device(
        I2C_BUS_CLOCK, ADDR_DS1307, &reg, 1, d, 3, pdMS_TO_TICKS(100));

    if (r != ESP_OK) { hora_cero(b); return; }

    int s = bcd2dec(d[0] & 0x7F);
    int m = bcd2dec(d[1]);
    int h = bcd2dec(d[2] & 0x3F);

    if (h > 23 || m > 59 || s > 59) { hora_cero(b); return; }

    b[0]='0'+(h/10); b[1]='0'+(h%10); b[2]=':';
    b[3]='0'+(m/10); b[4]='0'+(m%10); b[5]=':';
    b[6]='0'+(s/10); b[7]='0'+(s%10); b[8]='\0';
}

void pant_espera(void)
{
    disp_limpiar();
    disp_pos(0, 0); disp_str("Panel bloqueado");
    disp_pos(1, 0); disp_str("Acerque credencial");
}

void pant_ok(void)
{
    rtc_get(str_hora);
    disp_limpiar();
    disp_pos(0, 0); disp_str("Acceso concedido");
    disp_pos(1, 0); disp_str(str_hora);
}

void pant_error(void)
{
    disp_limpiar();
    disp_pos(0, 0); disp_str("Acceso denegado");
    disp_pos(1, 0); disp_str("UID no regist.");
}

void pant_activa(void)
{
    rtc_get(str_hora);
    disp_limpiar();
    disp_pos(0, 0); disp_str16(msg_display);
    disp_pos(1, 0); disp_str(str_hora);
}

void bz_setup(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = 2000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num   = SALIDA_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&c);
}

void bz_on(void)  { ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); }
void bz_off(void) { ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);   ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); }
void bz_beep(int ms) { bz_on(); vTaskDelay(pdMS_TO_TICKS(ms)); bz_off(); }

static void rfid_wr(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (reg << 1) & 0x7E, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_transmit(handle_rfid, &t);
}

static uint8_t rfid_rd(uint8_t reg)
{
    uint8_t tx[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx[2] = {0};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_transmit(handle_rfid, &t);
    return rx[1];
}

static void rfid_set(uint8_t reg, uint8_t mask) { rfid_wr(reg, rfid_rd(reg) | mask); }
static void rfid_clr(uint8_t reg, uint8_t mask) { rfid_wr(reg, rfid_rd(reg) & ~mask); }

static void rfid_reset(void)
{
    gpio_set_direction(RFID_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(RFID_RST, 0); vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(RFID_RST, 1); vTaskDelay(pdMS_TO_TICKS(50));
    rfid_wr(R_CMD, OP_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void rfid_antenna(void)
{
    if (!(rfid_rd(R_TX_CTRL) & 0x03))
        rfid_set(R_TX_CTRL, 0x03);
}

static void rfid_setup(void)
{
    rfid_reset();
    rfid_wr(R_TMODE,      0x8D);
    rfid_wr(R_TPRESCALER, 0x3E);
    rfid_wr(R_TRELOAD_L,  30);
    rfid_wr(R_TRELOAD_H,  0);
    rfid_wr(R_TX_ASK,     0x40);
    rfid_wr(R_MODE,       0x3D);
    rfid_antenna();
}

static int rfid_comm(uint8_t cmd, uint8_t *tx_buf, uint8_t tx_len,
                     uint8_t *rx_buf, uint16_t *rx_len)
{
    uint8_t wait_irq = (cmd == OP_TRANSCEIVE) ? 0x30 : 0x00;
    uint8_t n;
    uint16_t i;

    rfid_wr(R_IRQ,      0x7F);
    rfid_set(R_FIFO_LVL, 0x80);
    rfid_wr(R_CMD,       OP_IDLE);

    for (i = 0; i < tx_len; i++) rfid_wr(R_FIFO_DATA, tx_buf[i]);
    rfid_wr(R_CMD, cmd);

    if (cmd == OP_TRANSCEIVE) rfid_set(R_BIT_FRAME, 0x80);

    i = 5000;
    do { n = rfid_rd(R_IRQ); i--; }
    while (i && !(n & 0x01) && !(n & wait_irq));

    rfid_clr(R_BIT_FRAME, 0x80);
    if (!i || (rfid_rd(R_ERR) & 0x1B)) return 0;

    if (cmd == OP_TRANSCEIVE)
    {
        n = rfid_rd(R_FIFO_LVL);
        uint8_t ult = rfid_rd(R_CTRL) & 0x07;
        *rx_len = ult ? (n - 1) * 8 + ult : n * 8;
        if (n > 16) n = 16;
        for (i = 0; i < n; i++) rx_buf[i] = rfid_rd(R_FIFO_DATA);
    }
    return 1;
}

static int rfid_req(uint8_t modo, uint8_t *tipo)
{
    uint16_t bits;
    uint8_t b[1] = { modo };
    rfid_wr(R_BIT_FRAME, 0x07);
    return (rfid_comm(OP_TRANSCEIVE, b, 1, tipo, &bits) == 1 && bits == 0x10);
}

static int rfid_anticoll(uint8_t *sn)
{
    uint16_t lon;
    uint8_t b[2] = { TAG_ANTICOLL, 0x20 };
    rfid_wr(R_BIT_FRAME, 0x00);

    if (!rfid_comm(OP_TRANSCEIVE, b, 2, sn, &lon)) return 0;

    uint8_t chk = 0;
    for (int i = 0; i < 4; i++) chk ^= sn[i];
    return (chk == sn[4]);
}

int uid_valido(uint8_t *uid)
{
    for (int i = 0; i < 4; i++)
        if (uid[i] != uid_permitido[i]) return 0;
    return 1;
}

void permitir_entrada(void)
{
    printf("Acceso concedido\n");
    pant_ok();
    gpio_set_level(SALIDA_ROJA, 0);
    gpio_set_level(SALIDA_AZUL, 0);
    gpio_set_level(SALIDA_VERDE, 1);
    bz_beep(500);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(SALIDA_VERDE, 0);
    gpio_set_level(SALIDA_AZUL, 1);
    estado_panel = 1;
    strcpy(msg_display, "Sin mensajes");
    flag_lcd_bt = 1;
}

void denegar_entrada(void)
{
    printf("Acceso denegado\n");
    pant_error();
    gpio_set_level(SALIDA_VERDE, 0);
    gpio_set_level(SALIDA_AZUL, 0);
    bz_on();
    for (int i = 0; i < 3; i++)
    {
        gpio_set_level(SALIDA_ROJA, 1); vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(SALIDA_ROJA, 0); vTaskDelay(pdMS_TO_TICKS(300));
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    bz_off();
    gpio_set_level(SALIDA_ROJA, 1);
    estado_panel = 0;
    vTaskDelay(pdMS_TO_TICKS(1000));
    pant_espera();
}

void terminar_sesion(void)
{
    printf("Cerrando sesion\n");
    disp_limpiar();
    disp_pos(0, 0); disp_str("Cerrando sesion");
    disp_pos(1, 0); disp_str("Espere...");
    bz_beep(500);
    gpio_set_level(SALIDA_AZUL,  0);
    gpio_set_level(SALIDA_VERDE, 0);
    gpio_set_level(SALIDA_ROJA,  1);
    estado_panel = 0;
    strcpy(msg_display, "Sin mensajes");
    flag_lcd_bt = 0;
    vTaskDelay(pdMS_TO_TICKS(500));
    pant_espera();
}

void setup_gpio(void)
{
    gpio_config_t g = {
        .pin_bit_mask = (1ULL << SALIDA_ROJA)  |
                        (1ULL << SALIDA_VERDE)  |
                        (1ULL << SALIDA_AZUL),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&g);
    gpio_set_level(SALIDA_ROJA,  1);
    gpio_set_level(SALIDA_VERDE, 0);
    gpio_set_level(SALIDA_AZUL,  0);
}

static int cb_ble_rx(uint16_t ch, uint16_t ah,
                     struct ble_gatt_access_ctxt *ctx, void *arg)
{
    char tmp[64];
    uint16_t lon = OS_MBUF_PKTLEN(ctx->om);
    if (lon >= sizeof(tmp)) lon = sizeof(tmp) - 1;
    memset(tmp, 0, sizeof(tmp));
    os_mbuf_copydata(ctx->om, 0, lon, tmp);
    tmp[lon] = '\0';

    if (estado_panel)
    {
        memset(msg_display, 0, sizeof(msg_display));
        int n = strlen(tmp);
        if (n > 16) n = 16;
        strncpy(msg_display, tmp, n);
        flag_lcd_bt = 1;
        printf("Mensaje BLE recibido: %s\n", msg_display);
    }
    else
    {
        printf("Mensaje BLE ignorado porque el sistema esta bloqueado\n");
    }
    return 0;
}

static int cb_ble_tx(uint16_t ch, uint16_t ah,
                     struct ble_gatt_access_ctxt *ctx, void *arg) { return 0; }

static const struct ble_gatt_svc_def tabla_gatt[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = &chr_rx_uuid.u, .access_cb = cb_ble_rx,
              .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP },
            { .uuid = &chr_tx_uuid.u, .access_cb = cb_ble_tx,
              .flags = BLE_GATT_CHR_F_NOTIFY },
            {0}
        },
    },
    {0}
};

static void bt_broadcast(void)
{
    struct ble_gap_adv_params ap;
    struct ble_hs_adv_fields  af;
    memset(&af, 0, sizeof(af));
    af.flags            = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    af.name             = (uint8_t *)BT_NOMBRE;
    af.name_len         = strlen(BT_NOMBRE);
    af.name_is_complete = 1;
    ble_gap_adv_set_fields(&af);
    memset(&ap, 0, sizeof(ap));
    ap.conn_mode = BLE_GAP_CONN_MODE_UND;
    ap.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(bt_tipo_addr, NULL, BLE_HS_FOREVER, &ap, gap_cb, NULL);
}

static int gap_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            if (ev->connect.status == 0) printf("BLE conectado\n");
            else { printf("Fallo conexion BLE\n"); bt_broadcast(); }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            printf("BLE desconectado\n"); bt_broadcast(); break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            bt_broadcast(); break;
        default: break;
    }
    return 0;
}

static void bt_ready(void)
{
    ble_hs_id_infer_auto(0, &bt_tipo_addr);
    bt_broadcast();
}

void tarea_ble(void *p) { nimble_port_run(); nimble_port_freertos_deinit(); }

void bt_setup(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND)
    { nvs_flash_erase(); nvs_flash_init(); }

    nimble_port_init();
    ble_svc_gap_device_name_set(BT_NOMBRE);
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(tabla_gatt);
    ble_gatts_add_svcs(tabla_gatt);
    ble_hs_cfg.sync_cb = bt_ready;
    nimble_port_freertos_init(tarea_ble);
    printf("BLE iniciado como %s\n", BT_NOMBRE);
}

void app_main(void)
{
    printf("Iniciando sistema con LCD + RFID + BLE + RTC...\n");

    setup_gpio();
    bz_setup();
    setup_i2c();
    disp_setup();

    rtc_init();
    rtc_set(16, 23, 0);

    estado_panel = 0;
    pant_espera();

    bt_setup();

    spi_bus_config_t spi_bus = {
        .miso_io_num  = SPI_MISO,
        .mosi_io_num  = SPI_MOSI,
        .sclk_io_num  = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t spi_dev = {
        .clock_speed_hz = 500000,
        .mode           = 0,
        .spics_io_num   = SPI_SS,
        .queue_size     = 7
    };

    spi_bus_initialize(SPI_BUS, &spi_bus, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI_BUS, &spi_dev, &handle_rfid);

    rfid_setup();

    uint8_t ver = rfid_rd(R_VER);
    printf("Version RC522: 0x%02X\n", ver);

    if (ver == 0x00 || ver == 0xFF)
    {
        printf("No se detecta el RC522. Revisa conexiones SPI.\n");
        disp_limpiar();
        disp_pos(0, 0); disp_str("Error RC522");
        disp_pos(1, 0); disp_str("Revise cables");
    }
    else
    {
        printf("RC522 detectado correctamente.\n");
        printf("Acerque una tarjeta...\n");
    }

    while (1)
    {
        int64_t ts = esp_timer_get_time() / 1000;

        if (estado_panel)
        {
            gpio_set_level(SALIDA_AZUL, 1);

            if (flag_lcd_bt)
            {
                flag_lcd_bt = 0;
                pant_activa();
                ts_ultima_hora = ts;
            }

            if ((ts - ts_ultima_hora) >= 1000)
            {
                ts_ultima_hora = ts;
                pant_activa();
            }
        }

        uint8_t tipo_tag[2];
        uint8_t uid_leido[5];

        if ((ts - ts_ultima_rfid) > COOLDOWN_RFID_MS)
        {
            if (rfid_req(TAG_REQALL, tipo_tag))
            {
                if (rfid_anticoll(uid_leido))
                {
                    ts_ultima_rfid = ts;

                    printf("Tarjeta detectada. UID: ");
                    for (int i = 0; i < 4; i++) printf("%02X ", uid_leido[i]);
                    printf("\n");

                    disp_limpiar();
                    disp_pos(0, 0); disp_str("UID detectado");
                    disp_pos(1, 0);

                    char hex_uid[17];
                    const char HEX[] = "0123456789ABCDEF";
                    hex_uid[0]  = HEX[(uid_leido[0] >> 4) & 0x0F];
                    hex_uid[1]  = HEX[uid_leido[0] & 0x0F];
                    hex_uid[2]  = ' ';
                    hex_uid[3]  = HEX[(uid_leido[1] >> 4) & 0x0F];
                    hex_uid[4]  = HEX[uid_leido[1] & 0x0F];
                    hex_uid[5]  = ' ';
                    hex_uid[6]  = HEX[(uid_leido[2] >> 4) & 0x0F];
                    hex_uid[7]  = HEX[uid_leido[2] & 0x0F];
                    hex_uid[8]  = ' ';
                    hex_uid[9]  = HEX[(uid_leido[3] >> 4) & 0x0F];
                    hex_uid[10] = HEX[uid_leido[3] & 0x0F];
                    hex_uid[11] = '\0';
                    disp_str(hex_uid);
                    vTaskDelay(pdMS_TO_TICKS(700));

                    if (uid_valido(uid_leido))
                    {
                        if (!estado_panel) permitir_entrada();
                        else              terminar_sesion();
                    }
                    else
                    {
                        if (!estado_panel)
                        {
                            denegar_entrada();
                        }
                        else
                        {
                            printf("UID no autorizado ignorado en estado activo\n");
                            pant_activa();
                            gpio_set_level(SALIDA_AZUL, 1);
                        }
                    }

                    vTaskDelay(pdMS_TO_TICKS(1500));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
