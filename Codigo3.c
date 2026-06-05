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

// CONFIGURACIÓN DE PANTALLA LCD Y RELOJ RTC (BUSES SEPARADOS)
#define DISP_ADDR    0x27
#define RTC_CHIP_ADDR 0x68

// Bus I2C 0: Dedicado exclusivamente a la LCD
#define PIN_SDA_DISP     13
#define PIN_SCL_DISP     16
#define BUS_I2C_DISP     I2C_NUM_0

// Bus I2C 1: Dedicado exclusivamente al RTC DS1307
#define PIN_SDA_RTC      21
#define PIN_SCL_RTC      22
#define BUS_I2C_RTC      I2C_NUM_1

#define DISP_LUZ      0x08
#define DISP_ENABLE   0x04
#define DISP_RS       0x01


// RC522 SPI

#define RC_MISO  27
#define RC_MOSI  26
#define RC_SCK   32
#define RC_CS    12
#define RC_RST   14

#define BUS_SPI_USADO SPI2_HOST

static spi_device_handle_t handle_spi_rc;

// LEDs y buzzer
#define PIN_LED_VERDE GPIO_NUM_5
#define PIN_LED_ROJO  GPIO_NUM_4
#define PIN_LED_AZUL  GPIO_NUM_15
#define PIN_BUZZER    GPIO_NUM_18

// Estado del sistema
volatile int panel_activo = 0;
volatile int refrescar_disp_ble = 0;

char msg_pantalla[21] = "Sin mensajes";
char str_hora[9] = "00:00:00";

int64_t ts_ultima_rfid = 0;
int64_t ts_ultima_hora = 0;

#define RFID_ESPERA_MS 4000

// UID autorizado: TARJETA PERMITIDA
uint8_t uid_valido[4] = {0x5A, 0xD5, 0xFA, 0x03};

// Registros RC522
#define REG_CMD         0x01
#define REG_IRQ         0x04
#define REG_ERR         0x06
#define REG_FIFO_DATA   0x09
#define REG_FIFO_NIVEL  0x0A
#define REG_CTRL        0x0C
#define REG_FRAMING     0x0D
#define REG_MODO        0x11
#define REG_TX_CTRL     0x14
#define REG_TX_ASK      0x15
#define REG_TMODE       0x2A
#define REG_TPRESCALER  0x2B
#define REG_TRELOAD_H   0x2C
#define REG_TRELOAD_L   0x2D
#define REG_VERSION     0x37

#define CMD_IDLE        0x00
#define CMD_TRANSCEIVE  0x0C
#define CMD_SOFTRESET   0x0F

#define PICC_REQ_ALL    0x52
#define PICC_ANTI       0x93

// BLE NUS
#define NOMBRE_DISP "PanelHMI"

static uint8_t tipo_addr_ble;

static const ble_uuid128_t uuid_svc_nus =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

static const ble_uuid128_t uuid_chr_rx =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

static const ble_uuid128_t uuid_chr_tx =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static int cb_gap_event(struct ble_gap_event *event, void *arg);

// LCD INDEPENDIENTE (Puerto 0)
void iniciar_i2c(void)
{
    // 1. Configurar Bus 0 para Pantalla LCD
    i2c_config_t cfg_disp = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA_DISP,
        .scl_io_num = PIN_SCL_DISP,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(BUS_I2C_DISP, &cfg_disp);
    i2c_driver_install(BUS_I2C_DISP, cfg_disp.mode, 0, 0, 0);

    // 2. Configurar Bus 1 para Reloj RTC
    i2c_config_t cfg_rtc = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA_RTC,
        .scl_io_num = PIN_SCL_RTC,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(BUS_I2C_RTC, &cfg_rtc);
    i2c_driver_install(BUS_I2C_RTC, cfg_rtc.mode, 0, 0, 0);
}

void disp_enviar_byte(uint8_t dato)
{
    i2c_master_write_to_device(BUS_I2C_DISP, DISP_ADDR, &dato, 1, pdMS_TO_TICKS(100));
}

void disp_pulso_enable(uint8_t dato)
{
    disp_enviar_byte(dato | DISP_ENABLE | DISP_LUZ);
    esp_rom_delay_us(1000);

    disp_enviar_byte((dato & ~DISP_ENABLE) | DISP_LUZ);
    esp_rom_delay_us(1000);
}

void disp_nibble(uint8_t nibble, uint8_t modo)
{
    uint8_t dato = (nibble & 0xF0) | modo | DISP_LUZ;
    disp_pulso_enable(dato);
}

void disp_byte(uint8_t valor, uint8_t modo)
{
    disp_nibble(valor & 0xF0, modo);
    disp_nibble((valor << 4) & 0xF0, modo);
}

void disp_cmd(uint8_t cmd)
{
    disp_byte(cmd, 0);
}

void disp_char(uint8_t c)
{
    disp_byte(c, DISP_RS);
}

void disp_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    disp_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    disp_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    disp_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    disp_nibble(0x20, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    disp_cmd(0x28);
    vTaskDelay(pdMS_TO_TICKS(5));

    disp_cmd(0x08);
    vTaskDelay(pdMS_TO_TICKS(5));

    disp_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));

    disp_cmd(0x06);
    vTaskDelay(pdMS_TO_TICKS(5));

    disp_cmd(0x0C);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void disp_limpiar(void)
{
    disp_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void disp_cursor(uint8_t fila, uint8_t col)
{
    uint8_t pos;

    switch(fila)
    {
        case 0:  pos = 0x80 + col; break;
        case 1:  pos = 0xC0 + col; break;
        case 2:  pos = 0x94 + col; break;
        case 3:  pos = 0xD4 + col; break;
        default: pos = 0x80 + col; break;
    }
    disp_cmd(pos);
}

void disp_texto(const char *str)
{
    while (*str)
    {
        disp_char(*str++);
    }
}

void disp_texto_16(const char *str)
{
    char buf[17];

    for (int i = 0; i < 16; i++)
    {
        if (str[i] != '\0')
            buf[i] = str[i];
        else
            buf[i] = ' ';
    }

    buf[16] = '\0';
    disp_texto(buf);
}

// RTC DS1307 INDEPENDIENTE (Puerto 1)
int bcd_a_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

uint8_t dec_a_bcd(int val)
{
    return ((val / 10) << 4) | (val % 10);
}

void hora_por_defecto(char *buf)
{
    buf[0] = '0'; buf[1] = '0'; buf[2] = ':';
    buf[3] = '0'; buf[4] = '0'; buf[5] = ':';
    buf[6] = '0'; buf[7] = '0'; buf[8] = '\0';
}

void rtc_fijar_hora(int hora, int min, int seg)
{
    uint8_t trama[4];

    trama[0] = 0x00;
    trama[1] = dec_a_bcd(seg) & 0x7F;
    trama[2] = dec_a_bcd(min);
    trama[3] = dec_a_bcd(hora);

    i2c_master_write_to_device(BUS_I2C_RTC, RTC_CHIP_ADDR, trama, 4, pdMS_TO_TICKS(100));
}

// Arranque del RTC
void rtc_arrancar(void)
{
    uint8_t trama[2];
    trama[0] = 0x00;
    trama[1] = 0x00;

    i2c_master_write_to_device(BUS_I2C_RTC, RTC_CHIP_ADDR, trama, 2, pdMS_TO_TICKS(100));
}

void rtc_leer_hora(char *buf)
{
    uint8_t reg = 0x00;
    uint8_t datos[3] = {0};

    esp_err_t res = i2c_master_write_read_device(
        BUS_I2C_RTC,
        RTC_CHIP_ADDR,
        &reg,
        1,
        datos,
        3,
        pdMS_TO_TICKS(100)
    );

    if (res != ESP_OK)
    {
        hora_por_defecto(buf);
        return;
    }

    int seg  = bcd_a_dec(datos[0] & 0x7F);
    int min  = bcd_a_dec(datos[1]);
    int hora = bcd_a_dec(datos[2] & 0x3F);

    if (hora > 23 || min > 59 || seg > 59)
    {
        hora_por_defecto(buf);
        return;
    }

    buf[0] = '0' + (hora / 10);
    buf[1] = '0' + (hora % 10);
    buf[2] = ':';
    buf[3] = '0' + (min / 10);
    buf[4] = '0' + (min % 10);
    buf[5] = ':';
    buf[6] = '0' + (seg / 10);
    buf[7] = '0' + (seg % 10);
    buf[8] = '\0';
}


// Pantallas LCD
void pantalla_bloqueado(void)
{
    disp_limpiar();
    disp_cursor(0, 0);
    disp_texto("Panel bloqueado");

    disp_cursor(1, 0);
    disp_texto("Acerque credencial");
}

void pantalla_concedido(void)
{
    rtc_leer_hora(str_hora);

    disp_limpiar();
    disp_cursor(0, 0);
    disp_texto("Acceso concedido");

    disp_cursor(1, 0);
    disp_texto(str_hora);
}

void pantalla_denegado(void)
{
    disp_limpiar();
    disp_cursor(0, 0);
    disp_texto("Acceso denegado");

    disp_cursor(1, 0);
    disp_texto("UID no regist.");
}

void pantalla_activo(void)
{
    rtc_leer_hora(str_hora);

    disp_limpiar();

    disp_cursor(0, 0);
    disp_texto_16(msg_pantalla);

    disp_cursor(1, 0);
    disp_texto(str_hora);
}

// Buzzer PWM
void buzzer_setup(void)
{
    ledc_timer_config_t tmr = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&tmr);

    ledc_channel_config_t canal = {
        .gpio_num = PIN_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&canal);
}

void buzzer_encender(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_apagar(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_pitido(int ms)
{
    buzzer_encender();
    vTaskDelay(pdMS_TO_TICKS(ms));
    buzzer_apagar();
}

// RC522
static void rc_escribir(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {
        (reg << 1) & 0x7E,
        val
    };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx
    };

    spi_device_transmit(handle_spi_rc, &t);
}

static uint8_t rc_leer(uint8_t reg)
{
    uint8_t tx[2] = {
        ((reg << 1) & 0x7E) | 0x80,
        0x00
    };

    uint8_t rx[2] = {0};

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(handle_spi_rc, &t);

    return rx[1];
}

static void rc_set_bits(uint8_t reg, uint8_t mask)
{
    uint8_t tmp = rc_leer(reg);
    rc_escribir(reg, tmp | mask);
}

static void rc_clear_bits(uint8_t reg, uint8_t mask)
{
    uint8_t tmp = rc_leer(reg);
    rc_escribir(reg, tmp & (~mask));
}

static void rc_reset(void)
{
    gpio_set_direction(RC_RST, GPIO_MODE_OUTPUT);

    gpio_set_level(RC_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(RC_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    rc_escribir(REG_CMD, CMD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void rc_antena_on(void)
{
    uint8_t tmp = rc_leer(REG_TX_CTRL);

    if (!(tmp & 0x03))
    {
        rc_set_bits(REG_TX_CTRL, 0x03);
    }
}

static void rc_init(void)
{
    rc_reset();

    rc_escribir(REG_TMODE, 0x8D);
    rc_escribir(REG_TPRESCALER, 0x3E);
    rc_escribir(REG_TRELOAD_L, 30);
    rc_escribir(REG_TRELOAD_H, 0);

    rc_escribir(REG_TX_ASK, 0x40);
    rc_escribir(REG_MODO, 0x3D);

    rc_antena_on();
}

static int rc_transceive(uint8_t cmd, uint8_t *txBuf, uint8_t txLen,
                         uint8_t *rxBuf, uint16_t *rxLen)
{
    uint8_t irqWait = 0x00;
    uint8_t n;
    uint16_t i;

    if (cmd == CMD_TRANSCEIVE)
    {
        irqWait = 0x30;
    }

    rc_escribir(REG_IRQ, 0x7F);
    rc_set_bits(REG_FIFO_NIVEL, 0x80);
    rc_escribir(REG_CMD, CMD_IDLE);

    for (i = 0; i < txLen; i++)
    {
        rc_escribir(REG_FIFO_DATA, txBuf[i]);
    }
    rc_escribir(REG_CMD, cmd);

    if (cmd == CMD_TRANSCEIVE)
    {
        rc_set_bits(REG_FRAMING, 0x80);
    }

    i = 5000;

    do
    {
        n = rc_leer(REG_IRQ);
        i--;
    }
    while ((i != 0) && !(n & 0x01) && !(n & irqWait));

    rc_clear_bits(REG_FRAMING, 0x80);

    if (i == 0)
    {
        return 0;
    }

    if (rc_leer(REG_ERR) & 0x1B)
    {
        return 0;
    }

    if (cmd == CMD_TRANSCEIVE)
    {
        n = rc_leer(REG_FIFO_NIVEL);
        uint8_t ultBits = rc_leer(REG_CTRL) & 0x07;

        if (ultBits)
            *rxLen = (n - 1) * 8 + ultBits;
        else
            *rxLen = n * 8;

        if (n > 16)
            n = 16;

        for (i = 0; i < n; i++)
        {
            rxBuf[i] = rc_leer(REG_FIFO_DATA);
        }
    }

    return 1;
}

static int rc_solicitar(uint8_t modo, uint8_t *tipoTag)
{
    uint16_t bitsRx;
    uint8_t buf[1];

    buf[0] = modo;

    rc_escribir(REG_FRAMING, 0x07);

    int ok = rc_transceive(CMD_TRANSCEIVE, buf, 1, tipoTag, &bitsRx);

    if ((ok != 1) || (bitsRx != 0x10))
    {
        return 0;
    }

    return 1;
}

static int rc_anticol(uint8_t *uid)
{
    uint16_t longRx;
    uint8_t buf[2];

    buf[0] = PICC_ANTI;
    buf[1] = 0x20;

    rc_escribir(REG_FRAMING, 0x00);

    int ok = rc_transceive(CMD_TRANSCEIVE, buf, 2, uid, &longRx);

    if (ok)
    {
        uint8_t chk = 0;

        for (int i = 0; i < 4; i++)
        {
            chk ^= uid[i];
        }

        if (chk != uid[4])
        {
            return 0;
        }

        return 1;
    }

    return 0;
}

// Accesos
int uid_permitido(uint8_t *uid)
{
    for (int i = 0; i < 4; i++)
    {
        if (uid[i] != uid_valido[i])
        {
            return 0;
        }
    }

    return 1;
}

void accion_concedido(void)
{
    printf("Acceso concedido\n");

    pantalla_concedido();

    gpio_set_level(PIN_LED_ROJO, 0);
    gpio_set_level(PIN_LED_AZUL, 0);
    gpio_set_level(PIN_LED_VERDE, 1);

    buzzer_pitido(500);

    vTaskDelay(pdMS_TO_TICKS(1000));

    gpio_set_level(PIN_LED_VERDE, 0);
    gpio_set_level(PIN_LED_AZUL, 1);

    panel_activo = 1;

    strcpy(msg_pantalla, "Sin mensajes");
    refrescar_disp_ble = 1;
}

void accion_denegado(void)
{
    printf("Acceso denegado\n");

    pantalla_denegado();

    gpio_set_level(PIN_LED_VERDE, 0);
    gpio_set_level(PIN_LED_AZUL, 0);

    buzzer_encender();

    for (int i = 0; i < 3; i++)
    {
        gpio_set_level(PIN_LED_ROJO, 1);
        vTaskDelay(pdMS_TO_TICKS(300));

        gpio_set_level(PIN_LED_ROJO, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_apagar();

    gpio_set_level(PIN_LED_ROJO, 1);

    panel_activo = 0;

    vTaskDelay(pdMS_TO_TICKS(1000));
    pantalla_bloqueado();
}


void cerrar_sesion(void)
{
    printf("Cerrando sesion\n");

    disp_limpiar();
    disp_cursor(0, 0);
    disp_texto("Cerrando sesion");

    disp_cursor(1, 0);
    disp_texto("Espere...");

    buzzer_pitido(500);

    gpio_set_level(PIN_LED_AZUL, 0);
    gpio_set_level(PIN_LED_VERDE, 0);
    gpio_set_level(PIN_LED_ROJO, 1);

    panel_activo = 0;

    strcpy(msg_pantalla, "Sin mensajes");
    refrescar_disp_ble = 0;

    vTaskDelay(pdMS_TO_TICKS(500));
    pantalla_bloqueado();
}

void setup_salidas(void)
{
    gpio_config_t cfg_gpio = {
        .pin_bit_mask = (1ULL << PIN_LED_ROJO) |
                        (1ULL << PIN_LED_VERDE) |
                        (1ULL << PIN_LED_AZUL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&cfg_gpio);

    gpio_set_level(PIN_LED_ROJO, 1);
    gpio_set_level(PIN_LED_VERDE, 0);
    gpio_set_level(PIN_LED_AZUL, 0);
}

// BLE
static int cb_nus_rx(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char buf[64];
    uint16_t len_om = OS_MBUF_PKTLEN(ctxt->om);

    if (len_om >= sizeof(buf))
        len_om = sizeof(buf) - 1;

    memset(buf, 0, sizeof(buf));
    os_mbuf_copydata(ctxt->om, 0, len_om, buf);
    buf[len_om] = '\0';

    if (panel_activo == 1)
    {
        memset(msg_pantalla, 0, sizeof(msg_pantalla));

        int lon = strlen(buf);
        if (lon > 16)
            lon = 16;

        strncpy(msg_pantalla, buf, lon);
        msg_pantalla[lon] = '\0';

        refrescar_disp_ble = 1;

        printf("Mensaje BLE recibido: %s\n", msg_pantalla);
    }
    else
    {
        printf("Mensaje BLE ignorado porque el sistema esta bloqueado\n");
    }

    return 0;
}

static int cb_nus_tx(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;
}

static const struct ble_gatt_svc_def tabla_gatt[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_nus.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &uuid_chr_rx.u,
                .access_cb = cb_nus_rx,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &uuid_chr_tx.u,
                .access_cb = cb_nus_tx,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0}
        },
    },
    {0}
};

static void ble_iniciar_adv(void)
{
    struct ble_gap_adv_params params_adv;
    struct ble_hs_adv_fields campos;

    memset(&campos, 0, sizeof(campos));

    campos.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    campos.name = (uint8_t *)NOMBRE_DISP;
    campos.name_len = strlen(NOMBRE_DISP);
    campos.name_is_complete = 1;

    ble_gap_adv_set_fields(&campos);

    memset(&params_adv, 0, sizeof(params_adv));

    params_adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    params_adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(tipo_addr_ble, NULL, BLE_HS_FOREVER,
                      &params_adv, cb_gap_event, NULL);
}

static int cb_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0)
            {
                printf("BLE conectado\n");
            }
            else
            {
                printf("Fallo conexion BLE\n");
                ble_iniciar_adv();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf("BLE desconectado\n");
            ble_iniciar_adv();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_iniciar_adv();
            break;

        default:
            break;
    }

    return 0;
}

static void ble_sync_cb(void)
{
    ble_hs_id_infer_auto(0, &tipo_addr_ble);
    ble_iniciar_adv();
}

void tarea_ble_host(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_setup(void)
{
    esp_err_t res = nvs_flash_init();

    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    nimble_port_init();

    ble_svc_gap_device_name_set(NOMBRE_DISP);

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(tabla_gatt);
    ble_gatts_add_svcs(tabla_gatt);

    ble_hs_cfg.sync_cb = ble_sync_cb;

    nimble_port_freertos_init(tarea_ble_host);

    printf("BLE iniciado como %s\n", NOMBRE_DISP);
}


// Main
void app_main(void)
{
    printf("Iniciando sistema con LCD + RFID + BLE + RTC...\n");

    setup_salidas();
    buzzer_setup();

    // Inicializa ambos puertos I2C (Bus 0 para LCD, Bus 1 para RTC)
    iniciar_i2c();
    disp_init();

    // INICIO FORZADO DEL RTC
    rtc_arrancar();
    rtc_fijar_hora(16, 23, 0); //Ajuste de hora para que sea a la hora que el profe quiera, recuerda montarlo asi y despues comentarlo.

    panel_activo = 0;
    pantalla_bloqueado();

    ble_setup();

    spi_bus_config_t cfg_spi_bus = {
        .miso_io_num = RC_MISO,
        .mosi_io_num = RC_MOSI,
        .sclk_io_num = RC_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t cfg_spi_dev = {
        .clock_speed_hz = 500000,
        .mode = 0,
        .spics_io_num = RC_CS,
        .queue_size = 7
    };

    spi_bus_initialize(BUS_SPI_USADO, &cfg_spi_bus, SPI_DMA_CH_AUTO);
    spi_bus_add_device(BUS_SPI_USADO, &cfg_spi_dev, &handle_spi_rc);

    rc_init();

    uint8_t ver = rc_leer(REG_VERSION);

    printf("Version RC522: 0x%02X\n", ver);

    if (ver == 0x00 || ver == 0xFF)
    {
        printf("No se detecta el RC522. Revisa conexiones SPI.\n");

        disp_limpiar();
        disp_cursor(0, 0);
        disp_texto("Error RC522");

        disp_cursor(1, 0);
        disp_texto("Revise cables");
    }
    else
    {
        printf("RC522 detectado correctamente.\n");
        printf("Acerque una tarjeta...\n");
    }

    while (1)
    {
        int64_t ahora_ms = esp_timer_get_time() / 1000;

        if (panel_activo == 1)
        {
            gpio_set_level(PIN_LED_AZUL, 1);

            if (refrescar_disp_ble == 1)
            {
                refrescar_disp_ble = 0;
                pantalla_activo();
                ts_ultima_hora = ahora_ms;
            }

            if ((ahora_ms - ts_ultima_hora) >= 1000)
            {
                ts_ultima_hora = ahora_ms;
                pantalla_activo();
            }
        }

        uint8_t tipo_tag[2];
        uint8_t uid_leido[5];

        if ((ahora_ms - ts_ultima_rfid) > RFID_ESPERA_MS)
        {
            if (rc_solicitar(PICC_REQ_ALL, tipo_tag))
            {
                if (rc_anticol(uid_leido))
                {
                    ts_ultima_rfid = ahora_ms;

                    printf("Tarjeta detectada. UID: ");

                    for (int i = 0; i < 4; i++)
                    {
                        printf("%02X ", uid_leido[i]);
                    }

                    printf("\n");

                    disp_limpiar();
                    disp_cursor(0, 0);
                    disp_texto("UID detectado");

                    disp_cursor(1, 0);

                    char uid_str[17];
                    const char hex[] = "0123456789ABCDEF";

                    uid_str[0]  = hex[(uid_leido[0] >> 4) & 0x0F];
                    uid_str[1]  = hex[uid_leido[0] & 0x0F];
                    uid_str[2]  = ' ';
                    uid_str[3]  = hex[(uid_leido[1] >> 4) & 0x0F];
                    uid_str[4]  = hex[uid_leido[1] & 0x0F];
                    uid_str[5]  = ' ';
                    uid_str[6]  = hex[(uid_leido[2] >> 4) & 0x0F];
                    uid_str[7]  = hex[uid_leido[2] & 0x0F];
                    uid_str[8]  = ' ';
                    uid_str[9]  = hex[(uid_leido[3] >> 4) & 0x0F];
                    uid_str[10] = hex[uid_leido[3] & 0x0F];
                    uid_str[11] = '\0';

                    disp_texto(uid_str);

                    vTaskDelay(pdMS_TO_TICKS(700));

                    if (uid_permitido(uid_leido))
                    {
                        if (panel_activo == 0)
                        {
                            accion_concedido();
                        }
                        else
                        {
                            cerrar_sesion();
                        }
                    }
                    else
                    {
                        if (panel_activo == 0)
                        {
                            accion_denegado();
                        }
                        else
                        {
                            printf("UID no autorizado ignorado en estado activo\n");
                            pantalla_activo();
                            gpio_set_level(PIN_LED_AZUL, 1);
                        }
                    }

                    vTaskDelay(pdMS_TO_TICKS(1500));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}