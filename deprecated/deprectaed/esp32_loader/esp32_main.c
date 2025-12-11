// Sdkconfig-
// CONFIG_WIFI_AP_SSID="LPC4330_UPDATER"
// CONFIG_BUILD_SERVER_URL="http://192.168.4.2:5000/build"
// CONFIG_LPC4330_RST_GPIO=19


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "nvs_flash.h"

static const char *TAG = "LPC_UPDATER";

// GPIO for holding LPC4330 in reset
#define LPC4330_RST_GPIO  CONFIG_LPC4330_RST_GPIO

// QSPI flash pins & host
#define SPI_HOST_ID   HSPI_HOST
#define SPI_MISO_IO   12
#define SPI_MOSI_IO   13
#define SPI_SCLK_IO   14
#define SPI_CS_IO     15

// build-server URL from sdkconfig
#define BUILD_SERVER_URL  CONFIG_BUILD_SERVER_URL

static spi_device_handle_t flash_dev;

// ——— QSPI Flash Helpers ———
static void flash_wait_ready() {
    uint8_t cmd = 0x05, status = 0;
    spi_transaction_t t = { .length=8, .tx_buffer=&cmd, .rxlength=8, .rx_buffer=&status };
    do { spi_device_transmit(flash_dev, &t); } while (status & 0x01);
}
static void flash_write_enable() {
    uint8_t cmd = 0x06;
    spi_transaction_t t = { .length=8, .tx_buffer=&cmd };
    spi_device_transmit(flash_dev, &t);
}
static void flash_chip_erase() {
    flash_write_enable();
    uint8_t cmd = 0xC7;
    spi_transaction_t t = { .length=8, .tx_buffer=&cmd };
    spi_device_transmit(flash_dev, &t);
    flash_wait_ready();
}
static bool flash_page_program(uint32_t addr, const uint8_t *data, size_t len) {
    if (len > 256) return false;
    flash_write_enable();
    uint8_t cmd_addr[4] = {
        0x02,
        (addr>>16)&0xFF, (addr>>8)&0xFF, addr&0xFF
    };
    spi_transaction_t t = {
        .length = (4 + len)*8,
        .tx_buffer = cmd_addr,
        .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
        .cmd_bits = 0,
        .addr_bits = 0
    };
    // send opcode+addr
    spi_device_transmit(flash_dev, &t);
    // send data
    spi_transaction_t t2 = { .length = len*8, .tx_buffer = data };
    spi_device_transmit(flash_dev, &t2);
    flash_wait_ready();
    return true;
}
static void spi_flash_init() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_IO,
        .miso_io_num = SPI_MISO_IO,
        .sclk_io_num = SPI_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10*1000*1000,
        .mode = 0,
        .spics_io_num = SPI_CS_IO,
        .queue_size = 1,
    };
    spi_bus_initialize(SPI_HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI_HOST_ID, &devcfg, &flash_dev);
}

// ——— HTML page with drag-and-drop ———
static const char index_html[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'><title>LPC4330 Uploader</title>"
"<style>body{font-family:sans-serif;text-align:center;margin-top:50px;}#drop{border:2px dashed #666;padding:50px;}#drop.hover{border-color:#000;}#msg{margin-top:20px;}</style>"
"</head><body>"
"<h1>Drag & Drop .zip to upload</h1>"
"<div id='drop'>Drop ZIP here</div>"
"<div id='msg'></div>"
"<script>"
"let drop = document.getElementById('drop'), msg = document.getElementById('msg');"
"drop.addEventListener('dragover', e=>{e.preventDefault(); drop.classList.add('hover');});"
"drop.addEventListener('dragleave', e=>{e.preventDefault(); drop.classList.remove('hover');});"
"drop.addEventListener('drop', async e=>{"
" e.preventDefault(); drop.classList.remove('hover');"
" let file = e.dataTransfer.files[0];"
" msg.textContent='Uploading...';"
" try {"
"   let resp = await fetch('/upload',{method:'POST',body:file});"
"   let text = await resp.text(); msg.textContent=text;"
" } catch(err){ msg.textContent='Error:'+err; }"
"});"
"</script></body></html>";

// ——— HTTP Handlers ———
static esp_err_t index_get(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, sizeof(index_html)-1);
    return ESP_OK;
}

static esp_err_t upload_post(httpd_req_t *req) {
    ESP_LOGI(TAG, "Upload %d bytes", req->content_len);
    // 1) read ZIP
    uint8_t *zip = malloc(req->content_len);
    if (!zip) { httpd_resp_send_500(req); return ESP_FAIL; }
    int r = httpd_req_recv(req, (char*)zip, req->content_len);
    if (r <= 0) { free(zip); httpd_resp_send_500(req); return ESP_FAIL; }

    // 2) forward to build server
    esp_http_client_config_t cfg = { .url = BUILD_SERVER_URL };
    esp_http_client_handle_t cl = esp_http_client_init(&cfg);
    esp_http_client_set_method(cl, HTTP_METHOD_POST);
    esp_http_client_set_header(cl, "Content-Type", "application/zip");
    esp_http_client_open(cl, req->content_len);
    esp_http_client_write(cl, (char*)zip, req->content_len);

    // 3) read back firmware.bin
    size_t cap = 1024*1024, tot = 0;
    uint8_t *bin = malloc(cap);
    while (1) {
        int len = esp_http_client_read(cl, (char*)bin + tot, 1024);
        if (len <= 0) break;
        tot += len;
    }
    esp_http_client_cleanup(cl);
    free(zip);
    ESP_LOGI(TAG, "Got %u bytes firmware", tot);

    // 4) reflash external QSPI
    gpio_set_level(LPC4330_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    spi_flash_init();
    flash_chip_erase();
    for (uint32_t off=0; off<tot; off+=256) {
        size_t c = (tot-off>256?256:tot-off);
        if (!flash_page_program(off, bin+off, c)) {
            ESP_LOGE(TAG,"Flash failed @0x%08X",off);
            break;
        }
    }
    free(bin);
    gpio_set_level(LPC4330_RST_GPIO, 1);

    httpd_resp_sendstr(req, "✅ Firmware flashed—LPC4330 rebooting!");
    return ESP_OK;
}

// ——— Wi-Fi AP & HTTP server init ———
void wifi_ap_init(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wcfg);
    wifi_config_t ap_cfg = {
        .ap = {.ssid = CONFIG_WIFI_AP_SSID, .ssid_len=0, .channel=1,
               .authmode=WIFI_AUTH_OPEN, .max_connection=4}
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_cfg);
    esp_wifi_start();
    ESP_LOGI(TAG,"AP \"%s\" started", CONFIG_WIFI_AP_SSID);

    httpd_handle_t srv = NULL;
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    httpd_start(&srv, &hcfg);

    httpd_uri_t get_index = { .uri = "/", .method=HTTP_GET, .handler=index_get };
    httpd_uri_t post_upload = { .uri = "/upload", .method=HTTP_POST, .handler=upload_post };
    httpd_register_uri_handler(srv, &get_index);
    httpd_register_uri_handler(srv, &post_upload);
}

void app_main(void) {
    // prep reset pin
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL<<LPC4330_RST_GPIO
    };
    gpio_config(&io);
    gpio_set_level(LPC4330_RST_GPIO, 1);

    wifi_ap_init();
    ESP_LOGI(TAG,"Ready—point browser to http://192.168.4.1/");
    // nothing else to do; HTTP server handles everything
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}
