#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <freertos/semphr.h>
#include <iot_button.h>
#include "app_priv.h"
#include "board_esp32_devkitc.h"
#include "esp_sleep.h"
#include "rtc_io.h"
#include "driver/ledc.h"
#include "esp_err.h"

//====================ЗАПОЛНЯЕТСЯ ИНДИВИДУАЛЬНО НА КАЖДОЕ УСТРОЙСТВО======================
// CODE = "EP" // код предприятия
// MOD "01" //модификация устройства: 01_H - "УВНБУ 6-35"
//                                         01_L - "УННО-СИНХРО"
//                                         01_D - "ИТ-04"
//                                         02_H - "УВНБУ 35-110"
// TYPE "H" //тип устройства: "Н" указатель высокого напряжения;
//                                 "L" указатель низкого напряжения;
//                                 "D" цифровой индикатор;
// S_N "00401897" //серийный номер изделия
// VALUE "000" //численные показания для цифровых устройств
char name_const[] = "EP01H00401897000"; //ЗАПОЛНИТЬ "CODE#MOD#TYPE#S_N#VALUE" постоянная часть передаваемого сообщения
//ПРИМЕР: "EP01H00401897000" - всего 16 символов
int size_name_const = 17; //количество символов в строке 16+1
//=======================================================================================
// MESSAGE "01" //сообщение: 01 - "ОПАСНО!!!ВЫСОКОЕ НАПРЯЖЕНИЕ!!!";
//                                02 - "Тест проверки прошел";
//                                03 - "Тест проверки не прошел!!"
//==================ЗАДЕЙСТВОВАННЫЕ ВЫВОДЫ esp32====================================
// "EN" -  вход "сброс"
// "GPIO4"  -   вход пробуждения
gpio_num_t IN_AWAKEN = GPIO_NUM_4;
#define BUTTON_PIN_BITMASK 0x0010 // битовая маска только для контакта GPIO4
// "GPIO23" -   выход формирования сигнала самоконтроля
 gpio_num_t OUT_TEST = GPIO_NUM_23;  
// "GPIO22" -   вход кнопки "ТЕСТ"
 gpio_num_t IN_TEST = GPIO_NUM_22;
// "GPIO19" -   выход формирования звука
const gpio_num_t OUT_SOUND = GPIO_NUM_19;
// "GPIO21" -   выход светодиод зеленый
gpio_num_t OUT_GREEN = GPIO_NUM_21;
// "GPIO5"  -   выход светодиод красный 1
gpio_num_t OUT_RED1 = GPIO_NUM_5;
// "GPIO18" -   выход светодиод красный 2
gpio_num_t OUT_RED2 = GPIO_NUM_18;
//====================================================================================
const int freg_sound = 3200;//частота звука
const TickType_t PERIOD_TEST = 200 / portTICK_PERIOD_MS; //min импульс формирования тестового сигнала
const TickType_t PERIOD_OK = 1000 / portTICK_PERIOD_MS; //доп.время свечения зел. св.диода
const TickType_t PERIOD_SEND_BLE = 1000 /  portTICK_PERIOD_MS; //время передачи рекламы BLE
const TickType_t PERIOD_SIGNAL = 500 /  portTICK_PERIOD_MS; //полупериод св./зв. сигнала
#define PERIOD_AVD 360 //цикл посылки сообщений рекламы (PERIOD_AVD*0,625)мс
//структура - настройки звукового канала
ledc_channel_config_t sound_channel = {
        . gpio_num = OUT_SOUND,
        . speed_mode = LEDC_LOW_SPEED_MODE,
        . channel = LEDC_CHANNEL_0,
        . intr_type = 0,
        . timer_sel = 0,
        . duty = 0,
        . hpoint = 0,
    };
//структура - настройка таймера для звука
    ledc_timer_config_t sound_timer = {
    . duty_resolution = LEDC_TIMER_13_BIT, //  разрешение режима ШИМ
    . freq_hz = freg_sound,                // частота сигнала, Гц 
    . speed_mode = LEDC_LOW_SPEED_MODE,// режим таймера - низкая скорость
    . timer_num = LEDC_TIMER_0,      // канал таймера
    . clk_cfg = LEDC_AUTO_CLK,        // Автоматический выбор тактовых сигналов
    };

RTC_DATA_ATTR int bootCount = 0;//резервирование в памяти RTC ячейки для хранения числа "просыпаний"

/*=========================ОТНОСИТСЯ К BLE============================*/
static const char *tag = "BLE_ADV";
char *adv_name="NO"; //адрес, где располагается имя устройства

#define HCI_H4_CMD_PREAMBLE_SIZE           (4)

/* Поле группы опкодов команды HCI (OGF)  */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_BLE_WRITE_ADV_ENABLE           (0x000A | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_PARAMS           (0x0006 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)

#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)

#define BD_ADDR_LEN     (6)                     /*Длина адреса устройства */
typedef uint8_t bd_addr_t[BD_ADDR_LEN];         /*Адрес устройства */

#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};

static uint8_t hci_cmd_buf[128];//размер буфера интерфейса

/*
 * @brief: функция обратного вызова контроллера BT, используется для 
 * уведомления верхнего уровня о том, что
 * контроллер готов к приему команды
 */
static void controller_rcv_pkt_ready(void)
{
    printf("controller rcv pkt ready\n");
}

/*
 * @brief: функция обратного вызова контроллера BT для передачи 
 * пакета данных в верхний
 * контроллер готов к приему команды
 */
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    printf("host rcv pkt: ");
    for (uint16_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

static uint16_t make_cmd_reset(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);
    return HCI_H4_CMD_PREAMBLE_SIZE;
}

static uint16_t make_cmd_ble_set_adv_enable (uint8_t *buf, uint8_t adv_enable)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM (buf, adv_enable);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_ADV_ENABLE;
}

static uint16_t make_cmd_ble_set_adv_param (uint8_t *buf, uint16_t adv_int_min, uint16_t adv_int_max,
        uint8_t adv_type, uint8_t addr_type_own,
        uint8_t addr_type_dir, bd_addr_t direct_bda,
        uint8_t channel_map, uint8_t adv_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS );

    UINT16_TO_STREAM (buf, adv_int_min);
    UINT16_TO_STREAM (buf, adv_int_max);
    UINT8_TO_STREAM (buf, adv_type);
    UINT8_TO_STREAM (buf, addr_type_own);
    UINT8_TO_STREAM (buf, addr_type_dir);
    BDADDR_TO_STREAM (buf, direct_bda);
    UINT8_TO_STREAM (buf, channel_map);
    UINT8_TO_STREAM (buf, adv_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS;
}


static uint16_t make_cmd_ble_set_adv_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}

static void hci_cmd_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_start(void)
{
    uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void)
{
    uint16_t adv_intv_min = PERIOD_AVD; // (*х0,625)интервал рекламы
    uint16_t adv_intv_max = PERIOD_AVD; // (*х0,625)
    uint8_t adv_type = 0; // подключаемая ненаправленная реклама (ADV_IND)
    uint8_t own_addr_type = 0; // Адрес публичного устройства
    uint8_t peer_addr_type = 0; //Адрес публичного устройства
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t adv_chn_map = 0x07; // 37, 38, 39
    uint8_t adv_filter_policy = 0; //Обработка всех подключений и сканирование

    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                  adv_intv_min,
                  adv_intv_max,
                  adv_type,
                  own_addr_type,
                  peer_addr_type,
                  peer_addr,
                  adv_chn_map,
                  adv_filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}
//формирование данных объявления
static void hci_cmd_send_ble_set_adv_data(void)
{
    uint8_t name_len = (uint8_t)strlen(adv_name);
    uint8_t adv_data[31] = {0x02, 0x01, 0x06, 0x0, 0x09};
    uint8_t adv_data_len;

    adv_data[3] = name_len + 1;
    for (int i = 0; i < name_len; i++) {
        adv_data[5 + i] = (uint8_t)adv_name[i];
    }
    adv_data_len = 5 + name_len;

    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

/*
 * @brief: отправить команды HCI для выполнения рекламы BLE;
 */
void bleAdvtTask()//параметры отсутствуют
{
    int cmd_cnt = 0;//счетчик цикла подготовки отправки данных, вначале 0
    bool send_avail = false;//признак готовности к отправке, вначале false
    esp_vhci_host_register_callback(&vhci_host_cb);//регистрация обратного
    //вызова контроллера
    printf("BLE advt task start\n");// " Запуск задачи BLE advt \ n "
    while (cmd_cnt<4) {// цикл формирования сообщения\рекламы
        send_avail = esp_vhci_host_check_send_available();//используется для 
        //активной проверки, может ли хост отправлять пакет контроллеру или нет.
        //Возвращение: истина для готовности к отправке,
        // ложь означает невозможность отправки пакета
        if (send_avail) {
            switch (cmd_cnt) {
            case 0: hci_cmd_send_reset(); ++cmd_cnt; break;
            case 1: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;
            case 2: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; break;
            case 3: hci_cmd_send_ble_adv_start(); ++cmd_cnt; break;
            }
        }
        printf("BLE Advertise, flag_send_avail: %d, cmd_sent: %d\n", send_avail, cmd_cnt);
         
    }
       int tx_power = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
       printf("BLE TX POWER: %d\n", tx_power);
       vTaskDelay(PERIOD_SEND_BLE);//пауза 850мс, при интервале посылки 
       //сообщения в 300 мс формируется 3 посылки
       //выключить BLE
       hci_cmd_send_reset();
}
//==================================================================================
/*=============ОТНОСИТСЯ К ОБСЛУЖИВАНИЮ СОБЫТИЙ============================*/
void vToggleLED(int c) //переключение светодиодов (моргание)
{
  if(c==0||c==2||c==4)
  {
    gpio_set_level(OUT_RED1, 0); 
    gpio_set_level(OUT_RED2, 0);
    //отключить звук
    ledc_set_duty(sound_channel.speed_mode, sound_channel.channel, 0);
    ledc_update_duty(sound_channel.speed_mode, sound_channel.channel); 
  }
  if(c==1||c==3||c==5)
  {
    gpio_set_level(OUT_RED1, 1); 
    gpio_set_level(OUT_RED2, 1);
    //включить звук
    ledc_set_duty(sound_channel.speed_mode, sound_channel.channel, freg_sound);
    ledc_update_duty(sound_channel.speed_mode, sound_channel.channel); 
  }  
}
//функция формирования свето-звукового сигнала
static void signal()
{
    ledc_timer_config(& sound_timer);
    printf("Set sound timer\n");
    
    ledc_channel_config(& sound_channel);    
    printf("Set sound channel\n");

    for(int i=0; i<6; i++)
    {
    vToggleLED(i);
    vTaskDelay(PERIOD_SIGNAL);    
    }
  
}
//данная функция вызывается когда устройство "проснулось по входу ВН"
static void event_HV()
{
    char name[size_name_const];
    char message[] = "01";
    strcpy(name, name_const); //копирование строки
    adv_name = name; //назначение адреса имени - адрес копированной строки
    strcat(adv_name,message);//соединение строк с записью в 1 строку
        printf(adv_name);
        printf("\n");    
    bleAdvtTask();//вызов п\п формирования "рекламы"
    signal();//формирования свето-звукового сигнала
}

//данная функция вызывается когда ТЕСТ прошел
static void event_testOk()
{
    char name[size_name_const];
    char message[] = "02";
    strcpy(name, name_const); //копирование строки
    adv_name = name; //назначение адреса имени - адрес копированной строки
    strcat(adv_name,message);//соединение строк с записью в 1 строку
        printf(adv_name);
        printf("\n");
    bleAdvtTask();//вызов п\п формирования "рекламы"
    gpio_set_level(OUT_GREEN, 0);
    signal();//формирования свето-звукового сигнала
    vTaskDelay(PERIOD_OK);//доп.время свечения зел.св.диода
    gpio_set_level(OUT_GREEN, 1);
}
//данная функция вызывается когда ТЕСТ не прошел
static void event_testNo()
{
    char name[size_name_const];
    char message[] = "03";
    strcpy(name, name_const); //копирование строки
    adv_name = name; //назначение адреса имени - адрес копированной строки
    strcat(adv_name,message);//соединение строк с записью в 1 строку
        printf(adv_name);
        printf("\n");
    bleAdvtTask();//вызов п\п формирования "рекламы"
}


/*================================================================================*/
void app_main()
{
    printf("[%d] Wakeup caused by external signal using RTC_IO\n", bootCount);
    bootCount++;
    //получить битовую маску GPIO, вызвавшую пробуждение (ext1)
    int GPIO_reason = esp_sleep_get_ext1_wakeup_status();
    printf("GPIO that triggered the wake up: kod %d\n", GPIO_reason); 
   
 //==================ИНИЦИАЛИЗАЦИЯ GPIO======================================
    gpio_reset_pin(OUT_TEST);
    gpio_set_direction(OUT_TEST, GPIO_MODE_OUTPUT);
    gpio_set_level(OUT_TEST, 0);
    gpio_reset_pin(IN_TEST);
    gpio_set_direction(IN_TEST, GPIO_MODE_INPUT);
    gpio_reset_pin(OUT_SOUND);
    gpio_set_direction(OUT_SOUND, GPIO_MODE_OUTPUT);
    gpio_set_level(OUT_SOUND, 0);
    gpio_reset_pin(OUT_GREEN);
    gpio_set_direction(OUT_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_level(OUT_GREEN, 1);
    gpio_reset_pin(OUT_RED1);
    gpio_set_direction(OUT_RED1, GPIO_MODE_OUTPUT);
    gpio_set_level(OUT_RED1, 1);
    gpio_reset_pin(OUT_RED2);
    gpio_set_direction(OUT_RED2, GPIO_MODE_OUTPUT);
    gpio_set_level(OUT_RED2, 1);
//========================================================================== 
            
        //==============ИНИЦИАЛИЗАЦИЯ BLE==============================================
        // Инициализировать NVS - используется для хранения данных калибровки PHY 
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGI(tag, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        return;//"Ошибка освобождения классической памяти контроллера Bluetooth: % s"
    }

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGI(tag, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;//« Ошибка инициализации контроллера Bluetooth: % s »
    }
    
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGI(tag, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;//« Ошибка включения контроллера Bluetooth: % s »
    }
    //========================================================================
    //Анализируем код просыпания 
    switch(GPIO_reason) {
        case BUTTON_PIN_BITMASK: //событие - высокое напряжение
        event_HV();
        printf("Event HV\n"); 
        break;
        default: 
        printf("Wakeup was not caused by deep sleep\n");//пробуждение не
        //связано с режимом глубокого сна
        int level = gpio_get_level(IN_TEST);
        if (level == 0) //нажата кнопка "ТЕСТ"
        {
            //signal();//формирования свето-звукового сигнала
            gpio_set_level(OUT_TEST, 1);
            vTaskDelay(PERIOD_TEST);//выдержка времени
            int level_awaken = gpio_get_level(IN_AWAKEN);
            if (level_awaken) //есть сигнал ВН
            {
                event_testOk();
                printf("Test OK\n");
            }
            else
            { 
                event_testNo();
                printf("Test NO\n");
            }
            gpio_set_level(OUT_TEST, 0);
        }  
        break;
    }

   //задаем инициатора пробуждения типа ext1 в случае если хотя бы один ="1"
    esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
    //отключить RTC IO, sensors and ULP co-processor в спящем режиме
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    //отключить медленную память RTC
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    //отключить быструю память RTC
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    //отключить генератор XTAL
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    //отключить подбрасывающие сопротивления от выводов
    rtc_gpio_isolate(GPIO_NUM_0);
    rtc_gpio_isolate(GPIO_NUM_2);
    rtc_gpio_isolate(GPIO_NUM_4);
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    printf("Going to sleep now\n");//иду спать по новой
    //запуск режима глубокого сна
    esp_deep_sleep_start();
}    
 
  

