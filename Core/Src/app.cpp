#include "app.h"

extern "C" {
#include "backend.h"
}

#include "device_config.h"
#include "device_dpt.hpp"
#include "stm32h5xx_hal.h"
#include <string.h>
#include "main.h"

#include "max31855.h"

MAX31855_Data t_couple;

/* Локальная конфигурация и "сохранённая" копия (используем тип DeviceDPTConfig из device_lib) */
typedef struct {
    UniqId          UId;
    DeviceDPTConfig dpt_cfg;
} McuTcCfg;

static McuTcCfg g_cfg;
static McuTcCfg g_saved_cfg;

/* Виртуальный датчик ДПТ (виртуальное устройство с номером 1) */
static VDeviceDPT g_dpt(1);
static VDeviceCfg g_dpt_cfg;

/* Кольцевой буфер принятых CAN-пакетов */
#define APP_CAN_RX_RING_SIZE  32

typedef struct {
    uint32_t id;
    uint8_t  data[8];
} AppCanRxEntry;

static AppCanRxEntry can_rx_ring[APP_CAN_RX_RING_SIZE];
static volatile uint8_t can_rx_head = 0;
static volatile uint8_t can_rx_tail = 0;

/* Флаги активности шин CAN: 1 - были пакеты за последние 2 секунды, 0 - тишина */
volatile uint8_t CAN1_Active = 0;
volatile uint8_t CAN2_Active = 0;
static uint32_t can1_last_rx_tick = 0;
static uint32_t can2_last_rx_tick = 0;

/* callback статуса: отправляем его через CAN по протоколу backend */
static void VDeviceSetStatus(uint8_t DNum, uint8_t Code, const uint8_t *Parameters) {
    uint8_t data[7] = {0};
    for (uint8_t i = 0; i < 7; i++) {
        data[i] = Parameters[i];
    }
    /* DNum = 1 для виртуального ДПТ */
    SendMessage(DNum, Code, data, 0);
}

/* Заглушка сохранения конфига виртуального устройства */
static void SaveCfg(void) {
    /* пока ничего не делаем, конфиг хранится в g_cfg */
}

/* -------- C-интерфейс, который ожидает backend и main.c -------- */

extern "C" {

void DefaultConfig(void)
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    memset(&g_cfg, 0, sizeof(g_cfg));

    g_cfg.UId.UId0 = uid0;
    g_cfg.UId.UId1 = uid1;
    g_cfg.UId.UId2 = uid2;
    g_cfg.UId.UId3 = HAL_GetDEVID();
    g_cfg.UId.UId4 = 1;

    g_cfg.UId.devId.zone  = 0;
    g_cfg.UId.devId.l_adr = 0;

    uint8_t hadr = static_cast<uint8_t>(uid0 & 0xFFu);
    if (hadr == 0u) {
        hadr = static_cast<uint8_t>(uid1 & 0xFFu);
        if (hadr == 0u) {
            hadr = 1u;
        }
    }
    g_cfg.UId.devId.h_adr  = hadr;
    g_cfg.UId.devId.d_type = DEVICE_MCU_TC_TYPE;

    /* Значения порогов ДПТ по умолчанию */
    g_cfg.dpt_cfg.fire_threshold_ohm   = 680u;
    g_cfg.dpt_cfg.normal_threshold_ohm = 5380u;
    g_cfg.dpt_cfg.break_threshold_ohm  = 100000u;
    g_cfg.dpt_cfg.resistor_r1_ohm      = 10000u;
    g_cfg.dpt_cfg.resistor_r2_ohm      = 10000u;
    g_cfg.dpt_cfg.supply_voltage_mv    = 3300u;
    g_cfg.dpt_cfg.adc_resolution       = 4095u;
    g_cfg.dpt_cfg.is_limit_switch      = 0u;
}

uint16_t GetConfigSize(void)
{
    return static_cast<uint16_t>(sizeof(g_cfg));
}

uint32_t GetConfigWord(uint16_t num)
{
    uint32_t byte_index = static_cast<uint32_t>(num) * 4u;
    uint16_t cfg_size   = GetConfigSize();

    if (byte_index + 4u > cfg_size) {
        return 0u;
    }

    uint8_t *p = reinterpret_cast<uint8_t *>(&g_cfg);
    uint32_t word = 0u;
    word |= (static_cast<uint32_t>(p[byte_index + 0]) << 24);
    word |= (static_cast<uint32_t>(p[byte_index + 1]) << 16);
    word |= (static_cast<uint32_t>(p[byte_index + 2]) << 8);
    word |= (static_cast<uint32_t>(p[byte_index + 3]) << 0);

    return word;
}

void SetConfigWord(uint16_t num, uint32_t word)
{
    uint32_t byte_index = static_cast<uint32_t>(num) * 4u;
    uint16_t cfg_size   = GetConfigSize();

    if (byte_index + 4u > cfg_size) {
        return;
    }

    uint8_t *p = reinterpret_cast<uint8_t *>(&g_cfg);
    p[byte_index + 0] = static_cast<uint8_t>((word >> 24) & 0xFFu);
    p[byte_index + 1] = static_cast<uint8_t>((word >> 16) & 0xFFu);
    p[byte_index + 2] = static_cast<uint8_t>((word >> 8)  & 0xFFu);
    p[byte_index + 3] = static_cast<uint8_t>((word >> 0)  & 0xFFu);
}

void FlashWriteData(uint8_t *ConfigPtr, uint16_t ConfigSize)
{
    (void)ConfigPtr;
    (void)ConfigSize;
}

void SaveConfig(void)
{
    uint16_t size = GetConfigSize();
    (void)size;

    FlashWriteData(reinterpret_cast<uint8_t *>(&g_cfg), size);
    g_saved_cfg = g_cfg;
}

void ResetMCU(void)
{
    NVIC_SystemReset();
}

uint32_t GetID(void)
{
    uint32_t id0 = HAL_GetUIDw0();
    uint32_t id1 = HAL_GetUIDw1();
    uint32_t id2 = HAL_GetUIDw2();
    return (id0 ^ id1 ^ id2);
}

void CommandCB(uint8_t Dev, uint8_t Command, uint8_t *Parameters)
{
    switch (Dev) {
    case 0:
        /* сервисные команды физической платы — пока заглушка */
        break;
    case 1:
        /* команды для виртуального ДПТ */
        g_dpt.CommandCB(Command, Parameters);
        break;
    default:
        break;
    }
}

void ListenerCommandCB(uint32_t MsgID, uint8_t *MsgData)
{
    (void)MsgID;
    (void)MsgData;
}

void App_CanOnRx(uint8_t bus)
{
    uint32_t now = HAL_GetTick();
    if (bus == 1u) {
        CAN1_Active = 1u;
        can1_last_rx_tick = now;
    } else if (bus == 2u) {
        CAN2_Active = 1u;
        can2_last_rx_tick = now;
    }
}

void App_CanRxPush(uint32_t id, const uint8_t *data)
{
    uint8_t next = static_cast<uint8_t>(can_rx_head + 1u);
    if (next >= APP_CAN_RX_RING_SIZE) {
        next = 0u;
    }

    /* При переполнении затираем самый старый пакет */
    if (next == can_rx_tail) {
        can_rx_tail++;
        if (can_rx_tail >= APP_CAN_RX_RING_SIZE) {
            can_rx_tail = 0u;
        }
    }

    can_rx_ring[can_rx_head].id = id;
    memcpy(can_rx_ring[can_rx_head].data, data, 8u);
    can_rx_head = next;
}

void App_CanProcess(void)
{
    while (can_rx_head != can_rx_tail) {
        AppCanRxEntry *e = &can_rx_ring[can_rx_tail];
        can_rx_tail++;
        if (can_rx_tail >= APP_CAN_RX_RING_SIZE) {
            can_rx_tail = 0u;
        }

        ProtocolParse(e->id, e->data);
    }
}

void App_Init(void)
{
    extern Device BoardDevicesList[];
    extern uint8_t nDevs;

    DefaultConfig();
    g_saved_cfg = g_cfg;
    SetConfigPtr(reinterpret_cast<uint8_t *>(&g_saved_cfg),
                 reinterpret_cast<uint8_t *>(&g_cfg));

    /* Инициализируем виртуальный ДПТ */
    memset(&g_dpt_cfg, 0, sizeof(g_dpt_cfg));
    g_dpt.DeviceInit(&g_dpt_cfg);
    g_dpt.VDeviceSetStatus = VDeviceSetStatus;
    g_dpt.VDeviceSaveCfg   = SaveCfg;
    g_dpt.Init();

    /* Регистрируем физическую плату и одно виртуальное устройство ДПТ */
    nDevs = 1; /* Dev 0 зарезервирован под плату */

    if (nDevs < MAX_DEVS) {
        BoardDevicesList[0].zone  = g_cfg.UId.devId.zone;
        BoardDevicesList[0].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[0].l_adr = g_cfg.UId.devId.l_adr;
        BoardDevicesList[0].d_type = DEVICE_MCU_TC_TYPE;
    }

    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone  = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 1; /* виртуальный номер устройства */
        BoardDevicesList[nDevs].d_type = DEVICE_DPT_TYPE;
        nDevs++;
    }
    HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LINE1_EN_GPIO_Port, LINE1_EN_Pin, GPIO_PIN_SET);

}

void App_Timer1ms(void)
{
    static uint16_t led_cnt = 0u;
    static uint16_t tmax_cnt = 0u;

    static uint16_t probe_cnt = 0u;

    if (led_cnt < 1000u) {
        led_cnt++;
    } else {
        led_cnt = 0u;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    if(tmax_cnt < 100)
    	tmax_cnt++;
    else {
		if (MAX31855_ReadTemperature(&t_couple) == HAL_OK) {
			// t.thermocouple_temp_c — температура термопары (°C)
			// t.internal_temp_c     — внутренняя температура кристалла (°C)
			// t.fault, t.scv, t.scg, t.oc — флаги ошибок
		}
		tmax_cnt = 0;
    }

    if (probe_cnt < 2000u) {
    	probe_cnt++;
    } else {
    	probe_cnt = 0u;

        //HAL_GPIO_TogglePin(SWITCH_GPIO_Port, SWITCH_Pin);
       // HAL_GPIO_TogglePin(LINE1_EN_GPIO_Port, LINE1_EN_Pin);
    }

    /* Обновляем флаги активности CAN: если 2 секунды тишина — считаем шину неактивной */
    App_UpdateCanActivity();

    /* Обновление виртуального ДПТ (1 мс таймер) */
    g_dpt.Timer1ms();

    App_CanProcess();
    BackendProcess();
}

void App_UpdateCanActivity(void)
{
    uint32_t now = HAL_GetTick();

    if (can1_last_rx_tick != 0u) {
        if ((now - can1_last_rx_tick) >= 2000u) {
            CAN1_Active = 0u;
            can1_last_rx_tick = 0u;
        }
    }

    if (can2_last_rx_tick != 0u) {
        if ((now - can2_last_rx_tick) >= 2000u) {
            CAN2_Active = 0u;
            can2_last_rx_tick = 0u;
        }
    }
}

void App_SetDPTAdcValues(uint16_t ch1, uint16_t ch2)
{
    g_dpt.SetAdcValues(ch1, ch2);
}

} // extern "C"

