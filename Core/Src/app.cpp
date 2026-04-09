#include "app.h"

extern "C" {
#include "backend.h"
}

#include "device_config.h"
#include "device_dpt.hpp"
#include "mku_cfg_flash.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

/* Конфиг MKUCfg — секция .mku_cfg в STM32H523RETX_FLASH.ld (область FLASH_CFG),
 * адрес и размер — mku_cfg_flash.h (_mku_cfg_start, _mku_cfg_end) */
#define FLASH_CFG_SECTOR     (31u)         /* последний сектор Bank 2 (0x0807E000); в каждом банке 32 сектора 0–31 */
#define MKU_CFG_HEADER_MAGIC 0x4D4B5543u   /* 'MKUC' */
#define MAX_TEMP_SMA_SIZE    10u
#define MAX31855_SWITCH_BLANK_MS 100u
#define MAX_TC_STEP_LIMIT_C  40
#define MAX_TC_VALID_STREAK_REQUIRED 2u
#define MAX_FAULT_MASK_FAULT 0x01u
#define MAX_FAULT_MASK_SCV   0x02u
#define MAX_FAULT_MASK_SCG   0x04u
#define MAX_FAULT_MASK_OC    0x08u


#include <string.h>
#include "main.h"

#include "max31855.h"

MAX31855_Data t_couple;

/* Конфигурация платы — единая структура MKUCfg из device_config */
static MKUCfg g_cfg;
static MKUCfg g_saved_cfg;

/* Виртуальный датчик ДПТ (Devices[0], Dev=1) */
static VDeviceDPT g_dpt(1);

uint8_t mes_max_flag = 0;



/* Кольцевой буфер принятых CAN-пакетов */
#define APP_CAN_RX_RING_SIZE  256

typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  bus;
} AppCanRxEntry;

static AppCanRxEntry can_rx_ring[APP_CAN_RX_RING_SIZE];
static volatile uint8_t can_rx_head = 0;
static volatile uint8_t can_rx_tail = 0;

/* Флаги активности шин CAN: 1 - были пакеты за последние 2 секунды, 0 - тишина */
volatile uint8_t CAN1_Active = 0;
volatile uint8_t CAN2_Active = 0;
static uint32_t can1_last_rx_tick = 0;
static uint32_t can2_last_rx_tick = 0;

/* Напряжения из статуса ППКУ (мВ).
 * Приоритет: U (power), если оно >= 9 В, иначе U_res (резервное). */
static uint32_t g_ppky_u_mv    = 24000u;
static uint32_t g_ppky_ures_mv = 24000u;


void RcvStatusFire() {}


void RcvStopExtinguishment() {}
void RcvSetSystemTime(uint8_t *Data) {}
uint8_t DPT_status = 0;

/* Повторная отправка статуса пожара:
 * после SetStatusFire() каждые 200 мс до RcvReplyStatusFire() или RcvStartExtinguishment(). */
static uint8_t  g_fire_retry_active = 0;
static uint32_t g_fire_last_send_ms = 0;
static int32_t  g_max_temp_sma_buf[MAX_TEMP_SMA_SIZE];
static int32_t  g_max_temp_sma_sum = 0;
static uint8_t  g_max_temp_sma_idx = 0;
static uint8_t  g_max_temp_sma_fill = 0;
static uint32_t g_max_mode_switch_ms = 0u;
static int16_t  g_tc_hist3[3] = {0, 0, 0};
static uint8_t  g_tc_hist3_idx = 0u;
static uint8_t  g_tc_hist3_fill = 0u;
static int16_t  g_tc_prev_filtered = 0;
static uint8_t  g_tc_prev_valid = 0u;
static uint8_t  g_tc_valid_streak = 0u;
static int16_t	g_tc_prev_val = 0;

static int16_t Median3(int16_t a, int16_t b, int16_t c)
{
	if (a > b) { int16_t t = a; a = b; b = t; }
	if (b > c) { int16_t t = b; b = c; c = t; }
	if (a > b) { int16_t t = a; a = b; b = t; }
	return b;
}

static void App_DPT_SetResMeasureMode(void)
{
    /* 24В включены, реле на измерение сопротивления */
    HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LINE1_EN_GPIO_Port, LINE1_EN_Pin, GPIO_PIN_SET);
}

static void App_DPT_SetMaxMeasureMode(void)
{
    /* 24В отключены, реле на измерение линии по MAX */
	HAL_GPIO_WritePin(LINE1_EN_GPIO_Port, LINE1_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_Pin, GPIO_PIN_SET);
    g_max_mode_switch_ms = HAL_GetTick();
}

void MAXReadProcess() {
    if (MAX31855_ReadTemperature(&t_couple) == HAL_OK) {
    	//if(HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin) == GPIO_PIN_RESET)
    	//	t_couple.thermocouple_temp_c = g_max_temp_sma_buf[g_max_temp_sma_idx];

    	//if(t_couple.thermocouple_temp_c > (g_max_temp_sma_buf[g_max_temp_sma_idx] + 100))
    	//	t_couple.thermocouple_temp_c = g_max_temp_sma_buf[g_max_temp_sma_idx];

    	uint8_t valid = 1;

    	if(HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin) == GPIO_PIN_RESET)
    		valid = 0;

	/* После переключения реле в MAX-режим кратковременно подавляем данные термопары. */
	if ((HAL_GetTick() - g_max_mode_switch_ms) < MAX31855_SWITCH_BLANK_MS) {
		valid = 0;
	}

        int16_t tc = (int16_t)t_couple.thermocouple_temp_c;
        int16_t ti = (int16_t)t_couple.internal_temp_c;
        uint8_t fault_mask = 0u;
        if (t_couple.fault) { fault_mask |= MAX_FAULT_MASK_FAULT; }
        if (t_couple.scv)   { fault_mask |= MAX_FAULT_MASK_SCV; }
        if (t_couple.scg)   { fault_mask |= MAX_FAULT_MASK_SCG; }
        if (t_couple.oc)    { fault_mask |= MAX_FAULT_MASK_OC; }

        //if(t_couple.fault || t_couple.scv || t_couple.scg || t_couple.oc) <<< конкретно в этой плате отключаем эту проверку, т.к косяк в схеме
        //	valid = 0;

        if (valid) {
            g_tc_hist3[g_tc_hist3_idx] = tc;
            g_tc_hist3_idx++;
            if (g_tc_hist3_idx >= 3u) {
                g_tc_hist3_idx = 0u;
            }
            if (g_tc_hist3_fill < 3u) {
                g_tc_hist3_fill++;
            }

            int16_t tc_filtered = tc;
            if (g_tc_hist3_fill == 3u) {
                tc_filtered = Median3(g_tc_hist3[0], g_tc_hist3[1], g_tc_hist3[2]);
            }

            if (!valid) {
                g_tc_valid_streak = 0u;
            }

            if (valid) {
                if (g_tc_valid_streak < 255u) {
                    g_tc_valid_streak++;
                }
                if (g_tc_valid_streak < MAX_TC_VALID_STREAK_REQUIRED) {
                    valid = 0;
                } else {
                    tc = tc_filtered;
                    g_tc_prev_filtered = tc_filtered;
                    g_tc_prev_valid = 1u;
                }
            }
        } else
        	g_tc_valid_streak = 0u;



        if(valid) {
			if (g_max_temp_sma_fill == MAX_TEMP_SMA_SIZE) {
				g_max_temp_sma_sum -= g_max_temp_sma_buf[g_max_temp_sma_idx];
			} else {
				g_max_temp_sma_fill++;
			}
			g_max_temp_sma_buf[g_max_temp_sma_idx] = tc;
			g_max_temp_sma_sum += tc;
			g_max_temp_sma_idx++;
			if (g_max_temp_sma_idx >= MAX_TEMP_SMA_SIZE) {
				g_max_temp_sma_idx = 0u;
			}
			int16_t tc_avg = tc;
			if (g_max_temp_sma_fill > 0u) {
				tc_avg = (int16_t)(g_max_temp_sma_sum / (int32_t)g_max_temp_sma_fill);
			}
			g_tc_prev_val = tc_avg;
			g_dpt.SetMaxStatus(tc_avg, fault_mask, ti);
        } else
        	g_dpt.SetMaxStatus(g_tc_prev_val, fault_mask, ti);
    }
}

/* callback статуса: отправляем его через CAN по протоколу backend */
static void VDeviceSetStatus(uint8_t DNum, uint8_t Code, const uint8_t *Parameters) {
    uint8_t data[7] = {0};
    for (uint8_t i = 0; i < 7; i++) {
        data[i] = Parameters[i];
    }
    /* DNum = 1 для виртуального ДПТ */
    SendMessage(DNum, Code, data, 0, BUS_CAN12);
    DeviceDPTLineState state = (DeviceDPTLineState)data[0];
    extern Device BoardDevicesList[];
    if (BoardDevicesList[DNum].d_type == DEVICE_DPT_TYPE) {
        if (((DPT_status != state) || g_fire_retry_active) && (state == DeviceDPTLineState_Fire)) {
            DPT_status = state;
            can_ext_id_t can_id;
            can_id.ID = 0;
        	can_id.field.d_type = BoardDevicesList[DNum].d_type & 0x7F;
        	can_id.field.h_adr = BoardDevicesList[DNum].h_adr;
        	can_id.field.l_adr = BoardDevicesList[DNum].l_adr & 0x3F;
        	can_id.field.zone = BoardDevicesList[DNum].zone & 0x7F;
            uint8_t Data[7] = {(uint8_t)can_id.field.d_type, (uint8_t)can_id.field.l_adr, (uint8_t)can_id.field.zone,0 ,0 ,0, 0};
            SetStatusFire(Data);
            g_fire_retry_active = 1u;
            g_fire_last_send_ms = HAL_GetTick();
        }
    }

}

void RcvReplyStatusFire()
{
	/* Пришло подтверждение — останавливаем повторы */
	g_fire_retry_active = 0;
}

extern "C" void RcvStartExtinguishment(uint8_t *MsgData)
{
	(void)MsgData;
	/* Началось тушение — дальнейшие повторы статуса пожара не нужны */
	g_fire_retry_active = 0;
}

void SetHAdr(uint8_t h_adr) {
	g_cfg.UId.devId.h_adr = h_adr;
	extern uint8_t nDevs;
	extern Device BoardDevicesList[];
	for(uint8_t i = 0; i < nDevs; i++) {
		BoardDevicesList[i].h_adr = g_cfg.UId.devId.h_adr;
	}
	SaveConfig();
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

    /* VDtype: Devices[0]=DPT */
    g_cfg.VDtype[0] = DEVICE_DPT_TYPE;



    /*ДПТ config в Devices[0].reserv */
    DeviceDPTConfig *dpt_cfg = reinterpret_cast<DeviceDPTConfig*>(g_cfg.Devices[0].reserv);
    memset(dpt_cfg, 0, sizeof(DeviceDPTConfig));
    dpt_cfg->mode                  = 0;    // ДПТ
    dpt_cfg->use_max               = 1;    // использовать MAX
    dpt_cfg->max_fire_threshold_c  = 100;   // °C
    dpt_cfg->state_change_delay_ms = 100;  // мс

    /* reserv[64] — отличия платы ТС (пока пусто) */
}

uint32_t GetConfigSize(void)
{
    return static_cast<uint32_t>(sizeof(g_cfg));
}

uint32_t GetConfigWord(uint16_t num)
{
    uint32_t byte_index = static_cast<uint32_t>(num) * 4u;
    uint32_t cfg_size   = GetConfigSize();

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
    uint32_t cfg_size   = GetConfigSize();

    if (byte_index + 4u > cfg_size) {
        return;
    }

    uint8_t *p = reinterpret_cast<uint8_t *>(&g_cfg);
    p[byte_index + 0] = static_cast<uint8_t>((word >> 24) & 0xFFu);
    p[byte_index + 1] = static_cast<uint8_t>((word >> 16) & 0xFFu);
    p[byte_index + 2] = static_cast<uint8_t>((word >> 8)  & 0xFFu);
    p[byte_index + 3] = static_cast<uint8_t>((word >> 0)  & 0xFFu);
}

/* Заголовок конфига во Flash: magic (4) + size (4) = 8 байт, затем MKUCfg */
#define MKU_CFG_HEADER_SIZE  8u
#define QUADWORD_SIZE        16u   /* 128-bit для HAL_FLASH_Program */

static bool FlashReadConfig(MKUCfg *out)
{
    if (out == nullptr) {
        return false;
    }
    const uint32_t *p = reinterpret_cast<const uint32_t *>(FLASH_CFG_ADDR);
    if (p[0] != MKU_CFG_HEADER_MAGIC) {
        return false;
    }
    uint32_t sz = p[1];
    if (sz != sizeof(MKUCfg) || sz > FLASH_CFG_SIZE - MKU_CFG_HEADER_SIZE) {
        return false;
    }
    memcpy(out, p + 2, sz);
    return true;
}

void FlashWriteData(uint8_t *ConfigPtr, uint32_t ConfigSize)
{
    if (ConfigPtr == nullptr || ConfigSize != sizeof(MKUCfg) ||
        ConfigSize > FLASH_CFG_SIZE - MKU_CFG_HEADER_SIZE) {
        return;
    }

    /* Буфер: заголовок + MKUCfg, выровнен по 16 байт для quad-word программирования */
    __attribute__((aligned(16))) uint8_t buf[FLASH_CFG_SIZE_BYTES];
    uint32_t *hdr = reinterpret_cast<uint32_t *>(buf);
    hdr[0] = MKU_CFG_HEADER_MAGIC;
    hdr[1] = ConfigSize;
    memcpy(buf + MKU_CFG_HEADER_SIZE, ConfigPtr, ConfigSize);

    uint32_t total = MKU_CFG_HEADER_SIZE + ConfigSize;
    uint32_t n_quad = (total + QUADWORD_SIZE - 1u) / QUADWORD_SIZE;

    FLASH_EraseInitTypeDef erase;
    uint32_t sector_err = 0u;

#if defined(FLASH_TYPEERASE_SECTORS_NS)
    erase.TypeErase = FLASH_TYPEERASE_SECTORS_NS;
#else
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
#endif
    erase.Banks     = FLASH_BANK_2;
    erase.Sector    = FLASH_CFG_SECTOR;
    erase.NbSectors = 1u;

    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) {
        return;
    }

    st = HAL_FLASHEx_Erase(&erase, &sector_err);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

#if defined(FLASH_TYPEPROGRAM_QUADWORD_NS)
    uint32_t prog_type = FLASH_TYPEPROGRAM_QUADWORD_NS;
#else
    uint32_t prog_type = FLASH_TYPEPROGRAM_QUADWORD;
#endif

    for (uint32_t i = 0u; i < n_quad && st == HAL_OK; i++) {
        uint32_t addr = FLASH_CFG_ADDR + i * QUADWORD_SIZE;
        st = HAL_FLASH_Program(prog_type, addr,
                              reinterpret_cast<uint32_t>(buf + i * QUADWORD_SIZE));
    }

    HAL_FLASH_Lock();
}

void SaveConfig(void)
{
    uint32_t size = GetConfigSize();
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

void MCU_TCCommandCB(uint8_t Command, uint8_t *Parameters) {
	if(Command == 20) {
		g_cfg.UId.devId.zone = Parameters[0];
		SaveConfig();
	}
}

void CommandCB(uint8_t Dev, uint8_t Command, uint8_t *Parameters)
{
    switch (Dev) {
    case 0:
        /* сервисные команды физической платы  */
    	MCU_TCCommandCB(Command, Parameters);
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
    can_ext_id_t id;
    id.ID = MsgID;
    /* Интересует только статусная посылка от ППКУ (DEVICE_PPKY_TYPE, dir=1, Dev=0, Cmd=0) */
    if (id.field.d_type == DEVICE_PPKY_TYPE && id.field.dir == 1u && MsgData != NULL) {
    	uint8_t cmd = MsgData[0];
    	if (cmd == 0u) {
    		/* Формат как в ППКУ: status_data[1] = U (0.1 В), status_data[2] = U_res (0.1 В) */
    		uint8_t u_tenth    = MsgData[2];
    		uint8_t ures_tenth = MsgData[3];
    		g_ppky_u_mv    = (uint32_t)u_tenth    * 100u; /* 0.1 В → мВ */
    		g_ppky_ures_mv = (uint32_t)ures_tenth * 100u;
    	}
    }
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

void App_CanRxPush(uint32_t id, const uint8_t *data, uint8_t bus)
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
    can_rx_ring[can_rx_head].bus = bus;
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

        ProtocolParse(e->id, e->data, e->bus);
    }
}

void App_Init(void)
{
    extern Device BoardDevicesList[];
    extern uint8_t nDevs;

    if (!FlashReadConfig(&g_cfg)) {
        DefaultConfig();
        SaveConfig();   /* сохраняем дефолт при первом запуске */
    }
    g_saved_cfg = g_cfg;
    SetConfigPtr(reinterpret_cast<uint8_t *>(&g_saved_cfg),
                 reinterpret_cast<uint8_t *>(&g_cfg));

    /* Инициализируем виртуальный ДПТ (Devices[0]) */
    g_dpt.DeviceInit(&g_cfg.Devices[0]);
    g_dpt.VDeviceSetStatus = VDeviceSetStatus;
    g_dpt.VDeviceSaveCfg   = SaveConfig;
	g_dpt.DPT_SetResMeasureMode = App_DPT_SetResMeasureMode;
	g_dpt.DPT_SetMaxMeasureMode = App_DPT_SetMaxMeasureMode;

    //TODO:: delete!!

    g_cfg.zone_delay = 5 + g_cfg.UId.devId.zone * 2;
    g_cfg.module_delay[0] = 0;
    g_cfg.module_delay[1] = 2;
    g_cfg.module_delay[2] = 4;
	DeviceDPTConfig *dpt_cfg = reinterpret_cast<DeviceDPTConfig*>(g_cfg.Devices[0].reserv);
	dpt_cfg->use_max = 1;
	dpt_cfg->state_change_delay_ms = 100;
	dpt_cfg->max_fire_threshold_c = 80;
	dpt_cfg->mode                  = 0;    // ДПТ





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

    // MAX off, switch to Max On
    App_DPT_SetResMeasureMode();
    extern bool isListener;
    isListener = true;
}

void App_Timer1ms(void)
{
    static uint16_t led_cnt = 0u;
    static uint16_t tmax_cnt = 0u;
    static uint16_t status_cnt = 0u;
//    static uint16_t probe_cnt = 0u;

    uint32_t now = HAL_GetTick();

    /* Статус активности МКУ раз в секунду (Dev 0 — плата) */
    if (status_cnt < 1000u) {
        status_cnt++;
    } else {
        status_cnt = 0u;
        uint8_t status_data[7] = {
            (uint8_t)(now & 0xFFu),
            (uint8_t)((now >> 8) & 0xFFu),
            (uint8_t)((now >> 16) & 0xFFu),
            (uint8_t)((now >> 24) & 0xFFu),
            (uint8_t)(CAN1_Active | (CAN2_Active << 1)),
            0u,
            0u
        };
        SendMessage(0, 0, status_data, SEND_NOW, BUS_CAN12);
    }

    if (led_cnt < 1000u) {
        led_cnt++;
    } else {
        led_cnt = 0u;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    if(tmax_cnt < 100)
    	tmax_cnt++;
    else {
    	MAXReadProcess();
		tmax_cnt = 0;
    }
/*
    if (probe_cnt < 10000u) {
    	probe_cnt++;
    } else {
    	probe_cnt = 0u;

        HAL_GPIO_TogglePin(SWITCH_GPIO_Port, SWITCH_Pin);
        HAL_GPIO_TogglePin(LINE1_EN_GPIO_Port, LINE1_EN_Pin);
    }
*/
    /* Обновляем флаги активности CAN: если 3 секунды тишина — считаем шину неактивной */
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
        if ((now - can1_last_rx_tick) >= 3000u) {
            CAN1_Active = 0u;
            can1_last_rx_tick = 0u;
        }
    }

    if (can2_last_rx_tick != 0u) {
        if ((now - can2_last_rx_tick) >= 3000u) {
            CAN2_Active = 0u;
            can2_last_rx_tick = 0u;
        }
    }
}

void App_SetDPTAdcValues(uint16_t ch1, uint16_t ch2)
{
    /* ch1, ch2 — отфильтрованные коды АЦП для LINE_L и LINE_H.
     * Переводим их в напряжения и считаем сопротивление линии в Омах.
     */
    const uint32_t VREF_MV = 3300u;      /* опорное напряжение АЦП, мВ */
    const uint32_t ADC_MAX = 4095u;      /* 12-битный АЦП */
    const uint32_t R0_OHM  = 1925u;       /* ограничительные резисторы 24В→LINE при 24 В */
    const uint32_t DIV_K   = 11u;        /* коэффициент делителя 47к/4.7к */

    /* Напряжения на входах АЦП, мВ */
    uint32_t v_adc_l_mv = (uint32_t)ch1 * VREF_MV / ADC_MAX;
    uint32_t v_adc_h_mv = (uint32_t)ch2 * VREF_MV / ADC_MAX;

    /* Напряжения непосредственно на линии, мВ */
    uint32_t v_line_l_mv = v_adc_l_mv * DIV_K;
    uint32_t v_line_h_mv = v_adc_h_mv * DIV_K;
    if (v_line_l_mv == 0u) {
    	v_line_l_mv = 1u;
    }
    uint32_t r_line_ohm = 0u;

    /* Базовое сопротивление линии по идеальной формуле */
    if (v_line_h_mv > v_line_l_mv) {
        r_line_ohm = R0_OHM * (v_line_h_mv - v_line_l_mv) / v_line_l_mv;
    }

    /* Эмпирическая коррекция по напряжению питания ППКУ:
     * хотим, чтобы при 12 В (3100 Ом) и 20 В (1431 Ом) получалось примерно одно и то же значение.
     *
     * R_corr = R_meas * ( -0.346 + 0.0673 * U_В )
     * В целочисленном виде (k_scaled = k * 1000):
     * k_scaled = -346 + 0.0673 * U_В * 1000 = -346 + 0.0673 * u_src_mv
     * ≈ -346 + (673 * u_src_mv) / 10000
     */
    uint32_t u_src_mv = g_ppky_u_mv;
    if (u_src_mv < 9000u) {
    	u_src_mv = g_ppky_ures_mv;
    }
    if (u_src_mv < 9000u) {
    	u_src_mv = 12000u; /* минимально разумное значение, чтобы коэффициент не ушёл в ноль */
    }
// ниже перерасчёт по двум точкам (12В и 20В), всё в лаптях для макета
    int32_t k_scaled = -346;
    k_scaled += (int32_t)(673u * (u_src_mv / 10u)) / 1000; /* (673 * u_mv / 10000) ≈ 673*(u_mv/10)/1000 */

    /* Ограничиваем коэффициент в разумных пределах: от 0.3 до 1.5 */
    if (k_scaled < 300)  k_scaled = 300;
    if (k_scaled > 1500) k_scaled = 1500;

    /* Применяем коррекцию: r_corr = r_line_ohm * k_scaled / 1000 */
    uint32_t r_corr = (uint32_t)((r_line_ohm * (uint32_t)k_scaled + 500u) / 1000u);

    g_dpt.SetAdcValues((uint16_t)r_corr, 0);
}

} // extern "C"

