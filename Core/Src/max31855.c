#include "max31855.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

/* Чтение 32-битного слова из MAX31855 по SPI2 */
HAL_StatusTypeDef MAX31855_ReadRaw(uint32_t *raw)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    HAL_StatusTypeDef st;

    if (raw == NULL) {
        return HAL_ERROR;
    }

    /* Активируем CS (низкий уровень) */
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    /* MAX31855 выдаёт данные сразу после падения CS, так что просто читаем 4 байта */
    st = HAL_SPI_Receive(&hspi2, buf, 4, 50);

    /* Деактивируем CS */
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    if (st != HAL_OK) {
        return st;
    }

    *raw = ((uint32_t)buf[0] << 24) |
           ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8)  |
           ((uint32_t)buf[3] << 0);

    return HAL_OK;
}

/* Преобразование сырых данных MAX31855 в температуры */
HAL_StatusTypeDef MAX31855_ReadTemperature(MAX31855_Data *out)
{
    uint32_t v;
    if (out == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef st = MAX31855_ReadRaw(&v);
    if (st != HAL_OK) {
        return st;
    }

    /* Разбор формата MAX31855:
     * Биты 31..18: термопара (14 бит, знаковое, шаг 0.25 °C)
     * Бит 16: fault
     * Биты 15..4: внутренняя температура (12 бит, знаковое, шаг 0.0625 °C)
     * Бит 2: SCV, бит 1: SCG, бит 0: OC
     */

    /* Термопара */
    int32_t tc_raw = (int32_t)(v >> 18);        /* берём старшие 14 бит в знаковом виде */
    if (tc_raw & 0x2000) {                      /* знак для 14-битного значения */
        tc_raw |= ~0x3FFF;
    }
    out->thermocouple_temp_c = (float)tc_raw * 0.25f;

    /* Внутренняя температура */
    int32_t cj_raw = (int32_t)((v >> 4) & 0x0FFF); /* 12 бит */
    if (cj_raw & 0x0800) {                        /* знак для 12-битного значения */
        cj_raw |= ~0x0FFF;
    }
    out->internal_temp_c = (float)cj_raw * 0.0625f;

    /* Флаги неисправности */
    out->fault = (uint8_t)((v >> 16) & 0x1);
    out->scv   = (uint8_t)((v >> 2)  & 0x1);
    out->scg   = (uint8_t)((v >> 1)  & 0x1);
    out->oc    = (uint8_t)((v >> 0)  & 0x1);

    return HAL_OK;
}

