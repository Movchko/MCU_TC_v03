#include "main.h"
#include "app.h"

/* Буфер DMA для двух каналов АЦП */
uint16_t MCU_TC_ADC_VAL[MCU_TC_NUM_ADC_CHANNEL];

/* Фильтрация по скользящему среднему (по аналогии с PPKY) */
static uint16_t adc_filter_val[MCU_TC_NUM_ADC_CHANNEL];
static uint16_t adc_sma_buf[MCU_TC_NUM_ADC_CHANNEL][MCU_TC_FILTERSIZE];
static uint32_t adc_sma_sum[MCU_TC_NUM_ADC_CHANNEL];
static uint8_t  adc_sma_fill_index[MCU_TC_NUM_ADC_CHANNEL];
static uint8_t  adc_sma_index[MCU_TC_NUM_ADC_CHANNEL];

static uint16_t SmaProcess(uint8_t num, uint16_t val)
{
    uint16_t old_val = 0;

    if (adc_sma_fill_index[num] == MCU_TC_FILTERSIZE) {
        old_val = adc_sma_buf[num][adc_sma_index[num]];
        adc_sma_sum[num] -= old_val;
    } else {
        adc_sma_fill_index[num]++;
    }

    adc_sma_buf[num][adc_sma_index[num]] = val;
    adc_sma_sum[num] += val;

    adc_sma_index[num]++;
    if (adc_sma_index[num] >= MCU_TC_FILTERSIZE) {
        adc_sma_index[num] = 0;
    }

    uint32_t result = adc_sma_sum[num] / adc_sma_fill_index[num];
    return (uint16_t)result;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) {
        return;
    }

    /* MCU_TC_ADC_VAL содержит два канала: LINE1_L и LINE1_H */
    for (uint8_t i = 0; i < MCU_TC_NUM_ADC_CHANNEL; i++) {
        adc_filter_val[i] = SmaProcess(i, MCU_TC_ADC_VAL[i]);
    }

    /* Виртуальному ДПТ передаём уже отфильтрованные значения */
    App_SetDPTAdcValues(adc_filter_val[0], adc_filter_val[1]);
}

