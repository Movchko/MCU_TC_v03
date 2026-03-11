/**
 * @file mku_cfg_flash.h
 * @brief Область конфигов MKUCfg во Flash — объявления секции .mku_cfg
 *
 * Адрес и размер задаются в STM32H523RETX_FLASH.ld:
 *   FLASH_CFG: ORIGIN = 0x0807E000, LENGTH = 8K
 *   Секция .mku_cfg (NOLOAD) — читается/пишется в runtime
 */
#ifndef MCU_MKU_CFG_FLASH_H
#define MCU_MKU_CFG_FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Символы из linker script — границы области конфигов */
extern uint8_t _mku_cfg_start[];
extern uint8_t _mku_cfg_end[];

#define FLASH_CFG_ADDR       ((uint32_t)_mku_cfg_start)
#define FLASH_CFG_SIZE       ((uint32_t)(_mku_cfg_end - _mku_cfg_start))
#define FLASH_CFG_SIZE_BYTES 0x2000u   /* 8 KB — для массивов (константа времени компиляции) */

#ifdef __cplusplus
}
#endif

#endif /* MCU_MKU_CFG_FLASH_H */
