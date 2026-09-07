#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

#define MANIFEST_MAGIC 0x31564741UL /* "AGV1" little-endian */
#define MANIFEST_FORMAT 1UL
#define SRAM_BASE_ADDR 0x20000000UL
#define SRAM_END_ADDR  0x20020000UL

typedef struct {
  uint32_t magic;
  uint32_t format;
  uint32_t app_base;
  uint32_t app_size;
  uint32_t app_crc32;
  uint32_t header_crc32;
  uint32_t generation;
  uint32_t reserved;
} app_manifest_t;

static uint32_t crc32_bytes(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (len--) {
    crc ^= *data++;
    for (uint32_t bit = 0; bit < 8U; ++bit)
      crc = (crc >> 1U) ^ (0xEDB88320UL & (0U - (crc & 1U)));
  }
  return crc ^ 0xFFFFFFFFUL;
}
static void quiesce(void) {
  __disable_irq();
  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;
  for (uint32_t i = 0; i < 8U; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }
  __DSB();
  __ISB();
}

static bool vector_valid(uint32_t base, uint32_t limit) {
  const uint32_t sp = *(volatile const uint32_t *)base;
  const uint32_t reset = *(volatile const uint32_t *)(base + 4U);
  if (sp < SRAM_BASE_ADDR || sp > SRAM_END_ADDR || (sp & 3U) != 0U) return false;
  if ((reset & 1U) == 0U) return false;
  const uint32_t pc = reset & ~1UL;
  return pc >= base && pc < limit;
}

__attribute__((noreturn)) static void jump_vector(uint32_t base) {
  const uint32_t sp = *(volatile const uint32_t *)base;
  const uint32_t reset = *(volatile const uint32_t *)(base + 4U);
  quiesce();
  SCB->VTOR = base;
  __set_CONTROL(0U);
  __set_PSP(0U);
  __set_MSP(sp);
  __DSB();
  __ISB();
  __enable_irq();
  ((void (*)(void))reset)();
  while (1) {}
}

static uint32_t read_boot_request(void) {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;
  PWR->CR |= PWR_CR_DBP;
  for (volatile uint32_t i = 0; i < 1000U; ++i) __NOP();
  const uint32_t value = RTC->BKP0R;
  RTC->BKP0R = 0U;
  __DSB();
  return value;
}

__attribute__((noreturn)) static void jump_system_dfu(void) {
  if (!vector_valid(SYSTEM_MEMORY, 0x20000000UL)) {
    NVIC_SystemReset();
    while (1) {}
  }
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  (void)RCC->APB2ENR;
  SYSCFG->MEMRMP = 0x01U;
  jump_vector(SYSTEM_MEMORY);
}

static bool application_valid(void) {
  const app_manifest_t *m = (const app_manifest_t *)MANIFEST_ADDR;
  if (m->magic != MANIFEST_MAGIC || m->format != MANIFEST_FORMAT) return false;
  if (m->app_base != APP_BASE || m->app_size == 0U) return false;
  if (m->app_size > (APP_LIMIT - APP_BASE)) return false;
  if (crc32_bytes((const uint8_t *)m, 20U) != m->header_crc32) return false;
  if (!vector_valid(APP_BASE, APP_LIMIT)) return false;
  return crc32_bytes((const uint8_t *)APP_BASE, m->app_size) == m->app_crc32;
}

int main(void) {
  const uint32_t request = read_boot_request();
  if (request == BOOT_REQUEST_MAGIC) jump_system_dfu();
  if (application_valid()) jump_vector(APP_BASE);
  jump_system_dfu();
}
