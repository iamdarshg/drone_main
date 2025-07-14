MEMORY
{
  FLASH_M4 (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  FLASH_M0 (rx) : ORIGIN = 0x08080000, LENGTH = 256K
  SRAM      (rwx): ORIGIN = 0x10000000, LENGTH = 136K
}

SECTIONS
{
  /* M4 */
  .isr_vector_m4 : { KEEP(*(.m4_vector)) } > FLASH_M4
  .text_m4       : { *(.m4_text*) *(.text*) *(.rodata*) } > FLASH_M4
  .data_m4       : { *(.m4_data*) *(.data*) } > SRAM AT> FLASH_M4
  .bss_m4        : { *(.m4_bss*)  *(.bss*)  *(COMMON) } > SRAM

  /* M0 */
  .isr_vector_m0 : { KEEP(*(.m0_vector)) } > FLASH_M0
  .text_m0       : { *(.m0_text*) *(.text*) *(.rodata*) } > FLASH_M0
  .data_m0       : { *(.m0_data*) *(.data*) } > SRAM AT> FLASH_M0
  .bss_m0        : { *(.m0_bss*)  *(.bss*)  *(COMMON) } > SRAM
}
