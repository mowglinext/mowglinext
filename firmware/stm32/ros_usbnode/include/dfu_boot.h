#ifndef MOWGLI_DFU_BOOT_H
#define MOWGLI_DFU_BOOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /* Called by the F401 Reset_Handler immediately after loading SP, before C
   * runtime initialization. It consumes the NOLOAD marker on every reset. */
  void DFU_PreRuntime(void);

  /* Called only after the main-loop stop window. The marker occupies the
   * linker-reserved .dfu_magic NOLOAD section, never an arbitrary SRAM word. */
  void DFU_ArmAndReset(void);

#ifdef __cplusplus
}
#endif

#endif
