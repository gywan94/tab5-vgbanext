#ifndef GLOBALS_H
#define GLOBALS_H

#include "types.h"
#include <stdint.h>

/*performance boost tweaks. */
#if USE_TWEAKS
    #define USE_TWEAK_SPEEDHACK 1
	#define USE_TWEAK_MEMFUNC 1
#endif

#define PIX_BUFFER_SCREEN_WIDTH 256

extern int saveType;
extern bool useBios;
extern bool skipBios;
extern int cpuSaveType;
extern bool mirroringEnable;
extern bool enableRtc;
extern bool skipSaveGameBattery; /* skip battery data when reading save states */

extern int cpuDmaCount;

extern uint8_t *rom;
extern uint8_t *bios;
extern uint8_t *vram;
extern uint16_t *pix;
extern uint8_t *oam;
extern uint8_t *ioMem;
extern uint8_t *internalRAM;
extern uint8_t *workRAM;
extern uint8_t *paletteRAM;

/* tab5: ROM buffer sized to the actual ROM instead of a flat 32 MB (see gba.c).
 * g_rom_mask wraps every ROM access inside the smaller buffer. */
extern uint32_t g_rom_alloc;
extern uint32_t g_rom_mask;
extern uint32_t g_swi_base;

#endif /* GLOBALS_H */
