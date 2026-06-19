/*
  update this file for new releases
 */
#define VERSION_MAJOR 2
#define VERSION_MINOR 20

/*
 * EEPROM layout version.  Bump ONLY when the struct layout of EEprom_t
 * changes (field removed, resized, or reordered).  Adding new fields
 * in reserved / unused bytes does NOT need a bump.
 * v3 -> v4: added can.spd{} (SPEED_KP/KI/KD/FF_KDRAG/ADAPT_LO/HI)
 */
#define EEPROM_VERSION 4
