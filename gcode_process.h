#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

#include	"gcode_parse.h"


// the current tool
extern uint8_t tool;
// the tool to be changed when we get an M6
extern uint8_t next_tool;
#ifdef PRESSURE_ADV
// pressure advance k-factor
extern uint32_t k;
#endif

// when we have a whole line, feed it to this
void process_gcode_command(void);

#endif	/* _GCODE_PROCESS_H */
