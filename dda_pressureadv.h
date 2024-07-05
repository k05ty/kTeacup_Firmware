#ifndef DDA_PRESSUREADV_H
#define DDA_PRESSUREADV_H

#include <stdint.h>
#include "config_wrapper.h"
#include "dda.h"
#include "dda_maths.h"
#include "memory_barrier.h"
#include "debug.h"
#include "sersendf.h"

#ifdef PRESSURE_ADV

void dda_calculate_adv(DDA* dda);
void dda_join_adv(DDA* prev, DDA* current);

#endif
#endif