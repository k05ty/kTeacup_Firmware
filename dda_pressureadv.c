#include "dda_pressureadv.h"

#ifdef PRESSURE_ADV

void dda_calculate_adv(DDA* dda) {
    if (dda->delta[E] > 0 && dda->e_direction == 1 && dda->endpoint.k > 0)
    {
        // This block calculates amount of advanced steps. Result is slightly less that it should be, but i think it's ok
        // With G1 X20 E0.7982 F1200 its result is 15 (have to be 18)
        // With G1 X40 E1.5965 F4200 its result is 55 (have to be 64)
        dda->c_extruder = muldiv(dda->c_min, dda->total_steps, dda->delta[E]); // extruder's velocity (ticks/step)
        dda->adv_start = dda->endpoint.k / dda->c_extruder; // mm/mm/tick / ticks/step => ticks / ticks/step => ticks * step/ticks

        // This block calculates delta steps for advanced steps as like it is one more axis that have to travel all adv_steps during acceleration
        dda->adv_delta = muldiv(dda->total_steps, dda->adv_start, dda->rampup_steps_before_lookahead);
        if (dda->adv_delta > dda->total_steps)
            dda->adv_delta = dda->total_steps;
    }
    else
    {
        dda->adv_start = 0;
        dda->adv_delta = 0;
    }
    dda->adv_end = dda->adv_start;
}

void dda_join_adv(DDA* prev, DDA* current) {
    uint8_t prev_id, current_id;
    uint32_t prev_c_extruder, current_c_extruder, cross_c_extruder;
    uint32_t prev_adv_end, current_adv_start;
    uint32_t cross_F_extruder;
    DDA* slower;

    // Bail out if there's nothing to join.
    if (!prev || current->crossF == 0)
        return;

    // Make sure we have 2 moves and the previous move is not already active
    if (prev->live == 0) {
        // Copy DDA ids to verify later that nothing changed during calculations
        ATOMIC_START
            prev_id = prev->id;
            current_id = current->id;
        ATOMIC_END

        // Calculations
        slower = prev;
        if (current->endpoint.F < slower->endpoint.F)
            slower = current;

        cross_F_extruder = muldiv(current->delta[E], slower->endpoint.F, current->total_steps);
        
        prev_c_extruder = prev->c_extruder;
        current_c_extruder = current->c_extruder;
        cross_c_extruder = (((slower->distance * 2400) / slower->total_steps) * (F_CPU / 40000)) / cross_F_extruder;

        if (prev_c_extruder < cross_c_extruder) {
            // If we are decelerating from prev to cross (prev F_E > cross F_E)
            prev_adv_end = current->endpoint.k / (cross_c_extruder - prev_c_extruder);
        }
        else {
            prev_adv_end = 0;
        }

        if (current_c_extruder < cross_c_extruder) {
            // If we are accelerating from cross to current (current F_E > cross F_E)
            current_adv_start = current->endpoint.k / (cross_c_extruder - current_c_extruder);
        }
        else {
            current_adv_start = 0;
        }

        //sersendf_P(PSTR("\tprev_F_extruder[%lu]\n"), muldiv(prev->delta[E], prev->endpoint.F, prev->total_steps));
        //sersendf_P(PSTR("\tcurrent_F_extruder[%lu]\n"), muldiv(current->delta[E], current->endpoint.F, current->total_steps));

        sersendf_P(PSTR("\tprev_c_extruder[%lx]\n"), prev_c_extruder);
        sersendf_P(PSTR("\tcurrent_c_extruder[%lx]\n"), current_c_extruder);
        sersendf_P(PSTR("\tcross_F_extruder[%lu]\n"), cross_F_extruder);
        sersendf_P(PSTR("\tcross_c_extruder[%lx]\n"), cross_c_extruder);
        sersendf_P(PSTR("\tprev_adv_end[%lu]\n"), prev_adv_end);
        sersendf_P(PSTR("\tcurrent_adv_start[%lu]\n"), current_adv_start);

        #ifdef DEBUG
        uint8_t timeout = 0;
        #endif

        ATOMIC_START
        // Evaluation: determine how we did...

        // Determine if we are fast enough - if not, just leave the moves
        // Note: to test if the previous move was already executed and replaced by a new
        // move, we compare the DDA id.
        if (prev->live == 0 && prev->id == prev_id && current->live == 0 && current->id == current_id) {
            
            prev->adv_end = prev_adv_end;
            current->adv_start = current_adv_start;
            
        }
        #ifdef DEBUG
        else timeout = 1;
        #endif
        ATOMIC_END

        // sersendf_P(PSTR("Pressure advance {\n"));
        // sersendf_P(PSTR("\tprev_id[%su]\n"), prev_id);
        // sersendf_P(PSTR("\tprev_adv_start[%lu]\n"), prev->adv_start);
        // sersendf_P(PSTR("\tcurrent_id[%su]\n"), current_id);
        // sersendf_P(PSTR("\tcurrent_adv_start[%lu]\n"), current->adv_start);
        // sersendf_P(PSTR("}\n"));
        // If we were not fast enough, any feedback will happen outside the atomic block:
        #ifdef DEBUG
        if (timeout) {
            sersendf_P(PSTR("// Notice: pressure adv calculations not fast enough\n"));
        }
        #endif
    }
}

#endif