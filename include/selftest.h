/**
 * @file selftest.h
 * @brief Boot-time self-test: drive forward/backward one wheel revolution,
 * steer left/right, using PE0 hold-at-boot as the trigger.
 */

#ifndef __SELFTEST_H
#define __SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run the self-test sequence if PE0 is held down at boot. No-op
 * (returns immediately) if the button isn't held, or if the PD3 e-stop
 * switch is engaged.
 */
void selftest_run_if_requested(void);

/**
 * @brief Run the self-test sequence unconditionally (e.g. triggered by a
 * PE0 press during normal operation). Refuses (blinks 5x) if the PD3
 * e-stop switch is engaged.
 */
void selftest_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __SELFTEST_H */
