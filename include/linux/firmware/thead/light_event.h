/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _LIGHT_EVENT_H
#define _LIGHT_EVENT_H

enum light_rebootmode_index {
	/* C902 event rebootmode */
        LIGHT_EVENT_PMIC_RESET = 0x0,
        LIGHT_EVENT_PMIC_ONKEY,
        LIGHT_EVENT_PMIC_POWERUP,

	/* C910 event rebootmode */
        LIGHT_EVENT_SW_REBOOT = 0x20,
        LIGHT_EVENT_SW_WATCHDOG,
        LIGHT_EVENT_SW_PANIC,
        LIGHT_EVENT_SW_HANG,
        LIGHT_EVENT_MAX,
};

#if IS_ENABLED(CONFIG_LIGHT_REBOOTMODE)
extern int light_event_set_rebootmode(enum light_rebootmode_index mode);
extern int light_event_get_rebootmode(enum light_rebootmode_index *mode);
#else
static int light_event_set_rebootmode(enum light_rebootmode_index mode)
{
	return 0;
}
static int light_event_get_rebootmode(enum light_rebootmode_index *mode)
{
	*mode = LIGHT_EVENT_MAX;

	return 0;
}
#endif

#endif
