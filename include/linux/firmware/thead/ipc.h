// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#ifndef _SC_IPC_H
#define _SC_IPC_H

#include <linux/device.h>
#include <linux/types.h>

#define AON_RPC_MSG_MAGIC	       (0xef)
#define TH1520_AON_RPC_VERSION		2
#define TH1520_AON_RPC_MSG_NUM		7

struct th1520_aon_ipc;

enum th1520_aon_rpc_svc {
	TH1520_AON_RPC_SVC_UNKNOWN = 0,
	TH1520_AON_RPC_SVC_PM =	1,
	TH1520_AON_RPC_SVC_MISC = 2,
	TH1520_AON_RPC_SVC_AVFS = 3,
	TH1520_AON_RPC_SVC_SYS = 4,
	TH1520_AON_RPC_SVC_WDG = 5,
	TH1520_AON_RPC_SVC_LPM = 6,
	TH1520_AON_RPC_SVC_MAX = 0x3F,
};

enum th1520_aon_misc_func {
	TH1520_AON_MISC_FUNC_UNKNOWN = 0,
	TH1520_AON_MISC_FUNC_SET_CONTROL = 1,
	TH1520_AON_MISC_FUNC_GET_CONTROL = 2,
	TH1520_AON_MISC_FUNC_REGDUMP_CFG = 3,
};

enum th1520_aon_wdg_func {
	TH1520_AON_WDG_FUNC_UNKNOWN = 0,
	TH1520_AON_WDG_FUNC_START = 1,
	TH1520_AON_WDG_FUNC_STOP = 2,
	TH1520_AON_WDG_FUNC_PING = 3,
	TH1520_AON_WDG_FUNC_TIMEOUTSET = 4,
	TH1520_AON_WDG_FUNC_RESTART = 5,
	TH1520_AON_WDG_FUNC_GET_STATE = 6,
	TH1520_AON_WDG_FUNC_POWER_OFF = 7,
	TH1520_AON_WDG_FUNC_AON_WDT_ON  = 8,
	TH1520_AON_WDG_FUNC_AON_WDT_OFF = 9,
};

enum th1520_aon_sys_func {
	TH1520_AON_SYS_FUNC_UNKNOWN = 0,
	TH1520_AON_SYS_FUNC_AON_RESERVE_MEM = 1,
};

enum th1520_aon_lpm_func {
	TH1520_AON_LPM_FUNC_UNKNOWN = 0,
	TH1520_AON_LPM_FUNC_REQUIRE_STR = 1,
	TH1520_AON_LPM_FUNC_RESUME_STR = 2,
	TH1520_AON_LPM_FUNC_REQUIRE_STD = 3,
	TH1520_AON_LPM_FUNC_CPUHP = 4,
	TH1520_AON_LPM_FUNC_REGDUMP_CFG = 5,
};

enum th1520_aon_pm_func {
	TH1520_AON_PM_FUNC_UNKNOWN = 0,
	TH1520_AON_PM_FUNC_SET_RESOURCE_REGULATOR = 1,
	TH1520_AON_PM_FUNC_GET_RESOURCE_REGULATOR = 2,
	TH1520_AON_PM_FUNC_SET_RESOURCE_POWER_MODE = 3,
	TH1520_AON_PM_FUNC_PWR_SET = 4,
	TH1520_AON_PM_FUNC_PWR_GET = 5,
	TH1520_AON_PM_FUNC_CHECK_FAULT = 6,
	TH1520_AON_PM_FUNC_GET_TEMPERATURE = 7,
};

struct th1520_aon_rpc_msg_hdr {
	uint8_t ver;    ///< version of msg hdr
	uint8_t size;   ///< msg size ,uinit in bytes,the size includes rpc msg header self.
	uint8_t svc;    ///< rpc main service id
	uint8_t func;   ///< rpc sub func id of specific service, sent by caller
} __packed __aligned(1);

struct th1520_aon_rpc_ack_common {
    struct th1520_aon_rpc_msg_hdr hdr;
	u8   err_code;
} __packed __aligned(1);

#define RPC_SVC_MSG_TYPE_DATA     0
#define RPC_SVC_MSG_TYPE_ACK      1
#define RPC_SVC_MSG_NEED_ACK      0
#define RPC_SVC_MSG_NO_NEED_ACK   1

#define RPC_GET_VER(MESG)			((MESG)->ver)
#define RPC_SET_VER(MESG, VER)			((MESG)->ver = (VER))
#define RPC_GET_SVC_ID(MESG)			((MESG)->svc & 0x3F)
#define RPC_SET_SVC_ID(MESG, ID)	        ((MESG)->svc |= 0x3F & (ID))
#define RPC_GET_SVC_FLAG_MSG_TYPE(MESG)	        (((MESG)->svc & 0x80) >> 7)
#define RPC_SET_SVC_FLAG_MSG_TYPE(MESG, TYPE)	((MESG)->svc |= (TYPE) << 7)
#define RPC_GET_SVC_FLAG_ACK_TYPE(MESG)		(((MESG)->svc & 0x40) >> 6)
#define RPC_SET_SVC_FLAG_ACK_TYPE(MESG, ACK)    ((MESG)->svc |= (ACK) << 6)

#define RPC_SET_BE64(MESG, OFFSET, SET_DATA)                                                                                  do {uint8_t* data = (uint8_t*)(MESG);       \
                                                                                                                               data[OFFSET + 7]  =     (SET_DATA) & 0xFF;  \
                                                                                                                               data[OFFSET + 6]  = ((SET_DATA) & 0xFF00) >> 8; \
                                                                                                                               data[OFFSET + 5]  = ((SET_DATA) & 0xFF0000) >> 16; \
                                                                                                                               data[OFFSET + 4]  = ((SET_DATA) & 0xFF000000) >> 24; \
                                                                                                                               data[OFFSET + 3]  = ((SET_DATA) & 0xFF00000000) >> 32; \
                                                                                                                               data[OFFSET + 2]  = ((SET_DATA) & 0xFF0000000000) >> 40; \
                                                                                                                               data[OFFSET + 1]  = ((SET_DATA) & 0xFF000000000000) >> 48; \
                                                                                                                               data[OFFSET + 0]  = ((SET_DATA) & 0xFF00000000000000) >> 56; \
                                                                                                                              } while(0)

#define RPC_SET_BE32(MESG, OFFSET, SET_DATA)                                                                          do { uint8_t* data = (uint8_t*)(MESG);    \
                                                                                                                               data[OFFSET +  3]  =     (SET_DATA) & 0xFF;  \
                                                                                                                               data[OFFSET + 2]  = ((SET_DATA) & 0xFF00) >> 8; \
                                                                                                                               data[OFFSET + 1]  = ((SET_DATA) & 0xFF0000) >> 16; \
                                                                                                                               data[OFFSET + 0]  = ((SET_DATA) & 0xFF000000) >> 24; \
                                                                                                                              } while(0)
#define RPC_SET_BE16(MESG, OFFSET, SET_DATA)                                                                          do { uint8_t* data = (uint8_t*)(MESG);   \
                                                                                                                               data[OFFSET  + 1]  = (SET_DATA) & 0xFF; \
                                                                                                                               data[OFFSET + 0]  = ((SET_DATA) & 0xFF00) >> 8; \
                                                                                                                              } while(0)
#define RPC_SET_U8(MESG, OFFSET, SET_DATA)                                                                          do { uint8_t* data = (uint8_t*)(MESG);  \
                                                                                                                         data[OFFSET]  = (SET_DATA) & 0xFF; \
																														} while(0)
#define RPC_GET_BE64(MESG, OFFSET, PTR) \
			    do {uint8_t* data = (uint8_t*)(MESG); \
                                *(uint32_t*)(PTR) = (data[OFFSET + 7] \
						    | data[OFFSET + 6] << 8 \
						    | data[OFFSET  + 5] << 16 \
						    | data[OFFSET + 4] << 24 \
						    | data[OFFSET + 3] << 32 \
						    | data[OFFSET + 2] << 40 \
						    | data[OFFSET + 1] << 48 \
						    | data[OFFSET + 0] << 56); \
				} while(0)
#define RPC_GET_BE32(MESG, OFFSET, PTR) \
			    do {uint8_t* data = (uint8_t*)(MESG); \
                                *(uint32_t*)(PTR) = (data[OFFSET + 3] \
						| data[OFFSET + 2] << 8 \
						| data[OFFSET  + 1] << 16 \
						| data[OFFSET + 0] << 24); \
				} while(0)
#define RPC_GET_BE16(MESG, OFFSET, PTR) \
			    do {uint8_t* data = (uint8_t*)(MESG); \
                                *(uint16_t*)(PTR) = (data[OFFSET  + 1] \
						| data[OFFSET + 0] << 8); \
				} while(0)
#define RPC_GET_U8(MESG, OFFSET, PTR) \
			    do {uint8_t* data = (uint8_t*)(MESG); \
                                *(uint8_t*)(PTR) = (data[OFFSET]);  \
                                } while(0)

/*
 * Defines for SC PM Power Mode
 */
#define TH1520_AON_PM_PW_MODE_OFF		0	/* Power off */
#define TH1520_AON_PM_PW_MODE_STBY		1	/* Power in standby */
#define TH1520_AON_PM_PW_MODE_LP		2	/* Power in low-power */
#define TH1520_AON_PM_PW_MODE_ON		3	/* Power on */

int th1520_aon_call_rpc(struct th1520_aon_ipc *ipc,
			void *msg, void *ack_msg, bool have_resp);
int th1520_aon_get_handle(struct th1520_aon_ipc **ipc);
int th1520_aon_misc_set_control(struct th1520_aon_ipc *ipc,
				u16 resource, u32 ctrl, u32 val);
int th1520_aon_misc_get_control(struct th1520_aon_ipc *ipc,
				u16 resource, u32 ctrl, u32 *val);
#endif /* _SC_IPC_H */
