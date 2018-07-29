/*
 * Copyright (c) 2018 Oticon A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NATIVE_POSIX_CMDLINE_H
#define _NATIVE_POSIX_CMDLINE_H

#include "cmdline_common.h"

#ifdef __cplusplus
extern "C" {
#endif

struct args_t {
	double stop_at;
	double rtc_offset;
	double rt_drift;
	double rt_ratio;
#if defined(CONFIG_FAKE_ENTROPY_NATIVE_POSIX)
	u32_t seed;
#endif
};

void native_handle_cmd_line(int argc, char *argv[]);
void native_get_cmd_line_args(int *argc, char ***argv);
void native_get_test_cmd_line_args(int *argc, char ***argv);
void native_add_command_line_opts(struct args_struct_t *args);
void native_cleanup_cmd_line(void);

#ifdef __cplusplus
}
#endif

#endif /* _NATIVE_POSIX_CMDLINE_H */
