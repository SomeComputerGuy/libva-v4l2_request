/*
 * vaprobe - a minimal vainfo: list the decode profiles a VA driver offers.
 * Only needs libva and libva-drm (vainfo also pulls in X11 and Wayland).
 *
 *   vaprobe [/dev/dri/renderD128]
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_str.h>

int main(int argc, char **argv)
{
	const char *node = argc > 1 ? argv[1] : "/dev/dri/renderD128";
	int fd = open(node, O_RDWR);
	VADisplay dpy;
	VAProfile *profiles;
	int major, minor, n = 0, ok = 0;

	if (fd < 0) {
		perror(node);
		return 2;
	}
	dpy = vaGetDisplayDRM(fd);
	if (!dpy || vaInitialize(dpy, &major, &minor) != VA_STATUS_SUCCESS) {
		fprintf(stderr, "vaInitialize failed on %s\n", node);
		return 1;
	}
	printf("VA-API %d.%d, driver: %s\n", major, minor, vaQueryVendorString(dpy));

	profiles = calloc(vaMaxNumProfiles(dpy), sizeof(*profiles));
	vaQueryConfigProfiles(dpy, profiles, &n);
	for (int i = 0; i < n; i++) {
		VAEntrypoint *eps = calloc(vaMaxNumEntrypoints(dpy), sizeof(*eps));
		int m = 0;

		vaQueryConfigEntrypoints(dpy, profiles[i], eps, &m);
		for (int j = 0; j < m; j++) {
			VAConfigAttrib attr = { .type = VAConfigAttribRTFormat };

			vaGetConfigAttributes(dpy, profiles[i], eps[j], &attr, 1);
			printf("  %-32s %-22s rt_format 0x%x\n", vaProfileStr(profiles[i]),
			       vaEntrypointStr(eps[j]), attr.value);
			if (eps[j] == VAEntrypointVLD)
				ok++;
		}
		free(eps);
	}
	free(profiles);
	vaTerminate(dpy);
	close(fd);
	printf("%d decode profiles\n", ok);
	return ok ? 0 : 1;
}
