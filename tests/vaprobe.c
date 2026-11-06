/*
 * vaprobe - a minimal vainfo: list the decode profiles a VA driver offers,
 * with the surface pixel formats of each decode config.
 * Only needs libva and libva-drm (vainfo also pulls in X11 and Wayland).
 *
 *   vaprobe [/dev/dri/renderD128]
 *
 * Copyright (C) 2026 Arun Iyengar <aiyengar1990@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_str.h>

/* The surface pixel formats a decode config offers, in the driver's order. */
static void print_surface_formats(VADisplay dpy, VAProfile profile,
				  VAEntrypoint entrypoint)
{
	VASurfaceAttrib *attribs;
	unsigned int count = 0;
	VAConfigID config;
	const char *sep = " formats ";

	if (vaCreateConfig(dpy, profile, entrypoint, NULL, 0, &config) !=
	    VA_STATUS_SUCCESS)
		return;
	if (vaQuerySurfaceAttributes(dpy, config, NULL, &count) ==
	    VA_STATUS_SUCCESS && count) {
		attribs = calloc(count, sizeof(*attribs));
		if (attribs && vaQuerySurfaceAttributes(dpy, config, attribs,
							&count) ==
		    VA_STATUS_SUCCESS) {
			for (unsigned int i = 0; i < count; i++) {
				uint32_t f = attribs[i].value.value.i;

				if (attribs[i].type != VASurfaceAttribPixelFormat)
					continue;
				printf("%s%c%c%c%c", sep, f & 0xff, f >> 8 & 0xff,
				       f >> 16 & 0xff, f >> 24 & 0xff);
				sep = ",";
			}
		}
		free(attribs);
	}
	vaDestroyConfig(dpy, config);
}

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
			printf("  %-32s %-22s rt_format 0x%x", vaProfileStr(profiles[i]),
			       vaEntrypointStr(eps[j]), attr.value);
			if (eps[j] == VAEntrypointVLD) {
				print_surface_formats(dpy, profiles[i], eps[j]);
				ok++;
			}
			printf("\n");
		}
		free(eps);
	}
	free(profiles);
	vaTerminate(dpy);
	close(fd);
	printf("%d decode profiles\n", ok);
	return ok ? 0 : 1;
}
