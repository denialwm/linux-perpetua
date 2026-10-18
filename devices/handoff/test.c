// SPDX-License-Identifier: GPL-2.0-only
/*
 * Runs the kernel's device tree handoff (arch/arm64/kernel/fdt-handoff.c) on
 * the host: a device's built-in tree against a tree its bootloader passed.
 * See devices/handoff/test for the usage.
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/types.h>
#include "fdt-handoff.c"

char __dtb_start[1], __dtb_end[1];

static void *load(const char *path, size_t capacity)
{
	FILE *file = fopen(path, "rb");
	void *data = calloc(1, capacity);
	size_t n;

	if (!file || !data) {
		fprintf(stderr, "%s: %s\n", path, strerror(errno));
		exit(1);
	}
	n = fread(data, 1, capacity, file);
	if (!n || ferror(file) || !feof(file)) {
		fprintf(stderr, "%s: unreadable or larger than %zu bytes\n", path, capacity);
		exit(1);
	}
	fclose(file);
	if (fdt_check_header(data) || fdt_totalsize(data) > n) {
		fprintf(stderr, "%s: not a flattened device tree\n", path);
		exit(1);
	}
	return data;
}

static const char *chosen_prop(const void *fdt, const char *name, int *length)
{
	return fdt_getprop(fdt, fdt_path_offset(fdt, "/chosen"), name, length);
}

static void report(const void *builtin, const void *out)
{
	int node, length, handoff = fdt_path_offset(builtin, "/chosen/fdt-handoff");
	const fdt64_t *rd;

	printf("model: %s\n", (const char *)fdt_getprop(out, 0, "model", NULL));
	printf("bootargs: %s\n", chosen_prop(out, "bootargs", NULL) ?: "(none)");
	fdt_for_each_subnode(node, out, 0)
		if (fdt_handoff_is_memory(out, node) &&
		    fdt_getprop(out, node, "reg", &length))
			printf("memory: %d ranges, %d reservations\n", length / 16,
			       fdt_num_mem_rsv(out));
	rd = (const fdt64_t *)chosen_prop(out, "linux,initrd-start", &length);
	if (rd && length == 8)
		printf("initrd: %#llx-%#llx\n", (unsigned long long)fdt64_ld(rd),
		       (unsigned long long)fdt64_ld((const fdt64_t *)
					chosen_prop(out, "linux,initrd-end", NULL)));
	fdt_for_each_subnode(node, builtin, handoff) {
		const fdt32_t *target = fdt_getprop(builtin, node, "target", NULL);
		const char *property = fdt_getprop(builtin, node, "target-property", NULL);
		int to = target ? fdt_node_offset_by_phandle(out, fdt32_ld(target)) : -1;
		const u8 *value = to >= 0 ? fdt_getprop(out, to, property, &length) : NULL;

		printf("%s: ", fdt_get_name(builtin, node, NULL));
		if (!value) {
			puts("not imported (absent from the bootloader's tree)");
			continue;
		}
		for (int i = 0; i < length; i++)
			printf("%02x%s", value[i], i + 1 < length ? ":" : "\n");
	}
}

int main(int argc, char **argv)
{
	struct fdt_handoff_work *work = calloc(1, sizeof(*work));
	void *out = calloc(1, FDT_HANDOFF_CAPACITY);
	unsigned long long start, end;
	const char *builtin;
	void *stock;
	int ret;

	if (argc != 5 && argc != 6) {
		fputs("usage: test BUILTIN.dtb STOCK.dtb KERNEL-START KERNEL-END [OUTPUT.dtb]\n",
		      stderr);
		return 1;
	}
	builtin = load(argv[1], FDT_HANDOFF_CAPACITY);
	stock = load(argv[2], FDT_HANDOFF_CAPACITY);
	start = strtoull(argv[3], NULL, 0);
	end = strtoull(argv[4], NULL, 0);
	ret = fdt_handoff_prepare(stock, builtin, builtin + fdt_totalsize(builtin), out, work,
				  start, end);
	if (ret == -FDT_ERR_NOTFOUND) {
		puts("not this device: the kernel boots the bootloader's tree");
		return 2;
	}
	if (ret) {
		printf("broken: %s (%d); the kernel would stop\n", fdt_strerror(ret), ret);
		return 1;
	}
	puts("ready: the kernel boots its built-in tree");
	report(builtin, out);
	if (argc == 6) {
		FILE *file = fopen(argv[5], "wb");

		if (!file || fwrite(out, 1, fdt_totalsize(out), file) != fdt_totalsize(out) ||
		    fclose(file)) {
			fprintf(stderr, "%s: %s\n", argv[5], strerror(errno));
			return 1;
		}
	}
	return 0;
}
