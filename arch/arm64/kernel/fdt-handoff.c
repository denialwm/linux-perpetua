// SPDX-License-Identifier: GPL-2.0-only
/*
 * Device tree handoff from vendor bootloaders
 *
 * The Android bootloaders of phones and tablets only boot a vendor device
 * tree picked from their own images. They fix that tree up with the memory
 * map, the firmware reservations, the initramfs and the board addresses, but
 * mainline cannot use the rest of it. When the device tree built into the
 * image (CONFIG_BUILTIN_DTB_NAME) describes the bootloader's tree in its
 * /chosen/fdt-handoff node, the kernel boots the built-in tree instead and
 * carries over what only the bootloader knows. Any other bootloader tree
 * boots as usual. Documentation/devicetree/bindings/arm/fdt-handoff.yaml
 * describes the node.
 *
 * All the work happens on flattened trees in static buffers, before memblock
 * and before anything has been mapped from the bootloader's tree.
 */

#include <linux/init.h>
#include <linux/libfdt.h>
#include <linux/of_fdt.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/types.h>
#include <asm/processor.h>
#include <asm/setup.h>

#define FDT_HANDOFF_CAPACITY	(2 * 1024 * 1024)
#define FDT_HANDOFF_MAX_RANGES	256

struct fdt_handoff_range {
	u64 start, end;
};

/* Work buffers are owned by the caller, so this needs no large early stack. */
struct fdt_handoff_work {
	struct fdt_handoff_range ram[FDT_HANDOFF_MAX_RANGES];
	struct fdt_handoff_range reserved[FDT_HANDOFF_MAX_RANGES];
	struct fdt_handoff_range nomap[FDT_HANDOFF_MAX_RANGES];
	struct fdt_handoff_range own[FDT_HANDOFF_MAX_RANGES];
	int nr_ram, nr_reserved, nr_nomap, nr_own;
	fdt64_t memory_reg[FDT_HANDOFF_MAX_RANGES * 2];
};

/* Walks every node and property once, so later lookups cannot run off. */
static int __init fdt_handoff_check(const void *fdt)
{
	int ret, node = -1;

	ret = fdt_check_header(fdt);
	if (ret)
		return ret;
	if (fdt_totalsize(fdt) > FDT_HANDOFF_CAPACITY)
		return -FDT_ERR_NOSPACE;

	while ((node = fdt_next_node(fdt, node, NULL)) >= 0) {
		int prop = fdt_first_property_offset(fdt, node);

		while (prop >= 0) {
			const struct fdt_property *p;
			int length;

			p = fdt_get_property_by_offset(fdt, prop, &length);
			if (!p)
				return length;
			if (!fdt_get_string(fdt, fdt32_to_cpu(p->nameoff), &length))
				return length;
			prop = fdt_next_property_offset(fdt, prop);
		}
		if (prop != -FDT_ERR_NOTFOUND)
			return prop;
	}
	if (node != -FDT_ERR_NOTFOUND)
		return node;

	ret = fdt_num_mem_rsv(fdt);
	return ret < 0 ? ret : 0;
}

static const char * __init fdt_handoff_string(const void *fdt, int node, const char *name)
{
	int length;
	const char *value = fdt_getprop(fdt, node, name, &length);

	if (!value || length <= 0 || value[length - 1])
		return NULL;
	return value;
}

/* Finds KEY at the start of a word separated by spaces or newlines. */
static const char * __init fdt_handoff_word(const char *text, const char *key)
{
	const char *at = text;

	while ((at = strstr(at, key))) {
		if (at == text || at[-1] == ' ' || at[-1] == '\n')
			return at;
		at++;
	}
	return NULL;
}

static bool __init fdt_handoff_has_word(const char *text, const char *word)
{
	const char *at = fdt_handoff_word(text, word);
	char end;

	if (!at)
		return false;
	end = at[strlen(word)];
	return !end || end == ' ' || end == '\n';
}

enum fdt_handoff_identity {
	FDT_HANDOFF_COMPATIBLE,
	FDT_HANDOFF_MODEL,
	FDT_HANDOFF_BOOTARGS,
};

static const char *const fdt_handoff_identities[] __initconst = {
	[FDT_HANDOFF_COMPATIBLE] = "stock-compatible",
	[FDT_HANDOFF_MODEL] = "stock-model",
	[FDT_HANDOFF_BOOTARGS] = "stock-bootargs",
};

static bool __init fdt_handoff_holds(const void *stock, const char *bootargs,
				     enum fdt_handoff_identity identity,
				     const char *value)
{
	const char *model;

	switch (identity) {
	case FDT_HANDOFF_COMPATIBLE:
		return !fdt_node_check_compatible(stock, 0, value);
	case FDT_HANDOFF_MODEL:
		model = fdt_handoff_string(stock, 0, "model");
		return model && !strcmp(model, value);
	default:
		return bootargs && fdt_handoff_has_word(bootargs, value);
	}
}

/*
 * Matches the bootloader's tree STOCK against the description HANDOFF in the
 * built-in tree MAINLINE: it must hold one string of every identity property
 * there, and the root compatible one is required. Returns 0 on a match,
 * -FDT_ERR_NOTFOUND for another tree or -FDT_ERR_BADVALUE for a malformed
 * description.
 */
static int __init fdt_handoff_match(const void *stock, const char *bootargs,
				    const void *mainline, int handoff)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(fdt_handoff_identities); i++) {
		int length;
		const char *value = fdt_getprop(mainline, handoff,
						fdt_handoff_identities[i], &length);
		bool held = false;

		if (!value) {
			if (length != -FDT_ERR_NOTFOUND || i == FDT_HANDOFF_COMPATIBLE)
				return -FDT_ERR_BADVALUE;
			continue;
		}
		if (length <= 0 || value[length - 1])
			return -FDT_ERR_BADVALUE;
		for (const char *at = value; at < value + length; at += strlen(at) + 1) {
			if (!*at)
				return -FDT_ERR_BADVALUE;
			held = held || fdt_handoff_holds(stock, bootargs, i, at);
		}
		if (!held)
			return -FDT_ERR_NOTFOUND;
	}
	return 0;
}

static int __init fdt_handoff_add_range(struct fdt_handoff_range *ranges, int *count,
					u64 start, u64 size)
{
	if (!size)
		return 0;
	if (start + size < start || *count >= FDT_HANDOFF_MAX_RANGES)
		return -FDT_ERR_BADVALUE;
	ranges[(*count)++] = (struct fdt_handoff_range){ start, start + size };
	return 0;
}

/* Reads a number of one or two cells. */
static u64 __init fdt_handoff_cells(const fdt32_t *cells, int count)
{
	u64 value = 0;

	while (count--)
		value = value << 32 | fdt32_ld(cells++);
	return value;
}

static int __init fdt_handoff_ranges(const void *fdt, int node,
				     struct fdt_handoff_range *ranges, int *count)
{
	int length, parent = fdt_parent_offset(fdt, node);
	int address_cells = fdt_address_cells(fdt, parent);
	int size_cells = fdt_size_cells(fdt, parent);
	int stride = address_cells + size_cells;
	const fdt32_t *reg = fdt_getprop(fdt, node, "reg", &length);

	if (!reg)
		return length == -FDT_ERR_NOTFOUND ? 0 : length;
	if (address_cells < 1 || address_cells > 2 || size_cells < 1 ||
	    size_cells > 2 || length % (stride * 4))
		return -FDT_ERR_BADNCELLS;

	for (int i = 0; i < length / 4; i += stride) {
		int ret = fdt_handoff_add_range(ranges, count,
				fdt_handoff_cells(reg + i, address_cells),
				fdt_handoff_cells(reg + i + address_cells, size_cells));
		if (ret)
			return ret;
	}
	return 0;
}

static bool __init fdt_handoff_enabled(const void *fdt, int node)
{
	int size;
	const char *status = fdt_getprop(fdt, node, "status", &size);

	return !status || (size == 5 && !memcmp(status, "okay", 5)) ||
	       (size == 3 && !memcmp(status, "ok", 3));
}

static bool __init fdt_handoff_is_memory(const void *fdt, int node)
{
	int length;
	const char *type = fdt_getprop(fdt, node, "device_type", &length);

	return type && length == 7 && !memcmp(type, "memory", 7) &&
	       fdt_handoff_enabled(fdt, node);
}

static bool __init fdt_handoff_contains(const struct fdt_handoff_range *ranges,
					int count, u64 start, u64 end)
{
	if (end <= start)
		return false;
	/* Adjacent ranges may cover a single object together. */
	for (int pass = 0; pass < count && start < end; pass++) {
		u64 next = start;

		for (int i = 0; i < count; i++)
			if (ranges[i].start <= start && ranges[i].end > next)
				next = ranges[i].end;
		if (next == start)
			return false;
		start = next;
	}
	return start >= end;
}

static int __init fdt_handoff_subtract(struct fdt_handoff_range *ram, int *count,
				       struct fdt_handoff_range hole)
{
	int original = *count, used = 0;

	for (int i = 0; i < original; i++) {
		struct fdt_handoff_range old = ram[i];

		if (hole.start >= old.end || hole.end <= old.start)
			continue;
		ram[i].end = hole.start > old.start ? hole.start : old.start;
		if (hole.end < old.end) {
			int ret = fdt_handoff_add_range(ram, count, hole.end,
							old.end - hole.end);
			if (ret)
				return ret;
		}
	}
	for (int i = 0; i < *count; i++)
		if (ram[i].end > ram[i].start)
			ram[used++] = ram[i];
	*count = used;
	return 0;
}

static int __init fdt_handoff_hex(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

/*
 * Parses the first "KEY=XX:XX:XX:XX:XX:XX" of a command line or bootconfig,
 * byte-reversed for a little-endian property (local-bd-address). Returns
 * false when it is absent or malformed: the device then keeps its own
 * default address instead of the boot failing.
 */
static bool __init fdt_handoff_macaddr(const char *source, const char *key,
				       bool reverse, u8 address[6])
{
	const char *at = fdt_handoff_word(source, key);

	if (!at)
		return false;
	at += strlen(key);
	for (int i = 0; i < 6; i++) {
		int hi = fdt_handoff_hex(at[i * 3]);
		int lo = hi < 0 ? -1 : fdt_handoff_hex(at[i * 3 + 1]);
		char end = at[i * 3 + 2];

		if (lo < 0)
			return false;
		if (i < 5 ? end != ':' :
		    end && end != ' ' && end != '\n' && end != ',')
			return false;
		address[reverse ? 5 - i : i] = hi << 4 | lo;
	}
	return true;
}

/*
 * Carries a board address over from the bootloader's tree STOCK to OUTPUT, as
 * the child IMPORT of the handoff description in the built-in tree MAINLINE
 * says. The description is read from MAINLINE because OUTPUT moves as it
 * changes.
 */
static int __init fdt_handoff_import(void *output, const void *stock,
				     const void *mainline, int import)
{
	const char *path = fdt_handoff_string(mainline, import, "stock-path");
	const char *name = fdt_handoff_string(mainline, import, "stock-property");
	const char *key = fdt_handoff_string(mainline, import, "stock-key");
	const char *property = fdt_handoff_string(mainline, import, "target-property");
	bool reverse = !!fdt_getprop(mainline, import, "reverse-bytes", NULL);
	const fdt32_t *target;
	const char *source;
	int length, node;
	u8 address[6];

	target = fdt_getprop(mainline, import, "target", &length);
	if (!path || !name || !key || !*key || !property || !target || length != 4)
		return -FDT_ERR_BADVALUE;
	node = fdt_node_offset_by_phandle(output, fdt32_ld(target));
	if (node < 0)
		return node;

	source = fdt_handoff_string(stock, fdt_path_offset(stock, path), name);
	if (!source || !fdt_handoff_macaddr(source, key, reverse, address))
		return 0;
	return fdt_setprop(output, node, property, address, sizeof(address));
}

/*
 * Collects the fixed regions that the built-in tree reserves itself and wants
 * mapped. Its no-map regions are left out: the kernel cannot mark a region
 * no-map once the reservation map holds it, so they stay holes.
 */
static int __init fdt_handoff_read_own(const void *mainline,
				       struct fdt_handoff_work *work)
{
	int ret, child, node = fdt_path_offset(mainline, "/reserved-memory");

	if (node < 0)
		return node == -FDT_ERR_NOTFOUND ? 0 : node;
	fdt_for_each_subnode(child, mainline, node) {
		if (!fdt_handoff_enabled(mainline, child) ||
		    fdt_getprop(mainline, child, "no-map", NULL))
			continue;
		ret = fdt_handoff_ranges(mainline, child, work->own,
					 &work->nr_own);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Collects usable memory and the fixed reservations of the stock tree. The
 * built-in tree MAINLINE has the last word on the regions it reserves itself.
 */
static int __init fdt_handoff_read_memory(const void *stock,
					  const void *mainline,
					  struct fdt_handoff_work *work)
{
	int ret, node, child;

	fdt_for_each_subnode(node, stock, 0) {
		if (!fdt_handoff_is_memory(stock, node))
			continue;
		ret = fdt_handoff_ranges(stock, node, work->ram, &work->nr_ram);
		if (ret)
			return ret;
	}

	node = fdt_path_offset(stock, "/reserved-memory");
	if (node < 0)
		return node;
	fdt_for_each_subnode(child, stock, node) {
		if (!fdt_handoff_enabled(stock, child))
			continue;
		ret = fdt_handoff_ranges(stock, child, work->reserved,
					 &work->nr_reserved);
		if (ret)
			return ret;
		if (!fdt_getprop(stock, child, "no-map", NULL))
			continue;
		ret = fdt_handoff_ranges(stock, child, work->nomap,
					 &work->nr_nomap);
		if (ret)
			return ret;
	}

	/*
	 * Keep the firmware's no-map holes out of memory without importing the
	 * vendor reserved-memory nodes, which belong to vendor drivers and may
	 * conflict with mainline's. Other fixed reservations go to the
	 * reservation map: never allocated, but still mappable. A hole that the
	 * built-in tree reserves itself as mapped memory, such as a restricted
	 * DMA pool, stays memory.
	 */
	ret = fdt_handoff_read_own(mainline, work);
	if (ret)
		return ret;
	for (int i = 0; i < work->nr_nomap; i++) {
		if (fdt_handoff_contains(work->own, work->nr_own,
					 work->nomap[i].start, work->nomap[i].end))
			continue;
		ret = fdt_handoff_subtract(work->ram, &work->nr_ram,
					   work->nomap[i]);
		if (ret)
			return ret;
	}
	return work->nr_ram ? 0 : -FDT_ERR_BADVALUE;
}

/* Replaces the memory of OUTPUT and adds the stock reservations to it. */
static int __init fdt_handoff_write_memory(void *output, const void *stock,
					   struct fdt_handoff_work *work)
{
	int ret, node;

	/* The built-in tree describes its memory with a single node. */
	if (fdt_address_cells(output, 0) != 2 || fdt_size_cells(output, 0) != 2)
		return -FDT_ERR_BADNCELLS;
	fdt_for_each_subnode(node, output, 0)
		if (fdt_handoff_is_memory(output, node))
			break;
	if (node < 0)
		return node;
	for (int i = 0; i < work->nr_ram; i++) {
		work->memory_reg[i * 2] = cpu_to_fdt64(work->ram[i].start);
		work->memory_reg[i * 2 + 1] =
			cpu_to_fdt64(work->ram[i].end - work->ram[i].start);
	}
	ret = fdt_setprop(output, node, "reg", work->memory_reg,
			  work->nr_ram * 16);
	if (ret)
		return ret;

	for (int i = 0; i < work->nr_reserved; i++) {
		ret = fdt_add_mem_rsv(output, work->reserved[i].start,
				      work->reserved[i].end - work->reserved[i].start);
		if (ret)
			return ret;
	}
	for (int i = 0; i < fdt_num_mem_rsv(stock); i++) {
		u64 address, size;

		ret = fdt_get_mem_rsv(stock, i, &address, &size);
		if (ret)
			return ret;
		if (address + size < address)
			return -FDT_ERR_BADVALUE;
		ret = fdt_add_mem_rsv(output, address, size);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Fills /chosen of OUTPUT and carries the board addresses over, as the
 * description HANDOFF in the built-in tree MAINLINE says.
 */
static int __init fdt_handoff_write_chosen(void *output, const void *stock,
					   const void *mainline, int handoff,
					   bool keep_initrd, u64 initrd_start,
					   u64 initrd_end)
{
	int ret, import, chosen = fdt_path_offset(output, "/chosen");

	if (chosen < 0)
		return chosen;
	if (keep_initrd) {
		ret = fdt_setprop_u64(output, chosen, "linux,initrd-start",
				      initrd_start);
		if (ret)
			return ret;
		ret = fdt_setprop_u64(output, chosen, "linux,initrd-end",
				      initrd_end);
		if (ret)
			return ret;
	}
	ret = fdt_setprop_empty(output, chosen, "linux,fdt-handoff");
	if (ret)
		return ret;

	fdt_for_each_subnode(import, mainline, handoff) {
		ret = fdt_handoff_import(output, stock, mainline, import);
		if (ret)
			return ret;
	}
	return import == -FDT_ERR_NOTFOUND ? 0 : import;
}

/* Bootloaders write the initramfs bounds with one or two cells. */
static int __init fdt_handoff_initrd_bound(const void *stock, int chosen,
					   const char *name, u64 *bound)
{
	int length;
	const fdt32_t *value = fdt_getprop(stock, chosen, name, &length);

	if (!value || (length != 4 && length != 8))
		return -FDT_ERR_BADVALUE;
	*bound = fdt_handoff_cells(value, length / 4);
	return 0;
}

static int __init fdt_handoff_initrd(const void *stock, u64 *start, u64 *end)
{
	int ret, chosen = fdt_path_offset(stock, "/chosen");

	ret = fdt_handoff_initrd_bound(stock, chosen, "linux,initrd-start", start);
	if (ret)
		return ret;
	return fdt_handoff_initrd_bound(stock, chosen, "linux,initrd-end", end);
}

/*
 * Builds the tree to boot in OUTPUT from the bootloader's tree STOCK and the
 * built-in tree between BUILTIN and BUILTIN_END. The kernel image occupies
 * KERNEL_START to KERNEL_END and must lie in usable memory.
 *
 * Returns 0 when OUTPUT is ready, -FDT_ERR_NOTFOUND when the built-in tree
 * does not describe STOCK and the bootloader's tree should be booted, or
 * another negative libfdt error when the device cannot be handed off.
 */
static int __init fdt_handoff_prepare(const void *stock, const char *builtin,
				      const char *builtin_end, void *output,
				      struct fdt_handoff_work *work,
				      u64 kernel_start, u64 kernel_end)
{
	u64 initrd_start = 0, initrd_end = 0;
	const char *bootargs;
	int ret, handoff;
	bool keep_initrd;

	if (builtin + sizeof(struct fdt_header) > builtin_end ||
	    fdt_check_header(builtin) ||
	    builtin + fdt_totalsize(builtin) > builtin_end)
		return -FDT_ERR_NOTFOUND;
	ret = fdt_handoff_check(builtin);
	if (ret)
		return ret;
	handoff = fdt_path_offset(builtin, "/chosen/fdt-handoff");
	if (handoff < 0)
		return -FDT_ERR_NOTFOUND;

	ret = fdt_handoff_check(stock);
	if (ret)
		return ret;
	bootargs = fdt_handoff_string(stock, fdt_path_offset(stock, "/chosen"),
				      "bootargs");
	ret = fdt_handoff_match(stock, bootargs, builtin, handoff);
	if (ret)
		return ret;

	keep_initrd = !!fdt_getprop(builtin, handoff, "keep-initrd", NULL);
	if (keep_initrd) {
		ret = fdt_handoff_initrd(stock, &initrd_start, &initrd_end);
		if (ret)
			return ret;
	}
	work->nr_ram = work->nr_reserved = work->nr_nomap = work->nr_own = 0;
	ret = fdt_handoff_read_memory(stock, builtin, work);
	if (ret)
		return ret;
	if ((keep_initrd &&
	     !fdt_handoff_contains(work->ram, work->nr_ram, initrd_start,
				   initrd_end)) ||
	    !fdt_handoff_contains(work->ram, work->nr_ram, kernel_start,
				  kernel_end))
		return -FDT_ERR_BADVALUE;

	ret = fdt_open_into(builtin, output, FDT_HANDOFF_CAPACITY);
	if (ret)
		return ret;
	ret = fdt_handoff_write_memory(output, stock, work);
	if (ret)
		return ret;
	ret = fdt_handoff_write_chosen(output, stock, builtin, handoff,
				       keep_initrd, initrd_start, initrd_end);
	if (ret)
		return ret;
	ret = fdt_pack(output);
	if (ret)
		return ret;
	return fdt_handoff_check(output);
}

/* The tree to boot must outlive init: it stays initial_boot_params. */
static u8 fdt_handoff_output[FDT_HANDOFF_CAPACITY] __aligned(8);
static struct fdt_handoff_work fdt_handoff_work __initdata;

/*
 * Returns the tree to boot instead of the bootloader's tree STOCK, or NULL
 * when the built-in tree does not describe it. KERNEL_START and KERNEL_END
 * are the physical bounds of the kernel image.
 */
void * __init fdt_handoff(const void *stock, phys_addr_t kernel_start,
			  phys_addr_t kernel_end)
{
	int ret = fdt_handoff_prepare(stock, __dtb_start, __dtb_end,
				      fdt_handoff_output, &fdt_handoff_work,
				      kernel_start, kernel_end);

	if (ret == -FDT_ERR_NOTFOUND)
		return NULL;
	if (ret) {
		/*
		 * The built-in tree is this device's, so the bootloader's tree
		 * cannot run this kernel, and nothing can be reported this
		 * early. Stop here.
		 */
		pr_emerg("FDT handoff failed: %s\n", fdt_strerror(ret));
		while (true)
			cpu_relax();
	}
	return fdt_handoff_output;
}
