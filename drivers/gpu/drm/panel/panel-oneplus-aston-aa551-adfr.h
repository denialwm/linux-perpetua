/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Adaptive refresh floor, from qcom,mdss-dsi-adfr-min-fps-0 (120 Hz) and -6
 * (1 Hz) of timing@sdc_fhd_120 in the factory DTBO. Without new frames the
 * panel slows down to the floor on its own; the two differ in five values.
 */
static void aa551_min_fps(struct mipi_dsi_multi_context *dc, bool one_hz)
{
	u8 skip = one_hz ? 0x55 : 0x00;

	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x02);
	mipi_dsi_dcs_write_seq_multi(dc, 0xb0, 0x00, skip);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dc, 0x81, 0x02);
	mipi_dsi_dcs_write_seq_multi(dc, 0x80, 0x01);
	mipi_dsi_dcs_write_seq_multi(dc, 0x88, 0x78);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x09);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8e, one_hz ? 0x08 : 0x18, 0x10);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x01);
	mipi_dsi_dcs_write_seq_multi(dc, 0xf8, 0x0f, 0x0f, 0xf0, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0xcf, 0x00, 0x33, 0x00, one_hz ? 0x00 : 0x33, 0x33);
	mipi_dsi_dcs_write_seq_multi(dc, 0xfe, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x02);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa6, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa7, 0x40);
	mipi_dsi_dcs_write_seq_multi(dc, 0xb0, 0x00, skip, 0x91, 0x20);
	mipi_dsi_dcs_write_seq_multi(dc, 0xb1, 0x0b, 0x17, 0x77, one_hz ? 0x77 : 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0xb6, 0x80);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8b, 0x78);
	mipi_dsi_dcs_write_buffer_multi(dc, (u8[]){ MIPI_DCS_WRITE_MEMORY_START }, 1);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x00);
}
