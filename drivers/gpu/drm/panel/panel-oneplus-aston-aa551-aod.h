/* SPDX-License-Identifier: GPL-2.0-only */
/* From timing@sdc_fhd_120 of the exact factory DTBO; the 90 Hz node lacks
 * the page 02, 08 and 20 writes. */
static void aa551_factory_lp1_120(struct mipi_dsi_multi_context *dc)
{
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x02);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa7, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa2, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x20);
	mipi_dsi_dcs_write_seq_multi(dc, 0xbc, 0x52);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0x39, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dc, 0x81, 0x02);
	mipi_dsi_dcs_write_seq_multi(dc, 0x80, 0x01);
	mipi_dsi_dcs_write_seq_multi(dc, 0x88, 0x78);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x23);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa0, 0x0f);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa1, 0xff);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8a, 0x78);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x0b);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8c, 0x90);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8d, 0x03);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8e, 0xbf);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8b, 0x78);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0x51, 0x00, 0x00, 0x0d, 0xbb);
}

static void aa551_factory_nolp_120(struct mipi_dsi_multi_context *dc)
{
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x02);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa7, 0x40);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa2, 0x20);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x20);
	mipi_dsi_dcs_write_seq_multi(dc, 0xbc, 0x32);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x08);
	mipi_dsi_dcs_write_seq_multi(dc, 0xb0, 0x55);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0x38, 0x00);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dc, 0x81, 0x02);
	mipi_dsi_dcs_write_seq_multi(dc, 0x80, 0x01);
	mipi_dsi_dcs_write_seq_multi(dc, 0x88, 0x78);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x23);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa0, 0x0d);
	mipi_dsi_dcs_write_seq_multi(dc, 0xa1, 0xbb);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dc, 0x81, 0x01);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8a, 0x78);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x0b);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8c, 0x10);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8d, 0x03);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8e, 0xbf);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dc, 0x8b, 0x78);
	mipi_dsi_dcs_write_seq_multi(dc, 0xff, 0x08, 0x38, 0x00);
}
