/* Exact OPD2514 HAL UpdateNewNvmFormat: tag17 data[1] baud, tag27
 * data[1] bit0 IBS. Tag17 data[0] is deliberately preserved.
 * Caller gates this to explicit opt-in and iceland/WCN7860.
 */
static int pad4_nvm_transport_tag(u16 id, u8 *data, u16 len, u8 baud)
{
	switch (id) {
	case 17:
		if (len != 6)
			return -EINVAL;
		data[1] = baud;
		return 0;
	case 27:
		if (len != 14)
			return -EINVAL;
		data[1] |= 1;
		return 0;
	default:
		return -EINVAL;
	}
}
