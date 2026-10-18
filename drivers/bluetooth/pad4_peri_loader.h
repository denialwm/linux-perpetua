/* SPDX-License-Identifier: GPL-2.0-only */
/* Exact OPD2514 factory Peri protocol. Experimental, default off. */
static int pad4_peri_command(struct hci_uart *hu, u16 opcode,
			     const u8 *payload, size_t len, bool wait)
{
	struct qca_data *qca = hu->priv;
	struct pad4_peri_state *p = &qca->peri;
	struct sk_buff *skb;
	unsigned long flags;
	int ret = 0;

	if (!len || len > 246)
		return -EINVAL;
	skb = bt_skb_alloc(len + 5, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;
	skb_put_u8(skb, 0x31);
	skb_put_u8(skb, 0);
	put_unaligned_le16(opcode, skb_put(skb, 2));
	skb_put_u8(skb, len);
	skb_put_data(skb, payload, len);
	spin_lock_irqsave(&p->lock, flags);
	reinit_completion(&p->cmd_done);
	p->opcode = opcode;
	p->subcmd = payload[0];
	p->pending = wait;
	p->reply_len = 0;
	spin_unlock_irqrestore(&p->lock, flags);
	skb_queue_tail(&qca->txq, skb);
	hci_uart_tx_wakeup(hu);
	if (wait && !wait_for_completion_timeout(&p->cmd_done,
						msecs_to_jiffies(3000)))
		ret = -ETIMEDOUT;
	spin_lock_irqsave(&p->lock, flags);
	p->pending = false;
	spin_unlock_irqrestore(&p->lock, flags);
	if (ret)
		bt_dev_err(hu->hdev, "Peri command %04x/%02x timed out", opcode, payload[0]);
	return ret;
}

static int pad4_peri_status(struct hci_uart *hu, u16 opcode,
			   const u8 *payload, size_t len)
{
	struct qca_data *qca = hu->priv;
	int ret = pad4_peri_command(hu, opcode, payload, len, true);

	return ret ? ret : qca->peri.reply[8];
}

/* Validate entire NVM structure before changing the in-memory copy. */
static int pad4_peri_nvm(u8 *data, size_t size, u8 baud)
{
	size_t offset = 4, baud_offset = 0, ibs_offset = 0;
	u16 tag, len;

	if (size < 4 || data[0] != 2 ||
	    (get_unaligned_le32(data) >> 8) != size - 4)
		return -EINVAL;
	while (offset < size) {
		if (size - offset < 12)
			return -EINVAL;
		tag = get_unaligned_le16(data + offset);
		len = get_unaligned_le16(data + offset + 2);
		if (len > size - offset - 12)
			return -EINVAL;
		if (tag == 17) {
			if (len != 6 || baud_offset)
				return -EINVAL;
			baud_offset = offset + 13;
		} else if (tag == 27) {
			if (len != 14 || ibs_offset)
				return -EINVAL;
			ibs_offset = offset + 13;
		}
		offset += 12 + len;
	}
	if (!baud_offset || !ibs_offset)
		return -EINVAL;
	data[baud_offset] = baud;
	data[ibs_offset] |= 1;
	return 0;
}

static int pad4_peri_download(struct hci_uart *hu, const u8 *data,
			      size_t size, bool patch, u8 core)
{
	struct qca_data *qca = hu->priv;
	u8 payload[246];
	size_t offset, len;
	bool wait;
	int ret;

	WRITE_ONCE(qca->peri.download_error, 0);
	for (offset = 0; offset < size; offset += len) {
		ret = READ_ONCE(qca->peri.download_error);
		if (ret)
			return -EIO;
		len = min_t(size_t, 243, size - offset);
		payload[0] = 6;
		payload[1] = core;
		payload[2] = len;
		memcpy(payload + 3, data + offset, len);
		wait = !patch || offset + len == size;
		ret = pad4_peri_command(hu, 0xfff0, payload, len + 3, wait);
		if (ret)
			return ret;
		if (wait && (qca->peri.reply_len < 11 ||
			     qca->peri.reply[8] || qca->peri.reply[10] != core))
			return -EIO;
		/* Bound outstanding UART data without relying on a userspace
		 * write() flush. Hardware flow control remains enabled.
		 */
		if (!wait)
			usleep_range(1000, 1500);
	}
	return READ_ONCE(qca->peri.download_error) ? -EIO : 0;
}

static int pad4_peri_wake(struct hci_uart *hu)
{
	int ret = send_hci_ibs_cmd(HCI_IBS_WAKE_IND, hu);

	if (ret < 0)
		return ret;
	hci_uart_tx_wakeup(hu);
	msleep(10);
	return 0;
}

static int pad4_peri_load(struct hci_uart *hu, u8 baud)
{
	static const struct {
		const char *name;
		size_t size;
		u32 crc;
	} images[] = {
		{ "qca/pad4/brhperifw20.mbn", 31424, 0x38c07dca },
		{ "qca/pad4/brhperinv20.bin", 741, 0xad1a4b44 },
		{ "qca/pad4/brhperinv20.b43", 741, 0xd0f43e20 },
		{ "qca/pad4/brhperinv20.b45", 741, 0xd0f43e20 },
		{ "qca/pad4/tmel_peach_20.elf", 68952, 0x1c6d3b6d },
	};
	static const u8 version[] = { 5, 0 }, board[] = { 10 };
	static const u8 arbitrate[] = { 8, 0, 0 }, tme[] = { 8, 3, 0 };
	static const u8 tme_version[] = { 5, 3 };
	static const u8 reset[] = { 3 }, activate[] = { 0, 1, 1 };
	const struct firmware *fw[ARRAY_SIZE(images)] = { 0 };
	struct qca_data *qca = hu->priv;
	struct pad4_peri_state *p = &qca->peri;
	u8 *nvm = NULL;
	unsigned int i, selected;
	int ret;

	if (!pad4_peri_load_fw)
		return 0;
	if (!pad4_peri_probe || !hu->serdev ||
	    !of_machine_is_compatible("oneplus,iceland"))
		return -ENODEV;
	/* All files and allocation are checked before claiming ownership. */
	for (i = 0; i < ARRAY_SIZE(images); i++) {
		ret = request_firmware(&fw[i], images[i].name, &hu->hdev->dev);
		if (ret)
			goto out;
		if (fw[i]->size != images[i].size ||
		    (crc32_le(~0U, fw[i]->data, fw[i]->size) ^ ~0U) != images[i].crc) {
			ret = -EINVAL;
			goto out;
		}
	}
	nvm = kmalloc(741, GFP_KERNEL);
	if (!nvm) {
		ret = -ENOMEM;
		goto out;
	}
	ret = pad4_peri_wake(hu);
	if (ret)
		goto out;
	ret = pad4_peri_status(hu, 0xfff0, version, sizeof(version));
	if (ret || p->reply_len != 24 || p->reply[10] != 0 ||
	    get_unaligned_le32(p->reply + 11) != 0x21 ||
	    get_unaligned_le16(p->reply + 17) != 0x0200 ||
	    get_unaligned_le32(p->reply + 19) != 0x40210200) {
		ret = ret < 0 ? ret : -ENODEV;
		goto out;
	}
	reinit_completion(&p->ready);
	ret = pad4_peri_status(hu, 0xfff0, arbitrate, sizeof(arbitrate));
	bt_dev_info(hu->hdev, "Peri arbitration: %d", ret);
	if (ret < 0)
		goto out;
	if (p->reply_len < 11 || p->reply[10] != 0) {
		ret = -EPROTO;
		goto out;
	}
	if (ret == 0) {
		bt_dev_info(hu->hdev, "Loading exact factory Peri patch into RAM");
		ret = pad4_peri_download(hu, fw[0]->data, fw[0]->size, true, 0);
		if (ret)
			goto out;
		ret = pad4_peri_status(hu, 0xfff0, board, sizeof(board));
		if (ret || p->reply_len != 13 || p->reply[10] != 2 || p->reply[11]) {
			ret = ret < 0 ? ret : -EPROTO;
			goto out;
		}
		switch (p->reply[12]) {
		case 0: selected = 1; break;
		case 0x43: selected = 2; break;
		case 0x45: selected = 3; break;
		default: ret = -ENODEV; goto out;
		}
		memcpy(nvm, fw[selected]->data, 741);
		ret = pad4_peri_nvm(nvm, 741, baud);
		if (ret)
			goto out;
		bt_dev_info(hu->hdev, "Loading Peri NVM %s", images[selected].name);
		ret = pad4_peri_download(hu, nvm, 741, false, 0);
		if (ret)
			goto out;
		reinit_completion(&p->ready);
		ret = pad4_peri_status(hu, 0xfff1, reset, sizeof(reset));
		if (ret) {
			ret = ret < 0 ? ret : -EIO;
			goto out;
		}
		ret = 0x0b; /* Must receive Peri-ready notification after reset. */
	}
	if (ret == 0x0b) {
		if (!wait_for_completion_timeout(&p->ready, msecs_to_jiffies(5000))) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = READ_ONCE(p->ready_status) ? -EIO : 0x17;
	}
	if (ret != 0x17) {
		ret = ret < 0 ? ret : -EIO;
		goto out;
	}
	ret = pad4_peri_wake(hu);
	if (ret)
		goto out;
	ret = pad4_peri_status(hu, 0xfff0, version, sizeof(version));
	if (ret || p->reply_len != 24 ||
	    get_unaligned_le16(p->reply + 15) != 0x8af2) {
		ret = ret < 0 ? ret : -EIO;
		goto out;
	}
	/* Factory TmelSocInit: arbitrate, version, ELF (no NVM/reset),
	 * then a class3 ready notification for subsystem3.
	 */
	reinit_completion(&p->tme_ready);
	ret = pad4_peri_status(hu, 0xfff0, tme, sizeof(tme));
	bt_dev_info(hu->hdev, "Peri TMEL arbitration: %d", ret);
	if (ret < 0)
		goto out;
	if (p->reply_len < 11 || p->reply[10] != 3) {
		ret = -EPROTO;
		goto out;
	}
	if (ret == 0) {
		ret = pad4_peri_status(hu, 0xfff0, tme_version, sizeof(tme_version));
		if (ret || p->reply_len != 24 || p->reply[10] != 3 ||
		    (get_unaligned_le32(p->reply + 11) != 0x21 &&
		     get_unaligned_le32(p->reply + 11) != 0x1e) ||
		    get_unaligned_le16(p->reply + 17) != 0x0200 ||
		    get_unaligned_le32(p->reply + 19) != 0x40210200) {
			ret = ret < 0 ? ret : -ENODEV;
			goto out;
		}
		bt_dev_info(hu->hdev, "Loading exact factory TMEL patch into RAM");
		ret = pad4_peri_download(hu, fw[4]->data, fw[4]->size, true, 3);
		if (ret)
			goto out;
		ret = 0x0b;
	}
	if (ret == 0x0b) {
		if (!wait_for_completion_timeout(&p->tme_ready, msecs_to_jiffies(5000))) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = READ_ONCE(p->tme_status) ? -EIO : 0x17;
	}
	if (ret != 5 && ret != 6 && ret != 0x17) {
		ret = ret < 0 ? ret : -EIO;
		goto out;
	}
	reinit_completion(&p->active);
	ret = pad4_peri_status(hu, 0xfff1, activate, sizeof(activate));
	if (ret == 0) {
		if (!wait_for_completion_timeout(&p->active, msecs_to_jiffies(5000))) {
			ret = -ETIMEDOUT;
			goto out;
		}
		/* Intentionally strict until factory notification is observed. */
		ret = READ_ONCE(p->active_status);
		if (ret != 1 && ret != 0x17) {
			ret = -EPROTO;
			goto out;
		}
	} else if (ret != 0x17) {
		ret = ret < 0 ? ret : -EIO;
		goto out;
	}
	bt_dev_info(hu->hdev, "Factory Peri patch/NVM initialization completed");
	ret = 0;
out:
	if (ret)
		bt_dev_err(hu->hdev, "Peri initialization stopped: %d; no setup retry", ret);
	kfree(nvm);
	for (i = 0; i < ARRAY_SIZE(fw); i++)
		release_firmware(fw[i]);
	return ret;
}
