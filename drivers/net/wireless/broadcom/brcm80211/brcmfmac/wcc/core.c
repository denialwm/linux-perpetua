// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2022 Broadcom Corporation
 */
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>
#include <core.h>
#include <bus.h>
#include <fwvid.h>
#include <fwil.h>
#include <fweh.h>
#include <feature.h>
#include <cfg80211.h>

#include "vops.h"

/* event definitions */
#define BRCMF_WCC_E_AUTH_START		194
#define BRCMF_WCC_E_LAST		213

/* The firmware's "assoc_mgr_cmd" iovar: fine control of an association. */
#define BRCMF_WCC_ASSOC_MGR_VER			0
#define BRCMF_WCC_ASSOC_MGR_PAUSE_ON_EVT	0
#define BRCMF_WCC_ASSOC_MGR_SEND_AUTH		3
/* params of PAUSE_ON_EVT */
#define BRCMF_WCC_ASSOC_MGR_EVT_NONE		0	/* resume */
#define BRCMF_WCC_ASSOC_MGR_EVT_AUTH_RESP	1

/* Auth algorithm reported in BRCMF_E_AUTH for SAE frames. */
#define BRCMF_WCC_AUTH_SAE			3

struct brcmf_wcc_assoc_mgr_cmd_le {
	__le16 version;
	__le16 length;
	__le16 cmd;
	union {
		__le16 params;
		DECLARE_FLEX_ARRAY(u8, data);
	};
} __packed;

struct brcmf_wcc_ssid_le {
	__le32 len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
} __packed;

/* Payload of BRCMF_WCC_E_AUTH_START: the firmware asks the host to run
 * the authentication (SAE) with this peer.
 */
struct brcmf_wcc_auth_start_evt_le {
	__le16 version;
	__le16 len;
	struct brcmf_wcc_ssid_le ssid;
	u8 bssid[ETH_ALEN];
	u8 pad[2];
	__le32 key_mgmt_suite;
} __packed;

static int brcmf_wcc_set_sae_pwd(struct brcmf_if *ifp,
				 struct cfg80211_crypto_settings *crypto)
{
	return brcmf_set_wsec(ifp, crypto->sae_pwd, crypto->sae_pwd_len,
			      BRCMF_WSEC_PASSPHRASE);
}

static const struct brcmf_fweh_event_map brcmf_wcc_event_map = {
	.items = {
		{ BRCMF_E_AUTH_START, BRCMF_WCC_E_AUTH_START },
	},
	.n_items = 1
};

static int brcmf_wcc_alloc_fweh_info(struct brcmf_pub *drvr)
{
	struct brcmf_fweh_info *fweh;

	fweh = kzalloc_flex(*fweh, evt_handler, BRCMF_WCC_E_LAST);
	if (!fweh)
		return -ENOMEM;

	fweh->num_event_codes = BRCMF_WCC_E_LAST;
	fweh->event_map = &brcmf_wcc_event_map;
	drvr->fweh = fweh;
	return 0;
}

static int brcmf_wcc_assoc_pause(struct brcmf_if *ifp, u16 pause_event)
{
	struct brcmf_wcc_assoc_mgr_cmd_le cmd = {
		.version = cpu_to_le16(BRCMF_WCC_ASSOC_MGR_VER),
		.length = cpu_to_le16(sizeof(cmd)),
		.cmd = cpu_to_le16(BRCMF_WCC_ASSOC_MGR_PAUSE_ON_EVT),
		.params = cpu_to_le16(pause_event),
	};

	return brcmf_fil_iovar_data_set(ifp, "assoc_mgr_cmd", &cmd,
					sizeof(cmd));
}

/* Host SAE (firmware capability "extsae"): the firmware does not run SAE
 * itself. It reports BRCMF_WCC_E_AUTH_START and pauses the association;
 * the supplicant then runs SAE through cfg80211's external authentication:
 * its auth frames go out through assoc_mgr_cmd SEND_AUTH, the peer's come
 * back in BRCMF_E_AUTH events, and .external_auth resumes the association.
 */
static s32 brcmf_wcc_notify_auth_start(struct brcmf_if *ifp,
				       const struct brcmf_event_msg *e,
				       void *data)
{
	struct brcmf_pub *drvr = ifp->drvr;
	struct brcmf_wcc_auth_start_evt_le *evt = data;
	struct cfg80211_external_auth_params params = {};
	s32 err;

	if (e->datalen < sizeof(*evt)) {
		bphy_err(drvr, "auth start event too short (%u)\n", e->datalen);
		return -EINVAL;
	}

	params.action = NL80211_EXTERNAL_AUTH_START;
	params.key_mgmt_suite = WLAN_AKM_SUITE_SAE;
	params.status = WLAN_STATUS_SUCCESS;
	params.ssid.ssid_len = min_t(u32, le32_to_cpu(evt->ssid.len),
				     IEEE80211_MAX_SSID_LEN);
	memcpy(params.ssid.ssid, evt->ssid.ssid, params.ssid.ssid_len);
	memcpy(params.bssid, evt->bssid, ETH_ALEN);

	brcmf_dbg(CONN, "external auth start, event version %u\n",
		  le16_to_cpu(evt->version));

	err = cfg80211_external_auth_request(ifp->ndev, &params, GFP_KERNEL);
	if (err) {
		bphy_err(drvr, "external auth request failed (%d)\n", err);
		return err;
	}

	err = brcmf_wcc_assoc_pause(ifp, BRCMF_WCC_ASSOC_MGR_EVT_AUTH_RESP);
	if (err)
		bphy_err(drvr, "pausing the association failed (%d)\n", err);

	return err;
}

/* BRCMF_E_AUTH of an SAE exchange carries the peer's auth frame body. */
static s32 brcmf_wcc_notify_auth(struct brcmf_if *ifp,
				 const struct brcmf_event_msg *e, void *data)
{
	struct brcmf_cfg80211_info *cfg = ifp->drvr->config;
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	struct ieee80211_mgmt *mgmt;
	struct cfg80211_bss *bss;
	struct brcmu_chan ch;
	size_t hdrlen = offsetof(struct ieee80211_mgmt, u);
	size_t len;
	u32 chanspec;
	s32 freq;
	s32 err;

	if (e->auth_type != BRCMF_WCC_AUTH_SAE || !e->datalen ||
	    !brcmf_feat_is_enabled(ifp, BRCMF_FEAT_SAE_EXT))
		return 0;

	/* Before the association, the firmware's chanspec is not yet the
	 * peer's channel: take the channel the peer was scanned on.
	 */
	bss = cfg80211_get_bss(wiphy, NULL, e->addr, NULL, 0,
			       IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ANY);
	if (bss) {
		freq = bss->channel->center_freq;
		cfg80211_put_bss(wiphy, bss);
	} else {
		err = brcmf_fil_iovar_int_get(ifp, "chanspec", &chanspec);
		if (err)
			return err;
		ch.chspec = chanspec;
		cfg->d11inf.decchspec(&ch);
		freq = ieee80211_channel_to_frequency(ch.control_ch_num,
						      ch.control_ch_num <= 14 ?
						      NL80211_BAND_2GHZ :
						      NL80211_BAND_5GHZ);
	}

	len = hdrlen + e->datalen;
	mgmt = kzalloc(len, GFP_KERNEL);
	if (!mgmt)
		return -ENOMEM;

	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
					  IEEE80211_STYPE_AUTH);
	memcpy(mgmt->da, ifp->mac_addr, ETH_ALEN);
	memcpy(mgmt->sa, e->addr, ETH_ALEN);
	memcpy(mgmt->bssid, e->addr, ETH_ALEN);
	memcpy(&mgmt->u, data, e->datalen);

	brcmf_dbg(CONN, "SAE auth frame rx, len %zu, freq %d\n", len, freq);
	cfg80211_rx_mgmt(&ifp->vif->wdev, freq, 0, (u8 *)mgmt, len,
			 NL80211_RXMGMT_FLAG_EXTERNAL_AUTH);
	kfree(mgmt);
	return 0;
}

static int brcmf_wcc_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
			     struct cfg80211_mgmt_tx_params *params, u64 cookie)
{
	const struct ieee80211_mgmt *mgmt = (const void *)params->buf;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_wcc_assoc_mgr_cmd_le *cmd;
	size_t cmd_len;
	bool ack;
	int err;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	if (params->len < offsetof(struct ieee80211_mgmt, u) ||
	    !ieee80211_is_auth(mgmt->frame_control) ||
	    !brcmf_feat_is_enabled(vif->ifp, BRCMF_FEAT_SAE_EXT))
		return brcmf_cfg80211_mgmt_tx(wiphy, wdev, params, cookie);

	/* The whole frame, header included, follows the command word. */
	cmd_len = sizeof(*cmd) + params->len;
	cmd = kzalloc(cmd_len, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;
	cmd->version = cpu_to_le16(BRCMF_WCC_ASSOC_MGR_VER);
	cmd->length = cpu_to_le16(params->len);
	cmd->cmd = cpu_to_le16(BRCMF_WCC_ASSOC_MGR_SEND_AUTH);
	memcpy(cmd->data, params->buf, params->len);

	err = brcmf_fil_iovar_data_set(vif->ifp, "assoc_mgr_cmd", cmd, cmd_len);
	kfree(cmd);
	ack = !err;
	if (err)
		bphy_err(vif->ifp->drvr, "sending the auth frame failed (%d)\n",
			 err);
	else
		brcmf_dbg(CONN, "SAE auth frame tx, len %zu\n", params->len);

	cfg80211_mgmt_tx_status(wdev, cookie, params->buf, params->len, ack,
				GFP_KERNEL);
	return 0;
}

static int brcmf_wcc_external_auth(struct wiphy *wiphy,
				   struct net_device *dev,
				   struct cfg80211_external_auth_params *params)
{
	struct brcmf_if *ifp = netdev_priv(dev);
	struct brcmf_pub *drvr = ifp->drvr;
	int err;

	brcmf_dbg(CONN, "external auth done, status %u, %s PMKID\n",
		  params->status, params->pmkid ? "with" : "no");

	if (params->status != WLAN_STATUS_SUCCESS) {
		bphy_err(drvr, "external authentication failed (%u)\n",
			 params->status);
		return drvr->ops->disconnect(wiphy, dev,
					     WLAN_REASON_UNSPECIFIED);
	}

	if (params->pmkid) {
		struct cfg80211_pmksa pmksa = {
			.bssid = params->bssid,
			.pmkid = params->pmkid,
		};

		err = drvr->ops->set_pmksa(wiphy, dev, &pmksa);
		if (err)
			bphy_err(drvr, "setting the PMKSA failed (%d)\n", err);
	}

	err = brcmf_wcc_assoc_pause(ifp, BRCMF_WCC_ASSOC_MGR_EVT_NONE);
	if (err)
		bphy_err(drvr, "resuming the association failed (%d)\n", err);

	return err;
}

static void brcmf_wcc_get_cfg80211_ops(struct brcmf_pub *drvr)
{
	drvr->ops->mgmt_tx = brcmf_wcc_mgmt_tx;
	drvr->ops->external_auth = brcmf_wcc_external_auth;
}

static void brcmf_wcc_register_event_handlers(struct brcmf_pub *drvr)
{
	brcmf_fweh_register(drvr, BRCMF_E_AUTH_START,
			    brcmf_wcc_notify_auth_start);
	brcmf_fweh_register(drvr, BRCMF_E_AUTH, brcmf_wcc_notify_auth);
}

const struct brcmf_fwvid_ops brcmf_wcc_ops = {
	.set_sae_password = brcmf_wcc_set_sae_pwd,
	.alloc_fweh_info = brcmf_wcc_alloc_fweh_info,
	.get_cfg80211_ops = brcmf_wcc_get_cfg80211_ops,
	.register_event_handlers = brcmf_wcc_register_event_handlers,
};
