/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* Copyright (C) 2020 MediaTek Inc. */

#ifndef __MT7921_MCU_H
#define __MT7921_MCU_H

#include "../mt76_connac_mcu.h"

struct mt7921_mcu_tx_done_event {
	u8 pid;
	u8 status;
	__le16 seq;

	u8 wlan_idx;
	u8 tx_cnt;
	__le16 tx_rate;

	u8 flag;
	u8 tid;
	u8 rsp_rate;
	u8 mcs;

	u8 bw;
	u8 tx_pwr;
	u8 reason;
	u8 rsv0[1];

	__le32 delay;
	__le32 timestamp;
	__le32 applied_flag;
	u8 txs[28];

	u8 rsv1[32];
} __packed;

/* ext event table */
enum {
	MCU_EXT_EVENT_RATE_REPORT = 0x87,
};

struct mt7921_mcu_eeprom_info {
	__le32 addr;
	__le32 valid;
	u8 data[MT7921_EEPROM_BLOCK_SIZE];
} __packed;

#define MT_RA_RATE_NSS			GENMASK(8, 6)
#define MT_RA_RATE_MCS			GENMASK(3, 0)
#define MT_RA_RATE_TX_MODE		GENMASK(12, 9)
#define MT_RA_RATE_DCM_EN		BIT(4)
#define MT_RA_RATE_BW			GENMASK(14, 13)

enum {
	MT_EBF = BIT(0),	/* explicit beamforming */
	MT_IBF = BIT(1)		/* implicit beamforming */
};

struct mt7921_mcu_ant_id_config {
	u8 ant_id[4];
} __packed;

struct mt7921_txpwr_req {
	u8 ver;
	u8 action;
	__le16 len;
	u8 dbdc_idx;
	u8 rsv[3];
} __packed;

struct mt7921_txpwr_event {
	u8 ver;
	u8 action;
	__le16 len;
	struct mt7921_txpwr txpwr;
} __packed;

struct mt7921_wf_rf_pin_ctrl_event {
	u8 result;
	u8 value;
} __packed;

enum {
	TM_SWITCH_MODE,
	TM_SET_AT_CMD,
	TM_QUERY_AT_CMD,
};

enum {
	MT7921_TM_NORMAL,
	MT7921_TM_TESTMODE,
	MT7921_TM_ICAP,
	MT7921_TM_ICAP_OVERLAP,
	MT7921_TM_WIFISPECTRUM,
};

struct mt7921_rftest_cmd {
	u8 action;
	u8 rsv[3];
	__le32 param0;
	__le32 param1;
} __packed;

struct mt7921_rftest_evt {
	__le32 param0;
	__le32 param1;
} __packed;

struct mt7921_clc_info_tlv {
	__le16 tag;
	__le16 len;

	u8 chan_conf; /* BIT(0) : Enable UNII-4
		       * BIT(1) : Enable UNII-5
		       * BIT(2) : Enable UNII-6
		       * BIT(3) : Enable UNII-7
		       * BIT(4) : Enable UNII-8
		       */
	u8 rsv[63];
} __packed;
/* Per-feature commands, for firmware built without unified commands */
#define MT7921_LEGACY_STA_TYPE_AP	(BIT(0) | BIT(6))	/* legacy, AP role */
#define MT7921_LEGACY_STA_REC_NONE	0xfe
#define MT7921_LEGACY_WTBL_NONE		0xff
#define MT7921_LEGACY_DBDC_AUTO		4

enum {
	MT7921_LEGACY_CONNECTED,
	MT7921_LEGACY_DISCONNECTED,
};

enum {
	MT7921_LEGACY_STA_STATE_1,
	MT7921_LEGACY_STA_STATE_2,
	MT7921_LEGACY_STA_STATE_3,
};

enum {
	MT7921_LEGACY_ROC_ACQUIRE,
	MT7921_LEGACY_ROC_ABORT,
};

#define MT7921_LEGACY_PHY_HR_DSSS	BIT(0)
#define MT7921_LEGACY_PHY_ERP		BIT(1)
#define MT7921_LEGACY_PHY_OFDM		BIT(3)
#define MT7921_LEGACY_PHY_HT		BIT(4)
#define MT7921_LEGACY_PHY_VHT		BIT(5)
#define MT7921_LEGACY_PHY_HE		BIT(6)

enum {
	MT7921_LEGACY_CIPHER_TKIP = 2,
	MT7921_LEGACY_CIPHER_CCMP = 4,
	MT7921_LEGACY_CIPHER_CCMP_256 = 10,
	MT7921_LEGACY_CIPHER_GCMP = 11,
	MT7921_LEGACY_CIPHER_GCMP_256 = 12,
};

#define MT7921_LEGACY_RATE_CCK_1M	0x0000
#define MT7921_LEGACY_RATE_OFDM_6M	0x004b

/* request and grant event */
struct mt7921_legacy_roc {
	u8 bss_idx;
	u8 token;
	u8 action;	/* event: status */
	u8 primary_chan;
	u8 sco;
	u8 band;
	u8 width;
	u8 center_chan;
	u8 center_chan2;
	u8 req_type;
	u8 dbdc_band;
	u8 rsv0;
	__le32 max_interval;	/* ms */
	u8 rsv1[8];
} __packed;

struct mt7921_legacy_bss_info {
	u8 bss_idx;
	u8 conn_state;
	u8 op_mode;
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 bssid[ETH_ALEN];
	u8 qos;
	u8 version;
	__le16 op_rates;
	__le16 basic_rates;
	u8 ap_sta_idx;
	u8 rsv0;
	__le16 fixed_rate;
	u8 nonht_basic_phy;
	u8 auth_mode;
	u8 enc_status;
	u8 phy_types;
	u8 wapi_mode;
	u8 ap_mode;
	u8 bmc_wlan_idx;
	u8 hidden_ssid;
	u8 disconnect_threshold;
	u8 iot_ap;
	u8 rsv1[2];
	__le32 private_data;
	struct {
		u8 bss_idx;
		u8 band;
		u8 primary_chan;
		u8 sco;
		u8 erp_protect;
		u8 ht_protect;
		u8 gf_mode;
		u8 rifs_mode;
		__le16 ht_op_info3;
		__le16 ht_op_info2;
		u8 ht_op_info1;
		u8 short_preamble;
		u8 short_slot;
		u8 vht_width;
		u8 vht_center1;
		u8 vht_center2;
		__le16 vht_basic_mcs;
		u8 tx_nss;
		u8 rx_nss;
	} __packed rlm;
	u8 dbdc_band;
	u8 wmm_idx;
	u8 dbdc_action;
	u8 nss;
	u8 rsv2[2];
	u8 he_op[3];
	u8 bss_color;
	__le16 he_basic_mcs;
	u8 max_bssid_indicator;
	u8 mbssid_idx;
	u8 rsv3[12];
} __packed;

struct mt7921_legacy_sta_rec {
	u8 sta_idx;
	u8 sta_type;
	u8 addr[ETH_ALEN];
	__le16 aid;
	__le16 listen_interval;
	u8 bss_idx;
	u8 phy_types;
	__le16 rates;
	__le16 basic_rates;
	u8 qos;
	u8 uapsd;
	u8 state;
	u8 mcs;
	u8 mcs32;
	u8 version;
	u8 rx_mcs_mask[IEEE80211_HT_MCS_MASK_LEN];
	__le16 rx_highest;
	__le32 tx_rate_info;
	__le16 ht_cap;
	__le16 ht_ext_cap;
	__le32 txbf_cap;
	u8 ampdu_param;
	u8 asel_cap;
	u8 rcpi;
	u8 need_resp;
	u8 uapsd_ac;
	u8 uapsd_sp;
	u8 wlan_idx;
	u8 bmc_wlan_idx;
	__le32 vht_cap;
	__le16 vht_rx_mcs;
	__le16 vht_rx_highest;
	__le16 vht_tx_mcs;
	__le16 vht_tx_highest;
	u8 rts_policy;
	u8 vht_op_mode;
	u8 traffic_type;
	u8 gf_mode;
	u8 sgi_mode;
	u8 stbc_mode;
	__le16 fixed_rate;
	u8 tx_ampdu;
	u8 rx_ampdu;
	u8 rsv0[2];
	__le32 fixed_phy_rate;
	__le16 max_link_speed;
	__le16 min_link_speed;
	__le32 flags;
	u8 ba_size[4];
	__le16 pfmu_id;
	u8 txbf[28];
	u8 mlr_mode;
	u8 mlr_state;
	u8 tx_amsdu_in_ampdu;
	u8 rx_amsdu_in_ampdu;
	u8 rsv1[2];
	__le32 tx_max_amsdu_len;
	u8 he_mac_cap[6];
	u8 he_phy_cap[11];
	u8 rsv2[3];
	__le16 he_mcs[6];
	__le16 he_6ghz_cap;
	u8 rsv3[30];
	u8 rsv4[4];
} __packed;

struct mt7921_legacy_sta_remove {
	u8 action;
	u8 sta_idx;
	u8 bss_idx;
	u8 rsv;
} __packed;

struct mt7921_legacy_key {
	u8 add;
	u8 tx_key;
	u8 pairwise;
	u8 authenticator;
	u8 peer[ETH_ALEN];
	u8 bss_idx;
	u8 cipher;
	u8 key_idx;
	u8 key_len;
	u8 wlan_idx;
	u8 mgmt_prot;
	u8 key[32];
	u8 rsc[16];
} __packed;
#endif
