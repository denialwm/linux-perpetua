// SPDX-License-Identifier: GPL-2.0
#include <linux/cleanup.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include "audioreach.h"
#include "q6apm-hostless.h"
#include "q6apm-voice.h"
#include "q6apm-voice-backend.h"

#define VOICE_GRAPH_MAGIC 0x504f564d /* MVOP */
#define VOICE_MAX_SUBGRAPHS 8
#define VOICE_ACTIVE_MAGIC 0x4153564d /* MVSA */
#define VOICE_SESSION_MAX_COMMANDS 32
#define VOICE_CAL_TABLE 0x08001163

enum q6apm_voice_command_type {
	VOICE_OPEN = 1,
	VOICE_SET,
	VOICE_REGISTER,
	VOICE_PREPARE,
	VOICE_START,
};

struct q6apm_voice_command {
	u32 type;
	u32 graph;
	u32 destination;
	u32 sequence;
	const u8 *data;
	size_t size;
};

struct q6apm_voice_control {
	struct q6apm_voice *voice;
	bool ucm;
};

struct q6apm_voice_graph {
	struct audioreach_graph_info info;
	struct audioreach_sub_graph subgraphs[VOICE_MAX_SUBGRAPHS];
	struct q6apm_hostless *session;
	const u8 *payload;
	size_t size;
};

struct q6apm_voice {
	struct snd_soc_component *component;
	struct mutex lock; /* Serialize session state and retained resource cleanup. */
	struct mutex operation_lock; /* PCM lifecycle must run outside lock/pcm_mutex. */
	struct mutex ucm_lock; /* Cancel work before taking operation_lock. */
	struct delayed_work ucm_work;
	struct task_struct *backend_task;
	struct snd_pcm_substream *backends[2];
	const struct firmware *firmware;
	struct q6apm_voice_graph graphs[2];
	struct q6apm_voice_command commands[VOICE_SESSION_MAX_COMMANDS];
	struct q6apm_voice_control controls[2];
	u32 num_commands;
	/* Writable copy of the profile's CAL_KEYS command; its volume key follows the control. */
	struct q6apm_voice_command *keys_command;
	u8 *keys;
	size_t volume_offset;
	u32 volume;
	bool pinned;
	bool card_pinned;
	bool call_active;
	bool ucm_requested;
	bool ucm_pinned;
};

/* Preserve the full firmware payload; only extract ownership for safe CLOSE. */
static int q6apm_voice_subgraphs(struct q6apm_voice_graph *graph,
				 const u8 *data, size_t size)
{
	u32 count, i, j;

	if (size < sizeof(u32))
		return -EINVAL;
	count = get_unaligned_le32(data);
	data += sizeof(u32);
	size -= sizeof(u32);
	if (!count || count > VOICE_MAX_SUBGRAPHS - graph->info.num_sub_graphs)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		struct audioreach_sub_graph *sg;
		u32 id, props, seen = 0;

		if (size < 2 * sizeof(u32))
			return -EINVAL;
		id = get_unaligned_le32(data);
		props = get_unaligned_le32(data + sizeof(u32));
		data += 2 * sizeof(u32);
		size -= 2 * sizeof(u32);
		if (!id || !props)
			return -EINVAL;
		for (j = 0; j < graph->info.num_sub_graphs; j++)
			if (graph->subgraphs[j].sub_graph_id == id)
				return -EINVAL;
		sg = &graph->subgraphs[graph->info.num_sub_graphs];
		sg->sub_graph_id = id;
		for (j = 0; j < props; j++) {
			u32 property, bytes, value, bit = 0;

			if (size < 2 * sizeof(u32))
				return -EINVAL;
			property = get_unaligned_le32(data);
			bytes = get_unaligned_le32(data + sizeof(u32));
			data += 2 * sizeof(u32);
			size -= 2 * sizeof(u32);
			if (bytes > size)
				return -EINVAL;
			switch (property) {
			case APM_SUB_GRAPH_PROP_ID_PERF_MODE:
				bit = BIT(0);
				break;
			case APM_SUB_GRAPH_PROP_ID_DIRECTION:
				bit = BIT(1);
				break;
			case APM_SUB_GRAPH_PROP_ID_SCENARIO_ID:
				bit = BIT(2);
				break;
			case APM_SUB_GRAPH_PROP_ID_VSID:
				bit = BIT(3);
				break;
			}
			if (bit) {
				if (bytes != sizeof(u32) || (seen & bit))
					return -EINVAL;
				value = get_unaligned_le32(data);
				if (bit == BIT(0))
					sg->perf_mode = value;
				else if (bit == BIT(1))
					sg->direction = value;
				else if (bit == BIT(2))
					sg->scenario_id = value;
				else if (value != APM_SUB_GRAPH_VSID_DONT_CARE)
					return -EINVAL;
				seen |= bit;
			}
			data += bytes;
			size -= bytes;
		}
		if ((seen & 7) != 7 ||
		    (sg->scenario_id == APM_SUB_GRAPH_SID_AUDIO_VOICE_CALL && !(seen & BIT(3))))
			return -EINVAL;
		sg->info = &graph->info;
		INIT_LIST_HEAD(&sg->container_list);
		list_add_tail(&sg->node, &graph->info.sg_list);
		graph->info.num_sub_graphs++;
	}
	return size ? -EINVAL : 0;
}

static int q6apm_voice_parse_graph(struct q6apm_voice_graph *graph,
				   const u8 *data, size_t size)
{
	bool voice_config = false;
	u32 i;
	int ret;

	if (!size || size % 8)
		return -EINVAL;
	INIT_LIST_HEAD(&graph->info.sg_list);
	graph->payload = data;
	graph->size = size;
	while (size) {
		const u8 *body;
		u32 module, parameter, bytes;
		size_t block;

		if (size < APM_MODULE_PARAM_DATA_SIZE)
			return -EINVAL;
		body = data + APM_MODULE_PARAM_DATA_SIZE;
		module = get_unaligned_le32(data);
		parameter = get_unaligned_le32(data + 4);
		bytes = get_unaligned_le32(data + 8);
		if (get_unaligned_le32(data + 12) || bytes > size - APM_MODULE_PARAM_DATA_SIZE)
			return -EINVAL;
		block = APM_MODULE_PARAM_DATA_SIZE + ALIGN((size_t)bytes, 8);
		if (block > size)
			return -EINVAL;
		if (module == APM_MODULE_INSTANCE_ID) {
			switch (parameter) {
			case APM_PARAM_ID_SUB_GRAPH_CONFIG:
				ret = q6apm_voice_subgraphs(graph, body, bytes);
				if (ret)
					return ret;
				break;
			case APM_PARAM_ID_CONTAINER_CONFIG:
			case APM_PARAM_ID_MODULE_LIST:
			case APM_PARAM_ID_MODULE_PROP:
			case APM_PARAM_ID_MODULE_CONN:
			case APM_PARAM_ID_MODULE_CTRL_LINK_CFG:
				break;
			default:
				return -EINVAL;
			}
		} else if (module == VCPM_MODULE_INSTANCE_ID &&
			   parameter == VCPM_PARAM_ID_VOICE_CONFIG) {
			voice_config = true;
		} else {
			return -EINVAL;
		}
		data += block;
		size -= block;
	}
	if (!voice_config)
		return -EINVAL;
	for (i = 0; i < graph->info.num_sub_graphs; i++)
		if (graph->subgraphs[i].scenario_id == APM_SUB_GRAPH_SID_AUDIO_VOICE_CALL)
			return 0;
	return -EINVAL;
}

static int q6apm_voice_parse_data(struct q6apm_voice *voice,
				  const u8 *data, size_t size)
{
	u32 rx_size, tx_size, i, j;
	int ret;

	if (size < 16 || size > SZ_64K || get_unaligned_le32(data) != VOICE_GRAPH_MAGIC ||
	    get_unaligned_le32(data + 4) != 1)
		return -EINVAL;
	rx_size = get_unaligned_le32(data + 8);
	tx_size = get_unaligned_le32(data + 12);
	if (rx_size > size - 16 || tx_size != size - 16 - rx_size)
		return -EINVAL;
	ret = q6apm_voice_parse_graph(&voice->graphs[0], data + 16, rx_size);
	if (ret)
		return ret;
	ret = q6apm_voice_parse_graph(&voice->graphs[1], data + 16 + rx_size, tx_size);
	if (ret)
		return ret;
	for (i = 0; i < voice->graphs[0].info.num_sub_graphs; i++)
		for (j = 0; j < voice->graphs[1].info.num_sub_graphs; j++)
			if (voice->graphs[0].subgraphs[i].sub_graph_id ==
			    voice->graphs[1].subgraphs[j].sub_graph_id)
				return -EINVAL;
	return 0;
}

static bool q6apm_voice_has_module(const struct q6apm_voice_graph *graph, u32 id)
{
	const u8 *data = graph->payload;
	size_t size = graph->size;

	while (size >= APM_MODULE_PARAM_DATA_SIZE) {
		u32 bytes = get_unaligned_le32(data + 8);
		size_t block = APM_MODULE_PARAM_DATA_SIZE + ALIGN((size_t)bytes, 8);

		if (block > size)
			return false;
		if (get_unaligned_le32(data + 4) == APM_PARAM_ID_MODULE_LIST) {
			const u8 *body = data + APM_MODULE_PARAM_DATA_SIZE;
			size_t remaining = bytes;
			u32 groups, i, j;

			if (remaining < 4)
				return false;
			groups = get_unaligned_le32(body);
			body += 4;
			remaining -= 4;
			for (i = 0; i < groups; i++) {
				u32 modules;

				if (remaining < 12)
					return false;
				modules = get_unaligned_le32(body + 8);
				body += 12;
				remaining -= 12;
				if (modules > remaining / 8)
					return false;
				for (j = 0; j < modules; j++)
					if (get_unaligned_le32(body + j * 8 + 4) == id)
						return true;
				body += modules * 8;
				remaining -= modules * 8;
			}
		}
		data += block;
		size -= block;
	}
	return false;
}

static int q6apm_voice_validate_set(const struct q6apm_voice_graph *graph,
				    u32 destination, const u8 *data, size_t size)
{
	if (destination != APM_MODULE_INSTANCE_ID && destination != VCPM_MODULE_INSTANCE_ID &&
	    !q6apm_voice_has_module(graph, destination))
		return -EINVAL;
	while (size) {
		u32 module, parameter, bytes;
		size_t block;

		if (size < APM_MODULE_PARAM_DATA_SIZE)
			return -EINVAL;
		module = get_unaligned_le32(data);
		parameter = get_unaligned_le32(data + 4);
		bytes = get_unaligned_le32(data + 8);
		block = APM_MODULE_PARAM_DATA_SIZE + ALIGN((size_t)bytes, 8);
		if (!parameter || !bytes || block > size || get_unaligned_le32(data + 12))
			return -EINVAL;
		if (destination != APM_MODULE_INSTANCE_ID && module != destination)
			return -EINVAL;
		if (module == VCPM_MODULE_INSTANCE_ID) {
			if (parameter != VCPM_PARAM_ID_VSID &&
			    parameter != VCPM_PARAM_ID_CAL_KEYS &&
			    parameter != VCPM_PARAM_ID_TX_DEV_PP_CHANNEL_INFO)
				return -EINVAL;
		} else if (!q6apm_voice_has_module(graph, module)) {
			return -EINVAL;
		}
		data += block;
		size -= block;
	}
	return 0;
}

static int q6apm_voice_validate_cal(const struct q6apm_voice_graph *graph,
				    const u8 *data, size_t size, u32 *seen)
{
	u32 bytes, sgid, table_size, i;

	if (size < APM_MODULE_PARAM_DATA_SIZE + 12 ||
	    get_unaligned_le32(data) != VCPM_MODULE_INSTANCE_ID ||
	    get_unaligned_le32(data + 4) != VOICE_CAL_TABLE || get_unaligned_le32(data + 12))
		return -EINVAL;
	bytes = get_unaligned_le32(data + 8);
	if (bytes < 12 || size != APM_MODULE_PARAM_DATA_SIZE + ALIGN((size_t)bytes, 8))
		return -EINVAL;
	data += APM_MODULE_PARAM_DATA_SIZE;
	if (get_unaligned_le32(data) != 1)
		return -EINVAL;
	sgid = get_unaligned_le32(data + 4);
	table_size = get_unaligned_le32(data + 8);
	if (!table_size || table_size > bytes - 12)
		return -EINVAL;
	for (i = 0; i < graph->info.num_sub_graphs; i++) {
		const struct audioreach_sub_graph *sg = &graph->subgraphs[i];

		if (sg->sub_graph_id != sgid)
			continue;
		if (sg->scenario_id != APM_SUB_GRAPH_SID_AUDIO_VOICE_CALL || (*seen & BIT(i)))
			return -EINVAL;
		*seen |= BIT(i);
		return 0;
	}
	return -EINVAL;
}

static void q6apm_voice_patch_volume(struct q6apm_voice *voice)
{
	if (voice->keys)
		put_unaligned_le32(q6apm_voice_board()->volume_levels - 1 - voice->volume,
				   voice->keys + voice->volume_offset);
}

/* Find the CAL_KEYS command's volume key; a profile without one has a fixed volume. */
static int q6apm_voice_volume_keys(struct q6apm_voice *voice)
{
	u32 i, j, bytes, pairs;

	for (i = 0; i < voice->num_commands; i++) {
		struct q6apm_voice_command *command = &voice->commands[i];
		const u8 *data = command->data;

		if (command->type != VOICE_SET ||
		    command->destination != VCPM_MODULE_INSTANCE_ID ||
		    get_unaligned_le32(data + 4) != VCPM_PARAM_ID_CAL_KEYS)
			continue;
		/* One parameter: VSID, pair count, then key and value pairs */
		bytes = get_unaligned_le32(data + 8);
		if (bytes < 8 || command->size != APM_MODULE_PARAM_DATA_SIZE + ALIGN(bytes, 8))
			return -EINVAL;
		pairs = get_unaligned_le32(data + APM_MODULE_PARAM_DATA_SIZE + 4);
		if (pairs > (bytes - 8) / 8)
			return -EINVAL;
		for (j = 0; j < pairs; j++) {
			size_t offset = APM_MODULE_PARAM_DATA_SIZE + 8 + j * 8;

			if (get_unaligned_le32(data + offset) != VCPM_CAL_KEY_ID_VOLUME_LEVEL)
				continue;
			voice->keys = kmemdup(data, command->size, GFP_KERNEL);
			if (!voice->keys)
				return -ENOMEM;
			command->data = voice->keys;
			voice->keys_command = command;
			voice->volume_offset = offset + 4;
			q6apm_voice_patch_volume(voice);
			return 0;
		}
	}
	return 0;
}

static int q6apm_voice_parse_session(struct q6apm_voice *voice)
{
	const u8 *data = voice->firmware->data;
	size_t size = voice->firmware->size;
	u32 open_size, count, opened = 0, prepared = 0, calibrated[2] = { };
	u32 i, j, previous = 0, started = 0;
	int ret;

	if (size < 16 || size > SZ_256K ||
	    get_unaligned_le32(data) != VOICE_ACTIVE_MAGIC ||
	    get_unaligned_le32(data + 4) != 1)
		return -EINVAL;
	open_size = get_unaligned_le32(data + 8);
	count = get_unaligned_le32(data + 12);
	if (open_size > size - 16 || !count || count > VOICE_SESSION_MAX_COMMANDS)
		return -EINVAL;
	ret = q6apm_voice_parse_data(voice, data + 16, open_size);
	if (ret)
		return ret;
	data += 16 + open_size;
	size -= 16 + open_size;
	for (i = 0; i < count; i++) {
		struct q6apm_voice_command *command = &voice->commands[i];

		if (size < 24 || get_unaligned_le32(data + 20))
			return -EINVAL;
		command->type = get_unaligned_le32(data);
		command->graph = get_unaligned_le32(data + 4);
		command->destination = get_unaligned_le32(data + 8);
		command->size = get_unaligned_le32(data + 12);
		command->sequence = get_unaligned_le32(data + 16);
		data += 24;
		size -= 24;
		command->data = data;
		if (command->graph > 1 || command->size > size || command->size % 8 ||
		    command->sequence <= previous)
			return -EINVAL;
		if ((prepared & BIT(command->graph)) &&
		    !((command->type == VOICE_START && !(started & BIT(command->graph))) ||
		      (command->type == VOICE_SET && started == 3)))
			return -EINVAL;
		previous = command->sequence;
		if (command->type == VOICE_OPEN) {
			if (command->size || command->destination != APM_MODULE_INSTANCE_ID ||
			    (opened & BIT(command->graph)))
				return -EINVAL;
			opened |= BIT(command->graph);
		} else {
			const struct q6apm_voice_graph *graph;

			graph = &voice->graphs[command->graph];

			if (!(opened & BIT(command->graph)))
				return -EINVAL;
			switch (command->type) {
			case VOICE_SET:
				if (!command->size)
					return -EINVAL;
				ret = q6apm_voice_validate_set(graph, command->destination,
							       data, command->size);
				if (ret)
					return ret;
				break;
			case VOICE_REGISTER:
				if (command->destination != APM_MODULE_INSTANCE_ID)
					return -EINVAL;
				ret = q6apm_voice_validate_cal(graph, data, command->size,
							       &calibrated[command->graph]);
				if (ret)
					return ret;
				break;
			case VOICE_PREPARE:
				if (command->size || command->destination != APM_MODULE_INSTANCE_ID)
					return -EINVAL;
				prepared |= BIT(command->graph);
				break;
			case VOICE_START:
				if (command->size ||
				    command->destination != APM_MODULE_INSTANCE_ID ||
				    !(prepared & BIT(command->graph)) ||
				    (started & BIT(command->graph)))
					return -EINVAL;
				started |= BIT(command->graph);
				break;
			default:
				return -EINVAL;
			}
		}
		data += command->size;
		size -= command->size;
	}
	if (size || opened != 3 || prepared != 3 || started != 3)
		return -EINVAL;
	for (i = 0; i < 2; i++)
		for (j = 0; j < voice->graphs[i].info.num_sub_graphs; j++)
			if (voice->graphs[i].subgraphs[j].scenario_id ==
			    APM_SUB_GRAPH_SID_AUDIO_VOICE_CALL && !(calibrated[i] & BIT(j)))
				return -EINVAL;
	voice->num_commands = count;
	return q6apm_voice_volume_keys(voice);
}

bool q6apm_voice_pcm_allowed(struct q6apm_voice *voice)
{
	if (!voice)
		return true;
	guard(mutex)(&voice->lock);
	return !voice->pinned && !READ_ONCE(voice->ucm_requested);
}

bool q6apm_voice_backend_allowed(struct q6apm_voice *voice)
{
	return voice && READ_ONCE(voice->pinned) && READ_ONCE(voice->backend_task) == current;
}

static int q6apm_voice_stop_pair(struct q6apm_voice *voice)
{
	int i, ret;

	/* VCPM commands can affect both graphs; drain either pending request first. */
	for (i = 0; i < 2; i++)
		if (voice->graphs[i].session && !q6apm_hostless_is_idle(voice->graphs[i].session))
			return -EINPROGRESS;
	/* Stop both connected graphs before deregistering either graph's tables. */
	for (i = 1; i >= 0; i--) {
		if (!voice->graphs[i].session)
			continue;
		ret = q6apm_hostless_stop(voice->graphs[i].session);
		if (ret)
			return ret;
	}
	return 0;
}

static int q6apm_voice_cleanup(struct q6apm_voice *voice)
{
	int i, ret;

	ret = q6apm_voice_stop_pair(voice);
	if (ret)
		return ret;
	if (voice->backends[0] || voice->backends[1])
		return -EBUSY;
	for (i = 1; i >= 0; i--) {
		if (!voice->graphs[i].session)
			continue;
		ret = q6apm_hostless_destroy(voice->graphs[i].session);
		if (ret)
			return ret;
		voice->graphs[i].session = NULL;
	}
	kfree(voice->keys);
	voice->keys = NULL;
	voice->keys_command = NULL;
	release_firmware(voice->firmware);
	voice->firmware = NULL;
	memset(voice->graphs, 0, sizeof(voice->graphs));
	memset(voice->commands, 0, sizeof(voice->commands));
	voice->num_commands = 0;
	if (voice->card_pinned) {
		voice->card_pinned = false;
		module_put(voice->component->card->owner);
	}
	if (voice->pinned) {
		q6apm_hostless_release(voice->component->dev, voice);
		voice->pinned = false;
		module_put(THIS_MODULE);
	}
	return 0;
}

static int q6apm_voice_open_graph(struct q6apm_voice *voice, u32 index)
{
	struct q6apm_voice_graph *graph = &voice->graphs[index];
	int ret;

	graph->session = q6apm_hostless_create(voice->component->dev, &graph->info);
	if (IS_ERR(graph->session)) {
		ret = PTR_ERR(graph->session);
		graph->session = NULL;
		return ret;
	}
	return q6apm_hostless_open_payload(graph->session, graph->payload, graph->size);
}

static int q6apm_voice_run_session(struct q6apm_voice *voice)
{
	static const char * const names[] = {
		"", "OPEN", "SET_CFG", "REGISTER_CFG", "PREPARE", "START"
	};
	u32 i;
	int ret;

	for (i = 0; i < voice->num_commands; i++) {
		const struct q6apm_voice_command *command = &voice->commands[i];
		struct q6apm_hostless *session = voice->graphs[command->graph].session;

		switch (command->type) {
		case VOICE_OPEN:
			ret = q6apm_voice_open_graph(voice, command->graph);
			break;
		case VOICE_SET:
			ret = q6apm_hostless_set_payload(session, command->destination,
							 command->data, command->size);
			break;
		case VOICE_REGISTER:
			ret = q6apm_hostless_register_cfg(session, command->data, command->size);
			break;
		case VOICE_PREPARE:
			ret = q6apm_hostless_prepare(session);
			break;
		case VOICE_START:
			ret = q6apm_hostless_start(session);
			break;
		default:
			return -EINVAL;
		}
		if (ret) {
			dev_err(voice->component->dev, "voice command %u %s %s failed: %d\n",
				command->sequence, command->graph ? "TX" : "RX",
				names[command->type], ret);
			return ret;
		}
	}
	return 0;
}

static int q6apm_voice_get(struct snd_kcontrol *control, struct snd_ctl_elem_value *value)
{
	const struct q6apm_voice_control *request = (void *)control->private_value;
	struct q6apm_voice *voice = request->voice;

	guard(mutex)(&voice->lock);
	value->value.integer.value[0] = request->ucm ? READ_ONCE(voice->ucm_requested) :
		voice->call_active;
	return 0;
}

static int q6apm_voice_call_cleanup(struct q6apm_voice *voice)
{
	int i, ret;

	scoped_guard(mutex, &voice->lock) {
		voice->call_active = false;
		ret = q6apm_voice_stop_pair(voice);
	}
	if (ret)
		return ret;
	/* Keep physical clocks/ports alive until both DSP STOP replies are known. */
	for (i = 1; i >= 0; i--)
		q6apm_voice_backend_close(&voice->backends[i]);
	scoped_guard(mutex, &voice->component->card->pcm_mutex) {
		guard(mutex)(&voice->lock);
		return q6apm_voice_cleanup(voice);
	}
	return -EINVAL;
}

static int q6apm_voice_call_put(struct q6apm_voice *voice, bool requested)
{
	struct snd_soc_component *component = voice->component;
	int i, ret, cleanup;

	if (!requested) {
		ret = q6apm_voice_call_cleanup(voice);
		return ret ? ret : 1;
	}
	scoped_guard(mutex, &component->card->pcm_mutex) {
		guard(mutex)(&voice->lock);
		if (voice->call_active)
			return 0;
		if (voice->pinned || snd_soc_component_active(component))
			return -EBUSY;
		ret = request_firmware(&voice->firmware, q6apm_voice_board()->firmware,
			       component->dev);
		if (ret)
			return ret;
		ret = q6apm_voice_parse_session(voice);
		if (ret)
			goto out;
		ret = q6apm_hostless_claim(component->dev, voice);
		if (ret)
			goto out;
		if (!try_module_get(THIS_MODULE)) {
			q6apm_hostless_release(component->dev, voice);
			ret = -ENODEV;
			goto out;
		}
		if (!try_module_get(component->card->owner)) {
			module_put(THIS_MODULE);
			q6apm_hostless_release(component->dev, voice);
			ret = -ENODEV;
			goto out;
		}
		voice->card_pinned = true;
		WRITE_ONCE(voice->pinned, true);
	}
	/* ASoC takes pcm_mutex itself; no session lock may be held across its calls. */
	WRITE_ONCE(voice->backend_task, current);
	for (i = 0; i < 2; i++) {
		ret = q6apm_voice_backend_open(component, i, &voice->backends[i]);
		if (ret)
			break;
	}
	WRITE_ONCE(voice->backend_task, NULL);
	if (ret)
		goto out;
	scoped_guard(mutex, &voice->lock) {
		ret = q6apm_voice_run_session(voice);
		if (!ret)
			voice->call_active = true;
	}
	if (!ret)
		return 1;
out:
	cleanup = q6apm_voice_call_cleanup(voice);
	if (cleanup)
		dev_err(component->dev, "voice call cleanup: %d; retry %s=0\n", cleanup,
			voice->ucm_pinned ? "Voice Call UCM Switch" : "Voice Call Switch");
	return ret ? ret : cleanup;
}

static void q6apm_voice_ucm_work(struct work_struct *work)
{
	struct q6apm_voice *voice = container_of(to_delayed_work(work),
							    struct q6apm_voice, ucm_work);
	int ret;

	guard(mutex)(&voice->operation_lock);
	if (!READ_ONCE(voice->ucm_requested) || voice->call_active)
		return;
	/* A prior failed START may still own DMA, clocks or a pending DSP reply. */
	if (READ_ONCE(voice->pinned)) {
		ret = q6apm_voice_call_cleanup(voice);
		if (ret)
			goto retry;
	}
	ret = q6apm_voice_call_put(voice, true);
	if (ret >= 0)
		return;
	/* Malformed/unsupported configurations require a new off/on request. */
	if (ret != -EBUSY && !READ_ONCE(voice->pinned)) {
		dev_err(voice->component->dev, "voice UCM START failed: %d; disable UCM request\n",
			ret);
		return;
	}
retry:
	if (READ_ONCE(voice->ucm_requested))
		queue_delayed_work(system_long_wq, &voice->ucm_work, msecs_to_jiffies(100));
}

static int q6apm_voice_ucm_put(struct q6apm_voice *voice, bool requested)
{
	bool changed;
	int ret;

	guard(mutex)(&voice->ucm_lock);
	if (requested) {
		guard(mutex)(&voice->operation_lock);
		guard(mutex)(&voice->lock);
		if (voice->ucm_requested)
			return 0;
		if (voice->ucm_pinned || voice->pinned)
			return -EBUSY;
		if (!try_module_get(THIS_MODULE))
			return -ENODEV;
		voice->ucm_pinned = true;
		WRITE_ONCE(voice->ucm_requested, true);
		/* UCM card probing may briefly enable devices. Debounce START. */
		queue_delayed_work(system_long_wq, &voice->ucm_work, msecs_to_jiffies(300));
		return 1;
	}
	/* Never hold operation_lock while joining a worker that takes it. */
	changed = READ_ONCE(voice->ucm_requested) || voice->ucm_pinned;
	WRITE_ONCE(voice->ucm_requested, false);
	cancel_delayed_work_sync(&voice->ucm_work);
	guard(mutex)(&voice->operation_lock);
	ret = q6apm_voice_call_cleanup(voice);
	if (ret)
		return ret;
	if (voice->ucm_pinned) {
		voice->ucm_pinned = false;
		module_put(THIS_MODULE);
	}
	return changed;
}

static int q6apm_voice_put(struct snd_kcontrol *control, struct snd_ctl_elem_value *value)
{
	const struct q6apm_voice_control *request = (void *)control->private_value;
	struct q6apm_voice *voice = request->voice;
	long requested = value->value.integer.value[0];

	if (requested != 0 && requested != 1)
		return -EINVAL;
	if (request->ucm)
		return q6apm_voice_ucm_put(voice, requested);
	guard(mutex)(&voice->operation_lock);
	if (READ_ONCE(voice->ucm_requested) || voice->ucm_pinned)
		return -EBUSY;
	return q6apm_voice_call_put(voice, requested);
}

static int q6apm_voice_volume_info(struct snd_kcontrol *control,
				   struct snd_ctl_elem_info *info)
{
	info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	info->count = 1;
	info->value.integer.min = 0;
	info->value.integer.max = q6apm_voice_board()->volume_levels - 1;
	return 0;
}

static int q6apm_voice_volume_get(struct snd_kcontrol *control,
				  struct snd_ctl_elem_value *value)
{
	struct q6apm_voice *voice = (void *)control->private_value;

	guard(mutex)(&voice->lock);
	value->value.integer.value[0] = voice->volume;
	return 0;
}

/* Highest is loudest. A running call takes the new level at once. */
static int q6apm_voice_volume_put(struct snd_kcontrol *control,
				  struct snd_ctl_elem_value *value)
{
	struct q6apm_voice *voice = (void *)control->private_value;
	long level = value->value.integer.value[0];
	const struct q6apm_voice_command *command;
	u32 previous;
	int ret;

	if (level < 0 || level >= q6apm_voice_board()->volume_levels)
		return -EINVAL;
	guard(mutex)(&voice->lock);
	if (level == voice->volume)
		return 0;
	previous = voice->volume;
	voice->volume = level;
	q6apm_voice_patch_volume(voice);
	command = voice->keys_command;
	if (!voice->call_active || !command)
		return 1;
	ret = q6apm_hostless_set_payload(voice->graphs[command->graph].session,
					 command->destination, command->data, command->size);
	if (ret) {
		voice->volume = previous;
		q6apm_voice_patch_volume(voice);
		return ret;
	}
	return 1;
}

struct q6apm_voice *q6apm_voice_create(struct snd_soc_component *component)
{
	struct snd_kcontrol_new controls[3] = { {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Voice Call Switch",
		.info = snd_ctl_boolean_mono_info,
		.get = q6apm_voice_get,
		.put = q6apm_voice_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Voice Call UCM Switch",
		.info = snd_ctl_boolean_mono_info,
		.get = q6apm_voice_get,
		.put = q6apm_voice_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Voice Call Volume",
		.info = q6apm_voice_volume_info,
		.get = q6apm_voice_volume_get,
		.put = q6apm_voice_volume_put,
	} };
	struct q6apm_voice *voice;
	u32 i;
	int ret;

	if (!q6apm_voice_board())
		return NULL;
	voice = kzalloc_obj(*voice);
	if (!voice)
		return ERR_PTR(-ENOMEM);
	voice->component = component;
	mutex_init(&voice->lock);
	mutex_init(&voice->operation_lock);
	mutex_init(&voice->ucm_lock);
	INIT_DELAYED_WORK(&voice->ucm_work, q6apm_voice_ucm_work);
	voice->volume = q6apm_voice_board()->volume_levels - 1;
	for (i = 0; i < ARRAY_SIZE(voice->controls); i++) {
		voice->controls[i].voice = voice;
		voice->controls[i].ucm = i == 1;
		controls[i].private_value = (unsigned long)&voice->controls[i];
	}
	controls[2].private_value = (unsigned long)voice;
	ret = snd_soc_add_component_controls(component, controls, ARRAY_SIZE(controls));
	if (ret) {
		kfree(voice);
		return ERR_PTR(ret);
	}
	return voice;
}

void q6apm_voice_free(struct q6apm_voice *voice)
{
	int ret;

	if (IS_ERR_OR_NULL(voice))
		return;
	ret = q6apm_voice_ucm_put(voice, false);
	if (ret < 0) {
		/* Preserve callback data and descriptors for an unresolved DSP request. */
		dev_err(voice->component->dev, "voice removal cleanup failed: %d\n",
			ret);
		return;
	}
	kfree(voice);
}
