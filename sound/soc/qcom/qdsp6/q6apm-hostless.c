// SPDX-License-Identifier: GPL-2.0
#include <linux/completion.h>
#include <linux/cleanup.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/spinlock.h>
#include <dt-bindings/soc/qcom,gpr.h>
#include "q6apm.h"
#include "q6apm-hostless.h"

struct q6apm_hostless_buffer {
	struct list_head node;
	void *area;
	dma_addr_t dma_addr;
	u64 dsp_addr;
	size_t size;
	size_t allocation_size;
	u32 handle;
	u32 map_token;
	u32 command_token;
	u32 command_opcode;
	bool map_failed;
	bool unmapped;
	bool register_pending;
	struct apm_module_param_data header;
};

struct q6apm_hostless {
	struct device *dev;
	const struct audioreach_graph_info *info;
	gpr_port_t *port;
	struct mutex lock; /* Serialize session commands and lifetime transitions. */
	spinlock_t response_lock; /* Protect callback-visible transaction state. */
	struct completion response;
	struct list_head buffers;
	struct q6apm_hostless_buffer *pending_map;
	struct q6apm_hostless_buffer *pending_cal;
	u32 token;
	u32 opcode;
	u32 destination;
	u32 map_handle;
	u32 sid;
	bool has_sid;
	int result;
	bool waiting;
	bool uncertain;
	bool graph_pending;
	bool start_pending;
	bool prepared;
	bool ready;
};

static atomic_t hostless_token = ATOMIC_INIT(0);

struct q6apm_hostless_mgmt {
	struct apm_module_param_data param_data;
	u32 num_sub_graphs;
	u32 sub_graph_id_list[];
} __packed;

static int q6apm_hostless_callback(const struct gpr_resp_pkt *packet, void *priv, int op)
{
	struct q6apm_hostless *session = priv;
	gpr_port_t *port = READ_ONCE(session->port);
	const struct gpr_ibasic_rsp_result_t *basic;
	const struct apm_cmd_rsp_shared_mem_map_regions *map;
	unsigned long flags;
	int result;
	u32 handle = 0;

	spin_lock_irqsave(&session->response_lock, flags);
	if (!port || packet->hdr.src_domain != GPR_DOMAIN_ID_ADSP ||
	    packet->hdr.dest_port != port->id ||
	    packet->hdr.dest_domain != GPR_DOMAIN_ID_APPS)
		goto out;
	/* Retain the handle of a map whose response arrived after its timeout. */
	if (session->pending_map && packet->hdr.src_port == APM_MODULE_INSTANCE_ID &&
	    packet->hdr.token == session->pending_map->map_token && packet->payload) {
		if (packet->hdr.opcode == APM_CMD_RSP_SHARED_MEM_MAP_REGIONS &&
		    packet->payload_size >= (int)sizeof(*map)) {
			map = packet->payload;
			session->pending_map->handle = map->mem_map_handle;
		} else if (packet->hdr.opcode == GPR_BASIC_RSP_RESULT &&
			   packet->payload_size >= (int)sizeof(*basic)) {
			basic = packet->payload;
			if (basic->opcode == APM_CMD_SHARED_MEM_MAP_REGIONS && basic->status)
				session->pending_map->map_failed = true;
		}
	}
	if ((!session->waiting && !session->uncertain) || packet->hdr.token != session->token ||
	    packet->hdr.src_port != session->destination)
		goto out;
	if (packet->hdr.opcode == GPR_BASIC_RSP_RESULT) {
		if (packet->payload_size < (int)sizeof(*basic) || !packet->payload)
			goto out;
		basic = packet->payload;
		if (basic->opcode != session->opcode)
			goto out;
		/* A successful map is completed by the response carrying its handle. */
		if (!basic->status && session->opcode == APM_CMD_SHARED_MEM_MAP_REGIONS)
			goto out;
		result = basic->status ? -EREMOTEIO : 0;
		if (basic->status)
			dev_err(session->dev, "hostless opcode %#x destination %#x DSP status %#x\n",
				session->opcode, session->destination, basic->status);
	} else if (packet->hdr.opcode == APM_CMD_RSP_SHARED_MEM_MAP_REGIONS &&
		   session->opcode == APM_CMD_SHARED_MEM_MAP_REGIONS) {
		if (packet->payload_size < (int)sizeof(*map) || !packet->payload)
			goto out;
		map = packet->payload;
		handle = map->mem_map_handle;
		result = handle ? 0 : -EPROTO;
	} else {
		goto out;
	}
	session->map_handle = handle;
	if (session->pending_cal && packet->hdr.token == session->pending_cal->command_token &&
	    session->opcode == session->pending_cal->command_opcode) {
		if (!result && (session->opcode == APM_CMD_REGISTER_CFG ||
				session->opcode == APM_CMD_DEREGISTER_CFG)) {
			struct q6apm_hostless_buffer *buffer = session->pending_cal;
			struct apm_module_param_data *header = buffer->area;

			dma_rmb();
			if (header->module_instance_id != buffer->header.module_instance_id ||
			    header->param_id != buffer->header.param_id ||
			    header->param_size != buffer->header.param_size)
				result = -EPROTO;
			else if (header->error_code)
				result = -EREMOTEIO;
		}
		if (!result && (session->opcode == APM_CMD_GRAPH_OPEN ||
				session->opcode == APM_CMD_SET_CFG)) {
			struct q6apm_hostless_buffer *buffer = session->pending_cal;
			const u8 *data = buffer->area;
			size_t remaining = buffer->size;

			dma_rmb();
			while (remaining) {
				const struct apm_module_param_data *header = (const void *)data;
				size_t block;

				if (remaining < sizeof(*header) ||
				    header->param_size > remaining - sizeof(*header)) {
					result = -EPROTO;
					break;
				}
				block = sizeof(*header) + ALIGN((size_t)header->param_size, 8);
				if (block > remaining || header->error_code) {
					result = header->error_code ? -EREMOTEIO : -EPROTO;
					dev_err(session->dev,
						"hostless opcode %#x module %#x parameter %#x error %#x\n",
						session->opcode, header->module_instance_id,
						header->param_id, header->error_code);
					break;
				}
				data += block;
				remaining -= block;
			}
		}
		if (!result && session->opcode == APM_CMD_DEREGISTER_CFG)
			WRITE_ONCE(session->pending_cal->register_pending, false);
		if (!result && session->opcode == APM_CMD_SHARED_MEM_UNMAP_REGIONS)
			session->pending_cal->unmapped = true;
	}
	if (!result) {
		switch (session->opcode) {
		case APM_CMD_GRAPH_OPEN:
			WRITE_ONCE(session->ready, true);
			break;
		case APM_CMD_GRAPH_PREPARE:
			WRITE_ONCE(session->prepared, true);
			break;
		case APM_CMD_GRAPH_STOP:
			WRITE_ONCE(session->start_pending, false);
			break;
		case APM_CMD_GRAPH_CLOSE:
			WRITE_ONCE(session->graph_pending, false);
			WRITE_ONCE(session->ready, false);
			WRITE_ONCE(session->prepared, false);
			WRITE_ONCE(session->start_pending, false);
			break;
		}
	}
	session->result = result;
	session->waiting = false;
	session->uncertain = false;
	complete(&session->response);
out:
	spin_unlock_irqrestore(&session->response_lock, flags);
	return 0;
}

static bool q6apm_hostless_uses_buffer(const struct q6apm_hostless *session,
				       const struct gpr_pkt *packet)
{
	const struct apm_cmd_header *command;

	if (!session->pending_cal)
		return false;
	switch (packet->hdr.opcode) {
	case APM_CMD_REGISTER_CFG:
	case APM_CMD_DEREGISTER_CFG:
	case APM_CMD_SHARED_MEM_UNMAP_REGIONS:
		return true;
	case APM_CMD_GRAPH_OPEN:
	case APM_CMD_SET_CFG:
		if (packet->hdr.pkt_size < GPR_HDR_SIZE + APM_CMD_HDR_SIZE ||
		    !session->pending_cal->handle)
			return false;
		command = (const void *)packet + GPR_HDR_SIZE;
		return command->mem_map_handle == session->pending_cal->handle;
	default:
		return false;
	}
}

/* Caller holds session->lock. Tokens prevent late replies completing a retry. */
static int q6apm_hostless_send(struct q6apm_hostless *session, struct gpr_pkt *packet)
{
	unsigned long flags;
	int ret;
	u32 token;

	do {
		token = atomic_inc_return(&hostless_token);
	} while (!token);
	packet->hdr.src_port = session->port->id;
	packet->hdr.token = token;
	reinit_completion(&session->response);
	spin_lock_irqsave(&session->response_lock, flags);
	if (session->uncertain) {
		spin_unlock_irqrestore(&session->response_lock, flags);
		return -EINPROGRESS;
	}
	session->token = token;
	if (packet->hdr.opcode == APM_CMD_SHARED_MEM_MAP_REGIONS && session->pending_map)
		session->pending_map->map_token = token;
	if (q6apm_hostless_uses_buffer(session, packet)) {
		session->pending_cal->command_token = token;
		session->pending_cal->command_opcode = packet->hdr.opcode;
	}
	session->opcode = packet->hdr.opcode;
	session->destination = packet->hdr.dest_port;
	session->map_handle = 0;
	session->result = -ETIMEDOUT;
	session->waiting = true;
	spin_unlock_irqrestore(&session->response_lock, flags);
	ret = gpr_send_port_pkt(session->port, packet);
	if (ret >= 0 && !wait_for_completion_timeout(&session->response, 5 * HZ))
		ret = -ETIMEDOUT;
	spin_lock_irqsave(&session->response_lock, flags);
	if (ret >= 0)
		ret = session->result;
	if (ret == -ETIMEDOUT && session->waiting)
		session->uncertain = true;
	session->waiting = false;
	spin_unlock_irqrestore(&session->response_lock, flags);
	if (ret)
		dev_err(session->dev, "Hostless opcode %#x token %#x failed: %d\n",
			packet->hdr.opcode, token, ret);
	return ret;
}

static bool q6apm_hostless_idle(struct q6apm_hostless *session)
{
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&session->response_lock, flags);
	idle = !session->uncertain && !session->waiting;
	spin_unlock_irqrestore(&session->response_lock, flags);
	return idle;
}

bool q6apm_hostless_is_idle(struct q6apm_hostless *session)
{
	return q6apm_hostless_idle(session);
}
EXPORT_SYMBOL_GPL(q6apm_hostless_is_idle);

static int q6apm_hostless_graph_cmd(struct q6apm_hostless *session, u32 opcode)
{
	struct q6apm_hostless_mgmt *command;
	struct audioreach_sub_graph *sg;
	u32 count = session->info->num_sub_graphs;
	size_t size;
	u32 i = 0;

	size = ALIGN(struct_size(command, sub_graph_id_list, count), 8);
	if (size > GENMASK(23, 0) - GPR_HDR_SIZE - APM_CMD_HDR_SIZE)
		return -E2BIG;
	struct gpr_pkt *packet __free(kfree) = audioreach_alloc_apm_cmd_pkt(size, opcode, 0);
	if (IS_ERR(packet))
		return PTR_ERR(packet);
	command = (void *)packet + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;
	command->param_data.module_instance_id = APM_MODULE_INSTANCE_ID;
	command->param_data.param_id = APM_PARAM_ID_SUB_GRAPH_LIST;
	command->param_data.param_size = size - APM_MODULE_PARAM_DATA_SIZE;
	command->num_sub_graphs = count;
	list_for_each_entry(sg, &session->info->sg_list, node)
		command->sub_graph_id_list[i++] = sg->sub_graph_id;
	return q6apm_hostless_send(session, packet);
}

int q6apm_hostless_claim(struct device *dma_dev, const void *owner)
{
	struct q6apm *apm;

	if (!dma_dev || !dma_dev->parent || !owner)
		return -EINVAL;
	apm = dev_get_drvdata(dma_dev->parent);
	if (!apm)
		return -ENODEV;
	guard(mutex)(&apm->graph_lock);
	guard(mutex)(&apm->lock);
	if (READ_ONCE(apm->topology_uncertain))
		return -EUCLEAN;
	if (apm->hostless_owner || !idr_is_empty(&apm->graph_idr))
		return -EBUSY;
	apm->hostless_owner = owner;
	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_hostless_claim);

void q6apm_hostless_release(struct device *dma_dev, const void *owner)
{
	struct q6apm *apm = dev_get_drvdata(dma_dev->parent);

	guard(mutex)(&apm->graph_lock);
	if (apm->hostless_owner == owner)
		apm->hostless_owner = NULL;
}
EXPORT_SYMBOL_GPL(q6apm_hostless_release);

struct q6apm_hostless *q6apm_hostless_create(struct device *dma_dev,
					     const struct audioreach_graph_info *info)
{
	struct q6apm *apm;
	struct q6apm_hostless *session;
	struct audioreach_sub_graph *sg;
	struct of_phandle_args args;
	u32 count = 0;
	bool voice = false;

	if (!dma_dev || !dma_dev->parent || !info)
		return ERR_PTR(-EINVAL);
	if (!of_device_is_compatible(dma_dev->of_node, "qcom,q6apm-dais"))
		return ERR_PTR(-EINVAL);
	apm = dev_get_drvdata(dma_dev->parent);
	if (!apm)
		return ERR_PTR(-EINVAL);
	list_for_each_entry(sg, &info->sg_list, node) {
		if (!sg->sub_graph_id)
			return ERR_PTR(-EINVAL);
		voice |= sg->scenario_id == APM_SUB_GRAPH_SID_AUDIO_VOICE_CALL;
		count++;
	}
	if (!voice || !count || count != info->num_sub_graphs)
		return ERR_PTR(-EINVAL);
	if (!try_module_get(THIS_MODULE))
		return ERR_PTR(-ENODEV);
	session = kzalloc_obj(*session);
	if (!session) {
		module_put(THIS_MODULE);
		return ERR_PTR(-ENOMEM);
	}
	session->dev = get_device(dma_dev);
	session->info = info;
	mutex_init(&session->lock);
	spin_lock_init(&session->response_lock);
	init_completion(&session->response);
	INIT_LIST_HEAD(&session->buffers);
	if (!of_parse_phandle_with_fixed_args(dma_dev->of_node, "iommus", 1, 0, &args)) {
		session->sid = args.args[0] & 0xf;
		session->has_sid = true;
		of_node_put(args.np);
	}
	WRITE_ONCE(session->port,
		   gpr_alloc_port(apm->gdev, dma_dev, q6apm_hostless_callback, session));
	if (IS_ERR(session->port)) {
		void *error = session->port;

		put_device(session->dev);
		kfree(session);
		module_put(THIS_MODULE);
		return error;
	}
	return session;
}
EXPORT_SYMBOL_GPL(q6apm_hostless_create);

int q6apm_hostless_open(struct q6apm_hostless *session)
{
	int ret;

	guard(mutex)(&session->lock);
	if (!q6apm_hostless_idle(session))
		return -EINPROGRESS;
	if (READ_ONCE(session->graph_pending))
		return -EBUSY;
	struct gpr_pkt *packet __free(kfree) = audioreach_alloc_graph_pkt(NULL, session->info);
	if (IS_ERR(packet))
		return PTR_ERR(packet);
	WRITE_ONCE(session->graph_pending, true);
	WRITE_ONCE(session->prepared, false);
	WRITE_ONCE(session->ready, false);
	ret = q6apm_hostless_send(session, packet);
	if (!ret)
		WRITE_ONCE(session->ready, true);
	else if (ret != -ETIMEDOUT && ret != -EREMOTEIO)
		WRITE_ONCE(session->graph_pending, false);
	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_hostless_open);

int q6apm_hostless_set_cfg(struct q6apm_hostless *session, u32 destination,
			   u32 module, u32 parameter, const void *data, size_t size)
{
	struct apm_module_param_data *config;
	size_t packet_size;

	if (!module || !parameter || (size && !data) ||
	    (destination != APM_MODULE_INSTANCE_ID && destination != VCPM_MODULE_INSTANCE_ID))
		return -EINVAL;
	packet_size = size_add(sizeof(*config), ALIGN(size, 8));
	if (size > GENMASK(23, 0) ||
	    packet_size > GENMASK(23, 0) - GPR_HDR_SIZE - APM_CMD_HDR_SIZE)
		return -E2BIG;
	guard(mutex)(&session->lock);
	if (!q6apm_hostless_idle(session))
		return -EINPROGRESS;
	if (!READ_ONCE(session->ready))
		return -ENODEV;
	struct gpr_pkt *packet __free(kfree) = audioreach_alloc_cmd_pkt(packet_size,
					APM_CMD_SET_CFG, 0, session->port->id, destination);
	if (IS_ERR(packet))
		return PTR_ERR(packet);
	config = (void *)packet + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;
	config->module_instance_id = module;
	config->param_id = parameter;
	config->param_size = size;
	if (size)
		memcpy(config + 1, data, size);
	return q6apm_hostless_send(session, packet);
}
EXPORT_SYMBOL_GPL(q6apm_hostless_set_cfg);

static int q6apm_hostless_oob(struct q6apm_hostless *session,
			      struct q6apm_hostless_buffer *buffer, u32 opcode, u32 destination)
{
	struct apm_cmd_header *command;
	unsigned long flags;
	struct gpr_pkt *packet __free(kfree) = audioreach_alloc_pkt(sizeof(*command),
					opcode, 0, session->port->id, destination);
	if (IS_ERR(packet))
		return PTR_ERR(packet);
	command = (void *)packet + GPR_HDR_SIZE;
	command->payload_address_lsw = lower_32_bits(buffer->dsp_addr);
	command->payload_address_msw = upper_32_bits(buffer->dsp_addr);
	command->mem_map_handle = buffer->handle;
	command->payload_size = buffer->size;
	spin_lock_irqsave(&session->response_lock, flags);
	if (session->uncertain) {
		spin_unlock_irqrestore(&session->response_lock, flags);
		return -EINPROGRESS;
	}
	session->pending_cal = buffer;
	spin_unlock_irqrestore(&session->response_lock, flags);
	/* DSP may have written per-parameter result fields on a previous attempt. */
	if (opcode == APM_CMD_REGISTER_CFG || opcode == APM_CMD_DEREGISTER_CFG)
		memcpy(buffer->area, &buffer->header, sizeof(buffer->header));
	dma_wmb();
	return q6apm_hostless_send(session, packet);
}

static int q6apm_hostless_unmap(struct q6apm_hostless *session,
				struct q6apm_hostless_buffer *buffer)
{
	struct apm_cmd_shared_mem_unmap_regions *command;
	unsigned long flags;
	struct gpr_pkt *packet __free(kfree) = audioreach_alloc_pkt(sizeof(*command),
					APM_CMD_SHARED_MEM_UNMAP_REGIONS, 0,
					session->port->id, APM_MODULE_INSTANCE_ID);
	if (IS_ERR(packet))
		return PTR_ERR(packet);
	command = (void *)packet + GPR_HDR_SIZE;
	command->mem_map_handle = buffer->handle;
	spin_lock_irqsave(&session->response_lock, flags);
	if (session->uncertain) {
		spin_unlock_irqrestore(&session->response_lock, flags);
		return -EINPROGRESS;
	}
	session->pending_cal = buffer;
	spin_unlock_irqrestore(&session->response_lock, flags);
	return q6apm_hostless_send(session, packet);
}

static struct q6apm_hostless_buffer *
q6apm_hostless_map_blob(struct q6apm_hostless *session, const void *data, size_t size)
{
	struct apm_cmd_shared_mem_map_regions *map;
	struct apm_shared_map_region_payload *region;
	struct q6apm_hostless_buffer *buffer;
	struct gpr_pkt *packet __free(kfree) = NULL;
	unsigned long flags;
	int ret;

	if (session->pending_map)
		return ERR_PTR(-EUCLEAN);
	buffer = kzalloc_obj(*buffer);
	if (!buffer)
		return ERR_PTR(-ENOMEM);
	buffer->size = size;
	buffer->allocation_size = ALIGN(size, 4096);
	buffer->area = dma_alloc_coherent(session->dev, buffer->allocation_size,
					  &buffer->dma_addr, GFP_KERNEL);
	if (!buffer->area) {
		kfree(buffer);
		return ERR_PTR(-ENOMEM);
	}
	memset(buffer->area, 0, buffer->allocation_size);
	memcpy(buffer->area, data, size);
	if (session->has_sid && upper_32_bits(buffer->dma_addr)) {
		ret = -EOVERFLOW;
		goto free_buffer;
	}
	buffer->dsp_addr = buffer->dma_addr | ((u64)session->sid << 32);
	packet = audioreach_alloc_pkt(sizeof(*map) + sizeof(*region),
				      APM_CMD_SHARED_MEM_MAP_REGIONS, 0,
				      session->port->id, APM_MODULE_INSTANCE_ID);
	if (IS_ERR(packet)) {
		ret = PTR_ERR(packet);
		goto free_buffer;
	}
	map = (void *)packet + GPR_HDR_SIZE;
	map->mem_pool_id = APM_MEMORY_MAP_SHMEM8_4K_POOL;
	map->num_regions = 1;
	region = (void *)(map + 1);
	region->shm_addr_lsw = lower_32_bits(buffer->dsp_addr);
	region->shm_addr_msw = upper_32_bits(buffer->dsp_addr);
	region->mem_size_bytes = buffer->allocation_size;
	spin_lock_irqsave(&session->response_lock, flags);
	session->pending_map = buffer;
	spin_unlock_irqrestore(&session->response_lock, flags);
	dma_wmb();
	ret = q6apm_hostless_send(session, packet);
	if (ret == -ETIMEDOUT || ret == -EPROTO) {
		/* A late response can recover the handle; never free unknown DSP memory. */
		list_add_tail(&buffer->node, &session->buffers);
		return ERR_PTR(ret);
	}
	spin_lock_irqsave(&session->response_lock, flags);
	session->pending_map = NULL;
	spin_unlock_irqrestore(&session->response_lock, flags);
	if (ret)
		goto free_buffer;
	buffer->handle = session->map_handle;
	list_add_tail(&buffer->node, &session->buffers);
	return buffer;
free_buffer:
	dma_free_coherent(session->dev, buffer->allocation_size, buffer->area, buffer->dma_addr);
	kfree(buffer);
	return ERR_PTR(ret);
}

int q6apm_hostless_open_payload(struct q6apm_hostless *session,
				const void *data, size_t size)
{
	struct q6apm_hostless_buffer *buffer;
	int ret;

	if (!data || !size || size % 8)
		return -EINVAL;
	if (size > U32_MAX - 4095)
		return -EOVERFLOW;
	guard(mutex)(&session->lock);
	if (!q6apm_hostless_idle(session))
		return -EINPROGRESS;
	if (READ_ONCE(session->graph_pending))
		return -EBUSY;
	buffer = q6apm_hostless_map_blob(session, data, size);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);
	WRITE_ONCE(session->graph_pending, true);
	WRITE_ONCE(session->prepared, false);
	WRITE_ONCE(session->ready, false);
	ret = q6apm_hostless_oob(session, buffer, APM_CMD_GRAPH_OPEN, APM_MODULE_INSTANCE_ID);
	/* APM closes what a failed OPEN created: a DSP error leaves nothing open. */
	if (!ret)
		WRITE_ONCE(session->ready, true);
	else if (ret != -ETIMEDOUT && ret != -EPROTO)
		WRITE_ONCE(session->graph_pending, false);
	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_hostless_open_payload);

int q6apm_hostless_set_payload(struct q6apm_hostless *session, u32 destination,
			       const void *data, size_t size)
{
	struct q6apm_hostless_buffer *buffer;

	if (!destination || !data || size < APM_MODULE_PARAM_DATA_SIZE || size % 8)
		return -EINVAL;
	if (size > U32_MAX - 4095)
		return -EOVERFLOW;
	guard(mutex)(&session->lock);
	if (!q6apm_hostless_idle(session))
		return -EINPROGRESS;
	if (!READ_ONCE(session->ready))
		return -EBUSY;
	buffer = q6apm_hostless_map_blob(session, data, size);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);
	return q6apm_hostless_oob(session, buffer, APM_CMD_SET_CFG, destination);
}
EXPORT_SYMBOL_GPL(q6apm_hostless_set_payload);

int q6apm_hostless_register_cfg(struct q6apm_hostless *session, const void *data, size_t size)
{
	const struct apm_module_param_data *header = data;
	struct q6apm_hostless_buffer *buffer;
	int ret;

	if (!data || size < sizeof(*header) || size > U32_MAX - 4095)
		return -EINVAL;
	/* One persistent parameter per allocation, with optional final alignment. */
	if (!header->module_instance_id || !header->param_id ||
	    header->param_size > size - sizeof(*header) ||
	    size > ALIGN(sizeof(*header) + header->param_size, 8))
		return -EINVAL;
	guard(mutex)(&session->lock);
	if (!q6apm_hostless_idle(session))
		return -EINPROGRESS;
	if (!READ_ONCE(session->ready) || READ_ONCE(session->start_pending))
		return -EBUSY;
	buffer = q6apm_hostless_map_blob(session, data, size);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);
	buffer->header = *header;
	buffer->header.error_code = 0;
	WRITE_ONCE(buffer->register_pending, true);
	ret = q6apm_hostless_oob(session, buffer, APM_CMD_REGISTER_CFG, APM_MODULE_INSTANCE_ID);
	if (ret && ret != -ETIMEDOUT && ret != -EREMOTEIO && ret != -EPROTO)
		WRITE_ONCE(buffer->register_pending, false);
	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_hostless_register_cfg);

int q6apm_hostless_prepare(struct q6apm_hostless *session)
{
	int ret;

	guard(mutex)(&session->lock);
	if (!q6apm_hostless_idle(session))
		return -EINPROGRESS;
	if (!READ_ONCE(session->ready) || READ_ONCE(session->start_pending))
		return -EBUSY;
	ret = q6apm_hostless_graph_cmd(session, APM_CMD_GRAPH_PREPARE);
	if (!ret)
		WRITE_ONCE(session->prepared, true);
	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_hostless_prepare);

int q6apm_hostless_start(struct q6apm_hostless *session)
{
	guard(mutex)(&session->lock);
	if (!q6apm_hostless_idle(session))
		return -EINPROGRESS;
	if (!READ_ONCE(session->ready) || !READ_ONCE(session->prepared) ||
	    READ_ONCE(session->start_pending))
		return -EBUSY;
	WRITE_ONCE(session->start_pending, true);
	return q6apm_hostless_graph_cmd(session, APM_CMD_GRAPH_START);
}
EXPORT_SYMBOL_GPL(q6apm_hostless_start);

static int q6apm_hostless_stop_locked(struct q6apm_hostless *session)
{
	int ret;

	if (!q6apm_hostless_idle(session))
		return -EINPROGRESS;
	if (!READ_ONCE(session->start_pending))
		return 0;
	ret = q6apm_hostless_graph_cmd(session, APM_CMD_GRAPH_STOP);
	if (!ret)
		WRITE_ONCE(session->start_pending, false);
	return ret;
}

int q6apm_hostless_stop(struct q6apm_hostless *session)
{
	guard(mutex)(&session->lock);
	return q6apm_hostless_stop_locked(session);
}
EXPORT_SYMBOL_GPL(q6apm_hostless_stop);

int q6apm_hostless_destroy(struct q6apm_hostless *session)
{
	struct q6apm_hostless_buffer *buffer, *next;
	unsigned long flags;
	int ret;

	mutex_lock(&session->lock);
	ret = q6apm_hostless_stop_locked(session);
	if (ret)
		goto unlock;
	list_for_each_entry_safe(buffer, next, &session->buffers, node) {
		spin_lock_irqsave(&session->response_lock, flags);
		bool unknown = !buffer->handle && !buffer->map_failed;
		bool mapped = buffer->handle && !buffer->unmapped;

		spin_unlock_irqrestore(&session->response_lock, flags);
		if (unknown) {
			ret = -EUCLEAN;
			goto unlock;
		}
		if (READ_ONCE(buffer->register_pending)) {
			ret = q6apm_hostless_oob(session, buffer, APM_CMD_DEREGISTER_CFG,
						 APM_MODULE_INSTANCE_ID);
			if (ret)
				goto unlock;
			WRITE_ONCE(buffer->register_pending, false);
		}
		if (mapped) {
			ret = q6apm_hostless_unmap(session, buffer);
			if (ret)
				goto unlock;
		}
		spin_lock_irqsave(&session->response_lock, flags);
		if (session->pending_map == buffer)
			session->pending_map = NULL;
		if (session->pending_cal == buffer)
			session->pending_cal = NULL;
		spin_unlock_irqrestore(&session->response_lock, flags);
		list_del(&buffer->node);
		dma_free_coherent(session->dev, buffer->allocation_size,
				  buffer->area, buffer->dma_addr);
		kfree(buffer);
	}
	if (READ_ONCE(session->graph_pending)) {
		ret = q6apm_hostless_graph_cmd(session, APM_CMD_GRAPH_CLOSE);
		if (ret)
			goto unlock;
	}
	gpr_free_port_sync(session->port);
	mutex_unlock(&session->lock);
	put_device(session->dev);
	kfree(session);
	module_put(THIS_MODULE);
	return 0;
unlock:
	mutex_unlock(&session->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_hostless_destroy);
