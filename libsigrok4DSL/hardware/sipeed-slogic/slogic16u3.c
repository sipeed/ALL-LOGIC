/*
 * Sipeed SLogic driver for DSView / libsigrok4DSL
 *
 * USB protocol (Sipeed libsigrok slogic-dev):
 *   VID 0x359F  PID 0x3000 (SLogic8U2; 0x0300 on old firmware),
 *                 0x3031 (SLogic16U3), or
 *                 0x3032 (SLogic32U3)
 *   EP0 vendor control: 32-bit register read/write
 *     bRequest 0x00 REG_READ  / 0x01 REG_WRITE
 *     wValue   register address (byte), 4 bytes per transfer
 *   Registers:
 *     0x0004 R32_CTRL  bit0=RUN  bit1=RST
 *     0x000C R32_AUX   mailbox header + payload at 0x0010
 *   AUX commands: 1=channel mask  2=samplerate  3=vref  5=test mode
 *   Bulk IN: packed sample stream (no header).  Combo 8 uses EP 0x81;
 *   both U3 models use EP 0x82.
 *
 * DSView expects LA_CROSS_DATA (64-sample channel planes).
 */

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "slogic16u3.h"
#include "slogic/slogic.h"

#undef LOG_PREFIX
#define LOG_PREFIX "slogic16u3: "


/* Bulk transfer sizing for the convert staging buffer. */
#define SLOGIC_TRANSFER_ALIGN    (32 * 1024)

/* Upper bound on helper threads used to convert the wire format. */
#define SLOGIC_MAX_CONV_THREADS 8
#define SLOGIC_DEFAULT_SAMPLES   SR_Mn(1)
#define SLOGIC_HW_DEPTH          SR_Mn(128)
#define SLOGIC_MAX_PHYS_CH       32

#define SLOGIC_CROSS_SCALE       64
#define SLOGIC_CROSS_PLANE_BYTES 8

/* The application is often built as a debug binary. Keep the hot bit
 * transpose path optimized in that configuration as well; otherwise the
 * USB producer can outrun the converter at 800 MB/s. */
#if defined(__GNUC__) && !defined(__clang__)
#define SLOGIC_HOT __attribute__((optimize("O3"), hot))
#define SLOGIC_ALWAYS_INLINE __attribute__((always_inline))
#else
#define SLOGIC_HOT
#define SLOGIC_ALWAYS_INLINE
#endif

#define SLOGIC_VTH_MIN           0.0
#define SLOGIC_VTH_MAX           5.0
#define SLOGIC_VTH_DEFAULT       1.7

enum {
	SLOGIC_ST_IDLE = 0,
	SLOGIC_ST_START,
	SLOGIC_ST_DATA,
	SLOGIC_ST_STOP,
	SLOGIC_ST_FINISH,
};

enum {
	/* Keep the historical 16U3 IDs (0=16, 1=8, 2=4). */
	SLOGIC_CHMODE_16 = 0,
	SLOGIC_CHMODE_8 = 1,
	SLOGIC_CHMODE_4 = 2,
};




static const struct sr_list_item filter_list[] = {
	{ SR_FILTER_NONE, "None" },
	{ SR_FILTER_1T, "1 Sample Clock" },
	{ -1, NULL },
};

static const struct sr_list_item opmode_list[] = {
	{ LO_OP_STREAM, "Stream Mode" },
	{ -1, NULL },
};

static const struct sr_list_item channel_mode_16u3_list[] = {
	{ SLOGIC_CHMODE_16, "Use Channels 0~15 (Max 200MHz)" },
	{ SLOGIC_CHMODE_8, "Use Channels 0~7 (Max 400MHz)" },
	{ SLOGIC_CHMODE_4, "Use Channels 0~3 (Max 800MHz)" },
	{ -1, NULL },
};

static const struct sr_list_item channel_mode_combo8_list[] = {
	{ 0, "Use Channels 0~7 (Max 40MHz)" },
	{ 1, "Use Channels 0~3 (Max 80MHz)" },
	{ 2, "Use Channels 0~1 (Max 160MHz)" },
	{ -1, NULL },
};

static const struct sr_list_item channel_mode_32u3_list[] = {
	{ 0, "Use Channels 0~31 (Max 200MHz)" },
	{ 1, "Use Channels 0~15 (Max 400MHz)" },
	{ 2, "Use Channels 0~7 (Max 800MHz)" },
	{ 3, "Use Channels 0~3 (Max 1400MHz)" },
	{ -1, NULL },
};

static const char *pattern_modes[] = {
	"Normal",
	"USB connection test",
	"Emulation",
};

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_PATTERN_MODE,
	SR_CONF_FILTER,
	SR_CONF_VTH,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions[] = {
	SR_CONF_MAX_HEIGHT,
	SR_CONF_OPERATION_MODE,
	SR_CONF_PATTERN_MODE,
	SR_CONF_CHANNEL_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
	SR_CONF_VTH,
	SR_CONF_FILTER,
};

static const char *probe_names[] = {
	"0", "1", "2", "3", "4", "5", "6", "7",
	"8", "9", "10", "11", "12", "13", "14", "15",
	"16", "17", "18", "19", "20", "21", "22", "23",
	"24", "25", "26", "27", "28", "29", "30", "31",
	NULL,
};


struct slogic_context {
	const struct slogic_model *model;
	struct libusb_device *usb_dev;
	struct libusb_device_handle *devhdl;
	struct sr_context *sr_ctx;
	const struct sr_dev_inst *sdi;

	int channel_count;
	int ch_mode;
	int filter;
	int max_height;
	int op_mode;
	int is_loop;
	int pattern_mode;
	double vth;

	uint64_t samplerate;
	uint64_t limit_samples;
	uint64_t num_samples;
	uint64_t num_bytes;

	enum libusb_speed usb_speed;

	uint32_t filt_prev_in;
	uint32_t filt_prev_out;
	int filt_have_prev;

	volatile gint status;
	int source_added;
	int end_sent;

	/* libslogic core: owns the handle, event thread, ring and accounting. */
	slogic_dev *sdev;
	slogic_capture *cap;

	uint32_t *raw_pending;
	int raw_pending_len;
	int raw_pending_cap;

	uint8_t stream_res[4];
	int stream_res_len;

	uint8_t *cross_buf;
	int cross_cap;
	int cross_len;
	uint8_t *fast_pending;
	size_t fast_pending_len;
	size_t fast_pending_cap;

	uint8_t en_bits[SLOGIC_MAX_PHYS_CH];
	int en_count;

	/* Parallel LA_CROSS_DATA conversion.  The host has to bit-transpose the
	 * interleaved wire format into 64-sample channel planes, which costs more
	 * CPU time than the USB link needs to deliver the data.  A private pool
	 * splits each USB packet into contiguous group ranges so the output stays
	 * byte-for-byte identical to the serial conversion. */
	GThreadPool *conv_pool;
	int conv_threads;
};

SR_PRIV struct sr_dev_driver slogic16u3_driver_info;
static struct sr_dev_driver *di = &slogic16u3_driver_info;

/* -------------------- USB / speed -------------------- */

static const struct sr_list_item *slogic_channel_mode_list(
		const struct slogic_model *model)
{
	if (!model)
		return channel_mode_16u3_list;
	if (model->proto == SLOGIC_PROTO_COMBO8)
		return channel_mode_combo8_list;
	if (model->physical_channels == 32)
		return channel_mode_32u3_list;
	return channel_mode_16u3_list;
}

static int slogic_mode_channels(const struct slogic_context *devc, int mode)
{
	if (!devc || !devc->model || mode < 0 ||
		(size_t)mode >= devc->model->limit_count)
		return 16;
	return (devc->model->physical_channels >> (mode));
}

/* Link-aware ceiling for the current channel mode (USB2 cap in the core). */
static slogic_link al_link(const struct slogic_context *devc)
{
	return (devc && devc->usb_dev) ? slogic_usb_link(devc->usb_dev)
				       : SLOGIC_LINK_UNKNOWN;
}

static const struct slogic_model *al_model(const struct slogic_context *devc)
{
	return (devc && devc->model) ? devc->model
				     : slogic_model_for_pid(SLOGIC_PID_16U3);
}

static int al_nch(const struct slogic_context *devc)
{
	const struct slogic_model *model = al_model(devc);
	int mode = devc ? devc->ch_mode : 0;

	if (mode < 0 || (size_t)mode >= model->limit_count)
		mode = 0;
	return model->physical_channels >> mode;
}

static uint64_t al_link_max_rate(const struct slogic_context *devc)
{
	return slogic_link_max_rate(al_model(devc), al_nch(devc), al_link(devc));
}

/* "(10G) SLogic32 U3", "(480M) SLogic8 U2", "(old) SLogic8 U2": the same name
 * in every SLogic app (slogic_display_name); the prefix goes first so it
 * survives when the UI elides a long device name. */
static void slogic_apply_model_name(struct slogic_context *devc)
{
	char name[64];
	struct sr_dev_inst *sdi = (struct sr_dev_inst *)devc->sdi;

	g_free(sdi->name);
	sdi->name = g_strdup(slogic_display_name(devc->model, al_link(devc),
						 name, sizeof(name)));
}

static void slogic_update_usb_speed(struct slogic_context *devc)
{
	enum libusb_speed sp = LIBUSB_SPEED_UNKNOWN;

	if (!devc || !devc->usb_dev)
		return;
	sp = libusb_get_device_speed(devc->usb_dev);
	devc->usb_speed = sp;
	if (devc->sdi)
		slogic_apply_model_name(devc);
}

static uint64_t al_pick_rate(const struct slogic_context *devc, uint64_t want)
{
	return slogic_pick_rate(al_model(devc), al_nch(devc), al_link(devc), want, 0);
}

static void slogic_map_samplerate(struct slogic_context *devc, uint64_t want)
{
	devc->samplerate = al_pick_rate(devc, want);
}

static void slogic_build_rate_list(const struct slogic_context *devc,
				   uint64_t *out, int *out_n)
{
	/* The caller must size `out` for the whole list (32 entries max) plus
	 * the 0 terminator written below. DSView lists rates highest first. */
	uint64_t asc[32];
	size_t n = slogic_rate_list(al_model(devc), al_nch(devc), al_link(devc),
				    asc, 32);
	size_t i;

	for (i = 0; i < n; i++)
		out[i] = asc[n - 1 - i];
	out[n] = 0;
	*out_n = (int)n;
}

/* -------------------- libslogic core glue --------------------
 * The device, the transfer ring, stall recovery, exact sample accounting and
 * the throughput report live in the vendored slogic/ core. This driver maps
 * the DSView config onto it and turns the delivered samples (1 byte per
 * sample below 8 channels, else channels/8) into LA_CROSS_DATA. */
static void al_config(struct slogic_context *devc, slogic_config *c)
{
	*c = (slogic_config){
		.channel_count = devc->channel_count,
		.samplerate_hz = devc->samplerate,
		.threshold_v = devc->vth,
		.pattern_mode = devc->pattern_mode,
	};
}

static void al_log(void *user, int level, const char *msg)
{
	(void)user;
	switch (level) {
	case SLOGIC_LOG_ERR: sr_err("%s", msg); break;
	case SLOGIC_LOG_WARN: sr_warn("%s", msg); break;
	case SLOGIC_LOG_INFO: sr_info("%s", msg); break;
	case SLOGIC_LOG_DBG: sr_dbg("%s", msg); break;
	default: break;
	}
}

/* -------------------- probes / convert -------------------- */

static void slogic_apply_channel_mode(struct sr_dev_inst *sdi)
{
	struct slogic_context *devc = sdi->priv;
	GSList *l;
	int n = slogic_mode_channels(devc, devc->ch_mode);

	devc->channel_count = n;
	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC)
			continue;
		ch->enabled = (ch->index < n) ? TRUE : FALSE;
	}
}

static void slogic_refresh_enabled_bits(struct slogic_context *devc)
{
	GSList *l;
	int n = 0;
	int max_bit = devc->channel_count;

	for (l = devc->sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (!ch->enabled)
			continue;
		if (ch->index >= max_bit)
			continue;
		if (n >= SLOGIC_MAX_PHYS_CH)
			break;
		devc->en_bits[n++] = (uint8_t)ch->index;
	}
	if (n == 0) {
		int i;
		for (i = 0; i < max_bit; i++)
			devc->en_bits[i] = (uint8_t)i;
		n = max_bit;
	}
	devc->en_count = n;
}

static int slogic_ensure_cross_cap(struct slogic_context *devc, int need)
{
	uint8_t *p;

	if (need <= devc->cross_cap)
		return SR_OK;
	need = (need + 4095) & ~4095;
	p = g_try_realloc(devc->cross_buf, need);
	if (!p)
		return SR_ERR_MALLOC;
	devc->cross_buf = p;
	devc->cross_cap = need;
	return SR_OK;
}

static int slogic_ensure_pending_cap(struct slogic_context *devc, int need)
{
	uint32_t *p;

	if (need <= devc->raw_pending_cap)
		return SR_OK;
	need = (need + 1023) & ~1023;
	p = g_try_realloc(devc->raw_pending, (gsize)need * sizeof(uint32_t));
	if (!p)
		return SR_ERR_MALLOC;
	devc->raw_pending = p;
	devc->raw_pending_cap = need;
	return SR_OK;
}

static uint32_t slogic_apply_1t_filter(struct slogic_context *devc, uint32_t in)
{
	uint32_t outv;

	if (devc->filter != SR_FILTER_1T)
		return in;
	if (!devc->filt_have_prev) {
		devc->filt_prev_in = in;
		devc->filt_prev_out = in;
		devc->filt_have_prev = 1;
		return in;
	}
	outv = (in == devc->filt_prev_in) ? in : devc->filt_prev_out;
	devc->filt_prev_in = in;
	devc->filt_prev_out = outv;
	return outv;
}

static int slogic_push_sample(struct slogic_context *devc, uint32_t s)
{
	if (slogic_ensure_pending_cap(devc, devc->raw_pending_len + 1) != SR_OK)
		return SR_ERR_MALLOC;
	devc->raw_pending[devc->raw_pending_len++] =
		slogic_apply_1t_filter(devc, s);
	return SR_OK;
}

/*
 * Collect delivered samples as parallel words (bit i = Di). Below eight
 * channels the core delivers one sample per byte; eight, sixteen and
 * thirty-two channel modes use one, two and four little-endian bytes.
 */
static int slogic_unpack_append(struct slogic_context *devc,
				const uint8_t *raw, int raw_len)
{
	int nch = devc->channel_count;
	const uint8_t *p = raw;
	int left = raw_len;
	int bytes_per_sample;
	uint32_t mask;

	if (!raw || raw_len <= 0)
		return SR_OK;

	if (nch < 1)
		return SR_ERR_ARG;

	if (nch < 8) {
		/* The core already unpacked these: one sample per byte. */
		mask = (UINT32_C(1) << nch) - 1u;
		while (left-- > 0) {
			if (slogic_push_sample(devc, *p++ & mask) != SR_OK)
				return SR_ERR_MALLOC;
		}
		return SR_OK;
	}

	bytes_per_sample = (nch + 7) / 8;
	if (bytes_per_sample > (int)sizeof(devc->stream_res))
		return SR_ERR_ARG;
	if (devc->stream_res_len >= bytes_per_sample)
		devc->stream_res_len = 0;
	/* Complete a sample split over two USB transfers. */
	while (devc->stream_res_len > 0 &&
	       devc->stream_res_len < bytes_per_sample && left > 0) {
		devc->stream_res[devc->stream_res_len++] = *p++;
		left--;
		if (devc->stream_res_len == bytes_per_sample) {
			uint32_t s = 0;
			int i;
			for (i = 0; i < bytes_per_sample; i++)
				s |= (uint32_t)devc->stream_res[i] << (8 * i);
			if (slogic_push_sample(devc, s) != SR_OK)
				return SR_ERR_MALLOC;
			devc->stream_res_len = 0;
		}
	}
	while (left >= bytes_per_sample) {
		uint32_t s = 0;
		int i;
		for (i = 0; i < bytes_per_sample; i++)
			s |= (uint32_t)p[i] << (8 * i);
		if (slogic_push_sample(devc, s) != SR_OK)
			return SR_ERR_MALLOC;
		p += bytes_per_sample;
		left -= bytes_per_sample;
	}
	if (left > 0) {
		memcpy(devc->stream_res, p, (size_t)left);
		devc->stream_res_len = left;
	}
	return SR_OK;
}

static int slogic_ensure_fast_cap(struct slogic_context *devc, size_t need)
{
	uint8_t *p;

	if (need <= devc->fast_pending_cap)
		return SR_OK;
	/* Keep the pending area bounded to a few USB packets.  Normally only
	 * the final incomplete 64-sample group remains here. */
	if (need < 4096)
		need = 4096;
	need = (need + SLOGIC_TRANSFER_ALIGN - 1) &
		~(size_t)(SLOGIC_TRANSFER_ALIGN - 1);
	p = g_try_realloc(devc->fast_pending, need);
	if (!p)
		return SR_ERR_MALLOC;
	devc->fast_pending = p;
	devc->fast_pending_cap = need;
	return SR_OK;
}

/* Transpose eight bytes (eight consecutive samples of one byte lane). */
static inline SLOGIC_HOT SLOGIC_ALWAYS_INLINE uint64_t
slogic_transpose8x8(const uint8_t *in)
{
	uint64_t x, t;

	memcpy(&x, in, sizeof(x));
	t = (x ^ (x >> 7)) & UINT64_C(0x00aa00aa00aa00aa);
	x ^= t ^ (t << 7);
	t = (x ^ (x >> 14)) & UINT64_C(0x0000cccc0000cccc);
	x ^= t ^ (t << 14);
	t = (x ^ (x >> 28)) & UINT64_C(0x00000000f0f0f0f0);
	x ^= t ^ (t << 28);
	return x;
}

static void slogic_forward_cross(struct slogic_context *devc, int out_len)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	if (out_len <= 0)
		return;
	memset(&packet, 0, sizeof(packet));
	memset(&logic, 0, sizeof(logic));
	packet.type = SR_DF_LOGIC;
	packet.status = SR_PKT_OK;
	packet.payload = &logic;
	logic.length = (uint64_t)out_len;
	logic.format = LA_CROSS_DATA;
	logic.unitsize = (uint16_t)((devc->en_count + 7) / 8);
	if (logic.unitsize == 0)
		logic.unitsize = 1;
	logic.data = devc->cross_buf;
	ds_data_forward(devc->sdi, &packet);
	devc->num_bytes += (uint64_t)out_len;
}

/* Convert `groups` complete 64-sample groups starting at src into the channel
 * planes at dst.  Groups are independent and each one writes exactly
 * SLOGIC_CROSS_PLANE_BYTES * en bytes at a fixed offset, so a group range can
 * be converted on its own (see the worker pool below). */
static SLOGIC_HOT void slogic_convert_groups(const struct slogic_context *devc,
					     const uint8_t *src, uint8_t *dst,
					     size_t groups)
{
	const int bytes_per_sample = (devc->channel_count + 7) / 8;
	const size_t group_raw = (size_t)SLOGIC_CROSS_SCALE * bytes_per_sample;
	const size_t cross_group = (size_t)SLOGIC_CROSS_PLANE_BYTES *
				   (size_t)devc->en_count;
	size_t g, block, c;

	/* Channels that are not enabled leave their plane byte untouched. */
	memset(dst, 0, groups * cross_group);
	for (g = 0; g < groups; g++) {
		const uint8_t *gsrc = src + g * group_raw;
		uint8_t *gdst = dst + g * cross_group;

		for (block = 0; block < SLOGIC_CROSS_SCALE / 8; block++) {
			const uint8_t *samples =
				gsrc + block * (size_t)bytes_per_sample * 8;
			uint64_t transposed[4] = { 0, 0, 0, 0 };
			uint8_t lane_samples[8];
			int lane;

			for (lane = 0; lane < bytes_per_sample; lane++) {
				int sample;
				for (sample = 0; sample < 8; sample++)
					lane_samples[sample] =
						samples[sample * bytes_per_sample + lane];
				transposed[lane] = slogic_transpose8x8(
					lane_samples);
			}
			for (c = 0; c < (size_t)devc->en_count; c++) {
				int bit = devc->en_bits[c];
				int lane_bit;

				lane = bit / 8;
				if (lane >= bytes_per_sample)
					continue;
				lane_bit = bit % 8;
				gdst[c * SLOGIC_CROSS_PLANE_BYTES + block] =
					(uint8_t)(transposed[lane] >>
						  (lane_bit * 8));
			}
		}
	}
}

struct slogic_conv_batch {
	GMutex lock;
	GCond cond;
	gint left;
};

struct slogic_conv_chunk {
	const struct slogic_context *devc;
	const uint8_t *src;
	uint8_t *dst;
	size_t groups;
	struct slogic_conv_batch *batch;
};

static void slogic_conv_chunk_run(struct slogic_conv_chunk *chunk)
{
	slogic_convert_groups(chunk->devc, chunk->src, chunk->dst,
			      chunk->groups);
	g_mutex_lock(&chunk->batch->lock);
	if (--chunk->batch->left == 0)
		g_cond_signal(&chunk->batch->cond);
	g_mutex_unlock(&chunk->batch->lock);
}

static void slogic_conv_chunk_worker(gpointer data, gpointer user_data)
{
	(void)user_data;
	slogic_conv_chunk_run((struct slogic_conv_chunk *)data);
}

static SLOGIC_HOT int slogic_convert_fast(struct slogic_context *devc,
			       const uint8_t *raw, int raw_len)
{
	const int nch = devc->channel_count;
	const int en = devc->en_count;
	const int bytes_per_sample = (nch + 7) / 8;
	const size_t group_raw = (size_t)SLOGIC_CROSS_SCALE * bytes_per_sample;
	const int cross_group = SLOGIC_CROSS_PLANE_BYTES * en;
	const uint8_t *src;
	size_t groups, consumed;
	size_t raw_groups = 0;
	uint8_t *out;
	int out_len;

	if (en < 1 || raw_len <= 0 || bytes_per_sample < 1 || bytes_per_sample > 4)
		return SR_OK;
	/* Nothing staged and a whole number of 64-sample groups in this
	 * transfer (the normal case): convert straight out of the USB buffer.
	 * Staging a copy of it would add a full extra pass over 800 MB. */
	if (devc->fast_pending_len == 0 &&
	    ((size_t)raw_len % group_raw) == 0) {
		src = raw;
		groups = (size_t)raw_len / group_raw;
		raw_groups = groups;
	} else {
		if (slogic_ensure_fast_cap(devc, devc->fast_pending_len +
				(size_t)raw_len) != SR_OK)
			return SR_ERR_MALLOC;
		memcpy(devc->fast_pending + devc->fast_pending_len, raw,
			(size_t)raw_len);
		devc->fast_pending_len += (size_t)raw_len;
		src = devc->fast_pending;
		groups = devc->fast_pending_len / group_raw;
	}
	if (devc->limit_samples && !devc->is_loop) {
		uint64_t room = devc->num_samples < devc->limit_samples
			? devc->limit_samples - devc->num_samples : 0;
		size_t max_groups = (size_t)(room / SLOGIC_CROSS_SCALE);
		if (groups > max_groups)
			groups = max_groups;
	}
	if (groups == 0)
		return SR_OK;

	out_len = (int)(groups * (size_t)cross_group);
	if (out_len <= 0 || slogic_ensure_cross_cap(devc, out_len) != SR_OK)
		return SR_ERR_MALLOC;
	out = devc->cross_buf;
	if (devc->conv_pool && devc->conv_threads > 1 &&
	    groups >= (size_t)devc->conv_threads * 2) {
		const size_t n = (size_t)devc->conv_threads;
		struct slogic_conv_batch batch;
		struct slogic_conv_chunk *chunks =
			g_new0(struct slogic_conv_chunk, n);
		size_t base = 0;
		size_t i;

		g_mutex_init(&batch.lock);
		g_cond_init(&batch.cond);
		batch.left = (gint)n;
		for (i = 0; i < n; i++) {
			size_t count = groups / n + (i < groups % n ? 1 : 0);

			chunks[i].devc = devc;
			chunks[i].src = src + base * group_raw;
			chunks[i].dst = out + base * (size_t)cross_group;
			chunks[i].groups = count;
			chunks[i].batch = &batch;
			base += count;
			/* Fall back to running the chunk here if the pool refuses
			 * the job, otherwise the batch would never complete. */
			if (!g_thread_pool_push(devc->conv_pool, &chunks[i], NULL))
				slogic_conv_chunk_run(&chunks[i]);
		}
		g_mutex_lock(&batch.lock);
		while (batch.left > 0)
			g_cond_wait(&batch.cond, &batch.lock);
		g_mutex_unlock(&batch.lock);
		g_cond_clear(&batch.cond);
		g_mutex_clear(&batch.lock);
		g_free(chunks);
	} else {
		slogic_convert_groups(devc, src, out, groups);
	}
	slogic_forward_cross(devc, out_len);
	devc->num_samples += (uint64_t)groups * SLOGIC_CROSS_SCALE;
	consumed = groups * group_raw;
	if (raw_groups != 0) {
		/* Fast path: keep only the clamped tail for the next transfer. */
		size_t left = (size_t)raw_len - consumed;

		devc->fast_pending_len = 0;
		if (left > 0) {
			if (slogic_ensure_fast_cap(devc, left) != SR_OK)
				return SR_ERR_MALLOC;
			memcpy(devc->fast_pending, raw + consumed, left);
			devc->fast_pending_len = left;
		}
	} else {
		if (consumed < devc->fast_pending_len)
			memmove(devc->fast_pending, devc->fast_pending + consumed,
				devc->fast_pending_len - consumed);
		devc->fast_pending_len -= consumed;
	}
	return SR_OK;
}

static int slogic_convert_slow(struct slogic_context *devc,
			       const uint8_t *raw, int raw_len)
{
	const int en = devc->en_count;
	const int cross_group = SLOGIC_CROSS_PLANE_BYTES * en;
	int groups, g, c, i, consumed, left;
	uint8_t *out;
	int out_len;
	uint64_t samples_out;

	if (en < 1 || raw_len <= 0)
		return SR_OK;
	if (slogic_unpack_append(devc, raw, raw_len) != SR_OK)
		return SR_ERR_MALLOC;

	groups = devc->raw_pending_len / SLOGIC_CROSS_SCALE;
	if (groups <= 0)
		return SR_OK;

	if (devc->limit_samples && !devc->is_loop) {
		uint64_t room = (devc->num_samples < devc->limit_samples)
			? (devc->limit_samples - devc->num_samples) : 0;
		int max_groups = (int)(room / SLOGIC_CROSS_SCALE);
		if (max_groups <= 0) {
			devc->num_samples = devc->limit_samples;
			return SR_OK;
		}
		if (groups > max_groups)
			groups = max_groups;
	}

	out_len = groups * cross_group;
	if (slogic_ensure_cross_cap(devc, out_len) != SR_OK)
		return SR_ERR_MALLOC;

	out = devc->cross_buf;
	for (g = 0; g < groups; g++) {
		const uint32_t *blk = devc->raw_pending + g * SLOGIC_CROSS_SCALE;
		for (c = 0; c < en; c++) {
			uint64_t plane = 0;
			uint8_t bit = devc->en_bits[c];
			for (i = 0; i < SLOGIC_CROSS_SCALE; i++) {
				if ((blk[i] >> bit) & 1u)
					plane |= (1ULL << i);
			}
			out[0] = (uint8_t)(plane);
			out[1] = (uint8_t)(plane >> 8);
			out[2] = (uint8_t)(plane >> 16);
			out[3] = (uint8_t)(plane >> 24);
			out[4] = (uint8_t)(plane >> 32);
			out[5] = (uint8_t)(plane >> 40);
			out[6] = (uint8_t)(plane >> 48);
			out[7] = (uint8_t)(plane >> 56);
			out += SLOGIC_CROSS_PLANE_BYTES;
		}
	}

	samples_out = (uint64_t)groups * SLOGIC_CROSS_SCALE;

	slogic_forward_cross(devc, out_len);

	devc->num_samples += samples_out;

	consumed = groups * SLOGIC_CROSS_SCALE;
	left = devc->raw_pending_len - consumed;
	if (left > 0)
		memmove(devc->raw_pending, devc->raw_pending + consumed,
			(size_t)left * sizeof(uint32_t));
	devc->raw_pending_len = left;
	return SR_OK;
}

static int slogic_convert_push(struct slogic_context *devc,
			       const uint8_t *raw, int raw_len)
{
	/* Every mode arrives as whole bytes per sample now, so the fast
	 * transpose covers them all; only the 1T filter needs the slow path. */
	if (devc->filter == SR_FILTER_NONE)
		return slogic_convert_fast(devc, raw, raw_len);
	return slogic_convert_slow(devc, raw, raw_len);
}

/* -------------------- capture -------------------- */

static int al_sink_data(void *user, const uint8_t *samples, size_t n,
			unsigned unitsize)
{
	struct slogic_context *devc = user;

	if (slogic_convert_push(devc, samples, (int)(n * unitsize)) != SR_OK) {
		sr_err("format convert failed");
		return 1; /* ends the capture */
	}
	return 0;
}

/* Trigger point, called by the core between the pre- and post-trigger
 * samples: tell DSView where it is in the delivered stream. */
static void al_sink_trigger(void *user, uint64_t index)
{
	struct slogic_context *devc = user;
	struct sr_datafeed_packet packet;
	struct ds_trigger_pos pos;

	memset(&pos, 0, sizeof(pos));
	pos.status = 1;
	pos.real_pos = (uint32_t)index;
	memset(&packet, 0, sizeof(packet));
	packet.type = SR_DF_TRIGGER;
	packet.status = SR_PKT_OK;
	packet.payload = &pos;
	ds_data_forward(devc->sdi, &packet);
}

/* DSView simple trigger (stage TriggerStages): per channel 'X' don't care,
 * '0'/'1' level, 'R'/'F' edge, 'C' either edge -> core single-stage trigger.
 * Pre-trigger depth follows the trigger position slider (percent). */
static void al_map_trigger(struct slogic_context *devc, slogic_capture_config *c)
{
	int i;

	if (!trigger || !trigger->trigger_en)
		return;
	for (i = 0; i < devc->channel_count && i < SLOGIC_MAX_CHANNELS; i++) {
		switch (trigger->trigger0[TriggerStages][i]) {
		case '0': c->trigger.match[i] = SLOGIC_TRIG_ZERO; break;
		case '1': c->trigger.match[i] = SLOGIC_TRIG_ONE; break;
		case 'R': c->trigger.match[i] = SLOGIC_TRIG_RISING; break;
		case 'F': c->trigger.match[i] = SLOGIC_TRIG_FALLING; break;
		case 'C': c->trigger.match[i] = SLOGIC_TRIG_EDGE; break;
		default: break;
		}
	}
	if (!slogic_trigger_spec_empty(&c->trigger) && c->limit_samples)
		c->pretrigger_samples =
			c->limit_samples * ds_trigger_get_pos() / 100;
}

static void slogic_finish_session(struct slogic_context *devc)
{
	struct sr_datafeed_packet packet;

	if (devc->end_sent)
		return;
	devc->raw_pending_len = 0;
	devc->fast_pending_len = 0;

	/* The core logged its "slogic-report v1" line (rate, cold start). */
	sr_info("send SR_DF_END, samples=%" PRIu64, devc->num_samples);
	memset(&packet, 0, sizeof(packet));
	packet.type = SR_DF_END;
	packet.status = SR_PKT_OK;
	ds_data_forward(devc->sdi, &packet);
	devc->end_sent = 1;
	devc->status = SLOGIC_ST_FINISH;
	if (devc->source_added) {
		sr_session_source_remove(-1);
		devc->source_added = 0;
	}
}

static int receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
	struct slogic_context *devc = sdi->priv;

	(void)fd;
	(void)revents;

	/* Up to 8 buffers per tick (waiting at most 2 ms for one): enough to
	 * keep up with 800 MB/s at the session loop's tick rate while still
	 * returning to the UI loop between batches. */
	if (devc->cap &&
	    slogic_capture_poll(devc->cap, 8, 2) == SLOGIC_CAP_RUNNING)
		return TRUE;
	slogic_capture_destroy(devc->cap);
	devc->cap = NULL;
	slogic_finish_session(devc);
	return FALSE;
}

/* -------------------- driver API -------------------- */

static int hw_init(struct sr_context *sr_ctx)
{
	return std_hw_init(sr_ctx, di, LOG_PREFIX);
}

static int hw_cleanup(void)
{
	safe_free(di->priv);
	return SR_OK;
}

static void setup_channels(struct sr_dev_inst *sdi, int count)
{
	int i;
	char name[16];

	if (count > SLOGIC_MAX_PHYS_CH)
		count = SLOGIC_MAX_PHYS_CH;

	while (sdi->channels) {
		struct sr_channel *ch = sdi->channels->data;
		sdi->channels = g_slist_delete_link(sdi->channels, sdi->channels);
		g_free(ch->name);
		g_free(ch);
	}
	for (i = 0; i < count; i++) {
		const char *n = probe_names[i] ? probe_names[i] : name;
		if (!probe_names[i])
			snprintf(name, sizeof(name), "%d", i);
		sdi->channels = g_slist_append(
			sdi->channels,
			sr_channel_new(i, SR_CHANNEL_LOGIC, TRUE, n));
	}
}

static GSList *hw_scan(GSList *options)
{
	struct drv_context *drvc;
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	struct sr_dev_inst *sdi;
	struct slogic_context *devc;
	GSList *devices = NULL;
	int i;

	(void)options;
	drvc = di->priv;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		uint8_t bus, address;
		struct sr_usb_dev_inst *usb_info;
		const struct slogic_model *model;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		if (des.idVendor != SLOGIC_VID)
			continue;
		model = slogic_model_for_pid(des.idProduct);
		if (!model)
			continue;
		if (sr_usb_device_is_exists(devlist[i]))
			continue;

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc)
			break;

		bus = libusb_get_bus_number(devlist[i]);
		address = libusb_get_device_address(devlist[i]);

		devc->usb_dev = libusb_ref_device(devlist[i]);
		devc->sr_ctx = drvc->sr_ctx;
		devc->model = model;
		devc->ch_mode = 0;
		devc->channel_count = (model->physical_channels >> (0));
		devc->filter = SR_FILTER_NONE;
		devc->limit_samples = SLOGIC_DEFAULT_SAMPLES;
		devc->max_height = 1;
		devc->vth = SLOGIC_VTH_DEFAULT;
		devc->op_mode = LO_OP_STREAM;
		devc->is_loop = 0;
		devc->pattern_mode = 0;
		devc->usb_speed = LIBUSB_SPEED_UNKNOWN;

		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE,
				      "Sipeed", model->name, NULL);
		if (!sdi) {
			libusb_unref_device(devc->usb_dev);
			g_free(devc);
			break;
		}
		sdi->driver = di;
		sdi->dev_type = DEV_TYPE_USB;
		sdi->priv = devc;
		devc->sdi = sdi;
		sdi->handle = (ds_device_handle)devc->usb_dev;
		slogic_update_usb_speed(devc);
		slogic_map_samplerate(devc, slogic_max_rate(model, model->physical_channels >> (0)));

		usb_info = sr_usb_dev_inst_new(bus, address);
		if (usb_info) {
			usb_info->usb_dev = devc->usb_dev;
			sdi->conn = usb_info;
		} else {
			sdi->conn = NULL;
		}
		setup_channels(sdi, model->physical_channels);
		slogic_apply_channel_mode(sdi);

		sr_info("Found %s %04x:%04x bus=%u addr=%u speed=%s name=%s",
			model->name,
			des.idVendor, des.idProduct, bus, address,
			slogic_link_name(al_link(devc)),
			sdi->name ? sdi->name : "?");
		devices = g_slist_append(devices, sdi);
	}

	libusb_free_device_list(devlist, 0);
	return devices;
}

static const GSList *hw_dev_mode_list(const struct sr_dev_inst *sdi)
{
	static GSList *l;

	(void)sdi;
	if (!l)
		l = g_slist_append(NULL, (gpointer)&sr_mode_list[0]);
	return l;
}

static int hw_dev_open(struct sr_dev_inst *sdi)
{
	struct slogic_context *devc = sdi->priv;
	int r;

	if (sdi->status == SR_ST_ACTIVE)
		return SR_OK;

	r = slogic_dev_open(devc->sr_ctx->libusb_ctx, devc->usb_dev, al_log, devc,
			    &devc->sdev);
	if (r != SLOGIC_OK) {
		ds_set_last_error(r == SLOGIC_ERR_BUSY ? SR_ERR_DEVICE_IS_EXCLUSIVE
						       : SR_ERR_DEVICE_NO_DRIVER);
		return SR_ERR;
	}
	devc->devhdl = slogic_dev_handle(devc->sdev);

	/*
	 * The wire format has to be transposed into LA_CROSS_DATA on the host,
	 * which costs more than the USB link needs to deliver the samples.  Use
	 * a few helpers so a 32ch@200MHz stream stays ahead of real time.
	 *
	 * Half of the cores are left to the rest of the process (UI repaint,
	 * decode, other applications); the pool only has to convert faster than
	 * the ~790 MB/s the analyzer delivers, and measurements showed the
	 * remaining serial snapshot append - not the transpose - dominates the
	 * tail once this many helpers are running.
	 */
	if (!devc->conv_pool) {
		int cores = g_get_num_processors();
		int threads = cores / 2;

		if (threads < 1)
			threads = 1;
		if (threads > SLOGIC_MAX_CONV_THREADS)
			threads = SLOGIC_MAX_CONV_THREADS;
		devc->conv_pool = g_thread_pool_new(slogic_conv_chunk_worker,
						    NULL, threads, TRUE, NULL);
		devc->conv_threads = devc->conv_pool ? threads : 1;
	}

	devc->sdi = sdi;
	slogic_update_usb_speed(devc);
	slogic_map_samplerate(devc, devc->samplerate ? devc->samplerate
						     : SR_MHZ(100));

	sdi->status = SR_ST_ACTIVE;
	sr_info("open: name=\"%s\" speed=%s",
		sdi->name ? sdi->name : "?",
		slogic_link_name(al_link(devc)));
	return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct slogic_context *devc;

	if (!sdi || !sdi->priv)
		return SR_ERR;

	devc = sdi->priv;
	if (devc->source_added) {
		sr_session_source_remove(-1);
		devc->source_added = 0;
	}
	if (devc->sdev) {
		slogic_dev_close(devc->sdev); /* destroys a live capture first */
		devc->sdev = NULL;
		devc->cap = NULL;
		devc->devhdl = NULL;
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int hw_dev_destroy(struct sr_dev_inst *sdi)
{
	struct slogic_context *devc;

	if (!sdi)
		return SR_ERR;

	hw_dev_close(sdi);
	devc = sdi->priv;
	if (devc) {
		if (devc->conv_pool) {
			g_thread_pool_free(devc->conv_pool, FALSE, TRUE);
			devc->conv_pool = NULL;
			devc->conv_threads = 1;
		}
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		g_free(devc->raw_pending);
		g_free(devc->cross_buf);
		g_free(devc->fast_pending);
		g_free(devc);
		sdi->priv = NULL;
	}
	if (sdi->conn) {
		sr_usb_dev_inst_free(sdi->conn);
		sdi->conn = NULL;
	}
	sr_dev_inst_free(sdi);
	return SR_OK;
}

SR_PRIV void slogic16u3_on_usb_reconnected(struct sr_dev_inst *sdi,
						   struct libusb_device *new_dev)
{
	struct slogic_context *devc;
	struct libusb_device_descriptor des;
	const struct slogic_model *model;
	const struct slogic_model *old_model;

	if (!sdi || !sdi->priv || !new_dev)
		return;
	if (!sdi->driver || !sdi->driver->name ||
	    strcmp(sdi->driver->name, "slogic-16u3") != 0)
		return;

	devc = sdi->priv;
	old_model = devc->model;
	if (libusb_get_device_descriptor(new_dev, &des) != 0 ||
		 des.idVendor != SLOGIC_VID ||
		 !(model = slogic_model_for_pid(des.idProduct)))
		return;
	if (devc->usb_dev)
		libusb_unref_device(devc->usb_dev);
	devc->usb_dev = libusb_ref_device(new_dev);
	devc->model = model;
	if ((size_t)devc->ch_mode >= model->limit_count)
		devc->ch_mode = 0;
	devc->channel_count = (model->physical_channels >> (devc->ch_mode));
	devc->sdi = sdi;
	sdi->handle = (ds_device_handle)devc->usb_dev;
	if (old_model != model) {
		setup_channels(sdi, model->physical_channels);
		slogic_apply_channel_mode(sdi);
	}
	slogic_update_usb_speed(devc);
	slogic_map_samplerate(devc, devc->samplerate ? devc->samplerate
							     : slogic_max_rate(model, model->physical_channels >> (devc->ch_mode)));
	sr_info("reconnected: name=\"%s\" speed=%s max=%" PRIu64,
		sdi->name ? sdi->name : "?",
		slogic_link_name(al_link(devc)),
		al_link_max_rate(devc));
}

static int hw_dev_status_get(const struct sr_dev_inst *sdi,
			     struct sr_status *status, gboolean prg)
{
	(void)sdi;
	(void)prg;
	if (!status)
		return SR_ERR;
	memset(status, 0, sizeof(*status));
	return SR_OK;
}

static int slogic_mode_for_channels(const struct slogic_model *model, int channels)
{
	size_t i;

	if (!model)
		return -1;
	for (i = 0; i < model->limit_count; i++) {
		if ((model->physical_channels >> (i)) == channels)
			return (int)i;
	}
	return -1;
}

static int slogic_set_pattern_mode(struct slogic_context *devc, int mode)
{
	if (!devc || mode < 0 || mode >= (int)ARRAY_SIZE(pattern_modes))
		return SR_ERR_ARG;
	if (devc->model && devc->model->proto == SLOGIC_PROTO_COMBO8 && mode != 0)
		return SR_ERR_NA;
	/* Stored only; the shared core applies the pattern at the next
	 * slogic_reset + slogic_configure (done at acquisition start). */
	devc->pattern_mode = mode;
	return SR_OK;
}

static int config_get(int id, GVariant **data, const struct sr_dev_inst *sdi,
		      const struct sr_channel *ch,
		      const struct sr_channel_group *cg)
{
	struct slogic_context *devc;

	(void)ch;
	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_DEVICE_MODE:
		*data = g_variant_new_int16(sdi->mode);
		break;
	case SR_CONF_UNIT_BITS:
		*data = g_variant_new_byte(1);
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_string(maxHeights[devc->max_height]);
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		*data = g_variant_new_byte(devc->max_height);
		break;
	case SR_CONF_HW_DEPTH:
		*data = g_variant_new_uint64(devc->model ? SLOGIC_HW_DEPTH :
			SLOGIC_HW_DEPTH);
		break;
	case SR_CONF_TOTAL_CH_NUM:
		*data = g_variant_new_int16(devc->model ?
			devc->model->physical_channels : 16);
		break;
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int16(devc->channel_count);
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(TRUE);
		break;
	case SR_CONF_CAPTURE_RATIO:
		/* The core keeps the pre-trigger samples in host memory, so the
		 * trigger position is free (DSView enables its slider on this). */
		*data = g_variant_new_uint64(trigger ? ds_trigger_get_pos() : 0);
		break;
	case SR_CONF_VTH:
		*data = g_variant_new_double(devc->vth);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_int16(devc->op_mode);
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_string(pattern_modes[devc->pattern_mode]);
		break;
	case SR_CONF_CHANNEL_MODE:
		*data = g_variant_new_int16(devc->ch_mode);
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_int16(devc->filter);
		break;
	case SR_CONF_LOOP_MODE:
		*data = g_variant_new_boolean(devc->is_loop != 0);
		break;
	case SR_CONF_USB_SPEED:
		*data = g_variant_new_int32((int32_t)devc->usb_speed);
		break;
	case SR_CONF_USB30_SUPPORT:
		*data = g_variant_new_boolean(devc->model &&
			(devc->model->proto == SLOGIC_PROTO_U3));
		break;
	case SR_CONF_LA_CH32:
		*data = g_variant_new_boolean(devc->model &&
			devc->model->physical_channels >= 32);
		break;
	case SR_CONF_RLE:
	case SR_CONF_RLE_SUPPORT:
		*data = g_variant_new_boolean(FALSE);
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_set(int id, GVariant *data, struct sr_dev_inst *sdi,
		      struct sr_channel *ch,
		      struct sr_channel_group *cg)
{
	struct slogic_context *devc;
	const char *stropt;
	unsigned int i;

	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_SAMPLERATE:
		slogic_map_samplerate(devc, g_variant_get_uint64(data));
		sr_info("samplerate=%" PRIu64, devc->samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_EN:
		if (ch)
			ch->enabled = g_variant_get_boolean(data);
		break;
	case SR_CONF_MAX_HEIGHT:
		stropt = g_variant_get_string(data, NULL);
		for (i = 0; i < ARRAY_SIZE(maxHeights); i++) {
			if (!strcmp(stropt, maxHeights[i])) {
				devc->max_height = (int)i;
				break;
			}
		}
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		devc->max_height = g_variant_get_byte(data);
		break;
	case SR_CONF_VTH: {
		double v = g_variant_get_double(data);
		if (v < SLOGIC_VTH_MIN)
			v = SLOGIC_VTH_MIN;
		if (v > SLOGIC_VTH_MAX)
			v = SLOGIC_VTH_MAX;
		devc->vth = v;
		sr_info("VTH stored: %.2f V", devc->vth);
		break;
	}
	case SR_CONF_PATTERN_MODE: {
		const char *p = g_variant_get_string(data, NULL);
		int mode;
		for (mode = 0; mode < (int)ARRAY_SIZE(pattern_modes); mode++)
			if (!strcmp(p, pattern_modes[mode]))
				break;
		if (mode >= (int)ARRAY_SIZE(pattern_modes))
			return SR_ERR_ARG;
		return slogic_set_pattern_mode(devc, mode);
	}
	case SR_CONF_FILTER: {
		int nv = g_variant_get_int16(data);
		if (nv != SR_FILTER_NONE && nv != SR_FILTER_1T)
			return SR_ERR;
		devc->filter = nv;
		break;
	}
	case SR_CONF_CHANNEL_MODE: {
		int nv = g_variant_get_int16(data);
		if (!devc->model || nv < 0 ||
		    (size_t)nv >= devc->model->limit_count)
			return SR_ERR;
		devc->ch_mode = nv;
		slogic_apply_channel_mode(sdi);
		slogic_map_samplerate(devc, devc->samplerate);
		sr_info("channel mode -> %d ch, rate=%" PRIu64,
			devc->channel_count, devc->samplerate);
		break;
	}
	case SR_CONF_VLD_CH_NUM: {
		int nv = g_variant_get_int16(data);
		int mode = slogic_mode_for_channels(devc->model, nv);
		if (mode < 0)
			return SR_ERR;
		devc->ch_mode = mode;
		slogic_apply_channel_mode(sdi);
		slogic_map_samplerate(devc, devc->samplerate);
		break;
	}
	case SR_CONF_OPERATION_MODE: {
		int nv = g_variant_get_int16(data);
		if (nv != LO_OP_STREAM)
			return SR_ERR;
		devc->op_mode = LO_OP_STREAM;
		break;
	}
	case SR_CONF_LOOP_MODE:
		devc->is_loop = g_variant_get_boolean(data) ? 1 : 0;
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_list(int key, GVariant **data, const struct sr_dev_inst *sdi,
		       const struct sr_channel_group *cg)
{
	GVariant *gvar;
	GVariantBuilder gvb;

	(void)cg;

	switch (key) {
	case SR_CONF_DEVICE_OPTIONS:
		*data = g_variant_new_from_data(
			G_VARIANT_TYPE("ai"), hwoptions,
			ARRAY_SIZE(hwoptions) * sizeof(int32_t), TRUE, NULL, NULL);
		break;
	case SR_CONF_DEVICE_SESSIONS:
		*data = g_variant_new_from_data(
			G_VARIANT_TYPE("ai"), sessions,
			ARRAY_SIZE(sessions) * sizeof(int32_t), TRUE, NULL, NULL);
		break;
	case SR_CONF_SAMPLERATE: {
		/* Must hold the whole rate list plus its 0 terminator. */
		static uint64_t rate_buf[33];
		int rate_n = 0;
		const struct slogic_context *devc =
			(sdi && sdi->priv) ? (const struct slogic_context *)sdi->priv
					   : NULL;

		if (devc)
			slogic_build_rate_list(devc, rate_buf, &rate_n);
		else {
			unsigned int i;
			const slogic_model *m16 = slogic_model_for_pid(SLOGIC_PID_16U3); for (i = 0; m16 && i < m16->rate_count; i++)
				if (m16->rates[i] <= SR_MHZ(200))
					rate_buf[rate_n++] = m16->rates[i];
			rate_buf[rate_n] = 0;
		}
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_from_data(
			G_VARIANT_TYPE("at"), rate_buf,
			(gsize)rate_n * sizeof(uint64_t),
			TRUE, NULL, NULL);
		g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	}
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_strv(maxHeights, ARRAY_SIZE(maxHeights));
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_uint64((uint64_t)&filter_list);
		break;
	case SR_CONF_CHANNEL_MODE:
		*data = g_variant_new_uint64((uint64_t)slogic_channel_mode_list(
			(sdi && sdi->priv) ?
			((const struct slogic_context *)sdi->priv)->model : NULL));
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_uint64((uint64_t)&opmode_list);
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_strv(pattern_modes, ARRAY_SIZE(pattern_modes));
		break;
	default:
		return SR_ERR_ARG;
	}
	return SR_OK;
}

static int hw_dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data)
{
	struct slogic_context *devc = sdi->priv;
	slogic_capture_config c;
	int ret;

	(void)cb_data;

	if (sdi->status != SR_ST_ACTIVE || !devc->sdev) {
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}

	devc->sdi = sdi;
	devc->num_samples = 0;
	devc->num_bytes = 0;
	devc->status = SLOGIC_ST_START;
	devc->raw_pending_len = 0;
	devc->fast_pending_len = 0;
	devc->filt_have_prev = 0;
	devc->stream_res_len = 0;
	devc->source_added = 0;
	devc->end_sent = 0;

	slogic_refresh_enabled_bits(devc);
	slogic_map_samplerate(devc, devc->samplerate);

	memset(&c, 0, sizeof(c));
	al_config(devc, &c.dev);
	c.limit_samples = devc->is_loop ? 0 : devc->limit_samples; /* 0 = loop */
	c.sink.user = devc;
	c.sink.data = al_sink_data;
	c.sink.trigger = al_sink_trigger;
	al_map_trigger(devc, &c);

	sr_info("acq start: nch=%d en=%d rate=%" PRIu64 " loop=%d limit=%" PRIu64
		" speed=%s trigger=%s pre=%" PRIu64,
		devc->channel_count, devc->en_count, devc->samplerate,
		devc->is_loop, devc->limit_samples,
		slogic_link_name(al_link(devc)),
		slogic_trigger_spec_empty(&c.trigger) ? "off" : "on",
		c.pretrigger_samples);

	ret = slogic_capture_start(devc->sdev, &c, &devc->cap);
	if (ret != SLOGIC_OK) {
		sr_err("capture start failed (%d)", ret);
		return SR_ERR;
	}

	/* The core's event thread services libusb; this source drains the
	 * core's queue into LA_CROSS_DATA and ends the session. */
	ret = sr_session_source_add(-1, 0, 1, receive_data, sdi);
	if (ret != SR_OK) {
		slogic_capture_destroy(devc->cap);
		devc->cap = NULL;
		return ret;
	}
	devc->source_added = 1;
	std_session_send_df_header(sdi, LOG_PREFIX);
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct slogic_context *devc;

	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;

	devc = sdi->priv;
	if (devc->cap)
		slogic_capture_stop(devc->cap);
	return SR_OK;
}

SR_PRIV struct sr_dev_driver slogic16u3_driver_info = {
	.name = "slogic-16u3",
	.longname = "Sipeed SLogic Combo 8 / 16U3 / 32U3 Logic Analyzer",
	.api_version = 1,
	.driver_type = DRIVER_TYPE_HARDWARE,
	.init = hw_init,
	.cleanup = hw_cleanup,
	.scan = hw_scan,
	.dev_mode_list = hw_dev_mode_list,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = hw_dev_open,
	.dev_close = hw_dev_close,
	.dev_destroy = hw_dev_destroy,
	.dev_status_get = hw_dev_status_get,
	.dev_acquisition_start = hw_dev_acquisition_start,
	.dev_acquisition_stop = hw_dev_acquisition_stop,
	.priv = NULL,
};
