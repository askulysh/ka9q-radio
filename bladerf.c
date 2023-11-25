// Front end driver for bladeRF
// Copyright 2023, Andriy Skulysh

#define _GNU_SOURCE 1
#include <pthread.h>
#include <libbladeRF.h>
#include <errno.h>
#include <iniparser/iniparser.h>
#if defined(linux)
#include <bsd/string.h>
#endif
#include <sysexits.h>

#include "conf.h"
#include "misc.h"
#include "status.h"
#include "radio.h"
#include "config.h"

static const float power_smooth = 0.05; // Arbitrary exponential smoothing factor

// Anything generic should be in 'struct frontend' section 'sdr' in radio.h
struct sdrstate
{
	struct frontend *frontend;  /* Avoid references to external globals */
	struct bladerf	*dev;           /* Opaque pointer */
	pthread_t	monitor_thread;
	void		**buffers;      /* Transmit buffers */
	size_t		num_buffers;    /* Number of buffers */
	size_t		samples_per_buffer; /* Number of samples per buffer */
	unsigned int	idx;            /* The next one that needs to go out */
};

static double set_correct_freq(struct sdrstate *sdr, double freq);
static void * bladerf_monitor(void *p);

int bladerf_setup(struct frontend * const frontend,
		  dictionary * const Dictionary, char const * const section)
{
	char const *p;
	bladerf_channel ch = BLADERF_MODULE_RX;

	struct sdrstate * const sdr = calloc(1, sizeof(struct sdrstate));
	sdr->frontend = frontend;
	frontend->context = sdr;
	char const *device = config_getstring(Dictionary, section, "device",
			NULL);
	if (strcasecmp(device, "bladerf") != 0)
		return -1;

	bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_VERBOSE);
	int status;

	char const * const sn = config_getstring(Dictionary, section,
			"serial", NULL);
	if (sn != NULL) {
		struct bladerf_devinfo dev_info;

		bladerf_init_devinfo(&dev_info);
		strncpy(dev_info.serial, sn, sizeof(dev_info.serial) - 1);
		status = bladerf_open_with_devinfo(&sdr->dev, &dev_info);
	} else {
		status = bladerf_open(&sdr->dev, NULL);
	}
	if (status != 0) {
		fprintf(stderr, "Failed to open device: %s\n",
				bladerf_strerror(status));
		return -1;
	}

	status = bladerf_is_fpga_configured(sdr->dev);
	if (status < 0) {
		fprintf(stderr, "Failed to determine FPGA state: %s\n",
				bladerf_strerror(status));
		return -1;
	} else if (status == 0) {
		fprintf(stderr, "Error: FPGA is not loaded.\n");
		bladerf_close(sdr->dev);
		return -1;
	}

	sdr->idx = 0;
	sdr->num_buffers = 512;
	sdr->samples_per_buffer = 4096;

	frontend->samprate = 12000000;
	p = config_getstring(Dictionary, section, "samprate", NULL);
	if (p != NULL)
		frontend->samprate = parse_frequency(p, false);

	frontend->isreal = false;
	frontend->bitspersample = 12;
	frontend->calibrate = config_getdouble(Dictionary,section, "calibrate",
			0);

	fprintf(stdout, "Set sample rate %'u Hz\n", frontend->samprate);
	status = bladerf_set_sample_rate(sdr->dev, ch,
			(uint32_t)frontend->samprate, NULL);
	if (status != 0) {
		fprintf(stderr, "Failed to set samplerate: %s\n",
				bladerf_strerror(status));
		bladerf_close(sdr->dev);
		return -1;
	}

	uint32_t bw = 0, bw_actual = 0;
	p = config_getstring(Dictionary, section, "bandwidth", NULL);
	if (p != NULL)
		bw = parse_frequency(p, false);

	if (bw == 0)
		bw = frontend->samprate*0.8;

	status = bladerf_set_bandwidth(sdr->dev, ch, bw, &bw_actual);
	if (status != 0) {
		fprintf(stderr, "Failed to set bandwidth %u : %s\n",
				bw, bladerf_strerror(status));
		bladerf_close(sdr->dev);
		return -1;
	}
	fprintf(stdout, "Set bandwidth %'u Hz\n", bw_actual);
	frontend->calibrate = 0;
	frontend->max_IF = +frontend->samprate;
	frontend->min_IF = -frontend->samprate;

	frontend->rf_gain = config_getint(Dictionary, "bladerf", "gain",0);
	fprintf(stdout, "config gain %f\n", frontend->rf_gain);
	if (frontend->rf_gain != 0) {
		status = bladerf_set_gain_mode(sdr->dev, ch, BLADERF_GAIN_MGC);
		if (status < 0) {
			fprintf(stderr,
				"Failed to set gain mode on channel %d: %s\n",
				ch, bladerf_strerror(status));
		}
		status = bladerf_set_gain(sdr->dev, ch, frontend->rf_gain);
		if (status != 0) {
			fprintf(stderr, "Failed to set gain: %s\n",
					bladerf_strerror(status));
		}
	} else {
		status = bladerf_set_gain_mode(sdr->dev, ch, BLADERF_GAIN_AUTOMATIC);
		if (status < 0) {
			fprintf(stderr, "Failed to set AGC on channel %d: %s\n",
					ch, bladerf_strerror(status));
		}
	}

	bool antenna_bias = config_getboolean(Dictionary, section, "bias", false);
	bladerf_set_bias_tee(sdr->dev, ch, antenna_bias);
	bladerf_get_bias_tee(sdr->dev, ch, &antenna_bias);
	fprintf(stdout, "bias tee %d\n", antenna_bias);

	p = config_getstring(Dictionary, section, "description", NULL);
	if (p != NULL)
		strlcpy(frontend->description, p, sizeof(frontend->description));

	double init_frequency = 0;
	p = config_getstring(Dictionary, section, "frequency", NULL);
	if (p != NULL)
		init_frequency = parse_frequency(p, false);
	if (init_frequency != 0) {
		set_correct_freq(sdr, init_frequency);
		frontend->lock = true;
		fprintf(stdout, "Locked tuner frequency %'.3lf Hz\n",
				init_frequency);
	}

	return 0;
}

int bladerf_startup(struct frontend * const frontend)
{
	struct sdrstate * const sdr = (struct sdrstate *)frontend->context;
	pthread_create(&sdr->monitor_thread, NULL, bladerf_monitor, sdr);

	return 0;
}

static void *stream_callback(struct bladerf *,
			     struct bladerf_stream *,
			     struct bladerf_metadata *, void *samples,
			     size_t num_samples, void *user_data)
{
	struct sdrstate * const sdr = (struct sdrstate *)user_data;
	struct frontend * const frontend = sdr->frontend;
	float complex * const wptr = frontend->in.input_write_pointer.c;
	float energy = 0;

	const int16_t *sample = (const int16_t *)samples;
	for (size_t i=0; i < num_samples; i++) {
		float complex samp;
		int16_t s;

		s = sample[0] & 0xfff;
		frontend->overranges += s == 0x7ff || s == 0x800;
		if (s & 0x800)
			s |= 0xf000;
		__real__ samp = s;
		s = sample[1] & 0xfff;
		frontend->overranges += s == 0x7ff || s == 0x800;
		if (s & 0x800)
			s |= 0xf000;
		__imag__ samp = s;
		energy += cnrmf(samp);
		wptr[i] = samp;
		sample += 2;
	}
	// Update write pointer, invoke FFT
	write_cfilter(&frontend->in, NULL, num_samples);
	frontend->if_power += power_smooth * (energy / num_samples -
			frontend->if_power);
	frontend->samples += num_samples;

	void *rv = sdr->buffers[sdr->idx];
	sdr->idx = (sdr->idx + 1) % sdr->num_buffers;

	return rv ;
}

static void *bladerf_monitor(void *p)
{
	struct sdrstate * const sdr = (struct sdrstate *)p;
	pthread_setname("bladerf-mon");

	realtime();

	struct bladerf_stream *stream;

	int status = bladerf_init_stream(
			&stream,
			sdr->dev,
			stream_callback,
			&sdr->buffers,
			sdr->num_buffers,
			BLADERF_FORMAT_SC16_Q11,
			sdr->samples_per_buffer,
			sdr->num_buffers,
			sdr
			);

	status = bladerf_enable_module(sdr->dev, BLADERF_MODULE_RX, true);
	if (status < 0) {
		fprintf(stderr, "Failed to enable module: %s\n",
				bladerf_strerror(status));
		bladerf_deinit_stream(stream);
		bladerf_close(sdr->dev);
		goto out;
	}

	fprintf(stdout, "bladerf running\n");
	int readback = 0;
	status = bladerf_get_gain(sdr->dev, BLADERF_MODULE_RX, &readback);
	if (status != 0) {
		fprintf(stderr, "Failed to read back gain: %s\n",
				bladerf_strerror(status));
	}
	if (readback != 60)
		sdr->frontend->rf_gain = readback;
	printf("set gain = %d\n", readback);

	/* Start stream and stay there until we kill the stream */
	status = bladerf_stream(stream, BLADERF_MODULE_RX);
	if (status < 0) {
		fprintf(stderr, "Stream error: %s\n", bladerf_strerror(status));
	}

	status = bladerf_enable_module(sdr->dev, BLADERF_MODULE_RX, false);
	if (status < 0) {
		fprintf(stderr, "Failed to enable module: %s\n",
				bladerf_strerror(status));
	}

	bladerf_deinit_stream(stream);
	bladerf_close(sdr->dev);
	fprintf(stdout, "Device is no longer streaming, exiting\n");
out:
	exit(EX_NOINPUT); // Let systemd restart us
}

static double set_correct_freq(struct sdrstate * const sdr, double const freq)
{
	uint32_t f = freq;
	int status = bladerf_set_frequency(sdr->dev, BLADERF_MODULE_RX, f);

	if (status != 0) {
		fprintf(stderr,
			"Failed to set RX frequency to %u %s\n",
			f, bladerf_strerror(status));
		sdr->frontend->frequency = 0.0;
	} else {
		sdr->frontend->frequency = freq;
	}
	printf("tuned to %d\n", f);

	return sdr->frontend->frequency;
}

double bladerf_tune(struct frontend * const frontend, double const f)
{
	if (frontend->lock)
		return frontend->frequency;

	struct sdrstate * const sdr = frontend->context;

	return set_correct_freq(sdr, f);
}
