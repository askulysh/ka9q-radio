//#define SPECTRUM_DEBUG 1

// Spectral analysis service - far from complete - for ka9q-radio's radiod
// Copyright 2023-2025, Phil Karn, KA9Q
#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>

#include "misc.h"
#include "iir.h"
#include "filter.h"
#include "radio.h"

// Spectrum analysis thread
int demod_spectrum(void *arg){
  struct channel * const chan = arg;
  assert(chan != NULL);
  if(chan == NULL)
    return -1;
  {
    char name[100];
    snprintf(name,sizeof(name),"spect %u",chan->output.rtp.ssrc);
    pthread_setname(name);
  }
  pthread_mutex_init(&chan->status.lock,NULL);
  pthread_mutex_lock(&chan->status.lock);
  FREE(chan->status.command);
  FREE(chan->filter.energies);
  FREE(chan->spectrum.bin_data);
  delete_filter_output(&chan->filter.out);
  if(chan->output.opus != NULL){
    opus_encoder_destroy(chan->output.opus);
    chan->output.opus = NULL;
  }
  chan->status.output_interval = 0; // No automatic status updates
  chan->status.output_timer = 0; // No automatic status updates
  chan->output.silent = true; // we don't send anything there

  pthread_mutex_unlock(&chan->status.lock);

  // Parameters set by system input side
  float const blockrate = 1000.0f / Blocktime; // Typically 50 Hz

  int const L = Frontend.L;
  int const M = Frontend.M;
  int const N = L + M - 1;

  float const fe_fft_bin_spacing = blockrate * (float)L/N; // Input FFT bin spacing. Typically 40 Hz
  float binsperbin = 0; // can handle non-integer ratios

  // experiment - make array largest possible to temp avoid memory corruption
  chan->spectrum.bin_data = calloc(Frontend.in.bins,sizeof *chan->spectrum.bin_data);

  fftwf_plan plan = NULL;
  complex float *fft_in = NULL;
  complex float *fft_out = NULL;
  float gain = 0;
  int fft_index = 0;
  int old_bin_count = -1;
  float old_bin_bw = -1;
  int input_bins = 0;
  float *power_buffer = NULL;

  while(1){
    // Check user params
    int bin_count = chan->spectrum.bin_count <= 0 ? 64 : chan->spectrum.bin_count;
    float bin_bw = chan->spectrum.bin_bw <= 0 ? 1000 : chan->spectrum.bin_bw;
    int actual_bin_count = bin_count;

    if(bin_bw > 50){
      // large bins, use forward FFT directly

      if(bin_bw != old_bin_bw || bin_count != old_bin_count){
	// Params have changed, set everything up again
	old_bin_bw = bin_bw;
	old_bin_count = bin_count;

	// Get rid of anything old
	delete_filter_output(&chan->filter.out);
	if(plan != NULL){
	  fftwf_destroy_plan(plan);
	  plan = NULL;
	}
	FREE(fft_in);
	FREE(fft_out);
	FREE(chan->filter.energies);
	FREE(chan->status.command);
	FREE(power_buffer);

	binsperbin = bin_bw / fe_fft_bin_spacing;
	input_bins = ceilf(binsperbin * bin_count);
	if(Verbose > 1)
	  fprintf(stdout,"spectrum %d: freq %'lf bin_bw %'f binsperbin %'.1f bin_count %'d\n",chan->output.rtp.ssrc,chan->tune.freq,bin_bw,binsperbin,bin_count);

	chan->filter.max_IF = (bin_count * bin_bw)/2;
	chan->filter.min_IF = -chan->filter.max_IF;
	power_buffer = malloc((input_bins + 10) * sizeof *power_buffer);
	create_filter_output(&chan->filter.out,&Frontend.in,NULL,0,SPECTRUM);
	// Compute power (not amplitude) scale factor
	gain = 1.0f / (float) N;   // scale each bin value for our FFT
	gain *= gain;              // squared because the we're scaling the output of complex norm, not the input bin values
	if(chan->filter.out.master->in_type == REAL)
	  gain *= 2;               // we only see one side of the spectrum for real inputs
#if SPECTRUM_DEBUG
	fprintf(stdout,"direct mode binsperbin %'.1f bin_bw %.1f bin_count %d gain %.1f dB\n",
		binsperbin,bin_bw,bin_count,power2dB(gain));
#endif
      }
      if(downconvert(chan) != 0)
	break;

      // Look at downconverter's frequency bins directly
      //      chan->spectrum.bin_data = reallocf(&chan->spectrum.bin_data, bin_count * sizeof *chan->spectrum.bin_data);
      // Output flter is already waiting for the next job, so subtract 1 to get the current one
      unsigned int jobnum = (chan->filter.out.next_jobnum - 1) % ND;
      struct filter_in const * const master = chan->filter.out.master;
      complex float const * const fdomain = master->fdomain[jobnum];

      // Read the master's frequency bins directly
      // The layout depends on the master's time domain input:
      // 1. Complex 2. Real, upright spectrum 3. Real, inverted spectrum
      if(master->in_type == COMPLEX){
	int binp = -chan->filter.bin_shift - input_bins/2;
	if(binp < 0)
	  binp += master->bins; // Start in negative input region

	// Form array of bin energies from lowest frequency to high
	// Lowest frequency in power_buffer[0] to simplify interpolation
	for(int i = 0; i < input_bins; i++){
	  power_buffer[i] = cnrmf(fdomain[binp]);
	  if(++binp == master->bins)
	    binp = 0;
	}
      } else if(chan->filter.bin_shift <= 0){
	// Real input right side up
	int binp = -chan->filter.bin_shift - input_bins/2;
	int i = 0;
	if(binp < 0){
	  // Requested range starts below DC; skip
	  memset(power_buffer,0,input_bins * sizeof *power_buffer);
	  i = -binp / binsperbin;
	  if(i >= input_bins)
	    i -= input_bins;
	  binp = 0;
	}
	for(; i < input_bins && binp < master->bins;i++){
	  power_buffer[i] = cnrmf(fdomain[binp++]);
	}
      } else {
	// Real input spectrum is inverted, read in reverse order
	int binp = chan->filter.bin_shift + input_bins/2;
	int i = 0;
	if(binp >= master->bins){
	  // Requested range starts above top; skip
	  memset(power_buffer,0,input_bins * sizeof *power_buffer);
	  i = (master->bins - binp - 1) / binsperbin;
	  if(i >= input_bins)
	    i -= input_bins;
	  binp = master->bins - 1;
	}
	for(; i < input_bins && binp >= 0;i++)
	  power_buffer[i] = cnrmf(fdomain[binp--]);

      }
      // Merge the bins, negative output frequencies first
      float ratio = (float)bin_count / input_bins;

      int out = bin_count/2;
      float outf = (int)out;
      int in = 0;
      while(out < bin_count){
	float p = 0;
	int count = 0;
	while((int)outf == out){
	  p += power_buffer[in++];
	  outf += ratio;
	  count++;
	}
	chan->spectrum.bin_data[out++] = (p * gain) / count;
      }
      // Positive output frequencies
      out = 0;
      outf = (int)out;
      in = input_bins/2;
      while(out < bin_count/2){
	float p = 0;
	int count = 0;
	while((int)outf == out){
	  p += power_buffer[in++];
	  outf += ratio;
	  count++;
	}
	chan->spectrum.bin_data[out++] = (p * gain) / count;
      }
    } else {
      // ***FFT MODE***

      // For fine resolution better than the ~40 Hz from the main FFT, create an ordinary IQ channel
      // and feed it to a FFT. This also has parameter restrictions, mainly on the sample rate of the IQ channel
      // It will take several blocks to fill each FFT
      if(bin_bw != old_bin_bw || bin_count != old_bin_count){
	// Params have changed, set everything up againa
	old_bin_bw = bin_bw;
	old_bin_count = bin_count;

	if(Verbose > 1)
	  fprintf(stdout,"spectrum %d: freq %'lf bin_bw %'f bin_count %'d\n",chan->output.rtp.ssrc,chan->tune.freq,bin_bw,bin_count);

#if SPECTRUM_DEBUG
	fprintf(stdout,"spectrum creating IQ/FFT channel, requested bw = %.1f bin_count = %d\n",bin_bw,bin_count);
#endif
	delete_filter_output(&chan->filter.out);
	if(plan != NULL)
	  fftwf_destroy_plan(plan);
	plan = NULL;
	FREE(fft_in);
	FREE(fft_out);
	FREE(chan->filter.energies);
	FREE(chan->status.command);
	FREE(power_buffer);

	int samprate = bin_bw * bin_count;
	int valid_samprates = lcm(blockrate,L*blockrate/N);
	if(samprate % valid_samprates != 0){
	  // round up
	  samprate += valid_samprates - samprate % valid_samprates;
#if SPECTRUM_DEBUG
	  fprintf(stdout,"rounding samprate up to %d\n",samprate);
#endif
	  actual_bin_count = ceilf(samprate / bin_bw);
	} else
	  actual_bin_count = bin_count;
	// Should also round up to an efficient FFT size
	int frame_len = ceilf(samprate * Blocktime / 1000.);
	assert(actual_bin_count >= bin_count);

	chan->filter.min_IF = -samprate/2 + 200;
	chan->filter.max_IF = samprate/2 - 200;

	// The channel filter already normalizes for the size of the forward input FFT, we just handle our own FFT gain
	gain = 1.0f / (float) actual_bin_count;
	gain *= gain;                     // squared because the we're scaling the output of complex norm, not the input bin values

	int r = create_filter_output(&chan->filter.out,&Frontend.in,NULL,frame_len,COMPLEX);
	(void)r;
	assert(r == 0);

	set_filter(&chan->filter.out,chan->filter.min_IF,chan->filter.max_IF,11.0);
	chan->filter.remainder = NAN; // Force init of downconverter
	chan->filter.bin_shift = 1010101010; // Unlikely - but a kludge, force init of phase rotator

	// Should round FFT block size up to an efficient number
	fft_in = lmalloc(actual_bin_count * sizeof(complex float));
	fft_out = lmalloc(actual_bin_count * sizeof(complex float));
	fft_index = 0;
	assert(fft_in != NULL && fft_out != NULL);
#if SPECTRUM_DEBUG
	fprintf(stdout,"frame_len %d, actual bin count %d samprate %d, bin_bw %.1f gain %.1f dB\n",
		frame_len,actual_bin_count,samprate,bin_bw,power2dB(gain));
#endif
	pthread_mutex_lock(&FFTW_planning_mutex);
	fftwf_plan_with_nthreads(1);
	if((plan = fftwf_plan_dft_1d(actual_bin_count, fft_in, fft_out, FFTW_FORWARD, FFTW_WISDOM_ONLY|FFTW_planning_level)) == NULL){
	  suggest(FFTW_planning_level,actual_bin_count,FFTW_FORWARD,COMPLEX);
	  plan = fftwf_plan_dft_1d(actual_bin_count,fft_in,fft_out,FFTW_FORWARD,FFTW_MEASURE);
	}
	pthread_mutex_unlock(&FFTW_planning_mutex);
	if(fftwf_export_wisdom_to_filename(Wisdom_file) == 0)
	  fprintf(stdout,"fftwf_export_wisdom_to_filename(%s) failed\n",Wisdom_file);
      }
      if(downconvert(chan) != 0)
	break;

      // FFT mode for more precision
      // Should implement overlapping windowed FFTs here
      for(int i = 0; i < chan->sampcount; i++){
	assert(fft_index >= 0 && fft_index < actual_bin_count);
	fft_in[fft_index] = chan->baseband[i];
	if(++fft_index >= actual_bin_count){
	  // Time domain buffer is full, run the FFT
	  fft_index = 0;
	  fftwf_execute_dft(plan,fft_in,fft_out);
	  // Copy requested number of bins to user
	  // Should verify correctness for combinations of even and odd bin_count and actual_bin_count
	  for(int j = 0; j < bin_count/2; j++){
	    float p = gain * cnrmf(fft_out[j]); // Take power spectrum
#if SPECTRUM_DEBUG
	    if(p == 0)
	      fprintf(stdout,"spectrum[%d] = 0\n",j);
#endif
	    assert(j >= 0 && j < Frontend.in.bins);
	    chan->spectrum.bin_data[j] = p;
	  }
	  int offset = actual_bin_count - bin_count;
	  for(int j = bin_count/2; j < bin_count; j++){
	    float p = gain * cnrmf(fft_out[j+offset]); // Take power spectrum
#if SPECTRUM_DEBUG
	    if(p == 0)
	      fprintf(stdout,"spectrum[%d] = 0\n",j);
#endif
	    assert(j >= 0 && j < Frontend.in.bins);
	    chan->spectrum.bin_data[j] = p;
	  }
	  // Zero the Nyquist bin, if any
	  // Is this correct? The display will be showing decibels, and 0 -> -inf dB
	  if((bin_count & 1) == 0)
	    chan->spectrum.bin_data[bin_count/2] = 0;
	}
      }
    }
  }
  delete_filter_output(&chan->filter.out);
  if(plan != NULL)
    fftwf_destroy_plan(plan);
  plan = NULL;
  FREE(fft_in);
  FREE(fft_out);
  FREE(chan->filter.energies);
  FREE(chan->status.command);
  FREE(power_buffer);
  FREE(chan->spectrum.bin_data);
  return 0;
}
