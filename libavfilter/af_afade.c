/*
 * Copyright (c) 2013-2015 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * fade audio filter
 */

#include "config_components.h"

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AudioFadeContext {
    const AVClass *class;
    int nb_inputs;
    int type;
    int curve, curve2;
    int64_t nb_samples;
    int64_t start_sample;
    int64_t duration;
    int64_t start_time;
    double silence;
    double unity;
    int overlap;
    int64_t pts;
    int xfade_idx;

    /* Ring buffer for lazy crossfade (memory-efficient) */
    AVFrame *ring_buf;          /* Ring buffer holding last nb_samples from input 0 */
    int64_t ring_write_pos;     /* Write position in ring buffer (circular, 0 to nb_samples-1) */
    int64_t ring_filled;        /* Number of valid samples in ring buffer */
    int64_t crossfade_pos;      /* Current read position within crossfade (0 to nb_samples) */
    int crossfade_active;       /* Flag: currently in crossfade processing mode */

    void (*fade_samples)(uint8_t **dst, uint8_t * const *src,
                         int nb_samples, int channels, int direction,
                         int64_t start, int64_t range, int curve,
                         double silence, double unity);
    void (*scale_samples)(uint8_t **dst, uint8_t * const *src,
                          int nb_samples, int channels, double unity);
    void (*crossfade_samples)(uint8_t **dst, uint8_t * const *cf0,
                              uint8_t * const *cf1,
                              int nb_samples, int channels,
                              int curve0, int curve1,
                              int64_t offset, int64_t total);
} AudioFadeContext;

enum CurveType { NONE = -1, TRI, QSIN, ESIN, HSIN, LOG, IPAR, QUA, CUB, SQU, CBR, PAR, EXP, IQSIN, IHSIN, DESE, DESI, LOSI, SINC, ISINC, QUAT, QUATR, QSIN2, HSIN2, NB_CURVES };

#define OFFSET(x) offsetof(AudioFadeContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_S32, AV_SAMPLE_FMT_S32P,
        AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };

static double fade_gain(int curve, int64_t index, int64_t range, double silence, double unity)
{
#define CUBE(a) ((a)*(a)*(a))
    double gain;

    gain = av_clipd(1.0 * index / range, 0, 1.0);

    switch (curve) {
    case QSIN:
        gain = sin(gain * M_PI / 2.0);
        break;
    case IQSIN:
        /* 0.6... = 2 / M_PI */
        gain = 0.6366197723675814 * asin(gain);
        break;
    case ESIN:
        gain = 1.0 - cos(M_PI / 4.0 * (CUBE(2.0*gain - 1) + 1));
        break;
    case HSIN:
        gain = (1.0 - cos(gain * M_PI)) / 2.0;
        break;
    case IHSIN:
        /* 0.3... = 1 / M_PI */
        gain = 0.3183098861837907 * acos(1 - 2 * gain);
        break;
    case EXP:
        /* -11.5... = 5*ln(0.1) */
        gain = exp(-11.512925464970227 * (1 - gain));
        break;
    case LOG:
        gain = av_clipd(1 + 0.2 * log10(gain), 0, 1.0);
        break;
    case PAR:
        gain = 1 - sqrt(1 - gain);
        break;
    case IPAR:
        gain = (1 - (1 - gain) * (1 - gain));
        break;
    case QUA:
        gain *= gain;
        break;
    case CUB:
        gain = CUBE(gain);
        break;
    case SQU:
        gain = sqrt(gain);
        break;
    case CBR:
        gain = cbrt(gain);
        break;
    case DESE:
        gain = gain <= 0.5 ? cbrt(2 * gain) / 2: 1 - cbrt(2 * (1 - gain)) / 2;
        break;
    case DESI:
        gain = gain <= 0.5 ? CUBE(2 * gain) / 2: 1 - CUBE(2 * (1 - gain)) / 2;
        break;
    case LOSI: {
                   const double a = 1. / (1. - 0.787) - 1;
                   double A = 1. / (1.0 + exp(0 -((gain-0.5) * a * 2.0)));
                   double B = 1. / (1.0 + exp(a));
                   double C = 1. / (1.0 + exp(0-a));
                   gain = (A - B) / (C - B);
               }
        break;
    case SINC:
        gain = gain >= 1.0 ? 1.0 : sin(M_PI * (1.0 - gain)) / (M_PI * (1.0 - gain));
        break;
    case ISINC:
        gain = gain <= 0.0 ? 0.0 : 1.0 - sin(M_PI * gain) / (M_PI * gain);
        break;
    case QUAT:
        gain = gain * gain * gain * gain;
        break;
    case QUATR:
        gain = pow(gain, 0.25);
        break;
    case QSIN2:
        gain = sin(gain * M_PI / 2.0) * sin(gain * M_PI / 2.0);
        break;
    case HSIN2:
        gain = pow((1.0 - cos(gain * M_PI)) / 2.0, 2.0);
        break;
    case NONE:
        gain = 1.0;
        break;
    }

    return silence + (unity - silence) * gain;
}

#define FADE_PLANAR(name, type)                                             \
static void fade_samples_## name ##p(uint8_t **dst, uint8_t * const *src,   \
                                     int nb_samples, int channels, int dir, \
                                     int64_t start, int64_t range,int curve,\
                                     double silence, double unity)          \
{                                                                           \
    int i, c;                                                               \
                                                                            \
    for (i = 0; i < nb_samples; i++) {                                      \
        double gain = fade_gain(curve, start + i * dir,range,silence,unity);\
        for (c = 0; c < channels; c++) {                                    \
            type *d = (type *)dst[c];                                       \
            const type *s = (type *)src[c];                                 \
                                                                            \
            d[i] = s[i] * gain;                                             \
        }                                                                   \
    }                                                                       \
}

#define FADE(name, type)                                                    \
static void fade_samples_## name (uint8_t **dst, uint8_t * const *src,      \
                                  int nb_samples, int channels, int dir,    \
                                  int64_t start, int64_t range, int curve,  \
                                  double silence, double unity)             \
{                                                                           \
    type *d = (type *)dst[0];                                               \
    const type *s = (type *)src[0];                                         \
    int i, c, k = 0;                                                        \
                                                                            \
    for (i = 0; i < nb_samples; i++) {                                      \
        double gain = fade_gain(curve, start + i * dir,range,silence,unity);\
        for (c = 0; c < channels; c++, k++)                                 \
            d[k] = s[k] * gain;                                             \
    }                                                                       \
}

FADE_PLANAR(dbl, double)
FADE_PLANAR(flt, float)
FADE_PLANAR(s16, int16_t)
FADE_PLANAR(s32, int32_t)

FADE(dbl, double)
FADE(flt, float)
FADE(s16, int16_t)
FADE(s32, int32_t)

#define SCALE_PLANAR(name, type)                                            \
static void scale_samples_## name ##p(uint8_t **dst, uint8_t * const *src,  \
                                     int nb_samples, int channels,          \
                                     double gain)                           \
{                                                                           \
    int i, c;                                                               \
                                                                            \
    for (i = 0; i < nb_samples; i++) {                                      \
        for (c = 0; c < channels; c++) {                                    \
            type *d = (type *)dst[c];                                       \
            const type *s = (type *)src[c];                                 \
                                                                            \
            d[i] = s[i] * gain;                                             \
        }                                                                   \
    }                                                                       \
}

#define SCALE(name, type)                                                   \
static void scale_samples_## name (uint8_t **dst, uint8_t * const *src,     \
                                  int nb_samples, int channels, double gain)\
{                                                                           \
    type *d = (type *)dst[0];                                               \
    const type *s = (type *)src[0];                                         \
    int i, c, k = 0;                                                        \
                                                                            \
    for (i = 0; i < nb_samples; i++) {                                      \
        for (c = 0; c < channels; c++, k++)                                 \
            d[k] = s[k] * gain;                                             \
    }                                                                       \
}

SCALE_PLANAR(dbl, double)
SCALE_PLANAR(flt, float)
SCALE_PLANAR(s16, int16_t)
SCALE_PLANAR(s32, int32_t)

SCALE(dbl, double)
SCALE(flt, float)
SCALE(s16, int16_t)
SCALE(s32, int32_t)

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioFadeContext *s  = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBL:  s->fade_samples = fade_samples_dbl;
                             s->scale_samples = scale_samples_dbl;
                             break;
    case AV_SAMPLE_FMT_DBLP: s->fade_samples = fade_samples_dblp;
                             s->scale_samples = scale_samples_dblp;
                             break;
    case AV_SAMPLE_FMT_FLT:  s->fade_samples = fade_samples_flt;
                             s->scale_samples = scale_samples_flt;
                             break;
    case AV_SAMPLE_FMT_FLTP: s->fade_samples = fade_samples_fltp;
                             s->scale_samples = scale_samples_fltp;
                             break;
    case AV_SAMPLE_FMT_S16:  s->fade_samples = fade_samples_s16;
                             s->scale_samples = scale_samples_s16;
                             break;
    case AV_SAMPLE_FMT_S16P: s->fade_samples = fade_samples_s16p;
                             s->scale_samples = scale_samples_s16p;
                             break;
    case AV_SAMPLE_FMT_S32:  s->fade_samples = fade_samples_s32;
                             s->scale_samples = scale_samples_s32;
                             break;
    case AV_SAMPLE_FMT_S32P: s->fade_samples = fade_samples_s32p;
                             s->scale_samples = scale_samples_s32p;
                             break;
    }

    if (s->duration)
        s->nb_samples = av_rescale(s->duration, outlink->sample_rate, AV_TIME_BASE);
    s->duration = 0;
    if (s->start_time)
        s->start_sample = av_rescale(s->start_time, outlink->sample_rate, AV_TIME_BASE);
    s->start_time = 0;

    return 0;
}

#if CONFIG_AFADE_FILTER

static const AVOption afade_options[] = {
    { "type",         "set the fade direction",                      OFFSET(type),         AV_OPT_TYPE_INT,    {.i64 = 0    }, 0, 1, TFLAGS, .unit = "type" },
    { "t",            "set the fade direction",                      OFFSET(type),         AV_OPT_TYPE_INT,    {.i64 = 0    }, 0, 1, TFLAGS, .unit = "type" },
    { "in",           "fade-in",                                     0,                    AV_OPT_TYPE_CONST,  {.i64 = 0    }, 0, 0, TFLAGS, .unit = "type" },
    { "out",          "fade-out",                                    0,                    AV_OPT_TYPE_CONST,  {.i64 = 1    }, 0, 0, TFLAGS, .unit = "type" },
    { "start_sample", "set number of first sample to start fading",  OFFSET(start_sample), AV_OPT_TYPE_INT64,  {.i64 = 0    }, 0, INT64_MAX, TFLAGS },
    { "ss",           "set number of first sample to start fading",  OFFSET(start_sample), AV_OPT_TYPE_INT64,  {.i64 = 0    }, 0, INT64_MAX, TFLAGS },
    { "nb_samples",   "set number of samples for fade duration",     OFFSET(nb_samples),   AV_OPT_TYPE_INT64,  {.i64 = 44100}, 1, INT64_MAX, TFLAGS },
    { "ns",           "set number of samples for fade duration",     OFFSET(nb_samples),   AV_OPT_TYPE_INT64,  {.i64 = 44100}, 1, INT64_MAX, TFLAGS },
    { "start_time",   "set time to start fading",                    OFFSET(start_time),   AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX, TFLAGS },
    { "st",           "set time to start fading",                    OFFSET(start_time),   AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX, TFLAGS },
    { "duration",     "set fade duration",                           OFFSET(duration),     AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX, TFLAGS },
    { "d",            "set fade duration",                           OFFSET(duration),     AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX, TFLAGS },
    { "curve",        "set fade curve type",                         OFFSET(curve),        AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, TFLAGS, .unit = "curve" },
    { "c",            "set fade curve type",                         OFFSET(curve),        AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, TFLAGS, .unit = "curve" },
    { "nofade",       "no fade; keep audio as-is",                   0,                    AV_OPT_TYPE_CONST,  {.i64 = NONE }, 0, 0, TFLAGS, .unit = "curve" },
    { "tri",          "linear slope",                                0,                    AV_OPT_TYPE_CONST,  {.i64 = TRI  }, 0, 0, TFLAGS, .unit = "curve" },
    { "qsin",         "quarter of sine wave",                        0,                    AV_OPT_TYPE_CONST,  {.i64 = QSIN }, 0, 0, TFLAGS, .unit = "curve" },
    { "esin",         "exponential sine wave",                       0,                    AV_OPT_TYPE_CONST,  {.i64 = ESIN }, 0, 0, TFLAGS, .unit = "curve" },
    { "hsin",         "half of sine wave",                           0,                    AV_OPT_TYPE_CONST,  {.i64 = HSIN }, 0, 0, TFLAGS, .unit = "curve" },
    { "log",          "logarithmic",                                 0,                    AV_OPT_TYPE_CONST,  {.i64 = LOG  }, 0, 0, TFLAGS, .unit = "curve" },
    { "ipar",         "inverted parabola",                           0,                    AV_OPT_TYPE_CONST,  {.i64 = IPAR }, 0, 0, TFLAGS, .unit = "curve" },
    { "qua",          "quadratic",                                   0,                    AV_OPT_TYPE_CONST,  {.i64 = QUA  }, 0, 0, TFLAGS, .unit = "curve" },
    { "cub",          "cubic",                                       0,                    AV_OPT_TYPE_CONST,  {.i64 = CUB  }, 0, 0, TFLAGS, .unit = "curve" },
    { "squ",          "square root",                                 0,                    AV_OPT_TYPE_CONST,  {.i64 = SQU  }, 0, 0, TFLAGS, .unit = "curve" },
    { "cbr",          "cubic root",                                  0,                    AV_OPT_TYPE_CONST,  {.i64 = CBR  }, 0, 0, TFLAGS, .unit = "curve" },
    { "par",          "parabola",                                    0,                    AV_OPT_TYPE_CONST,  {.i64 = PAR  }, 0, 0, TFLAGS, .unit = "curve" },
    { "exp",          "exponential",                                 0,                    AV_OPT_TYPE_CONST,  {.i64 = EXP  }, 0, 0, TFLAGS, .unit = "curve" },
    { "iqsin",        "inverted quarter of sine wave",               0,                    AV_OPT_TYPE_CONST,  {.i64 = IQSIN}, 0, 0, TFLAGS, .unit = "curve" },
    { "ihsin",        "inverted half of sine wave",                  0,                    AV_OPT_TYPE_CONST,  {.i64 = IHSIN}, 0, 0, TFLAGS, .unit = "curve" },
    { "dese",         "double-exponential seat",                     0,                    AV_OPT_TYPE_CONST,  {.i64 = DESE }, 0, 0, TFLAGS, .unit = "curve" },
    { "desi",         "double-exponential sigmoid",                  0,                    AV_OPT_TYPE_CONST,  {.i64 = DESI }, 0, 0, TFLAGS, .unit = "curve" },
    { "losi",         "logistic sigmoid",                            0,                    AV_OPT_TYPE_CONST,  {.i64 = LOSI }, 0, 0, TFLAGS, .unit = "curve" },
    { "sinc",         "sine cardinal function",                      0,                    AV_OPT_TYPE_CONST,  {.i64 = SINC }, 0, 0, TFLAGS, .unit = "curve" },
    { "isinc",        "inverted sine cardinal function",             0,                    AV_OPT_TYPE_CONST,  {.i64 = ISINC}, 0, 0, TFLAGS, .unit = "curve" },
    { "quat",         "quartic",                                     0,                    AV_OPT_TYPE_CONST,  {.i64 = QUAT }, 0, 0, TFLAGS, .unit = "curve" },
    { "quatr",        "quartic root",                                0,                    AV_OPT_TYPE_CONST,  {.i64 = QUATR}, 0, 0, TFLAGS, .unit = "curve" },
    { "qsin2",        "squared quarter of sine wave",                0,                    AV_OPT_TYPE_CONST,  {.i64 = QSIN2}, 0, 0, TFLAGS, .unit = "curve" },
    { "hsin2",        "squared half of sine wave",                   0,                    AV_OPT_TYPE_CONST,  {.i64 = HSIN2}, 0, 0, TFLAGS, .unit = "curve" },
    { "silence",      "set the silence gain",                        OFFSET(silence),      AV_OPT_TYPE_DOUBLE, {.dbl = 0 },    0, 1, TFLAGS },
    { "unity",        "set the unity gain",                          OFFSET(unity),        AV_OPT_TYPE_DOUBLE, {.dbl = 1 },    0, 1, TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(afade);

static av_cold int init(AVFilterContext *ctx)
{
    AudioFadeContext *s = ctx->priv;

    if (INT64_MAX - s->nb_samples < s->start_sample)
        return AVERROR(EINVAL);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    AudioFadeContext *s     = inlink->dst->priv;
    AVFilterLink *outlink   = inlink->dst->outputs[0];
    int nb_samples          = buf->nb_samples;
    AVFrame *out_buf;
    int64_t cur_sample = av_rescale_q(buf->pts, inlink->time_base, (AVRational){1, inlink->sample_rate});

    if (s->unity == 1.0 &&
        ((!s->type && (s->start_sample + s->nb_samples < cur_sample)) ||
         ( s->type && (cur_sample + nb_samples < s->start_sample))))
        return ff_filter_frame(outlink, buf);

    if (av_frame_is_writable(buf)) {
        out_buf = buf;
    } else {
        out_buf = ff_get_audio_buffer(outlink, nb_samples);
        if (!out_buf)
            return AVERROR(ENOMEM);
        av_frame_copy_props(out_buf, buf);
    }

    if ((!s->type && (cur_sample + nb_samples < s->start_sample)) ||
        ( s->type && (s->start_sample + s->nb_samples < cur_sample))) {
        if (s->silence == 0.) {
            av_samples_set_silence(out_buf->extended_data, 0, nb_samples,
                                   out_buf->ch_layout.nb_channels, out_buf->format);
        } else {
            s->scale_samples(out_buf->extended_data, buf->extended_data,
                             nb_samples, buf->ch_layout.nb_channels,
                             s->silence);
        }
    } else if (( s->type && (cur_sample + nb_samples < s->start_sample)) ||
               (!s->type && (s->start_sample + s->nb_samples < cur_sample))) {
        s->scale_samples(out_buf->extended_data, buf->extended_data,
                         nb_samples, buf->ch_layout.nb_channels,
                         s->unity);
    } else {
        int64_t start;

        if (!s->type)
            start = cur_sample - s->start_sample;
        else
            start = s->start_sample + s->nb_samples - cur_sample;

        s->fade_samples(out_buf->extended_data, buf->extended_data,
                        nb_samples, buf->ch_layout.nb_channels,
                        s->type ? -1 : 1, start,
                        s->nb_samples, s->curve, s->silence, s->unity);
    }

    if (buf != out_buf)
        av_frame_free(&buf);

    return ff_filter_frame(outlink, out_buf);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    return config_output(ctx->outputs[0]);
}

static const AVFilterPad avfilter_af_afade_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad avfilter_af_afade_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_afade = {
    .p.name        = "afade",
    .p.description = NULL_IF_CONFIG_SMALL("Fade in/out input audio."),
    .p.priv_class  = &afade_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    .priv_size     = sizeof(AudioFadeContext),
    .init          = init,
    FILTER_INPUTS(avfilter_af_afade_inputs),
    FILTER_OUTPUTS(avfilter_af_afade_outputs),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .process_command = process_command,
};

#endif /* CONFIG_AFADE_FILTER */

#if CONFIG_ACROSSFADE_FILTER

static const AVOption acrossfade_options[] = {
    { "inputs",       "set number of input files to cross fade",       OFFSET(nb_inputs),    AV_OPT_TYPE_INT,    {.i64 = 2},     1, INT32_MAX, FLAGS },
    { "n",            "set number of input files to cross fade",       OFFSET(nb_inputs),    AV_OPT_TYPE_INT,    {.i64 = 2},     1, INT32_MAX, FLAGS },
    { "nb_samples",   "set number of samples for cross fade duration", OFFSET(nb_samples),   AV_OPT_TYPE_INT64,  {.i64 = 44100}, 1, INT64_MAX/2, FLAGS },
    { "ns",           "set number of samples for cross fade duration", OFFSET(nb_samples),   AV_OPT_TYPE_INT64,  {.i64 = 44100}, 1, INT64_MAX/2, FLAGS },
    { "duration",     "set cross fade duration",                       OFFSET(duration),     AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX/2, FLAGS },
    { "d",            "set cross fade duration",                       OFFSET(duration),     AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX/2, FLAGS },
    { "overlap",      "overlap 1st stream end with 2nd stream start",  OFFSET(overlap),      AV_OPT_TYPE_BOOL,   {.i64 = 1    }, 0,  1, FLAGS },
    { "o",            "overlap 1st stream end with 2nd stream start",  OFFSET(overlap),      AV_OPT_TYPE_BOOL,   {.i64 = 1    }, 0,  1, FLAGS },
    { "curve1",       "set fade curve type for 1st stream",            OFFSET(curve),        AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, FLAGS, .unit = "curve" },
    { "c1",           "set fade curve type for 1st stream",            OFFSET(curve),        AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, FLAGS, .unit = "curve" },
    {     "nofade",   "no fade; keep audio as-is",                     0,                    AV_OPT_TYPE_CONST,  {.i64 = NONE }, 0, 0, FLAGS, .unit = "curve" },
    {     "tri",      "linear slope",                                  0,                    AV_OPT_TYPE_CONST,  {.i64 = TRI  }, 0, 0, FLAGS, .unit = "curve" },
    {     "qsin",     "quarter of sine wave",                          0,                    AV_OPT_TYPE_CONST,  {.i64 = QSIN }, 0, 0, FLAGS, .unit = "curve" },
    {     "esin",     "exponential sine wave",                         0,                    AV_OPT_TYPE_CONST,  {.i64 = ESIN }, 0, 0, FLAGS, .unit = "curve" },
    {     "hsin",     "half of sine wave",                             0,                    AV_OPT_TYPE_CONST,  {.i64 = HSIN }, 0, 0, FLAGS, .unit = "curve" },
    {     "log",      "logarithmic",                                   0,                    AV_OPT_TYPE_CONST,  {.i64 = LOG  }, 0, 0, FLAGS, .unit = "curve" },
    {     "ipar",     "inverted parabola",                             0,                    AV_OPT_TYPE_CONST,  {.i64 = IPAR }, 0, 0, FLAGS, .unit = "curve" },
    {     "qua",      "quadratic",                                     0,                    AV_OPT_TYPE_CONST,  {.i64 = QUA  }, 0, 0, FLAGS, .unit = "curve" },
    {     "cub",      "cubic",                                         0,                    AV_OPT_TYPE_CONST,  {.i64 = CUB  }, 0, 0, FLAGS, .unit = "curve" },
    {     "squ",      "square root",                                   0,                    AV_OPT_TYPE_CONST,  {.i64 = SQU  }, 0, 0, FLAGS, .unit = "curve" },
    {     "cbr",      "cubic root",                                    0,                    AV_OPT_TYPE_CONST,  {.i64 = CBR  }, 0, 0, FLAGS, .unit = "curve" },
    {     "par",      "parabola",                                      0,                    AV_OPT_TYPE_CONST,  {.i64 = PAR  }, 0, 0, FLAGS, .unit = "curve" },
    {     "exp",      "exponential",                                   0,                    AV_OPT_TYPE_CONST,  {.i64 = EXP  }, 0, 0, FLAGS, .unit = "curve" },
    {     "iqsin",    "inverted quarter of sine wave",                 0,                    AV_OPT_TYPE_CONST,  {.i64 = IQSIN}, 0, 0, FLAGS, .unit = "curve" },
    {     "ihsin",    "inverted half of sine wave",                    0,                    AV_OPT_TYPE_CONST,  {.i64 = IHSIN}, 0, 0, FLAGS, .unit = "curve" },
    {     "dese",     "double-exponential seat",                       0,                    AV_OPT_TYPE_CONST,  {.i64 = DESE }, 0, 0, FLAGS, .unit = "curve" },
    {     "desi",     "double-exponential sigmoid",                    0,                    AV_OPT_TYPE_CONST,  {.i64 = DESI }, 0, 0, FLAGS, .unit = "curve" },
    {     "losi",     "logistic sigmoid",                              0,                    AV_OPT_TYPE_CONST,  {.i64 = LOSI }, 0, 0, FLAGS, .unit = "curve" },
    {     "sinc",     "sine cardinal function",                        0,                    AV_OPT_TYPE_CONST,  {.i64 = SINC }, 0, 0, FLAGS, .unit = "curve" },
    {     "isinc",    "inverted sine cardinal function",               0,                    AV_OPT_TYPE_CONST,  {.i64 = ISINC}, 0, 0, FLAGS, .unit = "curve" },
    {     "quat",     "quartic",                                       0,                    AV_OPT_TYPE_CONST,  {.i64 = QUAT }, 0, 0, FLAGS, .unit = "curve" },
    {     "quatr",    "quartic root",                                  0,                    AV_OPT_TYPE_CONST,  {.i64 = QUATR}, 0, 0, FLAGS, .unit = "curve" },
    {     "qsin2",    "squared quarter of sine wave",                  0,                    AV_OPT_TYPE_CONST,  {.i64 = QSIN2}, 0, 0, FLAGS, .unit = "curve" },
    {     "hsin2",    "squared half of sine wave",                     0,                    AV_OPT_TYPE_CONST,  {.i64 = HSIN2}, 0, 0, FLAGS, .unit = "curve" },
    { "curve2",       "set fade curve type for 2nd stream",            OFFSET(curve2),       AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, FLAGS, .unit = "curve" },
    { "c2",           "set fade curve type for 2nd stream",            OFFSET(curve2),       AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, FLAGS, .unit = "curve" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(acrossfade);

#define CROSSFADE_PLANAR(name, type)                                           \
static void crossfade_samples_## name ##p(uint8_t **dst, uint8_t * const *cf0, \
                                          uint8_t * const *cf1,                \
                                          int nb_samples, int channels,        \
                                          int curve0, int curve1,              \
                                          int64_t offset, int64_t total)       \
{                                                                              \
    int i, c;                                                                  \
                                                                               \
    for (i = 0; i < nb_samples; i++) {                                         \
        int64_t pos = offset + i;                                              \
        double gain0 = fade_gain(curve0, total - 1 - pos, total, 0., 1.);      \
        double gain1 = fade_gain(curve1, pos, total, 0., 1.);                  \
        for (c = 0; c < channels; c++) {                                       \
            type *d = (type *)dst[c];                                          \
            const type *s0 = (type *)cf0[c];                                   \
            const type *s1 = (type *)cf1[c];                                   \
                                                                               \
            d[i] = s0[i] * gain0 + s1[i] * gain1;                              \
        }                                                                      \
    }                                                                          \
}

#define CROSSFADE(name, type)                                               \
static void crossfade_samples_## name (uint8_t **dst, uint8_t * const *cf0, \
                                       uint8_t * const *cf1,                \
                                       int nb_samples, int channels,        \
                                       int curve0, int curve1,              \
                                       int64_t offset, int64_t total)       \
{                                                                           \
    type *d = (type *)dst[0];                                               \
    const type *s0 = (type *)cf0[0];                                        \
    const type *s1 = (type *)cf1[0];                                        \
    int i, c, k = 0;                                                        \
                                                                            \
    for (i = 0; i < nb_samples; i++) {                                      \
        int64_t pos = offset + i;                                           \
        double gain0 = fade_gain(curve0, total - 1 - pos, total, 0., 1.);   \
        double gain1 = fade_gain(curve1, pos, total, 0., 1.);               \
        for (c = 0; c < channels; c++, k++)                                 \
            d[k] = s0[k] * gain0 + s1[k] * gain1;                           \
    }                                                                       \
}

CROSSFADE_PLANAR(dbl, double)
CROSSFADE_PLANAR(flt, float)
CROSSFADE_PLANAR(s16, int16_t)
CROSSFADE_PLANAR(s32, int32_t)

CROSSFADE(dbl, double)
CROSSFADE(flt, float)
CROSSFADE(s16, int16_t)
CROSSFADE(s32, int32_t)

static int pass_frame(AVFilterLink *inlink, AVFilterLink *outlink, int64_t *pts)
{
    AVFrame *in;
    int ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    av_assert1(ret);
    in->pts = *pts;
    *pts += av_rescale_q(in->nb_samples,
            (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
    return ff_filter_frame(outlink, in);
}

/* Copy samples from frame to ring buffer (circular overwrite) */
static void copy_to_ring_buffer(AudioFadeContext *s, AVFrame *frame, int nb_channels, int is_planar)
{
    int bytes_per_sample = av_get_bytes_per_sample(frame->format);
    /* Clamp to ring buffer size; if frame is larger, keep only the last nb_samples */
    int samples_to_copy = FFMIN(frame->nb_samples, s->nb_samples);
    int src_offset = frame->nb_samples - samples_to_copy;
    int64_t dst_pos = s->ring_write_pos % s->nb_samples;
    int first_chunk = FFMIN(samples_to_copy, s->nb_samples - dst_pos);
    int second_chunk = samples_to_copy - first_chunk;

    if (is_planar) {
        for (int c = 0; c < nb_channels; c++) {
            memcpy(s->ring_buf->extended_data[c] + dst_pos * bytes_per_sample,
                   frame->extended_data[c] + src_offset * bytes_per_sample,
                   first_chunk * bytes_per_sample);
            if (second_chunk > 0)
                memcpy(s->ring_buf->extended_data[c],
                       frame->extended_data[c] + (src_offset + first_chunk) * bytes_per_sample,
                       second_chunk * bytes_per_sample);
        }
    } else {
        int stride = nb_channels * bytes_per_sample;
        memcpy(s->ring_buf->extended_data[0] + dst_pos * stride,
               frame->extended_data[0] + src_offset * stride,
               first_chunk * stride);
        if (second_chunk > 0)
            memcpy(s->ring_buf->extended_data[0],
                   frame->extended_data[0] + (src_offset + first_chunk) * stride,
                   second_chunk * stride);
    }

    s->ring_write_pos += samples_to_copy;
    s->ring_filled = FFMIN(s->ring_filled + samples_to_copy, s->nb_samples);
}

/* Read samples from ring buffer starting at crossfade_pos (circular read) */
static void read_from_ring_buffer(AudioFadeContext *s, uint8_t **dst, int nb_samples,
                                  int nb_channels, int is_planar, int bytes_per_sample)
{
    /* The ring buffer contains the last ring_filled samples from input 0.
     * We need to read starting from crossfade_pos within those samples.
     * ring_write_pos points to where the NEXT write would go, so the oldest
     * valid sample is at (ring_write_pos - ring_filled) % nb_samples */
    int64_t oldest_pos = (s->ring_write_pos - s->ring_filled + s->nb_samples) % s->nb_samples;
    int64_t read_start = (oldest_pos + s->crossfade_pos) % s->nb_samples;
    int first_chunk = FFMIN(nb_samples, s->nb_samples - read_start);
    int second_chunk = nb_samples - first_chunk;

    if (is_planar) {
        for (int c = 0; c < nb_channels; c++) {
            memcpy(dst[c],
                   s->ring_buf->extended_data[c] + read_start * bytes_per_sample,
                   first_chunk * bytes_per_sample);
            if (second_chunk > 0)
                memcpy(dst[c] + first_chunk * bytes_per_sample,
                       s->ring_buf->extended_data[c],
                       second_chunk * bytes_per_sample);
        }
    } else {
        int stride = nb_channels * bytes_per_sample;
        memcpy(dst[0],
               s->ring_buf->extended_data[0] + read_start * stride,
               first_chunk * stride);
        if (second_chunk > 0)
            memcpy(dst[0] + first_chunk * stride,
                   s->ring_buf->extended_data[0],
                   second_chunk * stride);
    }
}

/* Consume (read and remove) samples from the oldest position in ring buffer */
static void consume_from_ring_buffer(AudioFadeContext *s, uint8_t **dst, int nb_samples,
                                     int nb_channels, int is_planar, int bytes_per_sample)
{
    int64_t oldest_pos = (s->ring_write_pos - s->ring_filled + s->nb_samples) % s->nb_samples;
    int first_chunk = FFMIN(nb_samples, s->nb_samples - oldest_pos);
    int second_chunk = nb_samples - first_chunk;

    if (is_planar) {
        for (int c = 0; c < nb_channels; c++) {
            memcpy(dst[c],
                   s->ring_buf->extended_data[c] + oldest_pos * bytes_per_sample,
                   first_chunk * bytes_per_sample);
            if (second_chunk > 0)
                memcpy(dst[c] + first_chunk * bytes_per_sample,
                       s->ring_buf->extended_data[c],
                       second_chunk * bytes_per_sample);
        }
    } else {
        int stride = nb_channels * bytes_per_sample;
        memcpy(dst[0],
               s->ring_buf->extended_data[0] + oldest_pos * stride,
               first_chunk * stride);
        if (second_chunk > 0)
            memcpy(dst[0] + first_chunk * stride,
                   s->ring_buf->extended_data[0],
                   second_chunk * stride);
    }

    s->ring_filled -= nb_samples;
}

/* Copy partial frame data to ring buffer starting at src_offset */
static void copy_partial_to_ring_buffer(AudioFadeContext *s, AVFrame *frame,
                                        int src_offset, int nb_samples,
                                        int nb_channels, int is_planar)
{
    int bytes_per_sample = av_get_bytes_per_sample(frame->format);
    int64_t dst_pos = s->ring_write_pos % s->nb_samples;
    int first_chunk = FFMIN(nb_samples, s->nb_samples - dst_pos);
    int second_chunk = nb_samples - first_chunk;

    if (is_planar) {
        for (int c = 0; c < nb_channels; c++) {
            memcpy(s->ring_buf->extended_data[c] + dst_pos * bytes_per_sample,
                   frame->extended_data[c] + src_offset * bytes_per_sample,
                   first_chunk * bytes_per_sample);
            if (second_chunk > 0)
                memcpy(s->ring_buf->extended_data[c],
                       frame->extended_data[c] + (src_offset + first_chunk) * bytes_per_sample,
                       second_chunk * bytes_per_sample);
        }
    } else {
        int stride = nb_channels * bytes_per_sample;
        memcpy(s->ring_buf->extended_data[0] + dst_pos * stride,
               frame->extended_data[0] + src_offset * stride,
               first_chunk * stride);
        if (second_chunk > 0)
            memcpy(s->ring_buf->extended_data[0],
                   frame->extended_data[0] + (src_offset + first_chunk) * stride,
                   second_chunk * stride);
    }

    s->ring_write_pos += nb_samples;
    s->ring_filled += nb_samples;
}

/* Process crossfade for non-overlap mode (fade-out then fade-in) */
static int process_non_overlap_crossfade(AVFilterContext *ctx, const int idx1)
{
    AudioFadeContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *in1 = ctx->inputs[idx1];
    AVFrame *out, *cf = NULL;
    int ret;

    /* Phase 1: Fade-out from ring buffer */
    if (s->crossfade_pos < s->ring_filled) {
        int64_t remaining = s->ring_filled - s->crossfade_pos;
        int process_samples = FFMIN(remaining, 4096);  /* Process in chunks */
        int bytes_per_sample = av_get_bytes_per_sample(outlink->format);
        int is_planar = av_sample_fmt_is_planar(outlink->format);
        int nb_channels = outlink->ch_layout.nb_channels;

        out = ff_get_audio_buffer(outlink, process_samples);
        if (!out)
            return AVERROR(ENOMEM);

        /* Allocate temp buffer for ring buffer read */
        AVFrame *temp = ff_get_audio_buffer(outlink, process_samples);
        if (!temp) {
            av_frame_free(&out);
            return AVERROR(ENOMEM);
        }

        read_from_ring_buffer(s, temp->extended_data, process_samples,
                              nb_channels, is_planar, bytes_per_sample);

        /* Apply fade-out */
        s->fade_samples(out->extended_data, temp->extended_data, process_samples,
                        nb_channels, -1, s->ring_filled - 1 - s->crossfade_pos,
                        s->ring_filled, s->curve, 0., 1.);

        s->crossfade_pos += process_samples;
        out->pts = s->pts;
        s->pts += av_rescale_q(process_samples,
            (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
        av_frame_free(&temp);
        return ff_filter_frame(outlink, out);
    }

    /* Phase 2: Fade-in from input 1 */
    if (!ff_inlink_queued_samples(in1)) {
        if (ff_outlink_get_status(in1))
            return 0;  /* Input 1 is empty */
        FF_FILTER_FORWARD_WANTED(outlink, in1);
        return FFERROR_NOT_READY;
    }

    ret = ff_inlink_consume_frame(in1, &cf);
    if (ret < 0)
        return ret;
    if (!ret) {
        FF_FILTER_FORWARD_WANTED(outlink, in1);
        return FFERROR_NOT_READY;
    }

    int64_t fadein_pos = s->crossfade_pos - s->ring_filled;  /* Position in fade-in */
    int64_t fadein_remaining = s->nb_samples - fadein_pos;

    if (fadein_pos < s->nb_samples && fadein_remaining > 0) {
        int process_samples = FFMIN(cf->nb_samples, fadein_remaining);

        out = ff_get_audio_buffer(outlink, cf->nb_samples);
        if (!out) {
            av_frame_free(&cf);
            return AVERROR(ENOMEM);
        }

        /* Apply fade-in to the portion within crossfade region */
        s->fade_samples(out->extended_data, cf->extended_data, process_samples,
                        outlink->ch_layout.nb_channels, 1, fadein_pos,
                        s->nb_samples, s->curve2, 0., 1.);

        /* Copy remainder unchanged if frame extends past crossfade */
        if (cf->nb_samples > process_samples) {
            int bytes_per_sample = av_get_bytes_per_sample(outlink->format);
            int is_planar = av_sample_fmt_is_planar(outlink->format);
            int nb_channels = outlink->ch_layout.nb_channels;

            if (is_planar) {
                for (int c = 0; c < nb_channels; c++) {
                    memcpy(out->extended_data[c] + process_samples * bytes_per_sample,
                           cf->extended_data[c] + process_samples * bytes_per_sample,
                           (cf->nb_samples - process_samples) * bytes_per_sample);
                }
            } else {
                memcpy(out->extended_data[0] + process_samples * nb_channels * bytes_per_sample,
                       cf->extended_data[0] + process_samples * nb_channels * bytes_per_sample,
                       (cf->nb_samples - process_samples) * nb_channels * bytes_per_sample);
            }
        }

        s->crossfade_pos += cf->nb_samples;
        out->pts = s->pts;
        s->pts += av_rescale_q(cf->nb_samples,
            (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
        av_frame_free(&cf);

        /* Check if crossfade is complete */
        if (s->crossfade_pos >= s->ring_filled + s->nb_samples) {
            s->crossfade_active = 0;
        }

        return ff_filter_frame(outlink, out);
    }

    /* Past crossfade region - pass through */
    s->crossfade_active = 0;
    cf->pts = s->pts;
    s->pts += av_rescale_q(cf->nb_samples,
            (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
    return ff_filter_frame(outlink, cf);
}

/* Process one frame of overlapping crossfade using ring buffer + input 1 */
static int process_overlap_crossfade(AVFilterContext *ctx, const int idx1)
{
    AudioFadeContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *in1 = ctx->inputs[idx1];
    AVFrame *out, *cf1 = NULL;
    int ret;

    /* Check if crossfade is complete */
    if (s->crossfade_pos >= s->ring_filled) {
        s->crossfade_active = 0;
        return 0;
    }

    /* Get frame from input 1 */
    if (!ff_inlink_queued_samples(in1)) {
        if (ff_outlink_get_status(in1)) {
            /* Input 1 ended early - output remaining ring buffer with fade-out */
            int64_t remaining = s->ring_filled - s->crossfade_pos;
            if (remaining <= 0) {
                s->crossfade_active = 0;
                return 0;
            }
            int process_samples = FFMIN(remaining, 4096);
            int bytes_per_sample = av_get_bytes_per_sample(outlink->format);
            int is_planar = av_sample_fmt_is_planar(outlink->format);
            int nb_channels = outlink->ch_layout.nb_channels;

            out = ff_get_audio_buffer(outlink, process_samples);
            if (!out)
                return AVERROR(ENOMEM);

            AVFrame *temp = ff_get_audio_buffer(outlink, process_samples);
            if (!temp) {
                av_frame_free(&out);
                return AVERROR(ENOMEM);
            }

            read_from_ring_buffer(s, temp->extended_data, process_samples,
                                  nb_channels, is_planar, bytes_per_sample);

            s->fade_samples(out->extended_data, temp->extended_data, process_samples,
                            nb_channels, -1, s->ring_filled - 1 - s->crossfade_pos,
                            s->ring_filled, s->curve, 0., 1.);

            s->crossfade_pos += process_samples;
            out->pts = s->pts;
            s->pts += av_rescale_q(process_samples,
                (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
            av_frame_free(&temp);
            return ff_filter_frame(outlink, out);
        }
        FF_FILTER_FORWARD_WANTED(outlink, in1);
        return FFERROR_NOT_READY;
    }

    ret = ff_inlink_consume_frame(in1, &cf1);
    if (ret < 0)
        return ret;
    if (!ret) {
        FF_FILTER_FORWARD_WANTED(outlink, in1);
        return FFERROR_NOT_READY;
    }

    int64_t remaining_crossfade = s->ring_filled - s->crossfade_pos;
    int crossfade_samples = FFMIN(cf1->nb_samples, remaining_crossfade);
    int passthrough_samples = cf1->nb_samples - crossfade_samples;
    int bytes_per_sample = av_get_bytes_per_sample(outlink->format);
    int is_planar = av_sample_fmt_is_planar(outlink->format);
    int nb_channels = outlink->ch_layout.nb_channels;

    out = ff_get_audio_buffer(outlink, cf1->nb_samples);
    if (!out) {
        av_frame_free(&cf1);
        return AVERROR(ENOMEM);
    }

    if (crossfade_samples > 0) {
        /* Allocate temp buffer for ring buffer samples */
        AVFrame *temp = ff_get_audio_buffer(outlink, crossfade_samples);
        if (!temp) {
            av_frame_free(&out);
            av_frame_free(&cf1);
            return AVERROR(ENOMEM);
        }

        read_from_ring_buffer(s, temp->extended_data, crossfade_samples,
                              nb_channels, is_planar, bytes_per_sample);

        /* Apply crossfade */
        s->crossfade_samples(out->extended_data, temp->extended_data,
                             cf1->extended_data, crossfade_samples,
                             nb_channels, s->curve, s->curve2,
                             s->crossfade_pos, s->ring_filled);

        av_frame_free(&temp);
    }

    /* Copy any passthrough samples after crossfade region */
    if (passthrough_samples > 0) {
        if (is_planar) {
            for (int c = 0; c < nb_channels; c++) {
                memcpy(out->extended_data[c] + crossfade_samples * bytes_per_sample,
                       cf1->extended_data[c] + crossfade_samples * bytes_per_sample,
                       passthrough_samples * bytes_per_sample);
            }
        } else {
            memcpy(out->extended_data[0] + crossfade_samples * nb_channels * bytes_per_sample,
                   cf1->extended_data[0] + crossfade_samples * nb_channels * bytes_per_sample,
                   passthrough_samples * nb_channels * bytes_per_sample);
        }
    }

    s->crossfade_pos += crossfade_samples;
    out->pts = s->pts;
    s->pts += av_rescale_q(cf1->nb_samples,
        (AVRational){ 1, outlink->sample_rate }, outlink->time_base);

    av_frame_free(&cf1);

    /* Check if crossfade is complete */
    if (s->crossfade_pos >= s->ring_filled) {
        s->crossfade_active = 0;
    }

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AudioFadeContext *s   = ctx->priv;
    const int idx0        = s->xfade_idx;
    const int idx1        = s->xfade_idx + 1;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *in0     = ctx->inputs[idx0];

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    /* Last active input - just pass through */
    if (idx0 == s->nb_inputs - 1) {
        if (ff_inlink_queued_frames(in0))
            return pass_frame(in0, outlink, &s->pts);
        FF_FILTER_FORWARD_STATUS(in0, outlink);
        FF_FILTER_FORWARD_WANTED(outlink, in0);
        return FFERROR_NOT_READY;
    }

    AVFilterLink *in1 = ctx->inputs[idx1];

    /* If crossfade is active, process it */
    if (s->crossfade_active) {
        int ret;
        if (s->overlap) {
            ret = process_overlap_crossfade(ctx, idx1);
        } else {
            ret = process_non_overlap_crossfade(ctx, idx1);
        }

        if (ret < 0)
            return ret;

        /* If crossfade completed, move to next input pair */
        if (!s->crossfade_active) {
            s->xfade_idx++;
            s->crossfade_pos = 0;
            s->ring_filled = 0;
            s->ring_write_pos = 0;
            ff_filter_set_ready(ctx, 10);
        }
        return ret;
    }

    /* Allocate ring buffer if needed */
    if (!s->ring_buf) {
        s->ring_buf = ff_get_audio_buffer(outlink, s->nb_samples);
        if (!s->ring_buf)
            return AVERROR(ENOMEM);
    }

    /* Check if input 0 has reached EOF */
    int in0_eof = ff_outlink_get_status(in0);

    if (!in0_eof) {
        /* Still receiving from input 0 */
        if (ff_inlink_queued_frames(in0)) {
            AVFrame *frame;
            int ret = ff_inlink_consume_frame(in0, &frame);
            if (ret < 0)
                return ret;
            if (ret > 0) {
                int bytes_per_sample = av_get_bytes_per_sample(outlink->format);
                int is_planar = av_sample_fmt_is_planar(outlink->format);
                int nb_channels = outlink->ch_layout.nb_channels;

                if (s->overlap) {
                    /* For overlap mode: delay output by nb_samples.
                     * We buffer samples in ring_buf and only output when we have
                     * more than nb_samples buffered (the excess is safe to output).
                     *
                     * Strategy:
                     * 1. Add new frame samples to ring buffer
                     * 2. If ring buffer has more than nb_samples, output the excess
                     * 3. Keep exactly nb_samples in ring buffer for crossfade
                     */
                    int64_t total_after_add = s->ring_filled + frame->nb_samples;

                    if (total_after_add <= s->nb_samples) {
                        /* Still filling up - just buffer, don't output */
                        copy_to_ring_buffer(s, frame, nb_channels, is_planar);
                        av_frame_free(&frame);
                        return 0;
                    } else {
                        /* We have excess samples to output */
                        int64_t excess = total_after_add - s->nb_samples;

                        /* The excess comes from the oldest samples in ring buffer
                         * plus potentially some from the new frame */
                        int64_t from_ring = FFMIN(excess, s->ring_filled);
                        int64_t from_frame = excess - from_ring;

                        if (excess > 0) {
                            AVFrame *out = ff_get_audio_buffer(outlink, excess);
                            if (!out) {
                                av_frame_free(&frame);
                                return AVERROR(ENOMEM);
                            }

                            /* Copy from ring buffer first */
                            if (from_ring > 0) {
                                consume_from_ring_buffer(s, out->extended_data, from_ring,
                                                         nb_channels, is_planar, bytes_per_sample);
                            }

                            /* Copy from new frame */
                            if (from_frame > 0) {
                                if (is_planar) {
                                    for (int c = 0; c < nb_channels; c++) {
                                        memcpy(out->extended_data[c] + from_ring * bytes_per_sample,
                                               frame->extended_data[c],
                                               from_frame * bytes_per_sample);
                                    }
                                } else {
                                    memcpy(out->extended_data[0] + from_ring * nb_channels * bytes_per_sample,
                                           frame->extended_data[0],
                                           from_frame * nb_channels * bytes_per_sample);
                                }
                            }

                            out->pts = s->pts;
                            s->pts += av_rescale_q(excess,
                                (AVRational){ 1, outlink->sample_rate }, outlink->time_base);

                            /* Now add remaining samples from frame to ring buffer */
                            int remaining = frame->nb_samples - from_frame;
                            if (remaining > 0) {
                                copy_partial_to_ring_buffer(s, frame, from_frame, remaining,
                                                            nb_channels, is_planar);
                            }

                            av_frame_free(&frame);
                            return ff_filter_frame(outlink, out);
                        }
                    }
                } else {
                    /* Non-overlap mode: pass through immediately, keep copy in ring buffer */
                    copy_to_ring_buffer(s, frame, nb_channels, is_planar);

                    frame->pts = s->pts;
                    s->pts += av_rescale_q(frame->nb_samples,
                        (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
                    return ff_filter_frame(outlink, frame);
                }
            }
        }
        FF_FILTER_FORWARD_WANTED(outlink, in0);
        return FFERROR_NOT_READY;
    }

    /* Input 0 has reached EOF - start crossfade */
    if (!s->crossfade_active) {
        /* Handle case where input 0 was shorter than crossfade duration */
        if (s->ring_filled < s->nb_samples && s->ring_filled > 0) {
            av_log(ctx, AV_LOG_WARNING, "Input %d duration (%"PRId64" samples) "
                   "is shorter than crossfade duration (%"PRId64" samples), "
                   "crossfade will be shorter.\n",
                   idx0, s->ring_filled, s->nb_samples);
            if (s->ring_filled < 100)
                av_log(ctx, AV_LOG_WARNING, "Very short crossfade (%"PRId64" samples) "
                       "may cause audible artifacts.\n", s->ring_filled);
        }

        if (s->ring_filled == 0) {
            /* Input 0 was empty, skip to next */
            s->xfade_idx++;
            ff_filter_set_ready(ctx, 10);
            return 0;
        }

        s->crossfade_active = 1;
        s->crossfade_pos = 0;
        ff_filter_set_ready(ctx, 10);
    }

    /* Process crossfade */
    if (s->overlap) {
        return process_overlap_crossfade(ctx, idx1);
    } else {
        return process_non_overlap_crossfade(ctx, idx1);
    }
}

static av_cold int acrossfade_init(AVFilterContext *ctx)
{
    AudioFadeContext *s = ctx->priv;
    int ret;

    for (int i = 0; i < s->nb_inputs; i++) {
        AVFilterPad pad = {
            .name = av_asprintf("crossfade%d", i),
            .type = AVMEDIA_TYPE_AUDIO,
        };
        if (!pad.name)
            return AVERROR(ENOMEM);

        ret = ff_append_inpad_free_name(ctx, &pad);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static av_cold void acrossfade_uninit(AVFilterContext *ctx)
{
    AudioFadeContext *s = ctx->priv;
    av_frame_free(&s->ring_buf);
}

static int acrossfade_config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioFadeContext *s  = ctx->priv;

    outlink->time_base   = ctx->inputs[0]->time_base;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBL:  s->crossfade_samples = crossfade_samples_dbl;  break;
    case AV_SAMPLE_FMT_DBLP: s->crossfade_samples = crossfade_samples_dblp; break;
    case AV_SAMPLE_FMT_FLT:  s->crossfade_samples = crossfade_samples_flt;  break;
    case AV_SAMPLE_FMT_FLTP: s->crossfade_samples = crossfade_samples_fltp; break;
    case AV_SAMPLE_FMT_S16:  s->crossfade_samples = crossfade_samples_s16;  break;
    case AV_SAMPLE_FMT_S16P: s->crossfade_samples = crossfade_samples_s16p; break;
    case AV_SAMPLE_FMT_S32:  s->crossfade_samples = crossfade_samples_s32;  break;
    case AV_SAMPLE_FMT_S32P: s->crossfade_samples = crossfade_samples_s32p; break;
    }

    config_output(outlink);

    return 0;
}

static const AVFilterPad avfilter_af_acrossfade_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = acrossfade_config_output,
    },
};

const FFFilter ff_af_acrossfade = {
    .p.name        = "acrossfade",
    .p.description = NULL_IF_CONFIG_SMALL("Cross fade two input audio streams."),
    .p.priv_class  = &acrossfade_class,
    .p.flags       = AVFILTER_FLAG_DYNAMIC_INPUTS,
    .priv_size     = sizeof(AudioFadeContext),
    .init          = acrossfade_init,
    .uninit        = acrossfade_uninit,
    .activate      = activate,
    FILTER_OUTPUTS(avfilter_af_acrossfade_outputs),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
};

#endif /* CONFIG_ACROSSFADE_FILTER */
