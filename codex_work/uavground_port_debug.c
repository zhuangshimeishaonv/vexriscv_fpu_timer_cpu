
/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */
 

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdint.h>
#include "uart.h"
#include "axi_dma.h"
#include "cordic_math.h"
#define EXP 2.71828
#define _CRT_SECURE_NO_WARNINGS 1
#define MAX_PKT_LEN 1600 //一次突发传输的数据个数

#define pi 3.141592653589793
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline uint32_t read_cycle32(void) {
    uint32_t cycles;
    __asm__ volatile ("csrr %0, mcycle" : "=r"(cycles));
    return cycles;
}

extern float randn_gen_number[1600000];
int randn_gen_count = 0;
extern float rand_gen_number[160000];
int rand_gen_count = 0;

#ifndef USE_EXACT_CHANNEL_MODEL
#define USE_EXACT_CHANNEL_MODEL 1
#endif

#ifndef USE_FAST_SEPARABLE_LSP_FILTER
#define USE_FAST_SEPARABLE_LSP_FILTER 0
#endif

#define LN10_F 2.302585092994046f
static inline float pow10f_equiv(float x) {
    /* pow(10.0f, x) 的等价形式，避免调用通用 powf，软浮点下更快。 */
    return exp(LN10_F * x);
}

static inline float fast_exp2f_approx(float x) {
    if (x < -126.0f) {
        return 0.0f;
    }
    if (x > 127.0f) {
        x = 127.0f;
    }

    int ip = (int)x;
    if (x < 0.0f && (float)ip != x) {
        --ip;
    }
    const float f = x - (float)ip;
    const float f2 = f * f;
    const float f3 = f2 * f;
    const float f4 = f3 * f;
    const float poly = 1.0f + 0.69314718f * f + 0.24022651f * f2 + 0.05550411f * f3 + 0.00961813f * f4;

    union { uint32_t u; float f; } scale;
    scale.u = (uint32_t)(ip + 127) << 23;
    return scale.f * poly;
}

static inline float fast_pow10f_approx(float x) {
    return fast_exp2f_approx(3.32192809489f * x);
}


static inline void rand_normal_pair_fast(float* z0, float* z1) {
    *z0 = randn_gen_number[randn_gen_count++];
    *z1 = randn_gen_number[randn_gen_count++];
}

static inline float stage4_pow10_fast(float x) {
    return fast_pow10f_approx(x);
}

static inline float stage4_pow10_exact(float x) {
    return pow(10.0f, x);
}

static inline float stage4_pow10_model(float x) {
#if USE_EXACT_CHANNEL_MODEL
    return stage4_pow10_exact(x);
#else
    return stage4_pow10_fast(x);
#endif
}

static const float lsp_w_h20[20] = {
    0.0f, 0.05263158f, 0.10526316f, 0.15789474f, 0.21052632f,
    0.26315789f, 0.31578947f, 0.36842105f, 0.42105263f, 0.47368421f,
    0.52631579f, 0.57894737f, 0.63157895f, 0.68421053f, 0.73684211f,
    0.78947368f, 0.84210526f, 0.89473684f, 0.94736842f, 1.0f
};

static const float lsp_w_l20[20] = {
    1.0f, 0.94736842f, 0.89473684f, 0.84210526f, 0.78947368f,
    0.73684211f, 0.68421053f, 0.63157895f, 0.57894737f, 0.52631579f,
    0.47368421f, 0.42105263f, 0.36842105f, 0.31578947f, 0.26315789f,
    0.21052632f, 0.15789474f, 0.10526316f, 0.05263158f, 0.0f
};

#define MAX_CLUSTERS 20  //Nc0
#define ADD_CLUSTERS 200  //新簇的预分配 消失的簇数量不减
#define CLUSTER_RAYS 10// n[k] = (int)(1/ Nc0 / sum_weightT * para.NumRay); Nc0=20  para.NumRay=200  sum_weightT=1
#define CORDIC_DRIVER_MAX_BATCH 4095
#define TOTAL_CLUSTERS (MAX_CLUSTERS + ADD_CLUSTERS)
#define OUTPUT_NORMAL_CLUSTERS 39
#define OUTPUT_DUCT_CLUSTERS (TOTAL_CLUSTERS - OUTPUT_NORMAL_CLUSTERS)
#define MAX_TIME 141//T_length
#define MAX_MT  1 //发射天线数量
#define MAX_MR  1

    float sin_array[40][40];    // 存储所有 sin_val（fix32_30→float）
    float cos_array[40][40];    // 存储所有 cos_val（fix32_30→float）
    uint32_t delay_array[40][40];// 存储所有 delay_fix（原始 uint32_t）
    typedef struct {
    	float real;
    	float image;
    } complex;
complex H_FPGA[MAX_TIME][220];//存储响应
complex H_FPGA_duct[MAX_TIME][220];//存储响应
complex H[MAX_TIME][220];//存储响应
static int active_cluster_count[MAX_TIME];
static int active_cluster_count_duct[MAX_TIME];
typedef struct {
    int M_T;
    int nLayer;
    int M_R;
    float fc;
    int c;
    float lambda;
    int rand_seed;
    int randn_seed;
    float D_g;
    int square[2];
    int altitudes;
    int delet_altitude;
    // char r_track_type[10];
    int r_track_type;    
    float r_track_length;
    float r_speed;
    float r_accel;
    float r_samp_rate;
    float r_ini_pos[3];
    float r_ini_dir[3];
    // char t_track_type[10];
    int t_track_type;
    float t_track_length;
    float t_speed;
    float t_accel;
    float t_samp_rate;
    float t_ini_pos[3];
    float t_ini_dir[3];
    float t_distance;
    //float D_pr;
    float Corr_distance;
    float nu_A_T;
    float nu_E_T;
    float nu_A_R;
    float nu_E_R;
    float theta_v_R;
    float xi_v_R;
    float theta_v_T;
    float xi_v_T;
    float delta_tx;
    float delta_rx;
    float K_ricean_db_h_M;
    float K_ricean_db_l_M;
    float K_ricean_lin;
    float lambdaG;
    float lambdaR;
    float DS_miu_h;
    float DS_sigma_h;
    float DS_miu_l;
    float DS_sigma_l;
    float DS_r;
    float d_Rx_min;
    float d_Rx_mean;
    float d_Tx_min;
    float d_Tx_mean;
    float tau_tilder_mean;
    float tau_tilder_min;
    float mean_num_subpath;
    float mean_delay_subpath;
    float angular_variance_subpath;
    float A_Corr_distance;
    float T_Corr_distance;
    float T0;
    float Tend;
    float dT;
    float TimeLine[MAX_TIME];
    int T_length;
    float P_F;
    float v_F;
    float v_c;
    float v_R;
    float v_T;
    float Per_cluster_shadowing;
    float Polarization_Tx;
    float Polarization_Rx;
    float Polar_kappa;
    float Polar_kappa_linear;
    float (*FieldPatternTx)(float, float);
    float (*FieldPatternRx)(float, float);
    float Tx_Rot_x;
    float Tx_Rot_y;
    float Tx_Rot_z;
    float Rx_Rot_x;
    float Rx_Rot_y;
    float Rx_Rot_z;
    float Tsigmax;
    float Tsigmay;
    float Tsigmaz;
    float Rsigmax;
    float Rsigmay;
    float Rsigmaz;
    float sigmaxvar;
    float sigmayvar;
    float sigmazvar;
    float DS_l;
    float DS_h;
} Parameters;
// 声明全局变量 para
Parameters para;

// 大尺度参数矩阵固定为 init_parameters() 中的 200 x 200 x 20。
// 这些数组不能放在 main() 局部栈上：单个 float[200*200*20] 就约 3.2MB，
// linker 默认栈只有 512KB，放栈上会在 [3/7] 后直接栈溢出/跑飞。
#define LSP_ROWS 200
#define LSP_COLS 200
#define LSP_ALTITUDES 20
#define LSP_SQUARE_TOTAL (LSP_ROWS * LSP_COLS)
#define LSP_TOTAL (LSP_SQUARE_TOTAL * LSP_ALTITUDES)

static float K_ricean_db_l_input[LSP_SQUARE_TOTAL];
static float K_ricean_db_h_input[LSP_SQUARE_TOTAL];
static float DS_h_input[LSP_SQUARE_TOTAL];
static float DS_l_input[LSP_SQUARE_TOTAL];
static float AAoA_variance_l_input[LSP_SQUARE_TOTAL];
static float AAoD_variance_l_input[LSP_SQUARE_TOTAL];
static float EAoA_variance_l_input[LSP_SQUARE_TOTAL];
static float EAoD_variance_l_input[LSP_SQUARE_TOTAL];
static float AAoA_variance_h_input[LSP_SQUARE_TOTAL];
static float AAoD_variance_h_input[LSP_SQUARE_TOTAL];
static float EAoA_variance_h_input[LSP_SQUARE_TOTAL];
static float EAoD_variance_h_input[LSP_SQUARE_TOTAL];

static float K_ricean_db_l[LSP_SQUARE_TOTAL];
static float K_ricean_db_h[LSP_SQUARE_TOTAL];
static float K_ricean_db[LSP_TOTAL];
static float K_ricean_lin[LSP_TOTAL];

static float DS_h[LSP_SQUARE_TOTAL];
static float DS_l[LSP_SQUARE_TOTAL];
static float DS[LSP_TOTAL];

static float AAoA_variance_l[LSP_SQUARE_TOTAL];
static float AAoD_variance_l[LSP_SQUARE_TOTAL];
static float EAoA_variance_l[LSP_SQUARE_TOTAL];
static float EAoD_variance_l[LSP_SQUARE_TOTAL];
static float AAoA_variance_h[LSP_SQUARE_TOTAL];
static float AAoD_variance_h[LSP_SQUARE_TOTAL];
static float EAoA_variance_h[LSP_SQUARE_TOTAL];
static float EAoD_variance_h[LSP_SQUARE_TOTAL];

static float AAoA_variance[LSP_TOTAL];
static float AAoD_variance[LSP_TOTAL];
static float EAoA_variance[LSP_TOTAL];
static float EAoD_variance[LSP_TOTAL];

static float lsp_filter_tmp0[LSP_SQUARE_TOTAL];
static float lsp_filter_tmp1[LSP_SQUARE_TOTAL];
static float lsp_filter_tmp2[LSP_SQUARE_TOTAL];
static float lsp_filter_tmp3[LSP_SQUARE_TOTAL];
static float lsp_filter_tmp4[LSP_SQUARE_TOTAL];
static float lsp_filter_tmp5[LSP_SQUARE_TOTAL];
static float lsp_filter_tmp6[LSP_SQUARE_TOTAL];
static float lsp_filter_tmp7[LSP_SQUARE_TOTAL];

float InitPhi[(MAX_CLUSTERS + ADD_CLUSTERS) * CLUSTER_RAYS];
float InitPhi_duct[(MAX_CLUSTERS + ADD_CLUSTERS) * CLUSTER_RAYS];
float new_InitPhi[MAX_TIME][(MAX_CLUSTERS + ADD_CLUSTERS) * CLUSTER_RAYS];
float new_InitPhi_duct[MAX_TIME][(MAX_CLUSTERS + ADD_CLUSTERS) * CLUSTER_RAYS];
float Pow_FPGA[MAX_TIME][(MAX_CLUSTERS + ADD_CLUSTERS)];
float Pow_FPGA_duct[MAX_TIME][(MAX_CLUSTERS + ADD_CLUSTERS)];
float p1[(MAX_CLUSTERS + ADD_CLUSTERS)];//簇功率
float p1_duct[(MAX_CLUSTERS + ADD_CLUSTERS)];//簇功率
float iniphi[(MAX_CLUSTERS + ADD_CLUSTERS)][10];
float iniphi_duct[(MAX_CLUSTERS + ADD_CLUSTERS)][10];
float P1[MAX_TIME][(MAX_CLUSTERS + ADD_CLUSTERS)];
float P1_duct[MAX_TIME][(MAX_CLUSTERS + ADD_CLUSTERS)];
int pto;//有效功率簇索引
int cto;//有效相位簇索引
int pto_duct;//有效功率簇索引
int cto_duct;//有效相位簇索引

#define CHANNEL_DELAY_SLOTS (MAX_MT * MAX_MR * MAX_TIME * TOTAL_CLUSTERS)
#define CHANNEL_RAY_SLOTS (TOTAL_CLUSTERS * CLUSTER_RAYS)
#define CHANNEL_POS_SLOTS (TOTAL_CLUSTERS * 3)

static float Delay_t_storage[CHANNEL_DELAY_SLOTS];
static float D_Sum_storage[CHANNEL_DELAY_SLOTS];
static float Delay_t_permuted_storage[CHANNEL_DELAY_SLOTS];
static float Delay_t_duct_storage[CHANNEL_DELAY_SLOTS];
static float D_Sum_duct_storage[CHANNEL_DELAY_SLOTS];
static float Delay_t_permuted_duct_storage[CHANNEL_DELAY_SLOTS];

static float new_azimuthT_storage[CHANNEL_RAY_SLOTS];
static float new_elevationT_storage[CHANNEL_RAY_SLOTS];
static float new_rT_storage[CHANNEL_RAY_SLOTS];
static float new_azimuthR_storage[CHANNEL_RAY_SLOTS];
static float new_elevationR_storage[CHANNEL_RAY_SLOTS];
static float new_rR_storage[CHANNEL_RAY_SLOTS];
static float new_azimuthT_duct_storage[CHANNEL_RAY_SLOTS];
static float new_elevationT_duct_storage[CHANNEL_RAY_SLOTS];
static float new_rT_duct_storage[CHANNEL_RAY_SLOTS];
static float new_azimuthR_duct_storage[CHANNEL_RAY_SLOTS];
static float new_elevationR_duct_storage[CHANNEL_RAY_SLOTS];
static float new_rR_duct_storage[CHANNEL_RAY_SLOTS];

static float new_Delay_storage[TOTAL_CLUSTERS];
static float new_tau_tilder_storage[TOTAL_CLUSTERS];
static float new_ini_pos_t_storage[CHANNEL_POS_SLOTS];
static float new_ini_pos_r_storage[CHANNEL_POS_SLOTS];
static float new_Delay_duct_storage[TOTAL_CLUSTERS];
static float new_tau_tilder_duct_storage[TOTAL_CLUSTERS];
static float new_ini_pos_t_duct_storage[CHANNEL_POS_SLOTS];
static float new_ini_pos_r_duct_storage[CHANNEL_POS_SLOTS];

static float d_t_storage[MAX_TIME * TOTAL_CLUSTERS];
static float d_time_storage[MAX_TIME * TOTAL_CLUSTERS];
static float d_t_duct_storage[MAX_TIME * TOTAL_CLUSTERS];
static float d_time_duct_storage[MAX_TIME * TOTAL_CLUSTERS];
static float D_output_storage[MAX_TIME][TOTAL_CLUSTERS];

#define PL_CONV_BASE_ADDR 0x42000000u
#define PL_DMA_BASE_ADDR  0x41F00000u

#define PL_CONV_REG_CTRL                 0x00u
#define PL_CONV_REG_X_LEN                0x10u
#define PL_CONV_REG_Y_LEN                0x14u
#define PL_CONV_REG_SAMPLES_PER_SNAPSHOT 0x18u
#define PL_CONV_REG_NUM_SNAPSHOTS        0x1cu
#define PL_CONV_REG_NUM_PATHS            0x20u

#define PL_DMA_MM2S_DMACR   (PL_DMA_BASE_ADDR + 0x00u)
#define PL_DMA_MM2S_DMASR   (PL_DMA_BASE_ADDR + 0x04u)
#define PL_DMA_MM2S_SA      (PL_DMA_BASE_ADDR + 0x18u)
#define PL_DMA_MM2S_LENGTH  (PL_DMA_BASE_ADDR + 0x28u)
#define PL_DMA_S2MM_DMACR   (PL_DMA_BASE_ADDR + 0x30u)
#define PL_DMA_S2MM_DMASR   (PL_DMA_BASE_ADDR + 0x34u)
#define PL_DMA_S2MM_DA      (PL_DMA_BASE_ADDR + 0x48u)
#define PL_DMA_S2MM_LENGTH  (PL_DMA_BASE_ADDR + 0x58u)

#define PL_DMA_CR_RUNSTOP 0x00000001u
#define PL_DMA_CR_RESET   0x00000004u
#define PL_DMA_IRQ_IOC    0x00001000u
#define PL_DMA_IRQ_ERR    0x00004000u
#define PL_DMA_IRQ_ALL    0x00007000u
#define PL_DMA_TIMEOUT    10000000u

#define PL_CONV_MAX_X_LEN       2048
#define PL_CONV_MAX_Y_LEN       6144
#define PL_CONV_MAX_SNAPSHOTS   4
#define PL_CONV_MAX_PATHS       80
#define PL_CONV_INVALID_DELAY   0xffffu
#define PL_CONV_Q15             32767.0f
#define PL_CONV_Q15_INV         (1.0f / 32768.0f)
#define PL_DEMO_SPS             32
#define PL_DEMO_MAX_TIME        40
#define PL_DEMO_MAX_PRINT       32

typedef struct {
    int used_paths;
    int invalid_paths;
    int clipped_delay_paths;
    int max_delay_samples;
    uint32_t pack_cycles;
    uint32_t dma_setup_cycles;
    uint32_t dma_wait_cycles;
    uint32_t unpack_cycles;
    uint32_t total_cycles;
    int tx_words;
    int y_len;
    float h_scale;
    float max_abs_err;
    float rmse;
    float nmse_db;
} pl_conv_stats_t;

typedef struct {
    int max_delay_samples;
    int used_paths;
} ch_conv_info_t;

static uint32_t pl_tx_words[PL_CONV_MAX_X_LEN + PL_CONV_MAX_SNAPSHOTS * PL_CONV_MAX_PATHS * 2]
    __attribute__((aligned(64)));
static int32_t pl_rx_words[PL_CONV_MAX_Y_LEN * 2] __attribute__((aligned(64)));
static int g_pl_conv_verbose = 1;

static inline int16_t radian_to_fix16_13_bench(float radian) {
    float radian_mod = fmodf(radian, 2.0f * (float)M_PI);
    if (radian_mod < 0.0f) {
        radian_mod += 2.0f * (float)M_PI;
    }
    float phase = (radian_mod <= (float)M_PI) ? radian_mod : (radian_mod - 2.0f * (float)M_PI);
    float fixed_float = phase * (float)(1 << 13);
    return (int16_t)(fixed_float + (fixed_float >= 0.0f ? 0.5f : -0.5f));
}

static void software_cordic_q14(int16_t phase_q13, int16_t *sin_q14, int16_t *cos_q14) {
    static const int16_t atan_q13[14] = {
        6434, 3798, 2007, 1019, 511, 256, 128, 64, 32, 16, 8, 4, 2, 1
    };
    const int32_t pi_q13 = 25736;
    const int32_t half_pi_q13 = 12868;

    int32_t z = phase_q13;
    int32_t sign = 1;
    if (z > half_pi_q13) {
        z -= pi_q13;
        sign = -1;
    } else if (z < -half_pi_q13) {
        z += pi_q13;
        sign = -1;
    }

    int32_t x = 9949;  // 0.607252935 * 2^14
    int32_t y = 0;
    for (int i = 0; i < 14; ++i) {
        int32_t x_shift = x >> i;
        int32_t y_shift = y >> i;
        if (z >= 0) {
            int32_t x_new = x - y_shift;
            int32_t y_new = y + x_shift;
            x = x_new;
            y = y_new;
            z -= atan_q13[i];
        } else {
            int32_t x_new = x + y_shift;
            int32_t y_new = y - x_shift;
            x = x_new;
            y = y_new;
            z += atan_q13[i];
        }
    }

    x *= sign;
    y *= sign;
    if (x > 32767) x = 32767;
    if (x < -32768) x = -32768;
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;
    *cos_q14 = (int16_t)x;
    *sin_q14 = (int16_t)y;
}

static inline void pl_mmio_write(uint32_t addr, uint32_t value) {
    (*(volatile uint32_t *)addr) = value;
}

static inline uint32_t pl_mmio_read(uint32_t addr) {
    return (*(volatile uint32_t *)addr);
}

static inline void pl_memory_fence(void) {
    __asm__ volatile ("fence iorw, iorw" ::: "memory");
}

static int16_t pl_float_to_q15(float v) {
    if (!isfinite(v)) v = 0.0f;
    if (v > 0.999969f) v = 0.999969f;
    if (v < -1.0f) v = -1.0f;
    return (int16_t)(v * PL_CONV_Q15 + (v >= 0.0f ? 0.5f : -0.5f));
}

static int pl_valid_channel_path(complex h, float delay_seconds) {
    return isfinite(h.real) && isfinite(h.image) &&
           (h.real != 0.0f || h.image != 0.0f) &&
           isfinite(delay_seconds) &&
           delay_seconds > 0.0f &&
           delay_seconds < 1.0f;
}

static int channel_valid_ref_path(complex h, float delay_seconds) {
    return isfinite(h.real) && isfinite(h.image) &&
           isfinite(delay_seconds) &&
           (h.real != 0.0f || h.image != 0.0f) &&
           delay_seconds >= 0.0f &&
           delay_seconds < 1.0f;
}

static int channel_convolve_segmented_real_input(const float *x,
                                                 int x_len,
                                                 int samples_per_snapshot,
                                                 const complex *h,
                                                 const float *delay_seconds,
                                                 int num_snapshots,
                                                 int num_paths,
                                                 float fs_signal_hz,
                                                 complex *y,
                                                 int y_len,
                                                 ch_conv_info_t *info) {
    int total_used_paths = 0;
    int global_max_delay = 0;

    if (x == NULL || h == NULL || delay_seconds == NULL || y == NULL ||
        x_len <= 0 || y_len <= 0 || samples_per_snapshot <= 0 ||
        num_snapshots <= 0 || num_paths <= 0 || fs_signal_hz <= 0.0f) {
        return -1;
    }

    for (int i = 0; i < y_len; ++i) {
        y[i].real = 0.0f;
        y[i].image = 0.0f;
    }

    for (int t = 0; t < num_snapshots; ++t) {
        int seg_start = t * samples_per_snapshot;
        int seg_len = samples_per_snapshot;
        if (seg_start >= x_len) break;
        if (seg_start + seg_len > x_len) seg_len = x_len - seg_start;

        for (int l = 0; l < num_paths; ++l) {
            int path_idx = t * num_paths + l;
            complex h_l = h[path_idx];
            float d_seconds = delay_seconds[path_idx];
            if (!channel_valid_ref_path(h_l, d_seconds)) continue;

            int d = (int)floorf(d_seconds * fs_signal_hz + 0.5f);
            if (d < 0) d = 0;
            if (d > global_max_delay) global_max_delay = d;
            total_used_paths++;

            for (int n = 0; n < seg_len; ++n) {
                int out_idx = seg_start + n + d;
                if (out_idx >= y_len) break;
                y[out_idx].real += h_l.real * x[seg_start + n];
                y[out_idx].image += h_l.image * x[seg_start + n];
            }
        }
    }

    if (info != NULL) {
        info->max_delay_samples = global_max_delay;
        info->used_paths = total_used_paths;
    }
    return 0;
}

static int pl_estimate_required_y_len(const complex *h,
                                      const float *delay_seconds,
                                      int num_snapshots,
                                      int num_paths,
                                      int samples_per_snapshot,
                                      float fs_signal_hz,
                                      int max_extra_delay_samples) {
    int max_delay_samples = 0;
    if (h == NULL || delay_seconds == NULL || num_snapshots <= 0 ||
        num_paths <= 0 || samples_per_snapshot <= 0 || fs_signal_hz <= 0.0f) {
        return 0;
    }

    for (int t = 0; t < num_snapshots; ++t) {
        float min_delay = 1.0f;
        int has_valid_delay = 0;
        for (int l = 0; l < num_paths; ++l) {
            int idx = t * num_paths + l;
            if (pl_valid_channel_path(h[idx], delay_seconds[idx])) {
                if (!has_valid_delay || delay_seconds[idx] < min_delay) {
                    min_delay = delay_seconds[idx];
                    has_valid_delay = 1;
                }
            }
        }
        if (!has_valid_delay) {
            continue;
        }
        for (int l = 0; l < num_paths; ++l) {
            int idx = t * num_paths + l;
            if (pl_valid_channel_path(h[idx], delay_seconds[idx])) {
                int ds = (int)floorf((delay_seconds[idx] - min_delay) * fs_signal_hz + 0.5f);
                if (ds >= 0 && ds > max_delay_samples) {
                    max_delay_samples = ds;
                }
            }
        }
    }

    if (max_delay_samples > max_extra_delay_samples) {
        max_delay_samples = max_extra_delay_samples;
    }
    int y_len = num_snapshots * samples_per_snapshot + max_delay_samples;
    if (y_len < num_snapshots * samples_per_snapshot) {
        y_len = num_snapshots * samples_per_snapshot;
    }
    return y_len;
}

static int pl_compact_batch_paths(const complex *h_src,
                                  const float *d_src,
                                  int num_snapshots,
                                  int src_paths,
                                  complex *h_dst,
                                  float *d_dst,
                                  int dst_capacity_paths) {
    int max_valid_paths = 0;

    if (h_src == NULL || d_src == NULL || h_dst == NULL || d_dst == NULL ||
        num_snapshots <= 0 || src_paths <= 0 || dst_capacity_paths <= 0 ||
        num_snapshots > PL_CONV_MAX_SNAPSHOTS) {
        return -1;
    }

    for (int s = 0; s < num_snapshots; ++s) {
        int valid_count = 0;
        for (int p = 0; p < src_paths; ++p) {
            int idx = s * src_paths + p;
            if (pl_valid_channel_path(h_src[idx], d_src[idx])) {
                valid_count++;
            }
        }
        if (valid_count > max_valid_paths) {
            max_valid_paths = valid_count;
        }
    }

    if (max_valid_paths <= 0) {
        max_valid_paths = 1;
    }
    if (max_valid_paths > dst_capacity_paths) {
        return -2;
    }

    for (int s = 0; s < num_snapshots; ++s) {
        int write_idx = 0;
        for (int p = 0; p < src_paths && write_idx < max_valid_paths; ++p) {
            int src_idx = s * src_paths + p;
            if (pl_valid_channel_path(h_src[src_idx], d_src[src_idx])) {
                int dst_idx = s * max_valid_paths + write_idx;
                h_dst[dst_idx] = h_src[src_idx];
                d_dst[dst_idx] = d_src[src_idx];
                write_idx++;
            }
        }
        while (write_idx < max_valid_paths) {
            int dst_idx = s * max_valid_paths + write_idx;
            h_dst[dst_idx].real = 0.0f;
            h_dst[dst_idx].image = 0.0f;
            d_dst[dst_idx] = 2.0f;
            write_idx++;
        }
    }

    return max_valid_paths;
}

static void pl_channel_compare_outputs(const complex *ref,
                                       const complex *test,
                                       int len,
                                       pl_conv_stats_t *stats) {
    double err2_sum = 0.0;
    double ref2_sum = 0.0;
    double max_abs_err = 0.0;
    if (ref == NULL || test == NULL || stats == NULL || len <= 0) return;

    for (int i = 0; i < len; ++i) {
        double er = (double)test[i].real - (double)ref[i].real;
        double ei = (double)test[i].image - (double)ref[i].image;
        double rr = (double)ref[i].real;
        double ri = (double)ref[i].image;
        double eabs = sqrt(er * er + ei * ei);
        err2_sum += er * er + ei * ei;
        ref2_sum += rr * rr + ri * ri;
        if (eabs > max_abs_err) max_abs_err = eabs;
    }

    stats->max_abs_err = (float)max_abs_err;
    stats->rmse = (float)sqrt(err2_sum / (double)len);
    stats->nmse_db = (float)(10.0 * log10((err2_sum + 1.0e-30) /
                                          (ref2_sum + 1.0e-30)));
}

static void pl_dma_reset(void) {
    pl_mmio_write(PL_DMA_MM2S_DMACR, PL_DMA_CR_RESET);
    pl_mmio_write(PL_DMA_S2MM_DMACR, PL_DMA_CR_RESET);
    uint32_t guard = 0;
    while ((pl_mmio_read(PL_DMA_MM2S_DMACR) & PL_DMA_CR_RESET) && guard++ < PL_DMA_TIMEOUT) {}
    guard = 0;
    while ((pl_mmio_read(PL_DMA_S2MM_DMACR) & PL_DMA_CR_RESET) && guard++ < PL_DMA_TIMEOUT) {}
}

static int pl_dma_wait(uint32_t status_addr) {
    uint32_t guard = 0;
    while (guard++ < PL_DMA_TIMEOUT) {
        uint32_t status = pl_mmio_read(status_addr);
        if (status & PL_DMA_IRQ_ERR) return -2;
        if (status & PL_DMA_IRQ_IOC) return 0;
    }
    return -1;
}

static int pl_conv_wait_done(void) {
    uint32_t guard = 0;
    while (guard++ < PL_DMA_TIMEOUT) {
        uint32_t ctrl = pl_mmio_read(PL_CONV_BASE_ADDR + PL_CONV_REG_CTRL);
        if ((ctrl & 0x8u) != 0u) return -1;  // bit3: core_error
        if ((ctrl & 0x2u) != 0u) return 0;   // bit1: core_done
        if ((ctrl & 0x4u) == 0u) return 0;   // bit2: core_busy has cleared
    }
    return -1;
}

static int pl_channel_convolve_segmented_q15(const float *x,
                                             const complex *h,
                                             const float *delay_seconds,
                                             int num_snapshots,
                                             int num_paths,
                                             int samples_per_snapshot,
                                             float fs_signal_hz,
                                             complex *y_pl,
                                             int y_len,
                                             pl_conv_stats_t *stats) {
    int x_len;
    int total_paths;
    int tx_count;
    int used_paths = 0;
    int invalid_paths = 0;
    int clipped_delay_paths = 0;
    int max_delay_samples = 0;
    float h_abs_max = 0.0f;
    float h_scale = 1.0f;
    uint32_t total_start;
    uint32_t pack_cycles = 0;
    uint32_t dma_setup_cycles = 0;
    uint32_t dma_wait_cycles = 0;
    uint32_t unpack_cycles = 0;
    uint32_t stage_start;

    if (x == NULL || h == NULL || delay_seconds == NULL || y_pl == NULL ||
        num_snapshots <= 0 || num_paths <= 0 || samples_per_snapshot <= 0 ||
        fs_signal_hz <= 0.0f || y_len <= 0) {
        return -1;
    }
    if (num_snapshots > PL_CONV_MAX_SNAPSHOTS ||
        num_paths > PL_CONV_MAX_PATHS) {
        return -2;
    }

    x_len = num_snapshots * samples_per_snapshot;
    total_paths = num_snapshots * num_paths;
    if (x_len > PL_CONV_MAX_X_LEN || y_len > PL_CONV_MAX_Y_LEN) {
        return -3;
    }

    total_start = read_cycle32();
    stage_start = read_cycle32();
    for (int i = 0; i < y_len; i++) {
        y_pl[i].real = 0.0f;
        y_pl[i].image = 0.0f;
    }

    for (int i = 0; i < x_len; i++) {
        int16_t xq = pl_float_to_q15(x[i]);
        pl_tx_words[i] = (uint16_t)xq;
    }

    for (int i = 0; i < total_paths; i++) {
        float ar = fabsf(h[i].real);
        float ai = fabsf(h[i].image);
        if (isfinite(ar) && ar > h_abs_max) {
            h_abs_max = ar;
        }
        if (isfinite(ai) && ai > h_abs_max) {
            h_abs_max = ai;
        }
    }
    if (h_abs_max > 0.95f) {
        h_scale = h_abs_max / 0.95f;
    }

    for (int i = 0; i < total_paths; i++) {
        int16_t hr = pl_float_to_q15(h[i].real / h_scale);
        int16_t hi = pl_float_to_q15(h[i].image / h_scale);
        pl_tx_words[x_len + i] = ((uint32_t)(uint16_t)hi << 16) | (uint16_t)hr;
    }

    for (int t = 0; t < num_snapshots; t++) {
        float min_delay = 1.0f;
        int has_valid_delay = 0;
        for (int l = 0; l < num_paths; l++) {
            int idx = t * num_paths + l;
            if (pl_valid_channel_path(h[idx], delay_seconds[idx])) {
                if (!has_valid_delay || delay_seconds[idx] < min_delay) {
                    min_delay = delay_seconds[idx];
                    has_valid_delay = 1;
                }
            }
        }

        for (int l = 0; l < num_paths; l++) {
            int idx = t * num_paths + l;
            uint16_t dq = PL_CONV_INVALID_DELAY;
            if (has_valid_delay && pl_valid_channel_path(h[idx], delay_seconds[idx])) {
                float relative_delay = delay_seconds[idx] - min_delay;
                int ds = (int)floorf(relative_delay * fs_signal_hz + 0.5f);
                if (ds >= 0 && ds < (int)PL_CONV_INVALID_DELAY) {
                    dq = (uint16_t)ds;
                    used_paths++;
                    if (ds > max_delay_samples) {
                        max_delay_samples = ds;
                    }
                } else {
                    clipped_delay_paths++;
                }
            } else {
                invalid_paths++;
            }
            pl_tx_words[x_len + total_paths + idx] = dq;
        }
    }

    tx_count = x_len + total_paths + total_paths;
    const uint32_t tx_bytes = (uint32_t)tx_count * (uint32_t)sizeof(pl_tx_words[0]);
    const uint32_t rx_bytes = (uint32_t)y_len * 2u * (uint32_t)sizeof(pl_rx_words[0]);
    pl_memory_fence();
    pack_cycles = read_cycle32() - stage_start;

    if (g_pl_conv_verbose) {
        printf("PL_CONV_TX x0_7=%08lx,%08lx,%08lx,%08lx,%08lx,%08lx,%08lx,%08lx h0_3=%08lx,%08lx,%08lx,%08lx d0_3=%08lx,%08lx,%08lx,%08lx tx_count=%d\n",
               (unsigned long)pl_tx_words[0],
               (unsigned long)pl_tx_words[1],
               (unsigned long)pl_tx_words[2],
               (unsigned long)pl_tx_words[3],
               (unsigned long)pl_tx_words[4],
               (unsigned long)pl_tx_words[5],
               (unsigned long)pl_tx_words[6],
               (unsigned long)pl_tx_words[7],
               (unsigned long)pl_tx_words[x_len + 0],
               (unsigned long)((total_paths > 1) ? pl_tx_words[x_len + 1] : 0),
               (unsigned long)((total_paths > 2) ? pl_tx_words[x_len + 2] : 0),
               (unsigned long)((total_paths > 3) ? pl_tx_words[x_len + 3] : 0),
               (unsigned long)pl_tx_words[x_len + total_paths + 0],
               (unsigned long)((total_paths > 1) ? pl_tx_words[x_len + total_paths + 1] : 0),
               (unsigned long)((total_paths > 2) ? pl_tx_words[x_len + total_paths + 2] : 0),
               (unsigned long)((total_paths > 3) ? pl_tx_words[x_len + total_paths + 3] : 0),
               tx_count);
        fflush(stdout);
    }

    stage_start = read_cycle32();
    pl_mmio_write(PL_CONV_BASE_ADDR + PL_CONV_REG_X_LEN, (uint32_t)x_len);
    pl_mmio_write(PL_CONV_BASE_ADDR + PL_CONV_REG_Y_LEN, (uint32_t)y_len);
    pl_mmio_write(PL_CONV_BASE_ADDR + PL_CONV_REG_SAMPLES_PER_SNAPSHOT, (uint32_t)samples_per_snapshot);
    pl_mmio_write(PL_CONV_BASE_ADDR + PL_CONV_REG_NUM_SNAPSHOTS, (uint32_t)num_snapshots);
    pl_mmio_write(PL_CONV_BASE_ADDR + PL_CONV_REG_NUM_PATHS, (uint32_t)num_paths);

    if (g_pl_conv_verbose) {
        printf("PL_CONV_REGS x_len=%lu y_len=%lu sps=%lu snaps=%lu paths=%lu ctrl=0x%08lx\n",
               (unsigned long)pl_mmio_read(PL_CONV_BASE_ADDR + PL_CONV_REG_X_LEN),
               (unsigned long)pl_mmio_read(PL_CONV_BASE_ADDR + PL_CONV_REG_Y_LEN),
               (unsigned long)pl_mmio_read(PL_CONV_BASE_ADDR + PL_CONV_REG_SAMPLES_PER_SNAPSHOT),
               (unsigned long)pl_mmio_read(PL_CONV_BASE_ADDR + PL_CONV_REG_NUM_SNAPSHOTS),
               (unsigned long)pl_mmio_read(PL_CONV_BASE_ADDR + PL_CONV_REG_NUM_PATHS),
               (unsigned long)pl_mmio_read(PL_CONV_BASE_ADDR + PL_CONV_REG_CTRL));
        fflush(stdout);
    }

    pl_dma_reset();
    pl_mmio_write(PL_DMA_S2MM_DMASR, PL_DMA_IRQ_ALL);
    pl_mmio_write(PL_DMA_MM2S_DMASR, PL_DMA_IRQ_ALL);
    pl_mmio_write(PL_DMA_S2MM_DMACR, PL_DMA_CR_RUNSTOP);
    pl_mmio_write(PL_DMA_MM2S_DMACR, PL_DMA_CR_RUNSTOP);
    pl_mmio_write(PL_DMA_S2MM_DA, (uint32_t)pl_rx_words);
    pl_mmio_write(PL_DMA_S2MM_LENGTH, rx_bytes);
    pl_mmio_write(PL_CONV_BASE_ADDR + PL_CONV_REG_CTRL, 1u);
    pl_mmio_write(PL_DMA_MM2S_SA, (uint32_t)pl_tx_words);
    pl_mmio_write(PL_DMA_MM2S_LENGTH, tx_bytes);
    dma_setup_cycles = read_cycle32() - stage_start;

    stage_start = read_cycle32();
    int mm2s_rc = pl_dma_wait(PL_DMA_MM2S_DMASR);
    int s2mm_rc = pl_dma_wait(PL_DMA_S2MM_DMASR);
    int conv_rc = pl_conv_wait_done();
    pl_memory_fence();
    dma_wait_cycles = read_cycle32() - stage_start;
    if (mm2s_rc != 0 || s2mm_rc != 0 || conv_rc != 0) {
        printf("PL_CONV_DMA_ERROR mm2s=%d s2mm=%d conv=%d mm2s_sr=0x%08lx s2mm_sr=0x%08lx\n",
               mm2s_rc, s2mm_rc, conv_rc,
               (unsigned long)pl_mmio_read(PL_DMA_MM2S_DMASR),
               (unsigned long)pl_mmio_read(PL_DMA_S2MM_DMASR));
        return -20;
    }

    stage_start = read_cycle32();
    int32_t min_v = pl_rx_words[0], max_v = pl_rx_words[0];
    if (g_pl_conv_verbose) {
        for (int i = 0; i < y_len * 2; i++) {
            if (pl_rx_words[i] < min_v) min_v = pl_rx_words[i];
            if (pl_rx_words[i] > max_v) max_v = pl_rx_words[i];
        }
        printf("PL_CONV_RAW min=%ld max=%ld first8=%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
               (long)min_v, (long)max_v,
               (long)pl_rx_words[0], (long)pl_rx_words[1],
               (long)pl_rx_words[2], (long)pl_rx_words[3],
               (long)pl_rx_words[4], (long)pl_rx_words[5],
               (long)pl_rx_words[6], (long)pl_rx_words[7]);
        fflush(stdout);
    }

    for (int i = 0; i < y_len; i++) {
        y_pl[i].real = (float)pl_rx_words[2 * i] * PL_CONV_Q15_INV * h_scale;
        y_pl[i].image = (float)pl_rx_words[2 * i + 1] * PL_CONV_Q15_INV * h_scale;
    }
    unpack_cycles = read_cycle32() - stage_start;

    if (stats != NULL) {
        memset(stats, 0, sizeof(*stats));
        stats->used_paths = used_paths;
        stats->invalid_paths = invalid_paths;
        stats->clipped_delay_paths = clipped_delay_paths;
        stats->max_delay_samples = max_delay_samples;
        stats->pack_cycles = pack_cycles;
        stats->dma_setup_cycles = dma_setup_cycles;
        stats->dma_wait_cycles = dma_wait_cycles;
        stats->unpack_cycles = unpack_cycles;
        stats->total_cycles = read_cycle32() - total_start;
        stats->tx_words = tx_count;
        stats->y_len = y_len;
        stats->h_scale = h_scale;
    }

    return 0;
}

static int pl_channel_convolution_selftest(void) {
    printf("PL_SELFTEST_START\n");
    enum {
        SELF_X_LEN = 32,
        SELF_Y_LEN = 40,
        SELF_MAX_SNAPSHOTS = 2,
        SELF_MAX_PATHS = 2,
        SELF_SPS = 32
    };
    static float x[SELF_X_LEN];
    static complex h[SELF_MAX_SNAPSHOTS * SELF_MAX_PATHS];
    static float d[SELF_MAX_SNAPSHOTS * SELF_MAX_PATHS];
    static complex y[SELF_Y_LEN];
    pl_conv_stats_t stats;
    int fail_count = 0;

    for (int test = 0; test < 3; ++test) {
        int snapshots = 1;
        int paths = (test == 2) ? 2 : 1;
        float max_err = 0.0f;

        for (int i = 0; i < SELF_X_LEN; ++i) x[i] = (i & 1) ? 1.0f : 0.0f;
        for (int i = 0; i < SELF_MAX_SNAPSHOTS * SELF_MAX_PATHS; ++i) {
            h[i].real = 0.0f;
            h[i].image = 0.0f;
            d[i] = 2.0f;
        }
        for (int i = 0; i < SELF_Y_LEN; ++i) {
            y[i].real = 0.0f;
            y[i].image = 0.0f;
        }

        if (test == 0) {
            h[0].real = 0.5f;
            h[0].image = 0.0f;
            d[0] = 1.0e-9f;
        } else if (test == 1) {
            h[0].real = 0.5f;
            h[0].image = 0.0f;
            d[0] = 5.0e-9f;
        } else {
            h[0].real = 0.5f;
            h[0].image = 0.0f;
            d[0] = 1.0e-9f;
            h[1].real = 0.25f;
            h[1].image = -0.125f;
            d[1] = 5.0e-9f;
        }

        int rc = pl_channel_convolve_segmented_q15(x, h, d,
                                                   snapshots, paths,
                                                   SELF_SPS, 500000000.0f,
                                                   y, SELF_Y_LEN, &stats);
        if (rc != 0) {
            printf("PL_SELFTEST_ERROR test=%d rc=%d\n", test, rc);
            fail_count++;
            continue;
        }

        for (int i = 0; i < SELF_Y_LEN; ++i) {
            float expected_r = 0.0f;
            float expected_i = 0.0f;
            if (test == 0 || test == 1) {
                expected_r = (i < SELF_X_LEN && (i & 1)) ? 0.5f : 0.0f;
            } else {
                if (i < SELF_X_LEN && (i & 1)) expected_r += 0.5f;
                if (i >= 2 && i - 2 < SELF_X_LEN && ((i - 2) & 1)) {
                    expected_r += 0.25f;
                    expected_i += -0.125f;
                }
            }
            float er = fabsf(y[i].real - expected_r);
            float ei = fabsf(y[i].image - expected_i);
            if (er > max_err) max_err = er;
            if (ei > max_err) max_err = ei;
        }
        if (max_err > 2.0e-3f) fail_count++;

        printf("PL_SELFTEST_STATS test=%d paths=%d used_paths=%d invalid_paths=%d max_delay=%d h_scale=%.9f max_err=%.9e\n",
               test, paths, stats.used_paths, stats.invalid_paths,
               stats.max_delay_samples, stats.h_scale, max_err);
        printf("PL_SELFTEST_Y_FIRST test=%d\n", test);
        for (int i = 0; i < 12 && i < SELF_Y_LEN; ++i) {
            float expected_r = 0.0f;
            float expected_i = 0.0f;
            if (test == 0 || test == 1) {
                expected_r = (i < SELF_X_LEN && (i & 1)) ? 0.5f : 0.0f;
            } else {
                if (i < SELF_X_LEN && (i & 1)) expected_r += 0.5f;
                if (i >= 2 && i - 2 < SELF_X_LEN && ((i - 2) & 1)) {
                    expected_r += 0.25f;
                    expected_i += -0.125f;
                }
            }
            printf("y[%d]=%.9f %.9f expected=%.3f %.3f\n",
                   i, y[i].real, y[i].image, expected_r, expected_i);
        }
    }
    printf("PL_SELFTEST_END\n");
    fflush(stdout);
    return fail_count == 0 ? 0 : -30;
}

static void run_pl_channel_convolution_demo(complex H_in[MAX_TIME][220],
                                            float D_in[MAX_TIME][TOTAL_CLUSTERS],
                                            int total_snapshots,
                                            int num_paths,
                                            float fs_signal_hz) {
    enum {
        DEMO_SPS = 8,
        DEMO_SYMBOLS_PER_SNAPSHOT = 64,
        PL_DEMO_HW_BATCH_SNAPSHOTS = 4,
        PL_DEMO_MAX_PATHS = 80,
        PL_DEMO_MAX_DELAY_SAMPLES = 4096,
        PL_DEMO_MAX_PRINT_SAMPLES = 24576
    };
    int use_snapshots = total_snapshots;
    if (use_snapshots > PL_DEMO_MAX_TIME) use_snapshots = PL_DEMO_MAX_TIME;
    int use_paths = num_paths;
    if (use_paths > PL_DEMO_MAX_PATHS) use_paths = PL_DEMO_MAX_PATHS;
    if (use_snapshots <= 0 || use_paths <= 0) {
        printf("PL_CONV_DEMO_ERROR invalid snapshots_or_paths\n");
        return;
    }
    if (fs_signal_hz <= 0.0f) {
        printf("PL_CONV_DEMO_ERROR invalid fs\n");
        return;
    }

    const int samples_per_snapshot = DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT;
    const int x_len = use_snapshots * samples_per_snapshot;
    const int y_len = x_len + PL_DEMO_MAX_DELAY_SAMPLES;

    static float x[PL_DEMO_MAX_TIME * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT] __attribute__((aligned(64)));
    static complex h[PL_DEMO_MAX_TIME * PL_DEMO_MAX_PATHS] __attribute__((aligned(64)));
    static float d_raw[PL_DEMO_MAX_TIME * PL_DEMO_MAX_PATHS] __attribute__((aligned(64)));
    static float d_rel[PL_DEMO_MAX_TIME * PL_DEMO_MAX_PATHS] __attribute__((aligned(64)));
    static complex y_ref[PL_DEMO_MAX_TIME * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT + PL_DEMO_MAX_DELAY_SAMPLES] __attribute__((aligned(64)));
    static complex y_pl[PL_DEMO_MAX_TIME * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT + PL_DEMO_MAX_DELAY_SAMPLES] __attribute__((aligned(64)));
    static complex h_batch[PL_DEMO_HW_BATCH_SNAPSHOTS * PL_DEMO_MAX_PATHS] __attribute__((aligned(64)));
    static float d_batch[PL_DEMO_HW_BATCH_SNAPSHOTS * PL_DEMO_MAX_PATHS] __attribute__((aligned(64)));
    static complex y_batch[PL_DEMO_HW_BATCH_SNAPSHOTS * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT + PL_DEMO_MAX_DELAY_SAMPLES] __attribute__((aligned(64)));

    unsigned int lfsr = 0xA5u;
    int num_symbols = x_len / DEMO_SPS;
    for (int symbol_idx = 0; symbol_idx < num_symbols; ++symbol_idx) {
        int bit = (int)(lfsr & 1u);
        unsigned int feedback = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 4)) & 1u;
        lfsr = (lfsr >> 1) | (feedback << 7);
        for (int k = 0; k < DEMO_SPS; ++k) {
            x[symbol_idx * DEMO_SPS + k] = bit ? 1.0f : 0.0f;
        }
    }

    for (int i = 0; i < y_len; ++i) {
        y_ref[i].real = 0.0f;
        y_ref[i].image = 0.0f;
        y_pl[i].real = 0.0f;
        y_pl[i].image = 0.0f;
    }

    for (int t = 0; t < use_snapshots; ++t) {
        float min_delay = 1.0f;
        int has_valid_delay = 0;

        for (int c = 0; c < use_paths; ++c) {
            float dt = D_in[t][c];
            complex ht = H_in[t][c];
            if (isfinite(dt) && dt > 0.0f && dt < 1.0f &&
                isfinite(ht.real) && isfinite(ht.image) &&
                (ht.real != 0.0f || ht.image != 0.0f)) {
                if (!has_valid_delay || dt < min_delay) {
                    min_delay = dt;
                    has_valid_delay = 1;
                }
            }
        }

        for (int c = 0; c < use_paths; ++c) {
            int idx = t * use_paths + c;
            h[idx].real = H_in[t][c].real;
            h[idx].image = H_in[t][c].image;
            d_raw[idx] = D_in[t][c];
            d_rel[idx] = has_valid_delay ? (D_in[t][c] - min_delay) : D_in[t][c];
        }
    }

    ch_conv_info_t ref_info;
    int rc_ref = channel_convolve_segmented_real_input(x, x_len, samples_per_snapshot,
                                                       h, d_rel, use_snapshots, use_paths,
                                                       fs_signal_hz, y_ref, y_len, &ref_info);
    if (rc_ref != 0) {
        printf("PL_CONV_DEMO_ERROR ps_ref_rc=%d\n", rc_ref);
        return;
    }
    pl_conv_stats_t pl_stats;
    memset(&pl_stats, 0, sizeof(pl_stats));

    for (int batch_start = 0; batch_start < use_snapshots; batch_start += PL_DEMO_HW_BATCH_SNAPSHOTS) {
        int batch_snapshots = use_snapshots - batch_start;
        if (batch_snapshots > PL_DEMO_HW_BATCH_SNAPSHOTS) batch_snapshots = PL_DEMO_HW_BATCH_SNAPSHOTS;

        int compact_paths = pl_compact_batch_paths(&h[batch_start * use_paths],
                                                   &d_raw[batch_start * use_paths],
                                                   batch_snapshots, use_paths,
                                                   h_batch, d_batch,
                                                   PL_DEMO_MAX_PATHS);
        if (compact_paths <= 0) {
            printf("PL_CONV_DEMO_ERROR compact_paths=%d batch_start=%d\n",
                   compact_paths, batch_start);
            break;
        }

        int batch_x_len = batch_snapshots * samples_per_snapshot;
        int batch_y_len = pl_estimate_required_y_len(h_batch, d_batch,
                                                     batch_snapshots, compact_paths,
                                                     samples_per_snapshot,
                                                     fs_signal_hz,
                                                     PL_DEMO_MAX_DELAY_SAMPLES);
        if (batch_y_len <= 0) {
            batch_y_len = batch_x_len;
        }
        if (batch_y_len > batch_x_len + PL_DEMO_MAX_DELAY_SAMPLES) {
            batch_y_len = batch_x_len + PL_DEMO_MAX_DELAY_SAMPLES;
        }
        if (batch_y_len > (int)(sizeof(y_batch) / sizeof(y_batch[0]))) {
            batch_y_len = (int)(sizeof(y_batch) / sizeof(y_batch[0]));
        }

        pl_conv_stats_t stats;
        int rc = pl_channel_convolve_segmented_q15(&x[batch_start * samples_per_snapshot],
                                                   h_batch,
                                                   d_batch,
                                                   batch_snapshots,
                                                   compact_paths,
                                                   samples_per_snapshot,
                                                   fs_signal_hz,
                                                   y_batch,
                                                   batch_y_len,
                                                   &stats);
        if (rc != 0) {
            printf("PL_CONV_DEMO_ERROR pl_rc=%d batch_start=%d batch_snapshots=%d\n",
                   rc, batch_start, batch_snapshots);
            break;
        }

        int global_y_offset = batch_start * samples_per_snapshot;
        for (int i = 0; i < batch_y_len && global_y_offset + i < y_len; ++i) {
            y_pl[global_y_offset + i].real += y_batch[i].real;
            y_pl[global_y_offset + i].image += y_batch[i].image;
        }

        pl_stats.used_paths += stats.used_paths;
        pl_stats.invalid_paths += stats.invalid_paths;
        pl_stats.clipped_delay_paths += stats.clipped_delay_paths;
        if (stats.max_delay_samples > pl_stats.max_delay_samples) pl_stats.max_delay_samples = stats.max_delay_samples;
        if (stats.h_scale > pl_stats.h_scale) pl_stats.h_scale = stats.h_scale;

        printf("PL_CONV_BATCH start=%d snapshots=%d paths=%d source_paths=%d x_len=%d y_len=%d used_paths=%d max_delay=%d\n",
               batch_start, batch_snapshots, compact_paths, use_paths, batch_x_len, batch_y_len,
               stats.used_paths, stats.max_delay_samples);
        fflush(stdout);
    }

    pl_channel_compare_outputs(y_ref, y_pl, y_len, &pl_stats);

    int first_nonzero = -1;
    int last_nonzero = -1;
    for (int i = 0; i < y_len; ++i) {
        if (fabsf(y_pl[i].real) > 1.0e-12f || fabsf(y_pl[i].image) > 1.0e-12f) {
            if (first_nonzero < 0) first_nonzero = i;
            last_nonzero = i;
        }
    }
    int y_print_start = first_nonzero >= 0 ? first_nonzero : 0;
    if (y_print_start > 8) y_print_start -= 8;
    int y_print_end = y_print_start + PL_DEMO_MAX_PRINT_SAMPLES;
    if (y_print_end > y_len) y_print_end = y_len;

    printf("PL_CONV_DEMO_START\n");
    printf("snapshots=%d paths=%d x_len=%d y_len=%d fs=%.0f max_delay=%d used_paths=%d first_y=%d last_y=%d\n",
           use_snapshots, use_paths, x_len, y_len, fs_signal_hz,
           pl_stats.max_delay_samples, pl_stats.used_paths, first_nonzero, last_nonzero);
    printf("q15_scale=%.9f invalid_paths=%d clipped_delay_paths=%d ref_used_paths=%d ref_max_delay=%d\n",
           pl_stats.h_scale, pl_stats.invalid_paths, pl_stats.clipped_delay_paths,
           ref_info.used_paths, ref_info.max_delay_samples);
    printf("max_abs_err=%.9e rmse=%.9e nmse_db=%.3f\n",
           pl_stats.max_abs_err, pl_stats.rmse, pl_stats.nmse_db);
    printf("X_FIRST\n");
    for (int i = 0; i < x_len && i < PL_DEMO_MAX_PRINT_SAMPLES; ++i) {
        printf("x[%d]=%.6f\n", i, x[i]);
    }
    printf("Y_FIRST\n");
    for (int i = y_print_start; i < y_print_end; ++i) {
        printf("y[%d]=%.9f %.9f\n", i, y_pl[i].real, y_pl[i].image);
    }
    printf("PL_CONV_DEMO_END\n");
    fflush(stdout);
}

static void run_sw_channel_convolution_demo(complex H_in[MAX_TIME][220],
                                            float D_in[MAX_TIME][TOTAL_CLUSTERS],
                                            int total_snapshots,
                                            int num_paths,
                                            float fs_signal_hz) {
    enum {
        DEMO_SPS = 8,
        DEMO_SYMBOLS_PER_SNAPSHOT = 64,
        DEMO_MAX_SNAPSHOTS = 40,
        DEMO_MAX_PATHS = 80,
        DEMO_MAX_DELAY_SAMPLES = 4096,
        DEMO_MAX_PRINT_SAMPLES = 24576
    };
    int use_snapshots = total_snapshots;
    if (use_snapshots > DEMO_MAX_SNAPSHOTS) use_snapshots = DEMO_MAX_SNAPSHOTS;
    int use_paths = num_paths;
    if (use_paths > DEMO_MAX_PATHS) use_paths = DEMO_MAX_PATHS;
    if (use_snapshots <= 0 || use_paths <= 0) {
        printf("CONV_DEMO_ERROR invalid snapshot/path count\n");
        return;
    }
    if (fs_signal_hz <= 0.0f) {
        printf("CONV_DEMO_ERROR invalid fs\n");
        return;
    }

    const int samples_per_snapshot = DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT;
    const int x_len = use_snapshots * samples_per_snapshot;
    const int y_len = x_len + DEMO_MAX_DELAY_SAMPLES;
    static float x[DEMO_MAX_SNAPSHOTS * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT] __attribute__((aligned(64)));
    static complex h[DEMO_MAX_SNAPSHOTS * DEMO_MAX_PATHS] __attribute__((aligned(64)));
    static float d_rel[DEMO_MAX_SNAPSHOTS * DEMO_MAX_PATHS] __attribute__((aligned(64)));
    static complex y[DEMO_MAX_SNAPSHOTS * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT + DEMO_MAX_DELAY_SAMPLES] __attribute__((aligned(64)));

    unsigned int lfsr = 0xA5u;
    int num_symbols = x_len / DEMO_SPS;
    for (int symbol_idx = 0; symbol_idx < num_symbols; ++symbol_idx) {
        int bit = (int)(lfsr & 1u);
        unsigned int feedback = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 4)) & 1u;
        lfsr = (lfsr >> 1) | (feedback << 7);
        for (int k = 0; k < DEMO_SPS; ++k) {
            x[symbol_idx * DEMO_SPS + k] = bit ? 1.0f : 0.0f;
        }
    }

    for (int t = 0; t < use_snapshots; ++t) {
        float min_delay = 1.0f;
        int has_valid_delay = 0;
        for (int c = 0; c < use_paths; ++c) {
            float dt = D_in[t][c];
            complex ht = H_in[t][c];
            if (isfinite(dt) && dt > 0.0f && dt < 1.0f &&
                isfinite(ht.real) && isfinite(ht.image) &&
                (ht.real != 0.0f || ht.image != 0.0f)) {
                if (!has_valid_delay || dt < min_delay) {
                    min_delay = dt;
                    has_valid_delay = 1;
                }
            }
        }
        for (int c = 0; c < use_paths; ++c) {
            int idx = t * use_paths + c;
            h[idx].real = H_in[t][c].real;
            h[idx].image = H_in[t][c].image;
            d_rel[idx] = has_valid_delay ? (D_in[t][c] - min_delay) : D_in[t][c];
        }
    }

    ch_conv_info_t conv_info;
    int rc = channel_convolve_segmented_real_input(x, x_len, samples_per_snapshot,
                                                   h, d_rel, use_snapshots, use_paths,
                                                   fs_signal_hz, y, y_len, &conv_info);
    if (rc != 0) {
        printf("CONV_DEMO_ERROR rc=%d\n", rc);
        return;
    }

    int first_nonzero = -1;
    int last_nonzero = -1;
    for (int i = 0; i < y_len; ++i) {
        if (fabsf(y[i].real) > 1.0e-12f || fabsf(y[i].image) > 1.0e-12f) {
            if (first_nonzero < 0) first_nonzero = i;
            last_nonzero = i;
        }
    }
    int y_print_start = first_nonzero >= 0 ? first_nonzero : 0;
    if (y_print_start > 8) y_print_start -= 8;
    int y_print_end = y_print_start + DEMO_MAX_PRINT_SAMPLES;
    if (y_print_end > y_len) y_print_end = y_len;

    printf("CONV_DEMO_START\n");
    printf("snapshots=%d paths=%d x_len=%d y_len=%d fs=%.0f max_delay=%d used_paths=%d first_y=%d last_y=%d\n",
           use_snapshots, use_paths, x_len, y_len, fs_signal_hz,
           conv_info.max_delay_samples, conv_info.used_paths, first_nonzero, last_nonzero);
    printf("X_FIRST\n");
    for (int i = 0; i < x_len && i < DEMO_MAX_PRINT_SAMPLES; ++i) {
        printf("x[%d]=%.6f\n", i, x[i]);
    }
    printf("Y_FIRST\n");
    for (int i = y_print_start; i < y_print_end; ++i) {
        printf("y[%d]=%.9f %.9f\n", i, y[i].real, y[i].image);
    }
    printf("CONV_DEMO_END\n");
    fflush(stdout);
}

static void run_accel_benchmark(complex H_in[MAX_TIME][220],
                                float D_in[MAX_TIME][TOTAL_CLUSTERS],
                                int total_snapshots,
                                int num_paths,
                                float fs_signal_hz) {
    enum {
        TRIG_COUNT = 4096,
        DEMO_SPS = 8,
        DEMO_SYMBOLS_PER_SNAPSHOT = 64,
        BENCH_MAX_SNAPSHOTS = 40,
        BENCH_BATCH_SNAPSHOTS = 4,
        BENCH_MAX_PATHS = 80,
        BENCH_MAX_DELAY_SAMPLES = 4096
    };
    static float angles[TRIG_COUNT] __attribute__((aligned(64)));
    static float sw_sin[TRIG_COUNT] __attribute__((aligned(64)));
    static float sw_cos[TRIG_COUNT] __attribute__((aligned(64)));
    static float hw_sin[TRIG_COUNT] __attribute__((aligned(64)));
    static float hw_cos[TRIG_COUNT] __attribute__((aligned(64)));
    static int16_t phase_q13[TRIG_COUNT] __attribute__((aligned(64)));
    static int16_t nofpu_sin_q14[TRIG_COUNT] __attribute__((aligned(64)));
    static int16_t nofpu_cos_q14[TRIG_COUNT] __attribute__((aligned(64)));

    printf("BENCH_START\n");
    for (int i = 0; i < TRIG_COUNT; ++i) {
        angles[i] = -(float)M_PI + (2.0f * (float)M_PI * (float)i) / (float)(TRIG_COUNT - 1);
        phase_q13[i] = radian_to_fix16_13_bench(angles[i]);
    }

    uint32_t t0 = read_cycle32();
    for (int i = 0; i < TRIG_COUNT; ++i) {
        sw_sin[i] = sinf(angles[i]);
        sw_cos[i] = cosf(angles[i]);
    }
    uint32_t sw_trig_cycles = read_cycle32() - t0;

    t0 = read_cycle32();
    for (int i = 0; i < TRIG_COUNT; ++i) {
        software_cordic_q14(phase_q13[i], &nofpu_sin_q14[i], &nofpu_cos_q14[i]);
    }
    uint32_t nofpu_trig_cycles = read_cycle32() - t0;

    static float nofpu_full_sin[TRIG_COUNT] __attribute__((aligned(64)));
    static float nofpu_full_cos[TRIG_COUNT] __attribute__((aligned(64)));
    t0 = read_cycle32();
    for (int i = 0; i < TRIG_COUNT; ++i) {
        int16_t phase = radian_to_fix16_13_bench(angles[i]);
        int16_t s_q14;
        int16_t c_q14;
        software_cordic_q14(phase, &s_q14, &c_q14);
        nofpu_full_sin[i] = (float)s_q14 * (1.0f / (float)(1 << 14));
        nofpu_full_cos[i] = (float)c_q14 * (1.0f / (float)(1 << 14));
    }
    uint32_t nofpu_full_cycles = read_cycle32() - t0;

    static int16_t hw_raw_sin_q14[TRIG_COUNT] __attribute__((aligned(64)));
    static int16_t hw_raw_cos_q14[TRIG_COUNT] __attribute__((aligned(64)));
    int raw_hw_before = g_hw_used;
    int raw_fallback_before = g_sw_fallback;
    int raw_mm2s_before = g_dma_timeout_mm2s;
    int raw_s2mm_before = g_dma_timeout_s2mm;
    t0 = read_cycle32();
    int cordic_raw_rc = cordic_batch_q13_q14(phase_q13, TRIG_COUNT, hw_raw_sin_q14, hw_raw_cos_q14);
    uint32_t cordic_raw_cycles = read_cycle32() - t0;

    int hw_before = g_hw_used;
    int fallback_before = g_sw_fallback;
    int mm2s_before = g_dma_timeout_mm2s;
    int s2mm_before = g_dma_timeout_s2mm;
    t0 = read_cycle32();
    cordic_batch_sin_cos(angles, TRIG_COUNT, hw_sin, hw_cos);
    uint32_t cordic_cycles = read_cycle32() - t0;

    float trig_max_err = 0.0f;
    float nofpu_max_err = 0.0f;
    float nofpu_full_max_err = 0.0f;
    float cordic_raw_max_err = 0.0f;
    for (int i = 0; i < TRIG_COUNT; ++i) {
        float ds = hw_sin[i] - sw_sin[i];
        float dc = hw_cos[i] - sw_cos[i];
        float e = sqrtf(ds * ds + dc * dc);
        if (e > trig_max_err) trig_max_err = e;

        float nofpu_s = (float)nofpu_sin_q14[i] * (1.0f / (float)(1 << 14));
        float nofpu_c = (float)nofpu_cos_q14[i] * (1.0f / (float)(1 << 14));
        float ns = nofpu_s - sw_sin[i];
        float nc = nofpu_c - sw_cos[i];
        float ne = sqrtf(ns * ns + nc * nc);
        if (ne > nofpu_max_err) nofpu_max_err = ne;

        float fs = nofpu_full_sin[i] - sw_sin[i];
        float fc = nofpu_full_cos[i] - sw_cos[i];
        float fe = sqrtf(fs * fs + fc * fc);
        if (fe > nofpu_full_max_err) nofpu_full_max_err = fe;

        if (cordic_raw_rc == 0) {
            float raw_s = (float)hw_raw_sin_q14[i] * (1.0f / (float)(1 << 14));
            float raw_c = (float)hw_raw_cos_q14[i] * (1.0f / (float)(1 << 14));
            float rs = raw_s - sw_sin[i];
            float rc = raw_c - sw_cos[i];
            float re = sqrtf(rs * rs + rc * rc);
            if (re > cordic_raw_max_err) cordic_raw_max_err = re;
        }
    }
    float trig_speedup = cordic_cycles > 0 ? (float)sw_trig_cycles / (float)cordic_cycles : 0.0f;
    float nofpu_speedup = nofpu_trig_cycles > 0 ? (float)nofpu_trig_cycles / (float)cordic_cycles : 0.0f;
    printf("TRIG_BENCH count=%d sw_cycles=%u nofpu_cycles=%u cordic_cycles=%u speedup=%.6f nofpu_speedup=%.6f max_err=%.9e nofpu_max_err=%.9e hw_used=%d sw_fallback=%d mm2s_to=%d s2mm_to=%d\n",
           TRIG_COUNT,
           (unsigned)sw_trig_cycles,
           (unsigned)nofpu_trig_cycles,
           (unsigned)cordic_cycles,
           trig_speedup,
           nofpu_speedup,
           trig_max_err,
           nofpu_max_err,
           g_hw_used - hw_before,
           g_sw_fallback - fallback_before,
           g_dma_timeout_mm2s - mm2s_before,
           g_dma_timeout_s2mm - s2mm_before);
    printf("TRIG_DETAIL nofpu_raw_cycles=%u nofpu_full_cycles=%u cordic_raw_cycles=%u cordic_raw_rc=%d nofpu_full_max_err=%.9e cordic_raw_max_err=%.9e cordic_raw_hw_used=%d cordic_raw_fallback=%d cordic_raw_mm2s_to=%d cordic_raw_s2mm_to=%d\n",
           (unsigned)nofpu_trig_cycles,
           (unsigned)nofpu_full_cycles,
           (unsigned)cordic_raw_cycles,
           cordic_raw_rc,
           nofpu_full_max_err,
           cordic_raw_max_err,
           g_hw_used - raw_hw_before,
           g_sw_fallback - raw_fallback_before,
           g_dma_timeout_mm2s - raw_mm2s_before,
           g_dma_timeout_s2mm - raw_s2mm_before);

    int use_snapshots = total_snapshots;
    if (use_snapshots > BENCH_MAX_SNAPSHOTS) use_snapshots = BENCH_MAX_SNAPSHOTS;
    int use_paths = num_paths;
    if (use_paths > BENCH_MAX_PATHS) use_paths = BENCH_MAX_PATHS;
    if (use_snapshots <= 0 || use_paths <= 0 || fs_signal_hz <= 0.0f) {
        printf("CONV_BENCH_ERROR invalid_input snapshots=%d paths=%d fs=%.0f\n",
               use_snapshots, use_paths, fs_signal_hz);
        printf("BENCH_END\n");
        fflush(stdout);
        return;
    }

    const int samples_per_snapshot = DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT;
    const int x_len = use_snapshots * samples_per_snapshot;
    const int y_len = x_len + BENCH_MAX_DELAY_SAMPLES;
    static float x[BENCH_MAX_SNAPSHOTS * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT] __attribute__((aligned(64)));
    static complex h[BENCH_MAX_SNAPSHOTS * BENCH_MAX_PATHS] __attribute__((aligned(64)));
    static float d_raw[BENCH_MAX_SNAPSHOTS * BENCH_MAX_PATHS] __attribute__((aligned(64)));
    static float d_rel[BENCH_MAX_SNAPSHOTS * BENCH_MAX_PATHS] __attribute__((aligned(64)));
    static complex y_sw[BENCH_MAX_SNAPSHOTS * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT + BENCH_MAX_DELAY_SAMPLES] __attribute__((aligned(64)));
    static complex y_pl[BENCH_MAX_SNAPSHOTS * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT + BENCH_MAX_DELAY_SAMPLES] __attribute__((aligned(64)));
    static complex h_batch[BENCH_BATCH_SNAPSHOTS * BENCH_MAX_PATHS] __attribute__((aligned(64)));
    static float d_batch[BENCH_BATCH_SNAPSHOTS * BENCH_MAX_PATHS] __attribute__((aligned(64)));
    static complex y_batch[BENCH_BATCH_SNAPSHOTS * DEMO_SPS * DEMO_SYMBOLS_PER_SNAPSHOT + BENCH_MAX_DELAY_SAMPLES] __attribute__((aligned(64)));

    unsigned int lfsr = 0xA5u;
    int num_symbols = x_len / DEMO_SPS;
    for (int symbol_idx = 0; symbol_idx < num_symbols; ++symbol_idx) {
        int bit = (int)(lfsr & 1u);
        unsigned int feedback = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 4)) & 1u;
        lfsr = (lfsr >> 1) | (feedback << 7);
        for (int k = 0; k < DEMO_SPS; ++k) {
            x[symbol_idx * DEMO_SPS + k] = bit ? 1.0f : 0.0f;
        }
    }

    for (int i = 0; i < y_len; ++i) {
        y_sw[i].real = 0.0f;
        y_sw[i].image = 0.0f;
        y_pl[i].real = 0.0f;
        y_pl[i].image = 0.0f;
    }

    for (int t = 0; t < use_snapshots; ++t) {
        float min_delay = 1.0f;
        int has_valid_delay = 0;
        for (int c = 0; c < use_paths; ++c) {
            float dt = D_in[t][c];
            complex ht = H_in[t][c];
            if (isfinite(dt) && dt > 0.0f && dt < 1.0f &&
                isfinite(ht.real) && isfinite(ht.image) &&
                (ht.real != 0.0f || ht.image != 0.0f)) {
                if (!has_valid_delay || dt < min_delay) {
                    min_delay = dt;
                    has_valid_delay = 1;
                }
            }
        }
        for (int c = 0; c < use_paths; ++c) {
            int idx = t * use_paths + c;
            h[idx].real = H_in[t][c].real;
            h[idx].image = H_in[t][c].image;
            d_raw[idx] = D_in[t][c];
            d_rel[idx] = has_valid_delay ? (D_in[t][c] - min_delay) : D_in[t][c];
        }
    }

    ch_conv_info_t sw_info;
    t0 = read_cycle32();
    int sw_rc = channel_convolve_segmented_real_input(x, x_len, samples_per_snapshot,
                                                      h, d_rel, use_snapshots, use_paths,
                                                      fs_signal_hz, y_sw, y_len, &sw_info);
    uint32_t sw_conv_cycles = read_cycle32() - t0;
    if (sw_rc != 0) {
        printf("CONV_BENCH_ERROR sw_rc=%d\n", sw_rc);
        printf("BENCH_END\n");
        fflush(stdout);
        return;
    }

    pl_conv_stats_t pl_stats;
    memset(&pl_stats, 0, sizeof(pl_stats));
    int old_verbose = g_pl_conv_verbose;
    g_pl_conv_verbose = 0;
    int pl_rc = 0;
    int pl_total_y_len = 0;
    int pl_sum_compact_paths = 0;
    int pl_max_compact_paths = 0;
    t0 = read_cycle32();
    for (int batch_start = 0; batch_start < use_snapshots; batch_start += BENCH_BATCH_SNAPSHOTS) {
        int batch_snapshots = use_snapshots - batch_start;
        if (batch_snapshots > BENCH_BATCH_SNAPSHOTS) batch_snapshots = BENCH_BATCH_SNAPSHOTS;
        int compact_paths = pl_compact_batch_paths(&h[batch_start * use_paths],
                                                   &d_raw[batch_start * use_paths],
                                                   batch_snapshots, use_paths,
                                                   h_batch, d_batch,
                                                   BENCH_MAX_PATHS);
        if (compact_paths <= 0) {
            pl_rc = compact_paths;
            break;
        }
        pl_sum_compact_paths += compact_paths;
        if (compact_paths > pl_max_compact_paths) {
            pl_max_compact_paths = compact_paths;
        }
        int batch_x_len = batch_snapshots * samples_per_snapshot;
        int batch_y_len = pl_estimate_required_y_len(h_batch, d_batch,
                                                     batch_snapshots, compact_paths,
                                                     samples_per_snapshot,
                                                     fs_signal_hz,
                                                     BENCH_MAX_DELAY_SAMPLES);
        if (batch_y_len <= 0) {
            batch_y_len = batch_x_len;
        }
        if (batch_y_len > batch_x_len + BENCH_MAX_DELAY_SAMPLES) {
            batch_y_len = batch_x_len + BENCH_MAX_DELAY_SAMPLES;
        }
        if (batch_y_len > (int)(sizeof(y_batch) / sizeof(y_batch[0]))) {
            batch_y_len = (int)(sizeof(y_batch) / sizeof(y_batch[0]));
        }
        pl_conv_stats_t batch_stats;
        int rc = pl_channel_convolve_segmented_q15(&x[batch_start * samples_per_snapshot],
                                                   h_batch, d_batch,
                                                   batch_snapshots, compact_paths,
                                                   samples_per_snapshot, fs_signal_hz,
                                                   y_batch, batch_y_len, &batch_stats);
        if (rc != 0) {
            pl_rc = rc;
            break;
        }
        int global_y_offset = batch_start * samples_per_snapshot;
        for (int i = 0; i < batch_y_len && global_y_offset + i < y_len; ++i) {
            y_pl[global_y_offset + i].real += y_batch[i].real;
            y_pl[global_y_offset + i].image += y_batch[i].image;
        }
        pl_stats.used_paths += batch_stats.used_paths;
        pl_stats.invalid_paths += batch_stats.invalid_paths;
        pl_stats.clipped_delay_paths += batch_stats.clipped_delay_paths;
        pl_stats.pack_cycles += batch_stats.pack_cycles;
        pl_stats.dma_setup_cycles += batch_stats.dma_setup_cycles;
        pl_stats.dma_wait_cycles += batch_stats.dma_wait_cycles;
        pl_stats.unpack_cycles += batch_stats.unpack_cycles;
        pl_stats.total_cycles += batch_stats.total_cycles;
        pl_stats.tx_words += batch_stats.tx_words;
        pl_total_y_len += batch_stats.y_len;
        if (batch_stats.max_delay_samples > pl_stats.max_delay_samples) {
            pl_stats.max_delay_samples = batch_stats.max_delay_samples;
        }
        if (batch_stats.h_scale > pl_stats.h_scale) {
            pl_stats.h_scale = batch_stats.h_scale;
        }
    }
    uint32_t pl_conv_cycles = read_cycle32() - t0;
    g_pl_conv_verbose = old_verbose;

    if (pl_rc != 0) {
        printf("CONV_BENCH_ERROR pl_rc=%d\n", pl_rc);
        printf("BENCH_END\n");
        fflush(stdout);
        return;
    }

    pl_channel_compare_outputs(y_sw, y_pl, y_len, &pl_stats);
    float conv_speedup = pl_conv_cycles > 0 ? (float)sw_conv_cycles / (float)pl_conv_cycles : 0.0f;
    printf("CONV_BENCH snapshots=%d paths=%d x_len=%d y_len=%d sw_cycles=%u pl_cycles=%u speedup=%.6f max_abs_err=%.9e rmse=%.9e nmse_db=%.3f sw_used_paths=%d pl_used_paths=%d pack_cycles=%u dma_setup_cycles=%u dma_wait_cycles=%u unpack_cycles=%u pl_inner_cycles=%u tx_words=%d pl_y_words=%d clipped_delay_paths=%d max_delay=%d compact_path_sum=%d compact_path_max=%d\n",
           use_snapshots, use_paths, x_len, y_len,
           (unsigned)sw_conv_cycles,
           (unsigned)pl_conv_cycles,
           conv_speedup,
           pl_stats.max_abs_err,
           pl_stats.rmse,
           pl_stats.nmse_db,
           sw_info.used_paths,
           pl_stats.used_paths,
           (unsigned)pl_stats.pack_cycles,
           (unsigned)pl_stats.dma_setup_cycles,
           (unsigned)pl_stats.dma_wait_cycles,
           (unsigned)pl_stats.unpack_cycles,
           (unsigned)pl_stats.total_cycles,
           pl_stats.tx_words,
           pl_total_y_len,
           pl_stats.clipped_delay_paths,
           pl_stats.max_delay_samples,
           pl_sum_compact_paths,
           pl_max_compact_paths);
    printf("BENCH_END\n");
    fflush(stdout);
}

static void system_init(void) {
    uart_init();
    dma_reset();
    cordic_math_init();
}

static void compute_cluster_response(const float *phase_angles, int rays, complex *out) {
    float sin_out[CLUSTER_RAYS] = {0};
    float cos_out[CLUSTER_RAYS] = {0};
    cordic_batch_sin_cos(phase_angles, rays, sin_out, cos_out);
    out->real = 0.0f;
    out->image = 0.0f;
    for (int i = 0; i < rays; ++i) {
        out->real += cos_out[i];
        out->image += sin_out[i];
    }
}

static void compute_cluster_responses_batched(const float *phase_angles, int clusters, complex *out) {
    const int max_clusters_per_dma = CORDIC_DRIVER_MAX_BATCH / CLUSTER_RAYS;
    static float sin_out[CORDIC_DRIVER_MAX_BATCH] __attribute__((aligned(64)));
    static float cos_out[CORDIC_DRIVER_MAX_BATCH] __attribute__((aligned(64)));
    int done = 0;

    while (done < clusters) {
        int batch_clusters = clusters - done;
        if (batch_clusters > max_clusters_per_dma) {
            batch_clusters = max_clusters_per_dma;
        }

        const int angle_count = batch_clusters * CLUSTER_RAYS;
        cordic_batch_sin_cos(&phase_angles[done * CLUSTER_RAYS], angle_count, sin_out, cos_out);

        for (int c = 0; c < batch_clusters; ++c) {
            float real_sum = 0.0f;
            float imag_sum = 0.0f;
            const int base = c * CLUSTER_RAYS;
            for (int i = 0; i < CLUSTER_RAYS; ++i) {
                real_sum += cos_out[base + i];
                imag_sum += sin_out[base + i];
            }
            out[done + c].real = real_sum;
            out[done + c].image = imag_sum;
        }

        done += batch_clusters;
    }
}
// 假设这些外部变量存在，与原环境保持一致

void Angle(float* h_t_track, float* h_r_track, int num_points, float* Theta_LoS) {
    for (int i = 0; i < num_points; i++) {
    	float x1 = h_r_track[i * 3];    // 接收机轨迹的 x 坐标
    	float y1 = h_r_track[i * 3 + 1]; // 接收机轨迹的 y 坐标
    	float x2 = h_t_track[i * 3];    // 发射机轨迹的 x 坐标
    	float y2 = h_t_track[i * 3 + 1]; // 发射机轨迹的 y 坐标

        // 计算角度
        Theta_LoS[i] = atan2(y2 - y1, x2 - x1);
    }
}
float PJ_Model(float z_look, float Ta, float Ts, float U) {
    // 1. 计算 Rib (使用输入参数 Ta, Ts, U)
    // MATLAB: Rib = (98*z_look*(Ta-Ts))/((U^2)*Ta);
    float Rib = (98.0f * z_look * (Ta - Ts)) / ((U * U) * Ta);

    // 2. 计算 Hd (分段函数)
    float Hd = 0.0f;
    if (Rib <= -3.75f) {
        Hd = 0.05f;
    } else if (Rib > -3.75f && Rib <= -0.12f) {
        Hd = 0.065f + 0.004f + Rib;
    } else if (Rib > -0.12f && Rib <= 0.14f) {
        Hd = 0.109f + 0.367f * Rib;
    } else { // Rib > 0.14
        Hd = 0.155f + 0.021f * Rib;
    }

    // 3. 计算 L 和 x
    // MATLAB: L = (10*z_look*Hd)/Rib;
    float L = (10.0f * z_look * Hd) / Rib;
    float x = z_look / L;
    float z0 = 1.5e-4f;

    // 4. 计算空气和海表参数
    // 严格按照 MATLAB 代码硬编码这些值，覆盖/不使用输入的 Ta/Ts 计算这部分
    float Rh = 70.0f;
    float Tok = 273.2f + 25.3f;
    float Tsk = 273.2f + 20.0f;

    // 计算 e
    // MATLAB: e = (Rh/100)*(6.105*exp(25.22*((Tok-273.2)/Tok)-5.31*log(Tok/273.2)));
    float term_e_exp = 25.22f * ((Tok - 273.2f) / Tok) - 5.31f * log(Tok / 273.2f);
    float e = (Rh / 100.0f) * (6.105f * exp(term_e_exp));

    // 计算 Na
    // MATLAB: Na = (77.6/Tok)*(1000+4810*(e/Tok));
    float Na = (77.6f / Tok) * (1000.0f + 4810.0f * (e / Tok));

    // 计算 es
    // MATLAB: es = 6.105*exp(25.22*((Tsk-273.2)/Tsk)-5.31*log(Tsk/273.2));
    float term_es_exp = 25.22f * ((Tsk - 273.2f) / Tsk) - 5.31f * log(Tsk / 273.2f);
    float es = 6.105f * exp(term_es_exp);

    // 计算 Ns
    // MATLAB: Ns = (77.6/Tsk)*(1000+4810*(es/Tsk));
    float Ns = (77.6f / Tsk) * (1000.0f + 4810.0f * (es / Tsk));

    // MATLAB: Np = Na-Ns;
    float Np = Na - Ns;

    // 5. 计算波导高度 zd
    // MATLAB: zd = Np/(-0.125*(log(z_look/z0)+5.2*x)-5.2*(Np/L));
    float denom_zd = -0.125f * (log(z_look / z0) + 5.2f * x) - 5.2f * (Np / L);
    float zd = Np / denom_zd;

    // 6. 计算 dM_dz
    // MATLAB: dM_dz = 0.13-((0.13*zd)/(1+(5.2*zd/L)))*((L+5.2*z0)/(zd*L+z0*L+5.2*z0*zd));
    float term_dM_1 = (0.13f * zd) / (1.0f + (5.2f * zd / L));
    float term_dM_2 = (L + 5.2f * z0) / (zd * L + z0 * L + 5.2f * z0 * zd);
    float dM_dz = 0.13f - term_dM_1 * term_dM_2;

    // 7. 计算 dn_dz
    // MATLAB: dn_dz = (dM_dz-0.157)*10^(-6);
    float dn_dz = (dM_dz - 0.157f) * 1e-6f;

    // 8. 计算 Theta
    // MATLAB: Re = 6400000;
    float Re = 6400000.0f;

    // MATLAB: Theta = sqrt(2*((1/1.00035)*dn_dz-(1/Re))*(10-zd));
    float val_inside = 2.0f * ((1.0f / 1.00035f) * dn_dz - (1.0f / Re)) * (10.0f - zd);

    float Theta;
    // 处理负数开根号的情况（MATLAB返回复数，C返回NaN，物理上通常取0）
    if (val_inside < 0.0f) {
        Theta = 0.0f;
    } else {
        Theta = sqrt(val_inside);
    }

    // MATLAB: Theta_duct = Theta*180/pi;
    float Theta_duct = Theta * 180.0f / M_PI;

    return Theta_duct;
}
void truncated_gaussian(float mu, float sigma, float a, float b, int Nc0, float* AAOD) {
    // ---------------------------------------------------------
    // 1. 计算截断区间的 CDF 值 (内联 normcdf)
    //    normcdf(x) = 0.5 * (1 + erf((x - mu) / (sigma * sqrt(2))))
    // ---------------------------------------------------------
    float sqrt2 = 1.41421356f;

    // 计算 cdf_a
    // 注意：使用 erff (float版本的erf)
    float arg_a = (a - mu) / (sigma * sqrt2);
    float cdf_a = 0.5f * (1.0f + erff(arg_a));

    // 计算 cdf_b
    float arg_b = (b - mu) / (sigma * sqrt2);
    float cdf_b = 0.5f * (1.0f + erff(arg_b));

    float cdf_diff = cdf_b - cdf_a;

    // ---------------------------------------------------------
    // 准备 norminv 计算系数 (全部转为 float)
    // ---------------------------------------------------------
    const float a_cof[4] = { 2.50662823f, -18.6150006f, 41.3911977f, -25.4410605f };
    const float b_cof[4] = { -8.47351093f, 23.0833674f, -21.0622410f, 3.13082910f };
    const float c_cof[9] = { 0.33747548f, 0.97616902f, 0.16079797f,
                             0.02764388f, 0.00384057f, 0.00039519f,
                             0.00003218f, 0.00000029f, 0.00000040f };

    // ---------------------------------------------------------
    // 2. 循环生成
    // ---------------------------------------------------------
    for (int i = 0; i < Nc0; i++) {
        // [关键替换] 使用外部预置的 float 随机数

        float p_rand = rand_gen_number[rand_gen_count++];

        // 计算目标累积概率
        float target_p = cdf_a + p_rand * cdf_diff;

        // 边界保护 (使用 float 精度)
        if (target_p <= 1e-7f) target_p = 1e-7f;
        if (target_p >= (1.0f - 1e-7f)) target_p = 1.0f - 1e-7f;

        // -----------------------------------------------------
        // 3. 计算标准正态分布逆函数 (内联 norminv)
        // -----------------------------------------------------
        float z_std;
        float y = target_p - 0.5f;

        if (fabsf(y) < 0.42f) {
            // 中心区域
            float r = y * y;
            z_std = y * (((a_cof[3] * r + a_cof[2]) * r + a_cof[1]) * r + a_cof[0]) /
                        ((((b_cof[3] * r + b_cof[2]) * r + b_cof[1]) * r + b_cof[0]) * r + 1.0f);
        } else {
            // 尾部区域
            float r = target_p;
            if (y > 0.0f) r = 1.0f - target_p;

            // 使用 logf (float版本)
            r = log(-log(r));
            z_std = c_cof[0] + r * (c_cof[1] + r * (c_cof[2] + r * (c_cof[3] + r * (c_cof[4] +
                    r * (c_cof[5] + r * (c_cof[6] + r * (c_cof[7] + r * c_cof[8])))))));

            if (y < 0.0f) z_std = -z_std;
        }

        // -----------------------------------------------------
        // 4. 转换回一般正态分布
        // -----------------------------------------------------
        AAOD[i] = mu + sigma * z_std;
    }
}
void init_parameters(int r_track_type,int r_track_length,int r_samprate,int r_speed,int t_speed,int t_track_type,int fc) {
    para.M_T = MAX_MT;
    para.nLayer = 1;
    para.M_R = MAX_MR;
    para.fc = fc;
    para.c = 300000000;
    para.lambda = para.c / para.fc;
    para.rand_seed = 48;
    para.randn_seed = para.rand_seed ;//+ 20
    para.D_g = 10;
    para.square[0] = 200;//200
    para.square[1] = 200;
    para.altitudes = 20;
    para.delet_altitude = 5;
    // //fig5
    // para.r_track_type = 00;//00静止
    // para.r_track_length = 20;
    // para.r_speed = 2;
    // para.r_accel = 0;
    // para.r_samp_rate = 2;
    // para.r_ini_pos[0] = 1.0;
    // para.r_ini_pos[1] = 1.0;
    // para.r_ini_pos[2] = 1.0;
    // para.r_ini_dir[0] = 1.0;
    // para.r_ini_dir[1] = 0.0;
    // para.r_ini_dir[2] = 0.0;
    // para.T_length = MAX_TIME;
    // para.t_track_type = 02; //02圆形
    // para.t_track_length = 20;
    // para.t_speed = 4;
    // para.t_accel = 0;
    // para.t_samp_rate = 2;
    // para.t_ini_pos[0] = 80.0;
    // para.t_ini_pos[1] = 80.0;
    // para.t_ini_pos[2] = 5.0;
    // para.t_ini_dir[0] = 1.0;
    // para.t_ini_dir[1] = 1.0;
    // para.t_ini_dir[2] = 1.0;
    //fig6
    //  para.r_track_type = 00;//00静止
    // para.r_track_length = 20;
    // para.r_speed = 2;
    // para.r_accel = 0;
    // para.r_samp_rate = 2;
    // para.r_ini_pos[0] = 1.0;
    // para.r_ini_pos[1] = 1.0;
    // para.r_ini_pos[2] = 1.0;
    // para.r_ini_dir[0] = 1.0;
    // para.r_ini_dir[1] = 0.0;
    // para.r_ini_dir[2] = 0.0;
    // para.T_length = MAX_TIME;
    // para.t_track_type = 01; //01线形
    // para.t_track_length = 20;
    // para.t_speed = 4;
    // para.t_accel = 0;
    // para.t_samp_rate = 2;
    // para.t_ini_pos[0] = 80.0;
    // para.t_ini_pos[1] = 80.0;
    // para.t_ini_pos[2] = 5.0;
    // para.t_ini_dir[0] = 0.0;
    // para.t_ini_dir[1] = 0.0;
    // para.t_ini_dir[2] = 1.0;
    //fig7
    para.r_track_type = r_track_type;//01线性
    para.r_track_length = r_track_length;
    para.r_speed = r_speed;
    para.r_accel = 0;
    para.r_samp_rate = r_samprate;
    para.r_ini_pos[0] = 0;
    para.r_ini_pos[1] = 5;
    para.r_ini_pos[2] = 10;
    para.r_ini_dir[0] = 1.0;
    para.r_ini_dir[1] = 0.0;
    para.r_ini_dir[2] = 0.0;
    para.T_length = MAX_TIME;
    para.t_track_type = t_track_type;//02螺旋
    para.t_track_length = r_track_length;
    para.t_speed =t_speed;
    para.t_accel = 0;
    para.t_samp_rate = r_samprate;
    para.t_ini_pos[0] = 200.0;
    para.t_ini_pos[1] = 5.0;
    para.t_ini_pos[2] = 20.0;
    para.t_ini_dir[0] = 1.0;
    para.t_ini_dir[1] = 0.0;
    para.t_ini_dir[2] = 0.0;
    para.t_distance = para.t_track_length * para.t_speed + para.t_track_length * para.t_accel / 2;
    //para.D = 0;
    para.Corr_distance = 40;
    para.nu_A_T = pi / 3;
    para.nu_E_T = pi / 4;
    para.nu_A_R = pi / 4;
    para.nu_E_R = pi / 4;
    para.theta_v_R = pi / 3;
    para.xi_v_R = 0;
    para.theta_v_T = pi / 3;
    para.xi_v_T = 0;
    para.delta_tx = 0.5;
    para.delta_rx = 0.5;
    // para.K_ricean_db_h_M = 0;
    // para.K_ricean_db_l_M = 0;
    // para.K_ricean_lin = 0;
    para.lambdaG = 20;
    para.lambdaR = 1;
    para.DS_miu_h = -6.5;
    para.DS_sigma_h = 0.3;
    para.DS_miu_l = -7;
    para.DS_sigma_l = 0.4;
    para.DS_r = 2.5;
    para.d_Rx_min = 10;
    para.d_Rx_mean = 15;
    para.d_Tx_min = 10;
    para.d_Tx_mean = 20;
    para.tau_tilder_mean = 44.1e-9;
    para.tau_tilder_min = 27.3e-9;
    para.mean_num_subpath = 15;
    para.mean_delay_subpath = 15e-9;
    para.angular_variance_subpath = 1;
    para.A_Corr_distance = 30;
    para.T_Corr_distance = 5;
    para.T0 = 4;
    para.Tend = 4.8;
    para.dT = 0.001;

    para.P_F = 0.3;
    para.v_F = 0.5;
    para.v_c = 0.5;
    para.v_R = 25;
    para.v_T = 25;
    para.Per_cluster_shadowing = 3;
    para.Polarization_Tx = 0;
    para.Polarization_Rx = 0;
    para.Polar_kappa = -8;
    para.Polar_kappa_linear = pow(10, para.Polar_kappa / 10);
    //para.FieldPatternTx = OmniFieldPattern;
    //para.FieldPatternRx = OmniFieldPattern;
    para.Tx_Rot_x = 0;
    para.Tx_Rot_y = 0;
    para.Tx_Rot_z = 0;
    para.Rx_Rot_x = 0;
    para.Rx_Rot_y = 0;
    para.Rx_Rot_z = 0;
    para.Tsigmax = 1;
    para.Tsigmay = 1;
    para.Tsigmaz = 1;
    para.Rsigmax = 1;
    para.Rsigmay = 1;
    para.Rsigmaz = 1;
    para.sigmaxvar = 0.1;
    para.sigmayvar = 0.1;
    para.sigmazvar = 0.1;
}
// 生成正态分布随机数
float NorDis(float mu, float sigma) {
    // srand(para.randn_seed);    
    float u1 = randn_gen_number[randn_gen_count++];

        return u1;
}
float NorDis_2(float mu, float sigma) {
    // srand(para.randn_seed);    
    float u1 = rand_gen_number[rand_gen_count++];
    float u2 = rand_gen_number[rand_gen_count++];
    // 确保 u1 和 u2 在有效范围内
    if (u1 <= 0 || u1 >= 1) {
        u1 = 1e-10; // 避免除以零或对数非法值
    }
    if (u2 <= 0 || u2 >= 1) {
        u2 = 1e-10; // 避免除以零或对数非法值
    }

    return mu + sigma * sqrt(-2 * log(u1)) * cos(2 * pi * u2);
}
float NorDis3(float mu, float sigma) {
    // srand(para.randn_seed);
    float u1 = randn_gen_number[randn_gen_count++];

        return u1*sigma+mu;
}
void generatez(float* aa, float* bb,float* cc,int t, int Numpoint, float* z) {
    int I = 10;
    float rad[Numpoint * I];
    float w[Numpoint * I];
    float p[Numpoint * I];
    float h[Numpoint];
    // float tscale [Numpoint];
    // for (int t = 0; t < Numpoint; t++) {
    //     tscale[t] = t / Numpoint;
    // }

    // // 初始化随机数生成器
    // srand(48);
 for (int i=0;i<Numpoint;i++){
	 h[i]=0;
 }
 for (int i = 0; i < I; i++) {
        for (int j = 0; j < Numpoint; j++) {
            rad[j * I + i] = (rand_gen_number[rand_gen_count++]) * 2 * M_PI;
        }
    }
            //     printf("rad0 = %.16f",rad[0]/2/pi);
            // printf("rad1 = %.16f",rad[1]/2/pi);
            // printf("rad2 = %.16f",rad[2]/2/pi);
            // printf("rad11 = %.16f",rad[11]/2/pi);
            // printf("rad202 = %.16f",rad[202]/2/pi);
    for (int i = 0; i < I; i++) {
        for (int j = 0; j < Numpoint; j++) {

            w[j * I + i] = (rand_gen_number[rand_gen_count++]) * 10;
        }
    }
    for (int i = 0; i < I; i++) {
        for (int j = 0; j < Numpoint; j++) {

            p[j * I + i] = (rand_gen_number[rand_gen_count++]) * 2 * M_PI;
        }
    }



    for (int i = 0; i < Numpoint; i++) {
        // float t = tscale[i];
    	for (int j = 0; j < I - 1; j++) {
    	            int idx = i * I + j;
    	            float S1 = (0.78 / pow(w[idx], 5)) * exp(-0.74 * pow((9.8 / (5 * w[idx])), 4));
    	            float Q = exp(-pow((w[idx] * 5 / 9.8), 4) / 2);
    	            float D = (1 / M_PI) * (1 + (0.5 + 0.82 * Q) * cos(2 * p[idx]) + 0.32 * Q * cos(4 * p[idx]));
    	            float S = S1 * D;
    	            float W = w[idx] - w[idx + 1];
    	            float P = p[idx] - p[idx + 1];
    	            float a = sqrt(fabs(2 * S * W * P));
    	            float k = pow(w[idx], 2) / 9.8;
    	            float h1 = a * cos(w[idx] * t - k * (aa[i] * cos(p[idx]) + bb[i] * sin(p[idx])) + rad[idx]);

    	            h[j] = h1;
    	        }


    	// 处理最后一个波浪分量
    	        int last_idx = i *I + I - 1;
    	        float S1 = (0.78 / pow(w[last_idx], 5)) * exp(-0.74 * pow((9.8 / (5 * w[last_idx])), 4));
    	        float Q = exp(-pow((w[last_idx] * 5 / 9.8), 4) / 2);
    	        float D = (1 / M_PI) * (1 + (0.5 + 0.82 * Q) * cos(2 * p[last_idx]) + 0.32 * Q * cos(4 * p[last_idx]));
    	        float S = S1 * D;
    	        float W = w[last_idx] - w[last_idx - 1];
    	        float P = p[last_idx] - p[last_idx - 1];
    	        float a = sqrt(fabs(2 * S * W * P));
    	        float k = pow(w[i * I], 2) / 9.8;
    	        float h1 = a * cos(w[last_idx] * t - k * (aa[i] * cos(p[last_idx]) +  bb[i] * sin(p[i * I])) + rad[i * I]);
    	        h[0] = h1;

    	        // 计算总高度
    	        float sum_h = 0.0;
    	        for (int j = 0; j < I; j++) {
    	            sum_h += h[ j];
    	        }
    	        z[i] = sum_h;

    }

}
void generateh(int num_points,int samp_rate, float* h_r_track,  float* H) {

    // float c = 300000000;
        int I = 10;
    float rad[num_points * I];
    float w[num_points * I];
    float p[num_points * I];
    float h[num_points * I];
    float tscale [num_points];
    for (int t = 0; t < num_points; t++) {
        tscale[t] = (float)t / samp_rate;
    }
// printf("rand_gen_count = %d",rand_gen_count);
    // 生成随机数

    for (int i = 0; i < I; i++) {
        for (int j = 0; j < num_points; j++) {
            rad[j * I + i] = (rand_gen_number[rand_gen_count++]) * 2 * M_PI;
            //printf("rad[%d][%d]=%f\n",j,i,rad[j * I + i]);
        }
    }
            //     printf("rad0 = %.16f",rad[0]/2/pi);
            // printf("rad1 = %.16f",rad[1]/2/pi);
            // printf("rad2 = %.16f",rad[2]/2/pi);
            // printf("rad11 = %.16f",rad[11]/2/pi);
            // printf("rad202 = %.16f",rad[202]/2/pi);
    for (int i = 0; i < I; i++) {
        for (int j = 0; j < num_points; j++) {

            w[j * I + i] = (rand_gen_number[rand_gen_count++]) * 10;
        }
    }
    for (int i = 0; i < I; i++) {
        for (int j = 0; j < num_points; j++) {

            p[j * I + i] = (rand_gen_number[rand_gen_count++]) * 2 * M_PI;
        }
    }


    // 计算海浪高度
    for (int i = 0; i < num_points; i++) {
        float t = tscale[i];
        for (int j = 0; j < I - 1; j++) {
            int idx = i * I + j;
            float S1 = (0.78 / pow(w[idx], 5)) * exp(-0.74 * pow((9.8 / (2 * w[idx])), 4));
            float Q = exp(-pow((w[idx] * 5 / 9.8), 4) / 2);
            float D = (1 / M_PI) * (1 + (0.5 + 0.82 * Q) * cos(2 * p[idx]) + 0.32 * Q * cos(4 * p[idx]));
            float S = S1 * D;
            float W = w[idx] - w[idx + 1];
            float P = p[idx] - p[idx + 1];
            float a = sqrt(fabs(2 * S * W * P));
            float k = pow(w[idx], 2) / 9.8;
            float h1 = a * cos(w[idx] * t - k * (h_r_track[i * 3] * cos(p[idx]) + h_r_track[i * 3 + 1] * sin(p[idx])) + rad[idx]);
            h[idx] = h1;
        }

        // 处理最后一个波浪分量
        int last_idx = i *I + I - 1;
        float S1 = (0.78 / pow(w[last_idx], 5)) * exp(-0.74 * pow((9.8 / (2 * w[last_idx])), 4));
        float Q = exp(-pow((w[last_idx] * 5 / 9.8), 4) / 2);
        float D = (1 / M_PI) * (1 + (0.5 + 0.82 * Q) * cos(2 * p[last_idx]) + 0.32 * Q * cos(4 * p[last_idx]));
        float S = S1 * D;
        float W = w[last_idx] - w[last_idx - 1];
        float P = p[last_idx] - p[last_idx - 1];
        float a = sqrt(fabs(2 * S * W * P));
        float k = pow(w[i * I], 2) / 9.8;
        float h1 = a * cos(w[last_idx] * t - k * (h_r_track[i * 3] * cos(p[last_idx]) + h_r_track[i * 3 + 1] * sin(p[i * I])) + rad[i * I]);
        h[i * I] = h1;

        // 计算总高度
        float sum_h = 0.0;
        for (int j = 0; j < I; j++) {
            sum_h += h[i * I + j];
        }
        H[i] = sum_h;
      //  printf("H[%d]=%f\n",i,H[i]);
    }
}
void markov_mine(float* p, float* p0, int length, int* chain) {
    int state = 0; // 初始状态
    float choice = rand_gen_number[rand_gen_count++];
    // int r = rand() % 100;
    // float r1 = (float)r/100;
    // printf("choice = %.16f\n",choice);
    float sum = 0.0;
        // float r1 = rand_gen_number[rand_gen_count++];
    float sum_p = 0.0;

    // 使用 p0 选择初始状态
    //temp=find(prob>=choice);%找到累积概率大于这个随机数的位置
    for (int j = 0; j < 180; j++) {
        sum += p0[j];
        // printf("sum = %.16f\n",sum);
        if (choice <= sum) {
            state = j ;
            break;
        }
    }
    float r2[length];
    chain[0] = state;
    for(int i=0;i<length-1;i++){
    	r2[i]=rand_gen_number[rand_gen_count++];
    	//printf("r2[%d]=%f\n",i,r2[i]);
    }

    for (int i = 1; i < length; i++) {

       sum_p = 0.0;
    //    float r2 = rand_gen_number[rand_gen_count++];
        // int r_tmp = rand() % 100;
            // float r2 = (float)r_tmp/100;
                //  printf("r2 = %.16f\n",r2);
           for (int j = 0; j < 180; j++) {
            sum_p += p[(chain[i - 1]) * 180 + j];
            if (r2[i - 1] <= sum_p) {
                state = j;
                break;
            }
        }
        chain[i] = state;

        // printf("chain[%d] = %d\n",i,chain[i]);
    }
}
int poissrnd(float lambda) {
    float L = exp(-lambda);
    float p = 1.0;
    int k = 0;

    do {
        p *= rand_gen_number[rand_gen_count++];
        k++;
    } while (p > L);
    return k - 1;
}
// 轨迹生成子函数 //const char* track_type
void generate_uav_track(int track_type, float track_t, float speed, float accel, float* ini_pos, float* ini_dir, float samp_rate, float* track) {
    // 归一化方向向量
    float norm = sqrt(ini_dir[0] * ini_dir[0] + ini_dir[1] * ini_dir[1] + ini_dir[2] * ini_dir[2]);
    // printf("norm=%.16f \n", norm);
    // printf("ini_dir[3]=%.16f  %.16f  %.16f\n", ini_dir[0],ini_dir[1],ini_dir[2]);
    float ini_dir1[3] ={0.0};
    if (norm != 0.0) {
        ini_dir1[0] = ini_dir[0]/norm;
        ini_dir1[1] = ini_dir[1]/norm;
        ini_dir1[2] = ini_dir[2]/norm;
    }
// printf("ini_dir1[3]=%.16f  %.16f  %.16f\n", ini_dir1[0],ini_dir1[1],ini_dir1[2]);
    int num_points = (int)(track_t * samp_rate) + 1;
    float tscale [num_points];
    for (int t = 0; t < num_points; t++) {
        tscale[t] = t / samp_rate;
    }

    if (track_type == 00) {//strcmp(track_type, "static") == 0
        for (int t = 0; t < 3 * num_points; t = t + 3) {
                track[t] = ini_pos[0];
                track[t+1] = ini_pos[1];
                track[t+2] = ini_pos[2];              
        }
    }
    else if (track_type == 01) {//strcmp(track_type, "linear") == 0
        track[0] = ini_pos[0];
        track[1] = ini_pos[1];
        track[2] = ini_pos[2];

        for (int t = 3; t < 3 * num_points; t = t + 3) {
            int i = ceil(t/3);
            float delta_t = 1.0 / samp_rate;
            float current_speed = speed + accel * (tscale[i]);
            track[t] = track[t - 3] + delta_t * current_speed * ini_dir1[0];
            track[t + 1] = track[t - 2] + delta_t * current_speed * ini_dir1[1];
            track[t + 2] = track[t - 1] + delta_t * current_speed * ini_dir1[2];
        }
    }
    else if (track_type == 02) {//strcmp(track_type, "circle") == 0

        float r_vect[3] = { ini_dir1[1], -ini_dir1[0], 0 };
        //  printf("r_vect[3]=%.16f  %.16f  %.16f\n", r_vect[0],r_vect[1],r_vect[2]);
        float track_length = track_t * (2 * speed + accel * track_t) / 2;
    //   printf("track_length=%.16f\n", track_length);        
        float radius = track_length / (2 * pi);
        // printf("radius=%.16f\n", radius); 
        float center[3];
        center[0] = ini_pos[0] + radius * r_vect[0];
        center[1] = ini_pos[1] + radius * r_vect[1];
        center[2] = ini_pos[2] + radius * r_vect[2];
// printf("center[3]=%.16f  %.16f  %.16f\n", center[0],center[1],center[2]);
        float rad_tmp[num_points];
        float rad[num_points];
        rad_tmp[0] = 0.0;
        float norm_radius= sqrt((radius * r_vect[0]) * (radius * r_vect[0]) + (radius * r_vect[1]) * (radius * r_vect[1]) + (radius * r_vect[2]) * (radius * r_vect[2]));
        // printf("norm_radius=%.16f\n", norm_radius);
        for (int t = 1; t <  num_points; t++) {
            float delta_t = 1.0 / samp_rate;
            rad_tmp[t] = (rad_tmp[t - 1] + (speed + accel * (tscale[t]) )* delta_t / norm_radius) ;
            // printf("rad_tmp[%d]=%.16f\n", t,rad_tmp[t]);
        }
        for (int t = 0; t <  num_points; t++) {
            rad[t] = rad_tmp[t]*1.6 ;
            // printf("rad[%d]=%.16f\n", t,rad[t]);
        }

        for (int t = 0; t < 3 * num_points; t = t + 3) {
            int i = ceil(t/3);
            track[t] = center[0] + (-cos(rad[i]) * radius * r_vect[0] - sin(rad[i]) * radius * r_vect[1]);
            track[t + 1] = center[1] + (sin(rad[i]) * radius * r_vect[0] - cos(rad[i]) * radius * r_vect[1]);
            track[t + 2] = center[2] + (0.2 * i);
            // printf("i = %d\n",i);
        }
    }
    else if (track_type == 03) {//strcmp(track_type, "semicircle") == 0

        float v_vertical=2;
        float r_vect[3] = { ini_dir[1], -ini_dir[0], 0 };
        float track_length = track_t * (2 * speed + accel * track_t) / 2;
        float radius = track_length / (2 * pi);
        float center[3];
        center[0] = ini_pos[0] + radius * r_vect[0];
        center[1] = ini_pos[1] + radius * r_vect[1];
        center[2] = ini_pos[2] + radius * r_vect[2];

        float rad[num_points];
        rad[0] = 0.0;
        float norm_radius= sqrt((radius * r_vect[0]) * (radius * r_vect[0]) + (radius * r_vect[1]) * (radius * r_vect[1]) + (radius * r_vect[2]) * (radius * r_vect[2]));
        for (int t = 1; t <  num_points; t++) {
            float delta_t = 1.0 / samp_rate;
            float current_speed = speed + accel * (tscale[t]);
            rad[t] = rad[t - 1] + (current_speed * delta_t) / norm_radius;
        }
        // 将角度归一化到半圆范围内并乘以 π
        float last_rad = rad[num_points - 1];
        for (int t = 0; t < num_points; t++) {
         rad[t] = (rad[t] / last_rad) * pi;
        }

        for (int t = 0; t < 3 * num_points; t = t + 3) {
            int i = ceil(t/3);
            track[t] = center[0] + (-cos(rad[i]) * radius * r_vect[0] - sin(rad[i]) * radius * r_vect[1]);
            track[t + 1] = center[1] + (sin(rad[i]) * radius * r_vect[0] - cos(rad[i]) * radius * r_vect[1]);
            track[t + 2] = center[2] + v_vertical/samp_rate;
            // printf("i = %d\n",i);
        }
    }
    else if (track_type == 04) {//strcmp(track_type, "markov") == 0
            // 初始化马尔可夫链参数
            float p0[180];
            for (int i = 0; i < 180; i++) {
              p0[i] = 1.0 / 180.0;
               // printf("p0[%d] = %.16f\n",i,p0[i]);
              }
            float r[5];
             // 生成泊松分布随机数
            for (int i = 0; i < 5; i++) {
                   // 调用泊松生成函数
                   r[i] = poissrnd(3);
//                   printf("r[%d]=%f",i,r[i]);
               }

             int sum_r = 0;
             float r_1[5];
             for (int i = 0; i < 5; i++) {
            //    r[i] = poissrnd(3);
            //    printf("r[%d] = %f\n",i,r[i]);
               sum_r += r[i];
             }
             for (int i = 0; i < 5; i++) {
               r_1[i] = r[i] / sum_r;//(float)
            //    printf("r_1[%d] = %.16f\n",i,r_1[i]);
             }

             // 初始化转移概率矩阵 p
             float p[180 * 180] = {0.0};
             p[0 * 180 + 0] = r_1[2];
             p[0 * 180 + 1] = r_1[3];
             p[0 * 180 + 2] = r_1[4];
             p[0 * 180 + 178] = r_1[0];
             p[0 * 180 + 179] = r_1[1];


        // 生成后续行
        for (int i = 1; i < 180; i++) {
            //第一列是上一行的最后一列
            p[i * 180] = p[(i - 1) * 180 + 179 ]; // 循环移位
            //其余列
            for (int j = 1; j < 180; j++) {
                p[i * 180 + j] = p[(i - 1) * 180 + (j - 1) ]; // 循环移位
                //  printf("p[%d] = %.16f\n",i * 180 + j,p[i * 180 + j]);
            }
        }


            // 生成马尔可夫链
            int chain[num_points];
            markov_mine(p, p0, num_points, chain);

            // 初始化方向向量
            float ini_dir1[num_points * 3];
            for (int i = 0; i < num_points; i++) {
            	float angle = (chain[i] + 1) / 180.0 * pi;

                ini_dir1[i * 3 + 0] = cos(angle) * sin(angle);
                ini_dir1[i * 3 + 1] = sin(angle) * sin(angle);
                ini_dir1[i * 3 + 2] = cos(angle);
            }

            // 生成轨迹点
            track[0] = ini_pos[0];
            track[1] = ini_pos[1];
            track[2] = ini_pos[2];

            for (int t = 3; t < 3 * num_points; t = t + 3) {
              track[t] = track[t - 3] + speed / samp_rate * ini_dir1[t];
              track[t + 1] = track[t - 2] + speed / samp_rate * ini_dir1[t + 1];
              track[t + 2] = track[t - 1] + speed / samp_rate * ini_dir1[t + 2];

              // 高度限制
              if (track[t + 2] > 30.0) {
                track[t + 2] = 30.0;
              } else if (track[t + 2] < 10.0) {
                track[t + 2] = 10.0;
              }
            }
        }
    else {
        printf("Error: Unknown track type\n");
        exit(1);
    }
}
//////////////////////////////////////////////////
// 计算数组均值的函数
void calculateMean(float arr[], int size,float* mean_arr) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    *mean_arr = sum / size;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//travellength3dsim函数//////////////////////////////////////////////////////////
void TravelLength3DSim(float* loca_0, float* loca_t, float betaE, float betaA, float delta, float* r1, float* e1, float* a1, float* Dpt, float v_c, float theta_A, float theta_E, float tb) {
    // 设置默认值
    long long fc = 5e9;
    if (v_c == 0) v_c = 0.0;
    if (theta_A == 0) theta_A = 0.0;
    if (theta_E == 0) theta_E = 0.0;
    if (tb == 0) tb = 0.0;
    if (delta == 0) delta = (1 - 1) * 299792458 / fc / 2; // 第200根天线;
    // 检查输入参数
    if (r1[0] == 0) {
        for (int i = 0; i < CLUSTER_RAYS; i++) {
            Dpt[i] = 0;
        }
        return;
    }

    // 计算m向量
    float m[CLUSTER_RAYS * 3];
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        m[i * 3] = cos(e1[i]) * cos(a1[i]) * r1[i];
        m[i * 3 + 1] = cos(e1[i]) * sin(a1[i]) * r1[i];
        m[i * 3 + 2] = sin(e1[i]) * r1[i];
        // printf("e1[%d]=%.16f\n",i, e1[i]);
        // printf("m[i * 3]=%.16f\n", m[i * 3]);
    }

    // 计算v向量
    float v[3];
    v[0] = cos(theta_E) * cos(theta_A) * v_c * tb;
    v[1] = cos(theta_E) * sin(theta_A) * v_c * tb;
    v[2] = sin(theta_E) * v_c * tb;

    // 计算loca_s
    float loca_s[3 * CLUSTER_RAYS];
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        loca_s[i * 3] = loca_0[0] + m[i * 3] + v[0];
        loca_s[i * 3 + 1] = loca_0[1] + m[i * 3 + 1] + v[1];
        loca_s[i * 3 + 2] = loca_0[2] + m[i * 3 + 2] + v[2];
    }

    // 计算q_loc
    float q_loc[3];
    q_loc[0] = loca_t[0] + cos(betaE) * cos(betaA) * delta;
    q_loc[1] = loca_t[1] + cos(betaE) * sin(betaA) * delta;
    q_loc[2] = loca_t[2] + sin(betaE) * delta;
        //    printf("q_loc[2]=%.16f\n", q_loc[2]);
    // 计算Dpt
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        float dx = q_loc[0] - loca_s[i * 3];
        float dy = q_loc[1] - loca_s[i * 3 + 1];
        float dz = q_loc[2] - loca_s[i * 3 + 2];
        Dpt[i] = sqrt(dx * dx + dy * dy + dz * dz);
        // printf("Dpt[%d]=%.16f\n",i, Dpt[i]);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// /////LengthCACZ
void LengthCACZ(int t,float loc_tx0_x, float loc_tx0_y, float loc_tx0_z, float loc_rx0_x, float loc_rx0_y, float loc_rx0_z, float* e1, float* a1, float* r1, float* e2, float* a2, float* r2, float* dm) {
    // 计算发射器点集的坐标增量
    float m1[3*CLUSTER_RAYS];
//    if(t<1){
//    for(int i=0;i<10;i++){
//    	printf("e1[%d]=%f\n",i,e1[i]);
//    }
//    }
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        m1[3*i] = cos(e1[i]) * cos(a1[i]) * r1[i];
        m1[3*i+1] = cos(e1[i]) * sin(a1[i]) * r1[i];
        m1[3*i+2] = sin(e1[i]) * r1[i];
    }
//    if(t==2){
//    for(int i=0;i<10;i++){
//    	printf("m1[%d]=%f\n",i,m1[3*i+1]);
//    }
//    }
    // 计算接收器点集的坐标增量
    float m2[3*CLUSTER_RAYS];
//        if(t==2){
//        for(int i=0;i<10;i++){
//        	printf("e2[%d]=%f\n",i,e2[i]);
//        }
//        }
//        if(t==2){
//        for(int i=0;i<10;i++){
//        	printf("a2[%d]=%f\n",i,a2[i]);
//        }
//        }
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        m2[3*i] = cos(e2[i]) * cos(a2[i]) * r2[i];
        m2[3*i+1] = cos(e2[i]) * sin(a2[i]) * r2[i];
        m2[3*i+2] = sin(e2[i]) * r2[i];
    }
//    if(t==2){
//    for(int i=0;i<10;i++){
//    	printf("m2[%d]=%f\n",i,m2[3*i]);
//    }
//    }
    // 生成发射器点集的绝对坐标
    float loca_A[3*CLUSTER_RAYS];
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        loca_A[3*i] = loc_tx0_x + m1[3*i];
        loca_A[3*i+1] = loc_tx0_y + m1[3*i+1];
        loca_A[3*i+2] = loc_tx0_z + m1[3*i+2];
    }

    // 生成接收器点集的绝对坐标
    float loca_Z[3*CLUSTER_RAYS];
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        loca_Z[3*i] = loc_rx0_x + m2[3*i];
        loca_Z[3*i+1] = loc_rx0_y + m2[3*i+1];
        loca_Z[3*i+2] = loc_rx0_z + m2[3*i+2];
    }

    // 计算点集之间的距离
    float Dpt[CLUSTER_RAYS];
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        float dx = loca_A[3*i] - loca_Z[3*i];
        float dy = loca_A[3*i+1] - loca_Z[3*i+1];
        float dz = loca_A[3*i+2] - loca_Z[3*i+2];
        Dpt[i] = sqrt(dx * dx + dy * dy + dz * dz);
    }
   calculateMean( Dpt, CLUSTER_RAYS,dm);
    // // 计算平均距离
    // float sum = 0.0;
    // for (int i = 0; i < CLUSTER_RAYS; i++) {
    //     sum += Dpt[i];
    // }
    // dm = sum / CLUSTER_RAYS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//travellengthLos函数////////////////////////////////////////////////////////////
// 计算两个位置点之间的直线距离
void TravelLengthLoS(float* loca_t, float* loca_r,
    float betaE_t, float betaA_t, float deltap_t,
    float betaE_r, float betaA_r, float deltap_r,
    float* Dpqt) {
    // 设置默认值
    long long fc = 5e9;
    if (betaE_t == 0) betaE_t = pi / 6;
    if (betaA_t == 0) betaA_t = pi / 4;
    if (deltap_t == 0) deltap_t = (1 - 1) * 299792458 / fc / 2; // 第200根天线;

    // 扩展位置向量
    float q_loc[3];
    float l_loc[3];

    // 计算发射器最终位置 q_loc
    q_loc[0] = loca_t[0] + cos(betaE_t) * cos(betaA_t) * deltap_t;
    q_loc[1] = loca_t[1] + cos(betaE_t) * sin(betaA_t) * deltap_t;
    q_loc[2] = loca_t[2] + sin(betaE_t) * deltap_t;

    // 计算接收器最终位置 l_loc
    l_loc[0] = loca_r[0] + cos(betaE_r) * cos(betaA_r) * deltap_r;
    l_loc[1] = loca_r[1] + cos(betaE_r) * sin(betaA_r) * deltap_r;
    l_loc[2] = loca_r[2] + sin(betaE_r) * deltap_r;

    // 计算直线距离 Dpt

    float dx = q_loc[0] - l_loc[0];
    float dy = q_loc[1] - l_loc[1];
    float dz = q_loc[2] - l_loc[2];
    *Dpqt = sqrt(dx * dx + dy * dy + dz * dz);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////DiscrAngleLenEllipsoid函数////////////////////////////////////////////////////////////////////////////////////
// 
    // 子函数声明
// float NorDis(float mu, float sigma);
// void shuffleArray(float* array, int n);
// void cart2sph(float x, float y, float z, float* azimuth, float* elevation, float* r);
// void rotatePoints(float* x, float* y, float* z, float d0,int n, float roA, float roE,float *x2, float *y2, float *z2);
// 随机打乱数组顺序
void shuffleArray(float* array, int n) {
    for (int i = n - 1; i > 0; i--) {
        int j = randn_gen_number[randn_gen_count++];
        float temp = array[i];
        array[i] = array[j];
        array[j] = temp;
    }
}
// 直角坐标转球面坐标
void cart2sph(float x, float y, float z, float* azimuth, float* elevation, float* r) {
    *r = sqrt(x * x + y * y + z * z);
    *azimuth = atan2(y, x);
    *elevation = atan2(z, sqrt(x * x + y * y));
}

// 旋转点
#include <math.h>




void rotatePoints_z(int test,float* x, float* y, float* ztmp, int tind, float d0, int n, float roA, float roE, float* z_gen, float* azimuth, float* elevation, float* r) {

    // --- 1. 定义旋转矩阵 (对应 MATLAB 代码) ---
    // roA 对应 MATLAB 的 roA (phiA0)
    // roE 对应 MATLAB 的 roE (-phiE0)

    // MATLAB: Rz = [cos(roA), -sin(roA), 0; sin(roA), cos(roA), 0; 0, 0, 1];
    float Rz[9] = {
        cos(roA), -sin(roA), 0,
        sin(roA),  cos(roA), 0,
        0,       0,      1
    };

    // MATLAB: Ry = [cos(roE), 0, sin(roE); 0, 1, 0; -sin(roE), 0, cos(roE)];
    float Ry[9] = {
        cos(roE),  0, sin(roE),
        0,       1, 0,
        -sin(roE), 0, cos(roE)
    };

    // --- 2. 计算组合矩阵 (对应 MATLAB: Rz * Ry) ---
    float z_y[9]; // 存放 Rz * Ry 的结果

    // 矩阵乘法: Row_Rz * Col_Ry
    // Rz 第一行 * Ry 各列
    z_y[0] = Rz[0] * Ry[0] + Rz[1] * Ry[3] + Rz[2] * Ry[6];
    z_y[1] = Rz[0] * Ry[1] + Rz[1] * Ry[4] + Rz[2] * Ry[7];
    z_y[2] = Rz[0] * Ry[2] + Rz[1] * Ry[5] + Rz[2] * Ry[8];
    // Rz 第二行 * Ry 各列
    z_y[3] = Rz[3] * Ry[0] + Rz[4] * Ry[3] + Rz[5] * Ry[6];
    z_y[4] = Rz[3] * Ry[1] + Rz[4] * Ry[4] + Rz[5] * Ry[7];
    z_y[5] = Rz[3] * Ry[2] + Rz[4] * Ry[5] + Rz[5] * Ry[8];
    // Rz 第三行 * Ry 各列
    z_y[6] = Rz[6] * Ry[0] + Rz[7] * Ry[3] + Rz[8] * Ry[6];
    z_y[7] = Rz[6] * Ry[1] + Rz[7] * Ry[4] + Rz[8] * Ry[7];
    z_y[8] = Rz[6] * Ry[2] + Rz[7] * Ry[5] + Rz[8] * Ry[8];
//    if(test){
//		for (int i=0;i<9;i++){
//			printf("z_y[%d]=%f\n",i,z_y[i]);
//		}
//    }
    // --- 3. 准备临时数组存放旋转后的坐标 ---
    // 对应 MATLAB: x = a(1,:), y = a(2,:)
    // 注意：如果 CLUSTER_RAYS 很大，建议使用 malloc/std::vector 避免栈溢出
    float x_rot[CLUSTER_RAYS];
    float y_rot[CLUSTER_RAYS];
    float z_rot[CLUSTER_RAYS]; // 旋转后的几何 z，虽然 generatez 会生成新 z，但为了接口完整保留

    // --- 4. 执行坐标旋转 (对应 a = Rz*Ry*[x1+d0; y1; z1]) ---
    for (int i = 0; i < n; i++) {
        // 构造原始向量
        float x0 = x[i] + d0;
        float y0 = y[i];
        float z0 = ztmp[i];

        // 应用旋转矩阵
        x_rot[i] = z_y[0] * x0 + z_y[1] * y0 + z_y[2] * z0;
        y_rot[i] = z_y[3] * x0 + z_y[4] * y0 + z_y[5] * z0;
        z_rot[i] = z_y[6] * x0 + z_y[7] * y0 + z_y[8] * z0;
    }


    generatez(x_rot, y_rot, z_rot, tind, n, z_gen);

    // 使用旋转后的 x, y 和 *新生成* 的 z 计算
    for (int i = 0; i < n; i++) {
        float curr_x = x_rot[i];
        float curr_y = y_rot[i];
        float curr_z = z_gen[i]; // 使用 generatez 的结果，而不是几何旋转的 z_rot

        float dist_xy = sqrt(curr_x * curr_x + curr_y * curr_y);

        // 计算距离
        r[i] = sqrt(curr_x * curr_x + curr_y * curr_y + curr_z * curr_z);

        // 计算方位角 (Azimuth)
        azimuth[i] = atan2(curr_y, curr_x);

        // 计算仰角 (Elevation)
        // 注意：atan2(y, x)，对于仰角通常是 atan2(z, xy_dist)
        elevation[i] = atan2(curr_z, dist_xy);
    }
}
void rotatePoints(float* x, float* y, float* z,float d0, int n, float roA, float roE,float* azimuth, float* elevation, float* r) {


float Rz[9] = {cos(roA), -sin(roA), 0,
        sin(roA), cos(roA), 0,
        0, 0, 1
    };

    // 构造旋转矩阵 Ry
    float Ry[9] = {
        cos(roE), 0, sin(roE),
        0, 1, 0,
        -sin(roE), 0, cos(roE)
    };
    float z_y[9] = { 0 };

        // Rz * Ry
        z_y[0] = Rz[0] * Ry[0] + Rz[1] * Ry[3] + Rz[2] * Ry[6];
        z_y[1] = Rz[0] * Ry[1] + Rz[1] * Ry[4] + Rz[2] * Ry[7];
        z_y[2] = Rz[0] * Ry[2] + Rz[1] * Ry[5] + Rz[2] * Ry[8];
        z_y[3] = Rz[3] * Ry[0] + Rz[4] * Ry[3] + Rz[5] * Ry[6];
        z_y[4] = Rz[3] * Ry[1] + Rz[4] * Ry[4] + Rz[5] * Ry[7];
        z_y[5] = Rz[3] * Ry[2] + Rz[4] * Ry[5] + Rz[5] * Ry[8];
        z_y[6] = Rz[6] * Ry[0] + Rz[7] * Ry[3] + Rz[8] * Ry[6];
        z_y[7] = Rz[6] * Ry[1] + Rz[7] * Ry[4] + Rz[8] * Ry[7];
        z_y[8] = Rz[6] * Ry[2] + Rz[7] * Ry[5] + Rz[8] * Ry[8];
    float x2[CLUSTER_RAYS];
    float y2[CLUSTER_RAYS];
    float z2[CLUSTER_RAYS];

    for (int i = 0; i < n; i++) {
        // 原始坐标
        float x0 = x[i]+d0;
        float y0 = y[i];
        float z0 = z[i];
        
        // Ry * rotated
        x2[i] = z_y[0] * x0 + z_y[1] * y0 + z_y[2] * z0;
        y2[i] = z_y[3] * x0 + z_y[4] * y0 + z_y[5] * z0;
        z2[i] = z_y[6] * x0 + z_y[7] * y0 + z_y[8] * z0;
     r[i] = sqrt(x2[i] * x2[i] + y2[i] * y2[i] + z2[i] * z2[i]);
    azimuth[i] = atan2(y2[i], x2[i]);
    elevation[i] = atan2(z2[i], sqrt(x2[i] * x2[i] + y2[i] * y2[i]));
        //printf("x[%d] = %.16f\n", i, x[i]);
        //printf("y[%d] = %.16f\n", i, y[i]);
        //printf("z[%d] = %.16f\n", i, z[i]);
    }
}

void DiscrAngleLenEllipsoid(int test,int Num, float d0, float phiA0, float phiE0,
    float sigmax, float sigmay, float sigmaz,
    float* azimuth, float* elevation, float* r) {
    //  print("进入函数");
    // 设置默认值
    if (Num == 0) Num = 1000;
    if (d0 == 0) d0 = 500;
    if (phiA0 == 0) phiA0 = pi / 3;
    if (phiE0 == 0) phiE0 = pi / 6;
    if (sigmax == 0) sigmax = 60;
    if (sigmay == 0) sigmay = 100;
    if (sigmaz == 0) sigmaz = 30;
    // print("设置默认值");
    // 生成随机数
    float x1[CLUSTER_RAYS], y1[CLUSTER_RAYS], z1[CLUSTER_RAYS];
    for (int i = 0; i < Num; i++) {
        x1[i] = NorDis3(0, sigmax);
        if(test==1)
        printf("x1[%d] = %.16f\n", i, x1[i]);
    }
        for (int i = 0; i < Num; i++) {
        y1[i] = NorDis3(0, sigmay);
        if(test==1)
        printf("y1[%d] = %.16f\n", i, y1[i]);
    }
        for (int i = 0; i < Num; i++) {
        z1[i] = NorDis3(0, sigmaz);
        if(test==1)
        printf("z1[%d] = %.16f\n", i, z1[i]);
    }

    rotatePoints(x1, y1, z1,d0, Num, phiA0, -phiE0,azimuth,elevation,r);

}
void DiscrAngleLenEllipsoid222(int test,int Num, float d0, float phiA0, float phiE0,
		float sigmax, float sigmay, float sigmaz,
		float* h_t_track, int tind, float* ddd,
		float* azimuth, float* elevation, float* r) {
    //  print("进入函数");
    // 设置默认值
    // if (Num == 0) Num = 1000;
    // if (d0 == 0) d0 = 500;
    // if (phiA0 == 0) phiA0 = pi / 3;
    // if (phiE0 == 0) phiE0 = pi / 6;
    // if (sigmax == 0) sigmax = 60;
    // if (sigmay == 0) sigmay = 100;
    // if (sigmaz == 0) sigmaz = 30;
    // print("设置默认值");

    // print("NUM");
    // printf("Num = %d\n", Num);
    // 生成随机数
    float x1[CLUSTER_RAYS], y1[CLUSTER_RAYS], z1[CLUSTER_RAYS], ztmp[CLUSTER_RAYS];
    for (int i = 0; i < Num; i++) {
        x1[i] = NorDis3(0, sigmax);
        //printf("x1[%d] = %.16f\n", i, x1[i]);
    }
        for (int i = 0; i < Num; i++) {
        y1[i] = NorDis3(0, sigmay);
       // printf("y1[%d] = %.16f\n", i, y1[i]);
    }
        for (int i = 0; i < Num; i++) {
        ztmp[i] = NorDis3(0, sigmaz);
       // printf("z1[%d] = %.16f\n", i, z1[i]);
    }
    // print("随机数生成");

    // // 随机打乱数组
    // shuffleArray(x1, Num);
    // shuffleArray(y1, Num);
    // shuffleArray(ztmp, Num);
    // print("打乱数组");
    // 旋转点
	rotatePoints_z(test,x1, y1, ztmp,tind,d0, Num, phiA0, -phiE0,z1,azimuth,elevation,r);

        int i_dd=tind;
        for (int i = 0; i < Num; i++) {
        	float dx = h_t_track[i_dd * 3] - x1[i];
        	float dy = h_t_track[i_dd * 3 + 1] - y1[i];
        	float dz = h_t_track[i_dd * 3 + 2] - z1[i];
        ddd[i] = sqrt(dx * dx + dy * dy + dz * dz) / 300000000;
        }
    }




void GenerateParameters(
    int Nc0, float* par_Tsigmax, float* par_Tsigmay, float* par_Tsigmaz,
    float* par_Rsigmax, float* par_Rsigmay, float* par_Rsigmaz) {
    // float* array1,float* array2,float* array3,float* array4,float* array5,float* array6) {

    for (int i = 0; i < Nc0; i++) {
        par_Tsigmax[i] = NorDis3(para.Tsigmax, para.sigmaxvar);
    }
    for (int i = 0; i < Nc0; i++) {
        par_Tsigmay[i] = NorDis3(para.Tsigmay, para.sigmayvar);
    }
    for (int i = 0; i < Nc0; i++) {
        par_Tsigmaz[i] = NorDis3(para.Tsigmaz, para.sigmazvar);
    }
    for (int i = 0; i < Nc0; i++) {
        par_Rsigmax[i] = NorDis3(para.Rsigmax, para.sigmaxvar);
    }
    for (int i = 0; i < Nc0; i++) {
        par_Rsigmay[i] = NorDis3(para.Rsigmay, para.sigmayvar);
    }
    for (int i = 0; i < Nc0; i++) {
        par_Rsigmaz[i] = NorDis3(para.Rsigmaz, para.sigmazvar);
    }
    // GenerateNormalArray(par_Tsigmax, para.Tsigmax, para.sigmaxvar, Nc0);
    // GenerateNormalArray(par_Tsigmay, para.Tsigmay, para.sigmayvar, Nc0);
    // GenerateNormalArray(par_Tsigmaz, para.Tsigmaz, para.sigmazvar, Nc0);

    // // 生成 par.Rsigmax, par.Rsigmay, par.Rsigmaz
    // GenerateNormalArray(par_Rsigmax, para.Rsigmax, para.sigmaxvar, Nc0);
    // GenerateNormalArray(par_Rsigmay, para.Rsigmay, para.sigmayvar, Nc0);
    // GenerateNormalArray(par_Rsigmaz, para.Rsigmaz, para.sigmazvar, Nc0);

}



// 生成指数分布随机数
float Exprnd(float lambda) {
    float u = rand_gen_number[rand_gen_count++];
    return -log(u) * lambda;
}




// 指数滤波函数
#define EXP_FILTER_MAX_OFFSETS 1024
static int exp_cached_corr_num = -1;
static float exp_cached_corr_d = -1.0f;
static float exp_cached_D_g = -1.0f;
static int exp_di_list[EXP_FILTER_MAX_OFFSETS];
static int exp_dj_list[EXP_FILTER_MAX_OFFSETS];
static float exp_w_list[EXP_FILTER_MAX_OFFSETS];
static int exp_w_count = 0;
static float exp_inv_full_weight_sum = 1.0f;

static void prepare_exp_filter_weights(float corr_d, float D_g, int* corr_num_out) {
    const int corr_num = (int)ceil(corr_d / D_g);
    const int needed_offsets = (2 * corr_num + 1) * (2 * corr_num + 1);
    *corr_num_out = corr_num;

    if (needed_offsets > EXP_FILTER_MAX_OFFSETS) {
        printf("[4ERR] Exp_filter offset过多: %d > %d\n", needed_offsets, EXP_FILTER_MAX_OFFSETS);
        while (1) {}
    }

    if (exp_cached_corr_num != corr_num || exp_cached_corr_d != corr_d || exp_cached_D_g != D_g) {
        exp_w_count = 0;
        float full_weight_sum = 0.0f;
        for (int di = -corr_num; di <= corr_num; ++di) {
            for (int dj = -corr_num; dj <= corr_num; ++dj) {
                float d = sqrt((float)(di * di + dj * dj));
                float w = exp(-d * D_g / corr_d);
                exp_di_list[exp_w_count] = di;
                exp_dj_list[exp_w_count] = dj;
                exp_w_list[exp_w_count] = w;
                full_weight_sum += w;
                ++exp_w_count;
            }
        }
        exp_inv_full_weight_sum = (full_weight_sum != 0.0f) ? (1.0f / full_weight_sum) : 0.0f;
        exp_cached_corr_num = corr_num;
        exp_cached_corr_d = corr_d;
        exp_cached_D_g = D_g;
    }
}

void Exp_filter(float* u, int rows, int cols, float corr_d, float D_g,float* filtered_M) {
    const int m = rows;
    const int n = cols;
    int corr_num;
    prepare_exp_filter_weights(corr_d, D_g, &corr_num);

    for (int i = 0; i < m; i++) {
        const int row_base = i * n;
        const int interior_i = (i >= corr_num && i < (m - corr_num));
        for (int j = 0; j < n; j++) {
            float sum_M = 0.0f;

            if (interior_i && j >= corr_num && j < (n - corr_num)) {
                for (int k = 0; k < exp_w_count; ++k) {
                    sum_M += exp_w_list[k] * u[(i + exp_di_list[k]) * n + (j + exp_dj_list[k])];
                }
                filtered_M[row_base + j] = sum_M * exp_inv_full_weight_sum;
            } else {
                float sum_a = 0.0f;
                for (int k = 0; k < exp_w_count; ++k) {
                    int ii = i + exp_di_list[k];
                    int jj = j + exp_dj_list[k];
                    if ((unsigned)ii >= (unsigned)m || (unsigned)jj >= (unsigned)n) {
                        continue;
                    }
                    float w = exp_w_list[k];
                    sum_M += w * u[ii * n + jj];
                    sum_a += w;
                }
                if (sum_a != 0.0f) {
                    filtered_M[row_base + j] = sum_M / sum_a;
                }
            }
        }
    }

}

static void Exp_filter8(
    float* u0, float* u1, float* u2, float* u3,
    float* u4, float* u5, float* u6, float* u7,
    int rows, int cols, float corr_d, float D_g,
    float* o0, float* o1, float* o2, float* o3,
    float* o4, float* o5, float* o6, float* o7) {
    const int m = rows;
    const int n = cols;
    int corr_num;
    prepare_exp_filter_weights(corr_d, D_g, &corr_num);

    for (int i = 0; i < m; i++) {
        const int row_base = i * n;
        const int interior_i = (i >= corr_num && i < (m - corr_num));
        for (int j = 0; j < n; j++) {
            float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
            float s4 = 0.0f, s5 = 0.0f, s6 = 0.0f, s7 = 0.0f;
            float norm = exp_inv_full_weight_sum;

            if (interior_i && j >= corr_num && j < (n - corr_num)) {
                for (int k = 0; k < exp_w_count; ++k) {
                    const int idx = (i + exp_di_list[k]) * n + (j + exp_dj_list[k]);
                    const float w = exp_w_list[k];
                    s0 += w * u0[idx]; s1 += w * u1[idx]; s2 += w * u2[idx]; s3 += w * u3[idx];
                    s4 += w * u4[idx]; s5 += w * u5[idx]; s6 += w * u6[idx]; s7 += w * u7[idx];
                }
            } else {
                float sum_a = 0.0f;
                for (int k = 0; k < exp_w_count; ++k) {
                    const int ii = i + exp_di_list[k];
                    const int jj = j + exp_dj_list[k];
                    if ((unsigned)ii >= (unsigned)m || (unsigned)jj >= (unsigned)n) {
                        continue;
                    }
                    const int idx = ii * n + jj;
                    const float w = exp_w_list[k];
                    s0 += w * u0[idx]; s1 += w * u1[idx]; s2 += w * u2[idx]; s3 += w * u3[idx];
                    s4 += w * u4[idx]; s5 += w * u5[idx]; s6 += w * u6[idx]; s7 += w * u7[idx];
                    sum_a += w;
                }
                norm = (sum_a != 0.0f) ? (1.0f / sum_a) : 0.0f;
            }

            const int out_idx = row_base + j;
            o0[out_idx] = s0 * norm; o1[out_idx] = s1 * norm; o2[out_idx] = s2 * norm; o3[out_idx] = s3 * norm;
            o4[out_idx] = s4 * norm; o5[out_idx] = s5 * norm; o6[out_idx] = s6 * norm; o7[out_idx] = s7 * norm;
        }
    }
}

static void Exp_filter2(
    float* u0, float* u1,
    int rows, int cols, float corr_d, float D_g,
    float* o0, float* o1) {
    const int m = rows;
    const int n = cols;
    int corr_num;
    prepare_exp_filter_weights(corr_d, D_g, &corr_num);

    for (int i = 0; i < m; i++) {
        const int row_base = i * n;
        const int interior_i = (i >= corr_num && i < (m - corr_num));
        for (int j = 0; j < n; j++) {
            float s0 = 0.0f, s1 = 0.0f;
            float norm = exp_inv_full_weight_sum;

            if (interior_i && j >= corr_num && j < (n - corr_num)) {
                for (int k = 0; k < exp_w_count; ++k) {
                    const int idx = (i + exp_di_list[k]) * n + (j + exp_dj_list[k]);
                    const float w = exp_w_list[k];
                    s0 += w * u0[idx];
                    s1 += w * u1[idx];
                }
            } else {
                float sum_a = 0.0f;
                for (int k = 0; k < exp_w_count; ++k) {
                    const int ii = i + exp_di_list[k];
                    const int jj = j + exp_dj_list[k];
                    if ((unsigned)ii >= (unsigned)m || (unsigned)jj >= (unsigned)n) {
                        continue;
                    }
                    const int idx = ii * n + jj;
                    const float w = exp_w_list[k];
                    s0 += w * u0[idx];
                    s1 += w * u1[idx];
                    sum_a += w;
                }
                norm = (sum_a != 0.0f) ? (1.0f / sum_a) : 0.0f;
            }

            const int out_idx = row_base + j;
            o0[out_idx] = s0 * norm;
            o1[out_idx] = s1 * norm;
        }
    }
}

static void Exp_filter2_separable_fast(
    float* u0, float* u1,
    int rows, int cols, float corr_d, float D_g,
    float* o0, float* o1) {
    const int m = rows;
    const int n = cols;
    const int corr_num = (int)ceil(corr_d / D_g);
    if (corr_num * 2 + 1 > 32) {
        printf("[4ERR] separable filter2窗口过大: %d\n", corr_num);
        while (1) {}
    }

    float w1d[65];
    for (int d = -corr_num; d <= corr_num; ++d) {
        const int ad = (d < 0) ? -d : d;
        w1d[d + corr_num] = exp(-(float)ad * D_g / corr_d);
    }

    float inv_sum_x[LSP_COLS];
    float inv_sum_y[LSP_ROWS];
    for (int j = 0; j < n; ++j) {
        float s = 0.0f;
        for (int dj = -corr_num; dj <= corr_num; ++dj) {
            const int jj = j + dj;
            if ((unsigned)jj < (unsigned)n) {
                s += w1d[dj + corr_num];
            }
        }
        inv_sum_x[j] = (s != 0.0f) ? (1.0f / s) : 0.0f;
    }
    for (int i = 0; i < m; ++i) {
        float s = 0.0f;
        for (int di = -corr_num; di <= corr_num; ++di) {
            const int ii = i + di;
            if ((unsigned)ii < (unsigned)m) {
                s += w1d[di + corr_num];
            }
        }
        inv_sum_y[i] = (s != 0.0f) ? (1.0f / s) : 0.0f;
    }

    for (int i = 0; i < m; ++i) {
        const int row = i * n;
        for (int j = 0; j < n; ++j) {
            float s0 = 0.0f, s1 = 0.0f;
            for (int dj = -corr_num; dj <= corr_num; ++dj) {
                const int jj = j + dj;
                if ((unsigned)jj >= (unsigned)n) continue;
                const int idx = row + jj;
                const float w = w1d[dj + corr_num];
                s0 += w * u0[idx];
                s1 += w * u1[idx];
            }
            const int out = row + j;
            const float norm = inv_sum_x[j];
            lsp_filter_tmp0[out] = s0 * norm;
            lsp_filter_tmp1[out] = s1 * norm;
        }
    }

    for (int i = 0; i < m; ++i) {
        const int row = i * n;
        const float norm = inv_sum_y[i];
        for (int j = 0; j < n; ++j) {
            float s0 = 0.0f, s1 = 0.0f;
            for (int di = -corr_num; di <= corr_num; ++di) {
                const int ii = i + di;
                if ((unsigned)ii >= (unsigned)m) continue;
                const int idx = ii * n + j;
                const float w = w1d[di + corr_num];
                s0 += w * lsp_filter_tmp0[idx];
                s1 += w * lsp_filter_tmp1[idx];
            }
            const int out = row + j;
            o0[out] = s0 * norm;
            o1[out] = s1 * norm;
        }
    }
}

static void Exp_filter8_separable_fast(
    float* u0, float* u1, float* u2, float* u3,
    float* u4, float* u5, float* u6, float* u7,
    int rows, int cols, float corr_d, float D_g,
    float* o0, float* o1, float* o2, float* o3,
    float* o4, float* o5, float* o6, float* o7) {
    const int m = rows;
    const int n = cols;
    const int corr_num = (int)ceil(corr_d / D_g);
    if (corr_num * 2 + 1 > 32) {
        printf("[4ERR] separable filter窗口过大: %d\n", corr_num);
        while (1) {}
    }

    float w1d[65];
    for (int d = -corr_num; d <= corr_num; ++d) {
        int ad = (d < 0) ? -d : d;
        w1d[d + corr_num] = exp(-(float)ad * D_g / corr_d);
    }

    float inv_sum_x[LSP_COLS];
    float inv_sum_y[LSP_ROWS];
    for (int j = 0; j < n; ++j) {
        float s = 0.0f;
        for (int dj = -corr_num; dj <= corr_num; ++dj) {
            int jj = j + dj;
            if ((unsigned)jj < (unsigned)n) {
                s += w1d[dj + corr_num];
            }
        }
        inv_sum_x[j] = (s != 0.0f) ? (1.0f / s) : 0.0f;
    }
    for (int i = 0; i < m; ++i) {
        float s = 0.0f;
        for (int di = -corr_num; di <= corr_num; ++di) {
            int ii = i + di;
            if ((unsigned)ii < (unsigned)m) {
                s += w1d[di + corr_num];
            }
        }
        inv_sum_y[i] = (s != 0.0f) ? (1.0f / s) : 0.0f;
    }

    for (int i = 0; i < m; ++i) {
        const int row = i * n;
        for (int j = 0; j < n; ++j) {
            float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
            float s4 = 0.0f, s5 = 0.0f, s6 = 0.0f, s7 = 0.0f;
            for (int dj = -corr_num; dj <= corr_num; ++dj) {
                int jj = j + dj;
                if ((unsigned)jj >= (unsigned)n) continue;
                const int idx = row + jj;
                const float w = w1d[dj + corr_num];
                s0 += w * u0[idx]; s1 += w * u1[idx]; s2 += w * u2[idx]; s3 += w * u3[idx];
                s4 += w * u4[idx]; s5 += w * u5[idx]; s6 += w * u6[idx]; s7 += w * u7[idx];
            }
            const int out = row + j;
            const float norm = inv_sum_x[j];
            lsp_filter_tmp0[out] = s0 * norm; lsp_filter_tmp1[out] = s1 * norm;
            lsp_filter_tmp2[out] = s2 * norm; lsp_filter_tmp3[out] = s3 * norm;
            lsp_filter_tmp4[out] = s4 * norm; lsp_filter_tmp5[out] = s5 * norm;
            lsp_filter_tmp6[out] = s6 * norm; lsp_filter_tmp7[out] = s7 * norm;
        }
    }

    for (int i = 0; i < m; ++i) {
        const int row = i * n;
        const float norm = inv_sum_y[i];
        for (int j = 0; j < n; ++j) {
            float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
            float s4 = 0.0f, s5 = 0.0f, s6 = 0.0f, s7 = 0.0f;
            for (int di = -corr_num; di <= corr_num; ++di) {
                int ii = i + di;
                if ((unsigned)ii >= (unsigned)m) continue;
                const int idx = ii * n + j;
                const float w = w1d[di + corr_num];
                s0 += w * lsp_filter_tmp0[idx]; s1 += w * lsp_filter_tmp1[idx];
                s2 += w * lsp_filter_tmp2[idx]; s3 += w * lsp_filter_tmp3[idx];
                s4 += w * lsp_filter_tmp4[idx]; s5 += w * lsp_filter_tmp5[idx];
                s6 += w * lsp_filter_tmp6[idx]; s7 += w * lsp_filter_tmp7[idx];
            }
            const int out = row + j;
            o0[out] = s0 * norm; o1[out] = s1 * norm; o2[out] = s2 * norm; o3[out] = s3 * norm;
            o4[out] = s4 * norm; o5[out] = s5 * norm; o6[out] = s6 * norm; o7[out] = s7 * norm;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 高度相关矩阵生成函数
void Corr_Altitudes(float* LSP_l, float* LSP_h,int rows, int cols, int num, float* Corr_M) {
    const int m = rows;
    const int n = cols;
    if (num <= 1) {
        for (int i = 0; i < m * n; ++i) {
            Corr_M[i] = LSP_l[i];
        }
        return;
    }

    float weight_h[num];
    float weight_l[num];
    const float inv_den = 1.0f / (float)(num - 1);
    for (int k = 0; k < num; ++k) {
        weight_h[k] = (float)k * inv_den;
        weight_l[k] = 1.0f - weight_h[k];
    }

    for (int i = 0; i < m; i++) {
       for (int j = 0; j < n; j++) {
            int src_idx = i * n + j;
            float low = LSP_l[src_idx];
            float high = LSP_h[src_idx];
            float* corr = &Corr_M[src_idx * num];
            for (int k = 0; k < num; k++) {
                *corr++ = high * weight_h[k] + low * weight_l[k];
            }
        }
    }
}

static void Corr_Altitudes_K_with_lin(float* LSP_l, float* LSP_h, int rows, int cols, int num, float* Corr_M, float* Lin_M) {
    const int mn = rows * cols;
    if (num <= 1) {
        for (int i = 0; i < mn; ++i) {
            const float v = LSP_l[i];
            Corr_M[i] = v;
            Lin_M[i] = stage4_pow10_model(v * 0.1f);
        }
        return;
    }

    if (num == LSP_ALTITUDES) {
        for (int i = 0; i < mn; ++i) {
            const float low = LSP_l[i];
            const float high = LSP_h[i];
            float* corr = &Corr_M[i * LSP_ALTITUDES];
            float* lin = &Lin_M[i * LSP_ALTITUDES];
            for (int k = 0; k < LSP_ALTITUDES; ++k) {
                const float v = high * lsp_w_h20[k] + low * lsp_w_l20[k];
                *corr++ = v;
                *lin++ = stage4_pow10_model(v * 0.1f);
            }
        }
        return;
    }

    const float inv_den = 1.0f / (float)(num - 1);
    for (int i = 0; i < mn; ++i) {
        const float low = LSP_l[i];
        const float high = LSP_h[i];
        float* corr = &Corr_M[i * num];
        float* lin = &Lin_M[i * num];
        for (int k = 0; k < num; ++k) {
            const float wh = (float)k * inv_den;
            const float v = high * wh + low * (1.0f - wh);
            *corr++ = v;
            *lin++ = stage4_pow10_model(v * 0.1f);
        }
    }
}

static void Corr_Altitudes4(
    float* l0, float* h0, float* l1, float* h1,
    float* l2, float* h2, float* l3, float* h3,
    int rows, int cols, int num,
    float* o0, float* o1, float* o2, float* o3) {
    const int mn = rows * cols;
    if (num <= 1) {
        for (int i = 0; i < mn; ++i) {
            o0[i] = l0[i]; o1[i] = l1[i]; o2[i] = l2[i]; o3[i] = l3[i];
        }
        return;
    }

    if (num == LSP_ALTITUDES) {
        for (int i = 0; i < mn; ++i) {
            const float lo0 = l0[i], hi0 = h0[i];
            const float lo1 = l1[i], hi1 = h1[i];
            const float lo2 = l2[i], hi2 = h2[i];
            const float lo3 = l3[i], hi3 = h3[i];
            float* p0 = &o0[i * LSP_ALTITUDES];
            float* p1 = &o1[i * LSP_ALTITUDES];
            float* p2 = &o2[i * LSP_ALTITUDES];
            float* p3 = &o3[i * LSP_ALTITUDES];
            for (int k = 0; k < LSP_ALTITUDES; ++k) {
                const float wh = lsp_w_h20[k];
                const float wl = lsp_w_l20[k];
                *p0++ = hi0 * wh + lo0 * wl;
                *p1++ = hi1 * wh + lo1 * wl;
                *p2++ = hi2 * wh + lo2 * wl;
                *p3++ = hi3 * wh + lo3 * wl;
            }
        }
        return;
    }

    const float inv_den = 1.0f / (float)(num - 1);
    for (int i = 0; i < mn; ++i) {
        const float lo0 = l0[i], hi0 = h0[i];
        const float lo1 = l1[i], hi1 = h1[i];
        const float lo2 = l2[i], hi2 = h2[i];
        const float lo3 = l3[i], hi3 = h3[i];
        float* p0 = &o0[i * num];
        float* p1 = &o1[i * num];
        float* p2 = &o2[i * num];
        float* p3 = &o3[i * num];
        for (int k = 0; k < num; ++k) {
            const float wh = (float)k * inv_den;
            const float wl = 1.0f - wh;
            *p0++ = hi0 * wh + lo0 * wl;
            *p1++ = hi1 * wh + lo1 * wl;
            *p2++ = hi2 * wh + lo2 * wl;
            *p3++ = hi3 * wh + lo3 * wl;
        }
    }
}

//复数 × 复数
complex complex_mul_comp(complex comp1, complex comp2) {
    complex res;
    res.real = comp1.real * comp2.real - comp1.image * comp2.image;

    res.image = comp1.real * comp2.image + comp1.image * comp2.real;
    return res;
}
//复数*实数
complex complex_mul_x(complex comp1, float x) {
    complex res;
    res.real = comp1.real * x;

    res.image =comp1.image *x;
    return res;
}
complex complex_add(complex comp1, complex comp2) {
    complex res;
    res.real = comp1.real + comp2.real;  // 实部相加
    res.image = comp1.image + comp2.image;  // 虚部相加
    return res;
}
//复数求模
float complex_mod(complex c) {
    return c.real * c.real + c.image * c.image;
}
// 泊松分布随机数生成函数
float He[41];
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////
int main()
{

    system_init();
    printf("[1/7] 系统初始化完成\n");
   // print("参数导入……\n");
    int tp, le, sr, r_speed, t_speed, t_tp,Nc0,fc,num_col;
    printf("--- Zynq UAV Parameter Configuration ---\n");

        // 1. 接收轨迹类型
        scanf("%d", &tp);  // 注意这里的 &

        // 2. 采样时间
        scanf("%d", &le);

        // 3. 采样率
        scanf("%d", &sr);

        // 4. 接收速度
        scanf("%d", &r_speed);

        // 5. 发送速度
        scanf("%d", &t_speed);

        // 6. 发送轨迹类型
        scanf("%d", &t_tp);
        //接收最大簇的个数
        scanf("%d", &num_col);
        // 初始簇个数
        scanf("%d",&Nc0);
        // 载频
        scanf("%d",&fc);
        // 打印确认结果
        printf("\n--- Configuration Updated ---\n");
        printf("tp:%d, le:%d, sr:%d, r_speed:%d, t_speed:%d, t_tp:%d\n,num_col:%d\n,Nc0:%d\n,fc:%d\n",
                tp, le, sr, r_speed, t_speed, t_tp,num_col,Nc0,fc);
//    int tp=03;//接收轨迹类型
//    int le=20;//采样时间
//    int sr=2;//采样率
//    int r_speed=1;//速度
//    int t_speed=5;//速度
//    int t_tp=04;//发送轨迹
    init_parameters(tp,le,sr,r_speed,t_speed,t_tp,fc);



    ///////////////生成轨迹///////////////////////////////////////////////
    // 定义接收机和发射机的轨迹参数
    float r_ini_pos[3] = { para.r_ini_pos[0], para.r_ini_pos[1], para.r_ini_pos[2] };
    float r_ini_dir[3] = { para.r_ini_dir[0], para.r_ini_dir[1], para.r_ini_dir[2] };
    float t_ini_pos[3] = { para.t_ini_pos[0], para.t_ini_pos[1], para.t_ini_pos[2] };
    float t_ini_dir[3] = { para.t_ini_dir[0], para.t_ini_dir[1], para.t_ini_dir[2] };

    // 计算轨迹点数和分配内存
    int r_num_points = ceil(para.r_track_length * para.r_samp_rate)+1 ;

    int t_num_points = ceil(para.t_track_length * para.t_samp_rate)+1 ;
    if (r_num_points > MAX_TIME || t_num_points > MAX_TIME) {
        printf("[ERR] 轨迹点数超过MAX_TIME=%d: r=%d t=%d，请降低le/sr或提高MAX_TIME\n",
               MAX_TIME, r_num_points, t_num_points);
        while (1) {}
    }
    printf("[2/7] 参数初始化完成\n");
    float r_track[r_num_points * 3];
    float t_track[t_num_points * 3];
    // 生成接收机轨迹
    generate_uav_track(para.r_track_type, para.r_track_length, para.r_speed, para.r_accel, r_ini_pos, r_ini_dir, para.r_samp_rate, r_track);
    generateh(r_num_points, para.r_samp_rate, r_track, He);
//    for(int i=0;i<r_num_points;i++){
//    	printf("r_track[%d]=%f\n",i,r_track[i]);
//    }
//    for(int i=0;i<r_num_points;i++){
//        	printf("H[%d]=%f\n",i,H[i]);
//    }
    // 生成发射机轨迹
    for(int i=0;i<r_num_points;i++){
    	r_track[i*3+2]=He[i]+10;
    }
    generate_uav_track(para.t_track_type, para.t_track_length, para.t_speed, para.t_accel, t_ini_pos, t_ini_dir, para.t_samp_rate, t_track);
    printf("[3/7] 收发轨迹生成完成\n");
//    for(int i=0;i<r_num_points;i++){
//    	printf("t_track[%d]=%f\n",i,t_track[i]);
//    }
//    for(int i=0;i<r_num_points;i++){
//    	printf("r_track[%d]=%f\n",i,r_track[i]);
//    }
//    // //////////////////////////////////////////
    // 生成Rician因子 / DS / 角度方差输入矩阵
    const int square_total = para.square[0] * para.square[1];
    const int total = square_total * para.altitudes;
    if (para.square[0] != LSP_ROWS || para.square[1] != LSP_COLS || para.altitudes != LSP_ALTITUDES) {
        printf("[4ERR] LSP尺寸不匹配: rows=%d cols=%d alt=%d, 需要 %d x %d x %d\n",
               para.square[0], para.square[1], para.altitudes,
               LSP_ROWS, LSP_COLS, LSP_ALTITUDES);
        while (1) {}
    }
    uint32_t stage4_start = read_cycle32();
    uint32_t step_start = stage4_start;
    printf("[4A] 大尺度输入矩阵生成开始\n");
    const float log10_h = log10(100.0f);
    const float log10_l = log10(10.0f);
    const float exp_k_h = exp(-0.01659f * 100.0f);
    const float exp_k_l = exp(-0.01659f * 10.0f);
    const float exp_aaoa_l = 1.0f;
    const float exp_aaod_l = 1.0f;
    const float exp_eaoa_l = 1.0f;
    const float exp_eaod_l = 1.0f;
    const float exp_aaoa_h = exp(-0.0025f * 100.0f);
    const float exp_aaod_h = exp(-0.0008f * 100.0f);
    const float exp_eaoa_h = exp(-0.0068f * 100.0f);
    const float exp_eaod_h = exp(-0.0025f * 100.0f);
    const float k_h_base = -14.72f + 22.5f * log10_h;
    const float k_l_base = -14.72f + 22.5f * log10_l;
    const float k_h_sigma = 6.98f * exp_k_h;
    const float k_l_sigma = 6.98f * exp_k_l;
    const float aaoa_l_base = -2.214f * log10_l - 1.16f;
    const float aaod_l_base = -0.24f * log10_l - 0.15f;
    const float eaoa_l_base = -1.43f * log10_l - 0.89f;
    const float eaod_l_base = 0.81f * log10_l - 0.97f;
    const float aaoa_h_base = -2.214f * log10_h - 1.16f;
    const float aaod_h_base = -0.24f * log10_h - 0.15f;
    const float eaoa_h_base = -1.43f * log10_h - 0.89f;
    const float eaod_h_base = 0.81f * log10_h - 0.97f;
    const float aaoa_l_sigma = 2.56f * exp_aaoa_l;
    const float aaod_l_sigma = 1.7f * exp_aaod_l;
    const float eaoa_l_sigma = 2.2f * exp_eaoa_l;
    const float eaod_l_sigma = 1.5f * exp_eaod_l;
    const float aaoa_h_sigma = 2.56f * exp_aaoa_h;
    const float aaod_h_sigma = 1.7f * exp_aaod_h;
    const float eaoa_h_sigma = 2.2f * exp_eaoa_h;
    const float eaod_h_sigma = 1.5f * exp_eaod_h;

    for (int i = 0; i < square_total; i++) {
        K_ricean_db_h_input[i] = k_h_base + k_h_sigma * randn_gen_number[randn_gen_count++];
    }
    for (int i = 0; i < square_total; i++) {
        K_ricean_db_l_input[i] = k_l_base + k_l_sigma * randn_gen_number[randn_gen_count++];
    }
    for (int i = 0; i < square_total; i++) {
        DS_h_input[i] = para.DS_miu_h + para.DS_sigma_h * randn_gen_number[randn_gen_count++];
    }
    for (int i = 0; i < square_total; i++) {
        DS_l_input[i] = para.DS_miu_l + para.DS_sigma_l * randn_gen_number[randn_gen_count++];
    }
    for (int i = 0; i < square_total; i++) {
        AAoA_variance_l_input[i] = stage4_pow10_model(aaoa_l_base + aaoa_l_sigma * randn_gen_number[randn_gen_count++]);
    }
    for (int i = 0; i < square_total; i++) {
        AAoD_variance_l_input[i] = stage4_pow10_model(aaod_l_base + aaod_l_sigma * randn_gen_number[randn_gen_count++]);
    }
    for (int i = 0; i < square_total; i++) {
        EAoA_variance_l_input[i] = stage4_pow10_model(eaoa_l_base + eaoa_l_sigma * randn_gen_number[randn_gen_count++]);
    }
    for (int i = 0; i < square_total; i++) {
        EAoD_variance_l_input[i] = stage4_pow10_model(eaod_l_base + eaod_l_sigma * randn_gen_number[randn_gen_count++]);
    }
    for (int i = 0; i < square_total; i++) {
        AAoA_variance_h_input[i] = stage4_pow10_model(aaoa_h_base + aaoa_h_sigma * randn_gen_number[randn_gen_count++]);
    }
    for (int i = 0; i < square_total; i++) {
        AAoD_variance_h_input[i] = stage4_pow10_model(aaod_h_base + aaod_h_sigma * randn_gen_number[randn_gen_count++]);
    }
    for (int i = 0; i < square_total; i++) {
        EAoA_variance_h_input[i] = stage4_pow10_model(eaoa_h_base + eaoa_h_sigma * randn_gen_number[randn_gen_count++]);
    }
    for (int i = 0; i < square_total; i++) {
        EAoD_variance_h_input[i] = stage4_pow10_model(eaod_h_base + eaod_h_sigma * randn_gen_number[randn_gen_count++]);
    }
    printf("[4B] 大尺度输入矩阵生成完成 cycles=%u\n", (unsigned)(read_cycle32() - step_start));
    step_start = read_cycle32();
    

    
    // 应用指数滤波
#if USE_FAST_SEPARABLE_LSP_FILTER
    Exp_filter2_separable_fast(K_ricean_db_l_input, K_ricean_db_h_input,
                               para.square[0], para.square[1], para.Corr_distance, para.D_g,
                               K_ricean_db_l, K_ricean_db_h);
#else
    Exp_filter2(K_ricean_db_l_input, K_ricean_db_h_input,
                para.square[0], para.square[1], para.Corr_distance, para.D_g,
                K_ricean_db_l, K_ricean_db_h);
#endif
    // 已验证
     for (int i = 0; i < 3;i++){
     	 for(int j=0;j<para.square[1];j++){
//         printf("K_ricean_db_l[%d] = %.16f\n",i,K_ricean_db_l[i]);
        // printf("K_ricean_db_h[%d][%d] = %.16f\n",i,j,K_ricean_db_h[i*200+j]);
     }
}
    // print("K-滤波已生成\n");
    // 计算高度相关矩阵
    Corr_Altitudes_K_with_lin(K_ricean_db_l, K_ricean_db_h, para.square[0], para.square[1],para.altitudes, K_ricean_db, K_ricean_lin);
    printf("[4C] K因子滤波/高度相关完成 cycles=%u\n", (unsigned)(read_cycle32() - step_start));
    step_start = read_cycle32();

    //已验证
    // for (int i = 0; i < para.square[0] * para.square[1] * para.altitudes; i++) {
    //     printf("K_ricean_db[%d] = %.16f\n",i,K_ricean_db[i]);
    // } 
    // print("K-高度相关矩阵已生成\n");
    // // // ////////////////////////////
// printf("randn_gen_count = %.d\n",randn_gen_count);
/////////////////////////////////////////////////////
    //生成DS
 // 应用指数滤波
#if USE_FAST_SEPARABLE_LSP_FILTER
    Exp_filter2_separable_fast(DS_h_input, DS_l_input,
                               para.square[0], para.square[1], para.Corr_distance, para.D_g,
                               DS_h, DS_l);
#else
    Exp_filter2(DS_h_input, DS_l_input,
                para.square[0], para.square[1], para.Corr_distance, para.D_g,
                DS_h, DS_l);
#endif
    /*for (int i=0;i<2;i++){
    	for (int j=0;j<200;j++){
    		printf("Ds_j[%d][%d]=.4%f\n",i,j,DS_h[i*200+j]);
    	}
    }*/
    // 已验证DS_h与matlab生成相同
        //         printf("DS_h[0] = %.16f\n",DS_h[0]);
         //printf("DS_h[0] = %.16f\n",DS_h[0]);
    //     printf("DS_l[%d] = %.16f\n",i,DS_l[i]);
    // printf("DS_l[1407] = %.16f\n",DS_l[1407]);
    // }    
    // print("DS-滤波已生成\n");
    // 计算高度相关矩阵
    Corr_Altitudes(DS_l, DS_h, para.square[0], para.square[1],para.altitudes, DS);
    printf("[4D] DS滤波/高度相关完成 cycles=%u\n", (unsigned)(read_cycle32() - step_start));
    step_start = read_cycle32();
    //已验证与matlab相同
    // printf("DS[0] = %.16f\n",DS[0]);
    // printf("DS[1] = %.16f\n",DS[1]);
    // printf("DS[2] = %.16f\n",DS[2]);
    // printf("DS[281400] = %.16f\n",DS[281400]);
    //已验证
    // for (int i = 0; i < para.square[0] * para.square[1] * para.altitudes; i++) {
    //      printf("DS[%d] = %.16f\n",i,DS[i]);
    // } 
    // print("DS-高度相关矩阵已生成\n");
    // 计算 Nc0
//    int Nc0 = para.lambdaG / para.lambdaR;
    //Find_grid 函数
    int xx = 0, yy = 0, zz = 0;
    float ini_pos_t[3] = { t_track[0], t_track[1], t_track[2] };
    // float ini_pos_r[3] = { r_track[0], r_track[1], r_track[2] };
    xx = ceil(ini_pos_t[0] / para.D_g);
    yy = ceil(ini_pos_t[1] / para.D_g);
    zz = ceil(ini_pos_t[2] / para.delet_altitude);
    // /////验证找坐标格点
    // // 打印 xx、yy 和 zz 的值
    // printf("xx = %d\n", xx);
    // printf("yy = %d\n", yy);
    // printf("zz = %d\n", zz);
    // 生成 Delay
//    float Delay[MAX_CLUSTERS];
//    for (int i = 0; i < Nc0; i++) {//* pow(10, DS[zz].data[xx][yy])
//        float rand1 = rand_gen_number[rand_gen_count++] ;
//        Delay[i] = -para.DS_r  * pow(10, DS[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1]) * log(rand1);// 表示访问 DS 三维数组中第 zz 层、第 xx 行、第 yy 列的元素
//         //printf("para.DS_r = %.16f\n",para.DS_r);
//        // printf("DS = %.16f\n", DS[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1]);
//        // printf("rand1 = %.16f\n",rand1);
//        // printf("rand_gen_count = %.d\n",rand_gen_count);
//        //printf("Delay[%d] = %.16f\n", i, Delay[i]);
//     }
    // ///////////////////////////////////////////////
    // 生成 AAoA, AAoD, EAoA, EAoD
    // 创建输入矩阵

    // 应用指数滤波
    printf("[4E] 角度方差滤波开始\n");
#if USE_FAST_SEPARABLE_LSP_FILTER
    Exp_filter8_separable_fast(
        AAoA_variance_l_input, AAoD_variance_l_input, EAoA_variance_l_input, EAoD_variance_l_input,
        AAoA_variance_h_input, AAoD_variance_h_input, EAoA_variance_h_input, EAoD_variance_h_input,
        para.square[0], para.square[1], para.Corr_distance, para.D_g,
        AAoA_variance_l, AAoD_variance_l, EAoA_variance_l, EAoD_variance_l,
        AAoA_variance_h, AAoD_variance_h, EAoA_variance_h, EAoD_variance_h);
#else
    Exp_filter8(
        AAoA_variance_l_input, AAoD_variance_l_input, EAoA_variance_l_input, EAoD_variance_l_input,
        AAoA_variance_h_input, AAoD_variance_h_input, EAoA_variance_h_input, EAoD_variance_h_input,
        para.square[0], para.square[1], para.Corr_distance, para.D_g,
        AAoA_variance_l, AAoD_variance_l, EAoA_variance_l, EAoD_variance_l,
        AAoA_variance_h, AAoD_variance_h, EAoA_variance_h, EAoD_variance_h);
#endif
   // printf("AAoD_variance_l[0] = %.16f\n",AAoD_variance_l[0]);
        //  print("2\n");
    // print("角度滤波已生成\n");
    // 计算高度相关矩阵
    Corr_Altitudes4(
        AAoA_variance_l, AAoA_variance_h, AAoD_variance_l, AAoD_variance_h,
        EAoA_variance_l, EAoA_variance_h, EAoD_variance_l, EAoD_variance_h,
        para.square[0], para.square[1], para.altitudes,
        AAoA_variance, AAoD_variance, EAoA_variance, EAoD_variance);
    printf("[4F] 角度方差高度相关完成 cycles=%u\n", (unsigned)(read_cycle32() - step_start));
    printf("[4/7] 大尺度参数生成完成 total_cycles=%u\n", (unsigned)(read_cycle32() - stage4_start));
   // printf("AAoD_variance[0] = %.16f\n", AAoD_variance[0]);
    //printf("AAoD_variance[1] = %.16f\n", AAoD_variance[1]);
    // print("角度高度相关矩阵已生成\n");
    /*for (int i=0;i<400;i++){
    	printf("EAod_varience[%d]=%f\n",i,EAoD_variance_h[i]);
    }*/
// ////////////////////////////
   // ///////////////初始簇信息/////////////////////////////////////////////////////////////////////////////////////////////////////////
   // // //////////////////////
 
    // 计算 P_R
    float D_pr;
    /*para.D = norm(para.h_t_track(1, :) - para.h_t_track(2, :));*/
    D_pr = sqrt((t_track[0] - t_track[3]) * (t_track[0] - t_track[3])
        + (t_track[1] - t_track[4]) * (t_track[1] - t_track[4])
        + (t_track[2] - t_track[5]) * (t_track[2] - t_track[5]));
            //    printf("point %d: (%f, %f, %f)\n", 0, t_track[0], t_track[1], t_track[2]);
            //    printf("point %d: (%f, %f, %f)\n", 1, t_track[3], t_track[4], t_track[5]);
    float P_R = exp(-para.lambdaR * D_pr / para.Corr_distance);
    // printf("P_R = %f\n", P_R);
    // printf("para.lambdaR = %f\n", para.lambdaR);    
    // printf("P_R = %f\n", P_R);
    // 计算 N_new_avg
    float N_new_avg =1.2117;
    // printf("N_new_avg = %f\n", N_new_avg);
    //     printf("para.lambdaG = %f\n", para.lambdaG);
    //         printf("para.lambdaR = %f\n", para.lambdaR);
    //             printf("P_R = %f\n", P_R);
    //P_R = 0.951229
    //N_new_avg = 0.975412 已验证
	float Delay[MAX_CLUSTERS];
	for (int i=0;i<Nc0;i++){
		float rand1=rand_gen_number[rand_gen_count++] ;
		Delay[i]= -para.DS_r*stage4_pow10_model(DS[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1]) * logf(rand1);
		//printf("Delay[%d]=%.12f\n",i,Delay[i]);
	}
    for (int i=0;i<Nc0;i++){
    	P1[0][i]=stage4_pow10_model(-randn_gen_number[randn_gen_count++]*0.1f);
    	//printf("P1[%d]=%.6f\n",i,P1[0][i]);
    }
    float Theta_LoS[r_num_points];      // 存储计算的角度
    Angle(t_track,r_track,r_num_points,Theta_LoS);
//    for (int i=0;i<Nc0;i++){
//    	printf("Theta_Los[%d]=%.6f\n",i,Theta_LoS[i]);
//    }
    float Theta_duct;
    Theta_duct=PJ_Model(6,273.2+20.3,273.2+20,5);
    //printf("Theta_duct=%.6f\n",Theta_duct);
        //   print("2\n");
    //  print("延迟已生成\n");
    float AAoA[MAX_CLUSTERS], AAoD[MAX_CLUSTERS], EAoA[MAX_CLUSTERS], EAoD[MAX_CLUSTERS];
    for (int i = 0; i < Nc0; i++) {
        AAoA[i] = (AAoA_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1] * NorDis(0, 1) / 180 * pi) + para.nu_A_R;
         //已验证与matlab相同
         //printf("AAoA[%d] = %.16f\n", i, AAoA[i]);
    }
        for (int i = 0; i < Nc0; i++) {
        AAoD[i] = (AAoD_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1] * NorDis(0, 1) / 180 * pi) + para.nu_A_R;
         //printf("AAoD[%d] = %.16f\n", i, AAoD[i]);
    }
        truncated_gaussian(EAoA_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1] / 180 * pi,para.nu_A_R,-pi/2,-Theta_duct,Nc0,EAoA);
       // printf("EAoA_var=%f\n",EAoA_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1]);
//        for(int i=0;i<Nc0;i++){
//        	printf("EAoA[%d] = %.16f\n", i, EAoA[i]);
//        }
        truncated_gaussian(EAoD_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1] / 180 * pi,para.nu_A_R,-pi/2,-Theta_duct,Nc0,EAoD);
        //printf("EAoA_var=%f\n",EAoD_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1]);
//        for(int i=0;i<Nc0;i++){
//           printf("EAoD[%d] = %.16f\n", i, EAoD[i]);
//        }
    //print("角度已生成\n");
    //  print("3\n");
    // 生成 D_n_R 和 D_n_T
    float D_n_R[MAX_CLUSTERS], D_n_T[MAX_CLUSTERS];
//    printf("rand=%f\n",rand_gen_number[rand_gen_count++]);
    for (int i = 0; i < Nc0; i++) {
        D_n_R[i] = Exprnd(para.d_Rx_mean) + para.d_Rx_min;
                // D_n_R[i] = D_n_R_exp[i] + para.d_Rx_min;
        // D_n_T[i] = Exprnd(para.d_Tx_mean) + para.d_Tx_min;
        // printf("D_n_R[%d] = %.8f\n", i, D_n_R[i]);
    //    printf("D_n_T[%d] = %.16f\n", i, D_n_T[i]);
    }
        for (int i = 0; i < Nc0; i++) {
        // D_n_R[i] = Exprnd(para.d_Rx_mean) + para.d_Rx_min;
        D_n_T[i] = Exprnd(para.d_Tx_mean) + para.d_Tx_min;
                // D_n_T[i] = D_n_T_exp[i] + para.d_Tx_min;
        // printf("D_n_R[%d] = %.16f\n", i, D_n_R[i]);
       // printf("D_n_T[%d] = %.16f\n", i, D_n_T[i]);
    }
    // 生成 tau_tilder
    float tau_tilder[MAX_CLUSTERS];
    for (int i = 0; i < Nc0; i++) {
        tau_tilder[i] = Exprnd(para.tau_tilder_mean) + para.tau_tilder_min;
//        printf("tau_tilder[%d]=%.16f\n",i,tau_tilder[i]);
    }
//          print("4\n");
    // 定义数组
    float par_Tsigmax[MAX_CLUSTERS] = {0.0};
    float par_Tsigmay[MAX_CLUSTERS] = {0.0};
    float par_Tsigmaz[MAX_CLUSTERS] = {0.0};
    float par_Rsigmax[MAX_CLUSTERS] = {0.0};
    float par_Rsigmay[MAX_CLUSTERS] = {0.0};
    float par_Rsigmaz[MAX_CLUSTERS] = {0.0};
    
    // 调用 GenerateParameters 函数
    GenerateParameters( Nc0,par_Tsigmax, par_Tsigmay, par_Tsigmaz,
        par_Rsigmax, par_Rsigmay, par_Rsigmaz);
      for (int i = 0; i < Nc0; i++) {
       // printf("Tsigmaz[%d] = %.8f\n", i, par_Tsigmaz[i]);
     }

        //  print("5\n");
    float T_d0[MAX_CLUSTERS] = { 0.0 }, R_d0[MAX_CLUSTERS] = { 0.0 };
    float phiTA0[MAX_CLUSTERS] = { 0.0 }, phiTE0[MAX_CLUSTERS] = { 0.0 }, phiRA0[MAX_CLUSTERS] = { 0.0 }, phiRE0[MAX_CLUSTERS] = { 0.0 };
    // 转置数组D_n_T并赋值给par.T_d0
    for (int i = 0; i < Nc0; i++) {
        T_d0[i] = D_n_T[i];
        //已验证与matlab基本相同
        //printf("T_d0[%d] = %.16f\n", i, T_d0[i]);
    }
    // 转置数组D_n_R并赋值给par.R_d0
    for (int i = 0; i < Nc0; i++) {
        R_d0[i] = D_n_R[i];
        //printf("R_d0[%d] = %.16f\n", i, R_d0[i]);
    }
    // 转置数组AAoD并赋值给par.phiTA0
    for (int i = 0; i < Nc0; i++) {
        phiTA0[i] = AAoD[i];
        //printf("phiTA0[%d] = %.16f\n", i, phiTA0[i]);
    }
    // 转置数组EAoD并赋值给par.phiTE0
    for (int i = 0; i < Nc0; i++) {
        phiTE0[i] = EAoD[i];
       //printf("phiTE0[%d] = %.16f\n", i, phiTE0[i]);
    }
    // 转置数组AAoA并赋值给par.phiRA0
    for (int i = 0; i < Nc0; i++) {
        phiRA0[i] = AAoA[i];
        /*printf("phiRA0[%d] = %.16f\n", i, phiRA0[i]);*/
    }
    // 转置数组EAoA并赋值给par.phiRE0
    for (int i = 0; i < Nc0; i++) {
        phiRE0[i] = EAoA[i];
        //printf("phiRE0[%d] = %.16f\n", i, phiRE0[i]);
    }
            //  print("6\n");
    /////////////////////////////////////////////////////////////
    // 发射端初始簇信息计算
    float azimuthT[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
    float elevationT[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
    float rT[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
    // float new_ini_pos_t[MAX_CLUSTERS][3];
    int totalRays = 0;

    float a1T[CLUSTER_RAYS] = { 0.0 };
    float e1T[CLUSTER_RAYS] = { 0.0 };
    float r1T[CLUSTER_RAYS] = { 0.0 };
    //接收端初始簇信息计算
    float azimuthR[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
    float elevationR[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
    float rR[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
    // float new_ini_pos_r[MAX_CLUSTERS][3];
    //int totalRaysR = 0;
    float dd_T[CLUSTER_RAYS] = { 0.0 };
    float dd_R[CLUSTER_RAYS] = { 0.0 };
    float a1R[CLUSTER_RAYS] = { 0.0 };
    float e1R[CLUSTER_RAYS] = { 0.0 };
    float r1R[CLUSTER_RAYS] = { 0.0 };
    for (int k = 0; k < Nc0; k++) {

        // 调用 DiscrAngleLenEllipsoid 函数
    	//printf("k=%d\n",k);
    	if(k==7){
    		DiscrAngleLenEllipsoid222(1,CLUSTER_RAYS, T_d0[k], phiTA0[k], phiTE0[k], par_Tsigmax[k], par_Tsigmay[k], par_Tsigmaz[k],t_track,0,dd_T
    		                ,a1T, e1T, r1T);
    	}
    	else{
    		DiscrAngleLenEllipsoid222(0,CLUSTER_RAYS, T_d0[k], phiTA0[k], phiTE0[k], par_Tsigmax[k], par_Tsigmay[k], par_Tsigmaz[k],t_track,0,dd_T
    		                ,a1T, e1T, r1T);
    	}


//            printf("clus %d\n",k);
//        if(k==7){
//         for (int i = 0; i < CLUSTER_RAYS; i++) {
//             printf("a1T[%d] = %.8f\n", i, a1T[i]);
//             printf("e1T[%d] = %.8f\n", i, e1T[i]);
//             printf("r1T[%d] = %.8f\n", i, r1T[i]);
//         }
//        }
        // printf("函数");
        // 调用 DiscrAngleLenEllipsoid 函数
        DiscrAngleLenEllipsoid222(0,CLUSTER_RAYS, R_d0[k], phiRA0[k], phiRE0[k], par_Rsigmax[k], par_Rsigmay[k], par_Rsigmaz[k],r_track,0,dd_R,
            a1R, e1R, r1R);
//                if(k==7){
//                 for (int i = 0; i < CLUSTER_RAYS; i++) {
//                     printf("a1R[%d] = %.8f\n", i, a1R[i]);
//                     printf("e1R[%d] = %.8f\n", i, e1R[i]);
//                     printf("r1R[%d] = %.8f\n", i, r1R[i]);
//                 }
//                }
//        if(k==1){
//        	printf("R_d0[k] = %.16f\n", R_d0[k]);
//            printf("phiTA0[k] = %.16f\n",  phiTA0[k]);
//            printf("phiTE0[k] = %.16f\n",  phiTE0[k]);
//            printf("par_Tsigmax[k] = %.16f\n",  par_Tsigmax[k]);
//            printf("par_Tsigmay[k]= %.16f\n",  par_Tsigmay[k]);
//            printf("par_Tsigmaz[k] = %.16f\n",  par_Tsigmaz[k]);
//         for (int i = 0; i < CLUSTER_RAYS; i++) {
////             printf("a1R[%d] = %.16f\n", i, a1R[i]);
////             printf("e1R[%d] = %.16f\n", i, e1R[i]);
////             printf("r1R[%d] = %.16f\n", i, r1R[i]);
//         }
//        }
        //  print("7\n");
        // print("rand_gen_count = %d\n",rand_gen_count);
         
        // 将结果追加到数组中
        for (int i = 0; i < CLUSTER_RAYS; i++) {
            azimuthT[totalRays + i] = a1T[i];
            elevationT[totalRays + i] = e1T[i];
            rT[totalRays + i] = r1T[i];
        //     printf("a1T[%d] = %.16f\n",i, a1T[i]);
        //    printf("e1T[%d] = %.16f\n", i, e1T[i]);
        //    printf("r1T[%d] = %.16f\n", i, r1T[i]);
            azimuthR[totalRays + i] = a1R[i];
            elevationR[totalRays + i] = e1R[i];
            rR[totalRays + i] = r1R[i];
//            printf("N=%d,a1R[%d] = %.16f\n", k,i, a1R[i]);
//            printf("N=%d,e1R[%d] = %.16f\n", k,i, e1R[i]);
 //           printf("N=%d,r1R[%d] = %.16f\n", k,i, r1R[i]);
        }
        totalRays += CLUSTER_RAYS;
        // printf("totalRays = %d\n", totalRays);
    }
    // print("初始簇设置已生成");

    // ////////验证初始簇信息
//波导簇设置
totalRays=0;
float Delay_duct[MAX_CLUSTERS];
	for (int i=0;i<Nc0;i++){
		float rand1=rand_gen_number[rand_gen_count++] ;
		Delay_duct[i]= -para.DS_r*stage4_pow10_model(DS[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1]) * logf(rand1);
		//printf("Delay[%d]=%.12f\n",i,Delay[i]);
	}
for (int i=0;i<Nc0;i++){
	P1_duct[0][i]=stage4_pow10_model(-randn_gen_number[randn_gen_count++]*0.1f);
    	//printf("P1[%d]=%.6f\n",i,P1[0][i]);
}
float AAoA_duct[MAX_CLUSTERS], AAoD_duct[MAX_CLUSTERS], EAoA_duct[MAX_CLUSTERS], EAoD_duct[MAX_CLUSTERS];
truncated_gaussian(AAoA_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1] / 180 * pi,para.nu_A_R,-Theta_duct,Theta_duct,Nc0,AAoA_duct);
truncated_gaussian(AAoD_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1] / 180 * pi,para.nu_A_T,-Theta_duct,Theta_duct,Nc0,AAoD_duct);
//printf("AAoD_variance[%d]=%f",(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1,
//		AAoD_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1]);
truncated_gaussian(EAoA_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1] / 180 * pi,para.nu_A_R,-Theta_duct,Theta_duct,Nc0,EAoA_duct);
truncated_gaussian(EAoD_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1] / 180 * pi,para.nu_A_T,-Theta_duct,Theta_duct,Nc0,EAoD_duct);
//printf("EAoD_variance[%d]=%f",(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1,
//		EAoD_variance[(yy-1)*para.square[1]*para.altitudes + (xx-1)*para.altitudes + zz - 1]);
float D_n_R_duct[MAX_CLUSTERS], D_n_T_duct[MAX_CLUSTERS];
//    printf("rand=%f\n",rand_gen_number[rand_gen_count++]);
    for (int i = 0; i < Nc0; i++) {
        D_n_R_duct[i] = Exprnd(para.d_Rx_mean) + para.d_Rx_min;
                // D_n_R[i] = D_n_R_exp[i] + para.d_Rx_min;
        // D_n_T[i] = Exprnd(para.d_Tx_mean) + para.d_Tx_min;
        // printf("D_n_R[%d] = %.8f\n", i, D_n_R[i]);
    //    printf("D_n_T[%d] = %.16f\n", i, D_n_T[i]);
    }
        for (int i = 0; i < Nc0; i++) {
        // D_n_R[i] = Exprnd(para.d_Rx_mean) + para.d_Rx_min;
        D_n_T_duct[i] = Exprnd(para.d_Tx_mean) + para.d_Tx_min;
                // D_n_T[i] = D_n_T_exp[i] + para.d_Tx_min;
        // printf("D_n_R[%d] = %.16f\n", i, D_n_R[i]);
       // printf("D_n_T[%d] = %.16f\n", i, D_n_T[i]);
    }
    // 生成 tau_tilder
    float tau_tilder_duct[MAX_CLUSTERS];
    for (int i = 0; i < Nc0; i++) {
        tau_tilder_duct[i] = Exprnd(para.tau_tilder_mean) + para.tau_tilder_min;
       // printf("tau_tilder[%d]=%.16f\n",i,tau_tilder_duct[i]);
    }
    float par_Tsigmax_duct[MAX_CLUSTERS] = {0.0};
        float par_Tsigmay_duct[MAX_CLUSTERS] = {0.0};
        float par_Tsigmaz_duct[MAX_CLUSTERS] = {0.0};
        float par_Rsigmax_duct[MAX_CLUSTERS] = {0.0};
        float par_Rsigmay_duct[MAX_CLUSTERS] = {0.0};
        float par_Rsigmaz_duct[MAX_CLUSTERS] = {0.0};

        // 调用 GenerateParameters 函数
        GenerateParameters( Nc0,par_Tsigmax_duct, par_Tsigmay_duct, par_Tsigmaz_duct,
            par_Rsigmax_duct, par_Rsigmay_duct, par_Rsigmaz_duct);
        for (int i=0;i<Nc0;i++){
        	//printf("par_Rsigmaz_duct[%d]=%f\n",i,par_Rsigmaz_duct[i]);
        }
        float T_d0_duct[MAX_CLUSTERS] = { 0.0 }, R_d0_duct[MAX_CLUSTERS] = { 0.0 };
            float phiTA0_duct[MAX_CLUSTERS] = { 0.0 }, phiTE0_duct[MAX_CLUSTERS] = { 0.0 }, phiRA0_duct[MAX_CLUSTERS] = { 0.0 }, phiRE0_duct[MAX_CLUSTERS] = { 0.0 };
            // 转置数组D_n_T并赋值给par.T_d0
            for (int i = 0; i < Nc0; i++) {
                T_d0_duct[i] = D_n_T_duct[i];
                //已验证与matlab基本相同
                //printf("T_d0[%d] = %.16f\n", i, T_d0_duct[i]);
            }
            // 转置数组D_n_R并赋值给par.R_d0
            for (int i = 0; i < Nc0; i++) {
                R_d0_duct[i] = D_n_R_duct[i];
                //printf("R_d0[%d] = %.16f\n", i, R_d0[i]);
            }
            // 转置数组AAoD并赋值给par.phiTA0
            for (int i = 0; i < Nc0; i++) {
                phiTA0_duct[i] = AAoD_duct[i];
//                printf("phiTA0[%d] = %.16f\n", i, AAoD_duct[i]);
            }
            // 转置数组EAoD并赋值给par.phiTE0
            for (int i = 0; i < Nc0; i++) {
                phiTE0_duct[i] = EAoD_duct[i];
//               printf("phiTE0[%d] = %.16f\n", i, phiTE0_duct[i]);
            }
            // 转置数组AAoA并赋值给par.phiRA0
            for (int i = 0; i < Nc0; i++) {
                phiRA0_duct[i] = AAoA_duct[i];
//                printf("phiRA0[%d] = %.16f\n", i, phiRA0_duct[i]);
            }
            // 转置数组EAoA并赋值给par.phiRE0
            for (int i = 0; i < Nc0; i++) {
                phiRE0_duct[i] = EAoA_duct[i];
//                printf("phiRE0[%d] = %.16f\n", i, phiRE0_duct[i]);
            }
                    //  print("6\n");
            /////////////////////////////////////////////////////////////
            // 发射端初始簇信息计算
            float azimuthT_duct[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
            float elevationT_duct[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
            float rT_duct[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
            // float new_ini_pos_t[MAX_CLUSTERS][3];

            float a1T_duct[CLUSTER_RAYS] = { 0.0 };
            float e1T_duct[CLUSTER_RAYS] = { 0.0 };
            float r1T_duct[CLUSTER_RAYS] = { 0.0 };
            //接收端初始簇信息计算
            float azimuthR_duct[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
            float elevationR_duct[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
            float rR_duct[(MAX_CLUSTERS+ADD_CLUSTERS)*CLUSTER_RAYS];
            // float new_ini_pos_r[MAX_CLUSTERS][3];
            //int totalRaysR = 0;

            float a1R_duct[CLUSTER_RAYS] = { 0.0 };
            float e1R_duct[CLUSTER_RAYS] = { 0.0 };
            float r1R_duct[CLUSTER_RAYS] = { 0.0 };
            for (int k = 0; k < Nc0; k++) {


            	if(k==7){
            		DiscrAngleLenEllipsoid(0,CLUSTER_RAYS, T_d0_duct[k], phiTA0_duct[k], phiTE0_duct[k], par_Tsigmax_duct[k], par_Tsigmay_duct[k], par_Tsigmaz_duct[k]
            		                ,a1T_duct, e1T_duct, r1T_duct);
            	}
            	else{
            		DiscrAngleLenEllipsoid(0,CLUSTER_RAYS, T_d0_duct[k], phiTA0_duct[k], phiTE0_duct[k], par_Tsigmax_duct[k], par_Tsigmay_duct[k], par_Tsigmaz_duct[k]
            		                ,a1T_duct, e1T_duct, r1T_duct);
            	}

//                if(k==7){
//                 for (int i = 0; i < CLUSTER_RAYS; i++) {
//                     printf("a1T[%d] = %.8f\n", i, a1T_duct[i]);
//                     printf("e1T[%d] = %.8f\n", i, e1T_duct[i]);
//                     printf("r1T[%d] = %.8f\n", i, r1T_duct[i]);
//                 }
//                }

                DiscrAngleLenEllipsoid(0,CLUSTER_RAYS, R_d0_duct[k], phiRA0_duct[k], phiRE0_duct[k], par_Rsigmax_duct[k], par_Rsigmay_duct[k], par_Rsigmaz_duct[k],
                    a1R_duct, e1R_duct, r1R_duct);
//                        if(k==7){
//                         for (int i = 0; i < CLUSTER_RAYS; i++) {
//                             printf("a1R[%d] = %.8f\n", i, a1R_duct[i]);
//                             printf("e1R[%d] = %.8f\n", i, e1R_duct[i]);
//                             printf("r1R[%d] = %.8f\n", i, r1R_duct[i]);
//                         }
//                        }
        //        if(k==1){
        //        	printf("R_d0[k] = %.16f\n", R_d0[k]);
        //            printf("phiTA0[k] = %.16f\n",  phiTA0[k]);
        //            printf("phiTE0[k] = %.16f\n",  phiTE0[k]);
        //            printf("par_Tsigmax[k] = %.16f\n",  par_Tsigmax[k]);
        //            printf("par_Tsigmay[k]= %.16f\n",  par_Tsigmay[k]);
        //            printf("par_Tsigmaz[k] = %.16f\n",  par_Tsigmaz[k]);
        //         for (int i = 0; i < CLUSTER_RAYS; i++) {
        ////             printf("a1R[%d] = %.16f\n", i, a1R[i]);
        ////             printf("e1R[%d] = %.16f\n", i, e1R[i]);
        ////             printf("r1R[%d] = %.16f\n", i, r1R[i]);
        //         }
        //        }


                // 将结果追加到数组中
                for (int i = 0; i < CLUSTER_RAYS; i++) {
                    azimuthT_duct[totalRays + i] = a1T_duct[i];
                    elevationT_duct[totalRays + i] = e1T_duct[i];
                    rT_duct[totalRays + i] = r1T_duct[i];
                //     printf("a1T[%d] = %.16f\n",i, a1T[i]);
                //    printf("e1T[%d] = %.16f\n", i, e1T[i]);
                //    printf("r1T[%d] = %.16f\n", i, r1T[i]);
                    azimuthR_duct[totalRays + i] = a1R_duct[i];
                    elevationR_duct[totalRays + i] = e1R_duct[i];
                    rR_duct[totalRays + i] = r1R_duct[i];
        //            printf("N=%d,a1R[%d] = %.16f\n", k,i, a1R[i]);
        //            printf("N=%d,e1R[%d] = %.16f\n", k,i, e1R[i]);
         //           printf("N=%d,r1R[%d] = %.16f\n", k,i, r1R[i]);
                }
                totalRays += CLUSTER_RAYS;
                // printf("totalRays = %d\n", totalRays);
            }
//     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //      // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//       // /////////簇的生灭
       
    //分配内存
//波浪簇数据
    float *Delay_t = Delay_t_storage;//200
    float *D_Sum = D_Sum_storage;
    // float D_Sep[MAX_TIME * CLUSTER_RAYS * (MAX_CLUSTERS + 60)];
    // float Dpqt_los[MAX_MT * MAX_MR * MAX_TIME]= {0.0};
    float *Delay_t_permuted = Delay_t_permuted_storage;
    float *new_azimuthT = new_azimuthT_storage;
    float *new_elevationT = new_elevationT_storage;
    float *new_rT = new_rT_storage;
    float *new_azimuthR = new_azimuthR_storage;
    float *new_elevationR = new_elevationR_storage;
    float *new_rR = new_rR_storage;

    float *new_Delay = new_Delay_storage;
    float *new_tau_tilder = new_tau_tilder_storage;
    float Dpqt = 0.0;
    float *new_ini_pos_t = new_ini_pos_t_storage;
    float *new_ini_pos_r = new_ini_pos_r_storage;
//波导簇数据
    float *Delay_t_duct = Delay_t_duct_storage;//200
    float *D_Sum_duct = D_Sum_duct_storage;
    // float D_Sep[MAX_TIME * CLUSTER_RAYS * (MAX_CLUSTERS + 60)];
    // float Dpqt_los[MAX_MT * MAX_MR * MAX_TIME]= {0.0};
    float *Delay_t_permuted_duct = Delay_t_permuted_duct_storage;
    float *new_azimuthT_duct = new_azimuthT_duct_storage;
    float *new_elevationT_duct = new_elevationT_duct_storage;
    float *new_rT_duct = new_rT_duct_storage;
    float *new_azimuthR_duct = new_azimuthR_duct_storage;
    float *new_elevationR_duct = new_elevationR_duct_storage;
    float *new_rR_duct = new_rR_duct_storage;

    float *new_Delay_duct = new_Delay_duct_storage;
    float *new_tau_tilder_duct = new_tau_tilder_duct_storage;
    float Dpqt_duct = 0.0;
    float *new_ini_pos_t_duct = new_ini_pos_t_duct_storage;
    float *new_ini_pos_r_duct = new_ini_pos_r_duct_storage;
         // 初始化InitPhi
    for (int i = 0; i < CLUSTER_RAYS; i++) {
        for (int j = 0; j < MAX_CLUSTERS; j++) {
            InitPhi[j * CLUSTER_RAYS + i] =
                rand_gen_number[rand_gen_count++] * 2 * pi;
//            if(i==2){
//            printf("InitPhi[%d]=%.6f\n", j * CLUSTER_RAYS + i,
//                   InitPhi[j * CLUSTER_RAYS + i]);
//            }

        }
    }


        //初始化Delay_t   new_Delay
        for (int p = 0; p < MAX_MT; p++) {
             for (int q = 0; q < MAX_MR; q++) {
                 for (int tind = 0; tind < MAX_TIME; tind++) {
                     for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
                         Delay_t[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = 1.0;              
                     }
                }
             }
        }
        for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
                new_Delay[c] = 1.0;              
        }



        for (int i = 0; i < CLUSTER_RAYS; i++) {
                for (int j = 0; j < MAX_CLUSTERS; j++) {
                    InitPhi_duct[j * CLUSTER_RAYS + i] =
                        rand_gen_number[rand_gen_count++] * 2 * pi;
//                    if(j==2){
//                    printf("InitPhi_duct[%d]=%.6f\n", j * CLUSTER_RAYS + i,
//                    		InitPhi_duct[j * CLUSTER_RAYS + i]);
//                    }

                }
            }


                //初始化Delay_t   new_Delay
   for (int p = 0; p < MAX_MT; p++) {
     for (int q = 0; q < MAX_MR; q++) {
      for (int tind = 0; tind < MAX_TIME; tind++) {
       for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
        Delay_t_duct[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = 1.0;
     }
    }
  }
 }
 for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
    new_Delay_duct[c] = 1.0;
 }
 // 初始化 Nc
    int Nc[MAX_TIME];
    Nc[0] = Nc0;
    int Nc1[MAX_TIME];
    Nc1[0] = Nc0;
    float Dlos[MAX_TIME];
    float Dlos_duct[MAX_TIME];
    int cnt=19;
    int cnt1=19;
    // printf("Nc[0] = %d\n", Nc[0]);
        //      int N_new[41] = { 0,1,0,2,0,1,
        //      0,3,0,2,0,
        //      3,1,1,2,1,
        //      0,2,2,0,1,
        //      1,1,1,1,1,
        //      0,1,1,0,3,
        //      3,2,2,2,0,
        //      0,0,0,1,0};
        //    float rand_delay[42] ={
        //             0.5760,0.4331,0.9369,0.0099,0.1479,0.2452, 0.3569,0.8961,0.5920,0.9592
        //             ,0.9287,0.0453,0.3720,0.9966,0.8642,0.5418,0.6636,0.7015,0.1893,0.8195
        //             ,0.1559,0.3095,0.1392,0.1822,0.3867,0.7704,0.6749,0.3810,0.7562,0.4918
        //             ,0.3031,0.1608,0.1359,0.1599,0.8413,0.3714,0.5848,0.5531,0.7207,0.5339,
        //             0.3824,0.7011};
        //           int num_rand_delay = 0;  
    for (int tind = 0; tind < t_num_points-1; tind++) {

        // printf("tind = %d\n", tind);
        int xx_t = 0, yy_t = 0, zz_t = 0;
        float h_t_track_tind[3] = { t_track[3*tind], t_track[3*tind + 1], t_track[3*tind + 2] };
        xx_t = ceil(h_t_track_tind[0] / para.D_g);
        yy_t = ceil(h_t_track_tind[1] / para.D_g);
        zz_t = ceil(h_t_track_tind[2] / para.delet_altitude);
        //printf("xx_t = %d,yy_t = %d,zz_t = %d,\n", xx_t,yy_t,zz_t);
        /*Find_grid(h_t_track_tind, para.D_g, para.delet_altitude, &xx, &yy, &zz);*/
        // float t = para.TimeLine[tind];

        int new_totalRays = 0;
        if (tind == 0) {
            // 初始化数组 t=0时刻为初始簇
            for (int c = 0; c < Nc0; c++) {
                for (int i = 0; i < CLUSTER_RAYS; i++) {
                    new_azimuthT[c * CLUSTER_RAYS + i] = azimuthT[new_totalRays + i];
                    new_elevationT[c * CLUSTER_RAYS + i] = elevationT[new_totalRays + i];
                    new_rT[c * CLUSTER_RAYS + i] = rT[new_totalRays + i];
                    new_azimuthR[c * CLUSTER_RAYS + i] = azimuthR[new_totalRays + i];
                    new_elevationR[c * CLUSTER_RAYS + i] = elevationR[new_totalRays + i];
                    new_rR[c * CLUSTER_RAYS + i] = rR[new_totalRays + i];
                    new_InitPhi[0][c * CLUSTER_RAYS + i] = InitPhi[c * CLUSTER_RAYS + i];
                    //new2_ini_pos_t[c][i] = new_ini_pos_t[new_totalRays + i];
                    //new2_ini_pos_r[c][i] = new_ini_pos_r[new_totalRays + i];
                }
                /* totalRaysR += n[c];*/
                new_totalRays += CLUSTER_RAYS;
                new_ini_pos_t[c * 3] = t_track[0];
                new_ini_pos_t[c * 3 + 1] = t_track[1];
                new_ini_pos_t[c * 3 + 2] = t_track[2];
                new_ini_pos_r[c * 3] = r_track[0];
                new_ini_pos_r[c * 3 + 1] = r_track[1];
                new_ini_pos_r[c * 3 + 2] = r_track[2];
                new_tau_tilder[c] = tau_tilder[c];
                new_Delay[c] = Delay[c];
                // printf("new_Delay[%d] = %.16f\n", c, new_Delay[c]);
            }
//            for(int i=0;i<20;i++){
//            	for(int j=0;j<10;j++){
//            		printf("new_InitPhi[%d][%d]=%f\n",i,j,new_InitPhi[i*10+j]);
//            	}
//            }
        }
        else {
        	for (int clus=0;clus<(MAX_CLUSTERS + ADD_CLUSTERS);clus++){
        		P1[tind][clus]=P1[tind-1][clus];
        		for(int j=0;j<10;j++)
        			new_InitPhi[tind][clus*10+j]=new_InitPhi[tind-1][clus*10+j];
        	}
               Nc[tind] = Nc[tind - 1];
               if(tind==2){
            	   for(int i=0;i<25;i++){
            		   for(int c=0;c<10;c++){
            			   //printf("new_InitPhi[2][%d][%d]=%f\n",i,c,new_InitPhi[tind][i*10+c]);
            		   }
            	   }
               }
               // printf("Nc[%d] = %.16f\n", tind, Nc[tind]);
               int c = 0;
               //大于判断簇消失
               while (c < Nc[tind]) {
            	   float m=rand_gen_number[rand_gen_count++] ;
//            	   if(tind==14||tind==15)
//            	   printf("tind %d m=%f\n",tind,m);
                if (m > P_R) {
                    for (int i = 0; i < CLUSTER_RAYS; i++) {
                        new_azimuthT[c*CLUSTER_RAYS + i] = 0.0;
                        new_elevationT[c*CLUSTER_RAYS + i] = 0.0;
                        new_rT[c*CLUSTER_RAYS + i] = 0.0;
                        new_azimuthR[c*CLUSTER_RAYS + i] = 0.0;
                        new_elevationR[c*CLUSTER_RAYS + i] = 0.0;
                        new_rR[c*CLUSTER_RAYS + i] = 0.0;
                        new_InitPhi[tind][c*CLUSTER_RAYS + i] = 0.0;
                    }
                    P1[tind][c]=0;
                    new_tau_tilder[c] = 0.0;
                    new_Delay[c] = 1.0;
                    /*      printf("new_Delay[%d] = %.16f\n", c, new_Delay[c]);*/
                }
                c++;
               }
              //新簇的产生
               int N_new;
//               if(tind==14){
//            	   float test=rand_gen_number[rand_gen_count++];
//            	   printf("test[%d]=%f\n",tind,test);
//               }else {
                 N_new = poissrnd(N_new_avg);
//               }
                 //printf("N_new[%d]=%d\n",tind,N_new);
                // if (N_new[tind] > 0) {
                //  Nc[tind] = Nc[tind - 1] + N_new[tind];
                //  for (int n = 0; n < N_new[tind]; n++) {
                if (N_new > 0) {
                 Nc[tind] = Nc[tind] + N_new;
                 for (int n = 0; n < N_new; n++) {
                     int nn = CLUSTER_RAYS;
                     float D_n_Rt = Exprnd(para.d_Rx_mean) + para.d_Rx_min;
                     float D_n_Tt = Exprnd(para.d_Tx_mean) + para.d_Tx_min;
                    float AAoAt = (AAoA_variance[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1] * NorDis(0, 1) / 180) * pi + para.nu_A_R;
                    float AAoDt = (AAoD_variance[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1] * NorDis(0, 1) / 180) * pi + para.nu_A_R;
                    float EAoAt;
                    //printf("rand[%d]=%f\n",tind,rand_gen_number[rand_gen_count++]);
                    truncated_gaussian(EAoA_variance[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1] / 180 * pi,para.nu_A_R,-pi/2,-Theta_duct,1,&EAoAt);
                    float EAoDt;
                    //printf("next rand[%d]=%f\n",tind,rand_gen_number[rand_gen_count++]);
					truncated_gaussian(EAoD_variance[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1] / 180 * pi,para.nu_A_R,-pi/2,-Theta_duct,1,&EAoDt);
					//printf("%d,AAoDt=%.6f\n",tind,EAoDt);

                    // float D_n_Rt = D_n_Rt_exp[dnrnum] + para.d_Rx_min;
                    // float D_n_Tt = D_n_Tt_exp [dntnum]+ para.d_Tx_min;
                    // dnrnum  = dnrnum + 1;
                    //  dntnum  = dntnum + 1;

                   // printf("D_n_Tt=%.6f\n",D_n_Tt);
                    float Tsigmaxt = NorDis3(para.Tsigmax, para.sigmaxvar);
                    float Tsigmayt = NorDis3(para.Tsigmay, para.sigmayvar);
                    float Tsigmazt = NorDis3(para.Tsigmaz, para.sigmazvar);
                    float Rsigmaxt = NorDis3(para.Rsigmax, para.sigmaxvar);
                    float Rsigmayt = NorDis3(para.Rsigmay, para.sigmayvar);
                    float Rsigmazt = NorDis3(para.Rsigmaz, para.sigmazvar);
                    float dd_T1[CLUSTER_RAYS];
                    float dd_R1[CLUSTER_RAYS];
                    //printf("Rsigmazt = %.16f\n",Rsigmazt);
                    float new_T_a1[CLUSTER_RAYS] = {0};
                    float new_T_e1[CLUSTER_RAYS] = {0};
                    float new_T_r1[CLUSTER_RAYS] = {0};
                    float new_R_a1[CLUSTER_RAYS], new_R_e1[CLUSTER_RAYS], new_R_r1[CLUSTER_RAYS];
                    DiscrAngleLenEllipsoid222(0,CLUSTER_RAYS, D_n_Tt, AAoDt, EAoDt, Tsigmaxt, Tsigmayt, Tsigmazt,t_track,tind,dd_T1
                            ,new_T_a1, new_T_e1, new_T_r1);
                    DiscrAngleLenEllipsoid222(0,CLUSTER_RAYS, D_n_Rt, AAoAt, EAoAt, Rsigmaxt, Rsigmayt, Rsigmazt,r_track,tind,dd_R1
                            ,new_R_a1, new_R_e1, new_R_r1);
                    new_ini_pos_t[(Nc[tind - 1] + n) * 3] = t_track[tind * 3];
                    new_ini_pos_t[(Nc[tind - 1] + n) * 3 + 1] = t_track[tind * 3 + 1];
                    new_ini_pos_t[(Nc[tind - 1] + n) * 3 + 2] = t_track[tind * 3 + 2];
                    new_ini_pos_r[(Nc[tind - 1] + n) * 3] = r_track[tind * 3];
                    new_ini_pos_r[(Nc[tind - 1] + n) * 3 + 1] = r_track[tind * 3 + 1];
                    new_ini_pos_r[(Nc[tind - 1] + n) * 3 + 2] = r_track[tind * 3 + 2];

                    // 将结果追加到数组中 给新簇信息
                    for (int i = 0; i < nn; i++) {
                        new_azimuthT[(Nc[tind - 1] + n) * nn + i] = new_T_a1[i];
                        new_elevationT[(Nc[tind - 1] + n) * nn + i] = new_T_e1[i];
                        new_rT[(Nc[tind - 1] + n) * nn + i] = new_T_r1[i];

                        new_azimuthR[(Nc[tind - 1] + n) * nn + i] = new_R_a1[i];
                        new_elevationR[(Nc[tind - 1] + n) * nn + i] = new_R_e1[i];
                        new_rR[(Nc[tind - 1] + n) * nn + i] = new_R_r1[i];
//                        if(tind==2){
//                          printf("%d,new_azimuthT[%d][%d]=%.6f\n",tind,Nc[tind - 1] + n,i,new_azimuthT[(Nc[tind - 1] + n) * nn + i]);
//                          printf("%d,new_elevationT[%d][%d]=%.6f\n",tind,Nc[tind - 1] + n,i,new_elevationT[(Nc[tind - 1] + n) * nn + i]);
//                          printf("%d,new_rT[%d][%d]=%.6f\n",tind,Nc[tind - 1] + n,i,new_rT[(Nc[tind - 1] + n) * nn + i]);
//                        }
                    }

//                    float rand_delay = rand_gen_number[rand_gen_count++];
                    cnt++;

                    P1[tind][cnt]=stage4_pow10_model(-randn_gen_number[randn_gen_count++]*0.1f);

                    // printf("rand_delay = %.16f\n", rand_delay);
                    new_Delay[Nc[tind - 1] + n] = -para.DS_r * stage4_pow10_model(DS[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1]) * logf(rand_gen_number[rand_gen_count++]);//[num_rand_delay]
                    // num_rand_delay = num_rand_delay + 1;
                    for (int i = 0; i < nn; i++) {
                        new_InitPhi[tind][(Nc[tind - 1] + n) * nn + i] = (rand_gen_number[rand_gen_count++]) * 2 * pi;
                    }
                    //new_tau_tilder[Nc[tind - 1] + n] = 1;
                    new_tau_tilder[Nc[tind - 1] + n] = Exprnd(para.tau_tilder_mean) + para.tau_tilder_min;
                    //printf("tind %d tau=%f\n",tind,rand_gen_number[rand_gen_count++]);
                 }
              }
        }

        // 设置默认值
        float v_c = 0.0;
        float theta_A = 0.0;
        float theta_E = 0.0;
        float tb = 0.0;
        float DptT[CLUSTER_RAYS];
        float DqtR[CLUSTER_RAYS];
        //float Dpqt[MAX_RAYS];
        float mean_DptT = 0.0;
        float mean_DqtR = 0.0;

        //float T_pos_t[MAX_RAYS][3];
        //float R_pos_t[MAX_RAYS][3];
        float T_pos_t[3];
        float R_pos_t[3];

        //printf("Nc[%d] = %d\n", tind, Nc[tind]);

        for (int c = 0; c < Nc[tind]; c++) {//Nc[tind]MAX_CLUSTERS + 40
            for (int p = 0; p < MAX_MT; p++) {
                float deltap = p * para.delta_tx * para.lambda;
                //printf("deltap = %.16f\n", deltap);
                for (int q = 0; q < MAX_MR; q++) {
                    float deltaq = q * para.delta_rx * para.lambda;

                    T_pos_t[0] = t_track[tind * 3];
                    T_pos_t[1] = t_track[tind * 3 + 1];
                    T_pos_t[2] = t_track[tind * 3 + 2];
                    R_pos_t[0] = r_track[tind * 3];
                    R_pos_t[1] = r_track[tind * 3 + 1];
                    R_pos_t[2] = r_track[tind * 3 + 2];

                    float new2_ini_pos_t[3];
                    float new2_ini_pos_r[3];
                    new2_ini_pos_t[0] = new_ini_pos_t[c * 3];
                    new2_ini_pos_t[1] = new_ini_pos_t[c * 3 + 1];
                    new2_ini_pos_t[2] = new_ini_pos_t[c * 3 + 2];
                    new2_ini_pos_r[0] = new_ini_pos_r[c * 3];
                    new2_ini_pos_r[1] = new_ini_pos_r[c * 3 + 1];
                    new2_ini_pos_r[2] = new_ini_pos_r[c * 3 + 2];
                    //printf("tind = %d\n", tind);
                    //printf("Nc[%d] = %d\n", tind,Nc[tind]);
                    // 打印 T_pos_t 的所有元素
                    // printf("T_pos_t[0] = %.16f, T_pos_t[1] = %.16f, T_pos_t[2] = %.16f\n", T_pos_t[0], T_pos_t[1], T_pos_t[2]);
                    // // 打印 R_pos_t 的所有元素
                    // printf("R_pos_t[0] = %.16f, R_pos_t[1] = %.16f, R_pos_t[2] = %.16f\n", R_pos_t[0], R_pos_t[1], R_pos_t[2]);
                    // // 打印 new2_ini_pos_t 的所有元素
                    // printf("new2_ini_pos_t[0] = %.16f, new2_ini_pos_t[1] = %.16f, new2_ini_pos_t[2] = %.16f\n", new2_ini_pos_t[0], new2_ini_pos_t[1], new2_ini_pos_t[2]);
                    // // 打印 R_pos_t 的所有元素
                    // printf("new2_ini_pos_r[0] = %.16f, new2_ini_pos_r[1] = %.16f, new2_ini_pos_r[2] = %.16f\n", new2_ini_pos_r[0], new2_ini_pos_r[1], new2_ini_pos_r[2]);
                    float new2_azimuthR[CLUSTER_RAYS];
                    float new2_elevationR[CLUSTER_RAYS];
                    float new2_rR[CLUSTER_RAYS];
                    float new2_azimuthT[CLUSTER_RAYS];
                    float new2_elevationT[CLUSTER_RAYS];
                    float new2_rT[CLUSTER_RAYS];
                  
                    for (int i = 0; i < CLUSTER_RAYS; i++) {
                        new2_azimuthR[i] = new_azimuthR[c * CLUSTER_RAYS + i];
                        new2_elevationR[i] = new_elevationR[c * CLUSTER_RAYS + i];
                        new2_rR[i] = new_rR[c * CLUSTER_RAYS + i];
                        new2_azimuthT[i] = new_azimuthT[c * CLUSTER_RAYS + i];
                        new2_elevationT[i] = new_elevationT[c * CLUSTER_RAYS + i];
                        new2_rT[i] = new_rT[c * CLUSTER_RAYS + i]; 
                        if(1<tind&&tind<3){
//                        printf("%d new2_azimuthR[%d] = %.16f\n", c,i,new2_azimuthR[i]);
//                        printf("%d new2_elevationR[%d] = %.16f\n",tind,i,new2_elevationR[i]);
//                        printf("%d,new2_rR[%d] = %.16f\n", tind,i,new2_rR[i]);
//                        printf("%d new2_azimuthT[%d] = %.16f\n", c,i,new2_azimuthT[i]);
                       // printf("%d new2_elevationT[%d] = %.16f\n",c, i,new2_elevationT[i]);
//                        printf("%d,new2_rT[%d] = %.16f\n",tind, i,new2_rT[i]);
                        }
                    }

                    TravelLength3DSim(new2_ini_pos_t, T_pos_t, para.nu_E_T, para.nu_A_T, deltap, new2_rT, new2_elevationT, new2_azimuthT, DptT, 0, 0, 0, 0);
                    TravelLength3DSim(new2_ini_pos_r, R_pos_t, para.nu_E_R, para.nu_A_R, deltaq, new2_rR, new2_elevationR, new2_azimuthR, DqtR, 0, 0, 0, 0);
                    if(tind<5){
//                    printf("new_ini_pos_t[%d][0] = %.6f, new_ini_pos_t[%d][1] = %.6f, new_ini_pos_t[%d][2] = %.6f\n",c, new2_ini_pos_t[0], c, new2_ini_pos_t[1], c, new2_ini_pos_t[2]);
//
//                    printf("new_ini_pos_r[%d][0] = %.6f, new_ini_pos_r[%d][1] = %.6f, new_ini_pos_r[%d][2] = %.6f\n", c, new_ini_pos_r[c][0], c, new_ini_pos_r[c][1], c, new_ini_pos_r[c][2]);

                }
                    /////////已验证
//                    if(tind==2){
//						 for (int i = 0; i <10; i++) {
//							printf("%d,DptT[%d] = %.16f\n",c, i,DptT[i]);
//						  printf("DqtR[%d] = %.16f\n", i,DqtR[i]);
//						 }
//                    }
                    TravelLengthLoS(T_pos_t, R_pos_t, para.nu_E_T, para.nu_A_T, deltap, para.nu_E_R, para.nu_A_R, deltaq, &Dpqt);
                    Dlos[tind]=Dpqt;
                    // // 打印 T_pos_t 的所有元素
                    //printf("T_pos_t[0] = %.16f, T_pos_t[1] = %.16f, T_pos_t[2] = %.16f\n", T_pos_t[0], T_pos_t[1], T_pos_t[2]);
                    //// 打印 R_pos_t 的所有元素
                    // printf("R_pos_t[0] = %.16f, R_pos_t[1] = %.16f, R_pos_t[2] = %.16f\n", R_pos_t[0], R_pos_t[1], R_pos_t[2]);
//                    if(tind==3)
//                         printf("直线距离 Dpqt: %f\n", Dpqt);
                    if(1<tind&&tind<3){
                    	for(int i=0;i<10;i++){
//							printf("%d new2_azimuthR[%d] = %.16f\n", tind,i,new2_azimuthR[i]);
//							printf("%d new2_elevationR[%d] = %.16f\n",tind,i,new2_elevationR[i]);
//							printf("%d,new2_rR[%d] = %.16f\n", tind,i,new2_rR[i]);
//                    		printf("%d new2_azimuthT[%d] = %.16f\n", tind,i,new2_azimuthT[i]);
//							printf("%d new2_elevationT[%d] = %.16f\n",tind, i,new2_elevationT[i]);
//							printf("%d,new2_rT[%d] = %.16f\n",tind, i,new2_rT[i]);
                    	}
//                        printf("%d new2_ini_pos_t[0] = %.6f, new2_ini_pos_t[1] = %.6f, new2_ini_pos_t[2] = %.6f\n", tind,new2_ini_pos_t[0], new2_ini_pos_t[1], new2_ini_pos_t[2]);
//                        printf("%d new2_ini_pos_r[0] = %.6f, new2_ini_pos_r[1] = %.6f, new2_ini_pos_r[2] = %.6f\n", tind,new2_ini_pos_r[0], new2_ini_pos_r[1], new2_ini_pos_r[2]);
                    }
                    float dm = 0.0;
                    LengthCACZ(tind,new2_ini_pos_t[0], new2_ini_pos_t[1], new2_ini_pos_t[2], new2_ini_pos_r[0], new2_ini_pos_r[1], new2_ini_pos_r[2],
                        new2_elevationT, new2_azimuthT, new2_rT, new2_elevationR, new2_azimuthR, new2_rR, &dm);
                    if(tind==2){
                        //printf("%d,dm = %.16f\n",tind, dm);
                    }

                // float mean_DptT;
                // float mean_DqtR;
                calculateMean(DptT, CLUSTER_RAYS,&mean_DptT);
                calculateMean(DqtR, CLUSTER_RAYS,&mean_DqtR);
                // printf("mean_DptT = %.16f\n", mean_DptT);
                // printf("mean_DqtR = %.16f\n", mean_DqtR);
                D_Sum[p * MAX_MR * MAX_TIME *  (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind *  (MAX_CLUSTERS + ADD_CLUSTERS) + c] = mean_DptT + mean_DqtR + dm;
                // printf("D_Sum[%d]= %.16f\n", p * MAX_MR * MAX_TIME *  Nc[tind] + q * MAX_TIME * Nc[tind] + tind *  Nc[tind] + c,
                //        D_Sum[p * MAX_MR * MAX_TIME *  Nc[tind] + q * MAX_TIME * Nc[tind] + tind *  Nc[tind] + c]);
                // printf("D_Sum[%d]= %.16f\n", p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + 40) + q * MAX_TIME * (MAX_CLUSTERS + 40) + tind * (MAX_CLUSTERS + 40) + c,
                //        D_Sum[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + 40) + q * MAX_TIME * (MAX_CLUSTERS + 40) + tind * (MAX_CLUSTERS + 40) + c]);
            // xor_temp=Delay_t(p,q,tind,c)~=zeros(size(Delay_t(p,q,tind,c))); % 判断虚拟链路延迟是否为零
            // Delay_t(p,q,tind,c)=Delay_t(p,q,tind,c)+~xor_temp; % 如果延迟为零,则进行调整
                
                //nlos    
                Delay_t[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = 
                         new_Delay[c] + D_Sum[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] / para.c;
                        //  printf("para.c= %d\n", para.c);
                        //printf("new_Delay [%d]= %.16f\n",c, new_Delay[c]);
                // 判断延迟是否为零
                // float xor_temp;
                // xor_temp = (Delay_t[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] != 0);
                // 如果延迟为零,则进行调整
                if (Delay_t[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] == 0) {
                 Delay_t[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = 1;
                }
                //los
                int index_Delay_t_los = p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + 0;
                Delay_t[index_Delay_t_los] = Dpqt / para.c;
                // printf("Delay_t[%d]= %.16f\n", p * MAX_MR * MAX_TIME *  Nc[tind] + q * MAX_TIME * Nc[tind] + tind * Nc[tind] + c,
                //        Delay_t[p * MAX_MR * MAX_TIME *  Nc[tind] + q * MAX_TIME * Nc[tind] + tind *  Nc[tind] + c]);
                }
            }
        }
        //波导簇演进
       int  new_totalRays_duct = 0;
        if (tind == 0) {
                    // 初始化数组 t=0时刻为初始簇
                    for (int c = 0; c < Nc0; c++) {
                        for (int i = 0; i < CLUSTER_RAYS; i++) {
                            new_azimuthT_duct[c * CLUSTER_RAYS + i] = azimuthT_duct[new_totalRays_duct + i];
                            new_elevationT_duct[c * CLUSTER_RAYS + i] = elevationT_duct[new_totalRays_duct + i];
                            new_rT_duct[c * CLUSTER_RAYS + i] = rT_duct[new_totalRays_duct + i];
                            new_azimuthR_duct[c * CLUSTER_RAYS + i] = azimuthR_duct[new_totalRays_duct + i];
                            new_elevationR_duct[c * CLUSTER_RAYS + i] = elevationR_duct[new_totalRays_duct + i];
                            new_rR_duct[c * CLUSTER_RAYS + i] = rR_duct[new_totalRays_duct + i];
                            new_InitPhi_duct[0][c * CLUSTER_RAYS + i] = InitPhi_duct[c * CLUSTER_RAYS + i];
                            //new2_ini_pos_t[c][i] = new_ini_pos_t[new_totalRays + i];
                            //new2_ini_pos_r[c][i] = new_ini_pos_r[new_totalRays + i];
                        }
                        /* totalRaysR += n[c];*/
                        new_totalRays_duct += CLUSTER_RAYS;
                        new_ini_pos_t_duct[c * 3] = t_track[0];
                        new_ini_pos_t_duct[c * 3 + 1] = t_track[1];
                        new_ini_pos_t_duct[c * 3 + 2] = t_track[2];
                        new_ini_pos_r_duct[c * 3] = r_track[0];
                        new_ini_pos_r_duct[c * 3 + 1] = r_track[1];
                        new_ini_pos_r_duct[c * 3 + 2] = r_track[2];
                        new_tau_tilder_duct[c] = tau_tilder_duct[c];
                        new_Delay_duct[c] = Delay_duct[c];
                        // printf("new_Delay[%d] = %.16f\n", c, new_Delay[c]);
                    }
        //            for(int i=0;i<20;i++){
        //            	for(int j=0;j<10;j++){
        //            		printf("new_InitPhi[%d][%d]=%f\n",i,j,new_InitPhi[i*10+j]);
        //            	}
        //            }
                }
                else {
                	for (int clus=0;clus<(MAX_CLUSTERS + ADD_CLUSTERS);clus++){
                		P1_duct[tind][clus]=P1_duct[tind-1][clus];
                		for(int j=0;j<10;j++)
                			new_InitPhi_duct[tind][clus*10+j]=new_InitPhi_duct[tind-1][clus*10+j];
                	}
                       Nc1[tind] = Nc1[tind - 1];
                       if(tind==2){
                    	   for(int i=0;i<25;i++){
                    		   for(int c=0;c<10;c++){
                    			   //printf("new_InitPhi[2][%d][%d]=%f\n",i,c,new_InitPhi[tind][i*10+c]);
                    		   }
                    	   }
                       }
                       // printf("Nc[%d] = %.16f\n", tind, Nc[tind]);
                       int c = 0;
                       //大于判断簇消失
                       while (c < Nc1[tind]) {
                    	   float m=rand_gen_number[rand_gen_count++] ;
        //            	   if(tind==14||tind==15)
        //            	   printf("tind %d m=%f\n",tind,m);
                        if (m > P_R) {
                            for (int i = 0; i < CLUSTER_RAYS; i++) {
                                new_azimuthT_duct[c*CLUSTER_RAYS + i] = 0.0;
                                new_elevationT_duct[c*CLUSTER_RAYS + i] = 0.0;
                                new_rT_duct[c*CLUSTER_RAYS + i] = 0.0;
                                new_azimuthR_duct[c*CLUSTER_RAYS + i] = 0.0;
                                new_elevationR_duct[c*CLUSTER_RAYS + i] = 0.0;
                                new_rR_duct[c*CLUSTER_RAYS + i] = 0.0;
                                new_InitPhi_duct[tind][c*CLUSTER_RAYS + i] = 0.0;
                            }
                            P1_duct[tind][c]=0;
                            new_tau_tilder_duct[c] = 0.0;
                            new_Delay_duct[c] = 1.0;
                            /*      printf("new_Delay[%d] = %.16f\n", c, new_Delay[c]);*/
                        }
                        c++;
                       }
                      //新簇的产生
                       int N_new;
        //               if(tind==14){
        //            	   float test=rand_gen_number[rand_gen_count++];
        //            	   printf("test[%d]=%f\n",tind,test);
        //               }else {
                         N_new = poissrnd(N_new_avg);
        //               }
                         //printf("N_new[%d]=%d\n",tind,N_new);
                        // if (N_new[tind] > 0) {
                        //  Nc[tind] = Nc[tind - 1] + N_new[tind];
                        //  for (int n = 0; n < N_new[tind]; n++) {
                        if (N_new > 0) {
                         Nc1[tind] = Nc1[tind] + N_new;
                         for (int n = 0; n < N_new; n++) {
                             int nn = CLUSTER_RAYS;
                             float D_n_Rt = Exprnd(para.d_Rx_mean) + para.d_Rx_min;
                             float D_n_Tt = Exprnd(para.d_Tx_mean) + para.d_Tx_min;
                            float AAoAt_duct ;
                            float AAoDt_duct ;
                            float EAoAt_duct;
                            float EAoDt_duct;
                            truncated_gaussian(AAoA_variance[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1] / 180 * pi,para.nu_A_R,Theta_LoS[tind]-Theta_duct,Theta_LoS[tind]+Theta_duct,1,&AAoAt_duct);
                            truncated_gaussian(AAoD_variance[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1] / 180 * pi,para.nu_A_R,Theta_LoS[tind]-Theta_duct,Theta_LoS[tind]+Theta_duct,1,&AAoDt_duct);

                            //printf("rand[%d]=%f\n",tind,rand_gen_number[rand_gen_count++]);
                            truncated_gaussian(EAoA_variance[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1] / 180 * pi,para.nu_A_R,-Theta_duct,Theta_duct,1,&EAoAt_duct);

                            //printf("next rand[%d]=%f\n",tind,rand_gen_number[rand_gen_count++]);
        					truncated_gaussian(EAoD_variance[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1] / 180 * pi,para.nu_A_R,-Theta_duct,Theta_duct,1,&EAoDt_duct);
        					//printf("%d,AAoDt=%.6f\n",tind,EAoDt_duct);

                            // float D_n_Rt = D_n_Rt_exp[dnrnum] + para.d_Rx_min;
                            // float D_n_Tt = D_n_Tt_exp [dntnum]+ para.d_Tx_min;
                            // dnrnum  = dnrnum + 1;
                            //  dntnum  = dntnum + 1;

                           // printf("D_n_Tt=%.6f\n",D_n_Tt);
                            float Tsigmaxt_duct = NorDis3(para.Tsigmax, para.sigmaxvar);
                            float Tsigmayt_duct = NorDis3(para.Tsigmay, para.sigmayvar);
                            float Tsigmazt_duct = NorDis3(para.Tsigmaz, para.sigmazvar);
                            float Rsigmaxt_duct = NorDis3(para.Rsigmax, para.sigmaxvar);
                            float Rsigmayt_duct = NorDis3(para.Rsigmay, para.sigmayvar);
                            float Rsigmazt_duct = NorDis3(para.Rsigmaz, para.sigmazvar);
//                            if(tind==2)
//                            printf("Rsigmazt = %.16f\n",Rsigmazt_duct);
                            float new_T_a1_duct[CLUSTER_RAYS] = {0};
                            float new_T_e1_duct[CLUSTER_RAYS] = {0};
                            float new_T_r1_duct[CLUSTER_RAYS] = {0};
                            float new_R_a1_duct[CLUSTER_RAYS], new_R_e1_duct[CLUSTER_RAYS], new_R_r1_duct[CLUSTER_RAYS];
                            DiscrAngleLenEllipsoid(0,CLUSTER_RAYS, D_n_Tt, AAoDt_duct, EAoDt_duct, Tsigmaxt_duct, Tsigmayt_duct, Tsigmazt_duct
                                    ,new_T_a1_duct, new_T_e1_duct, new_T_r1_duct);
                            DiscrAngleLenEllipsoid(0,CLUSTER_RAYS, D_n_Rt, AAoAt_duct, EAoAt_duct, Rsigmaxt_duct, Rsigmayt_duct, Rsigmazt_duct
                                    ,new_R_a1_duct, new_R_e1_duct, new_R_r1_duct);
                            new_ini_pos_t_duct[(Nc1[tind - 1] + n) * 3] = t_track[tind * 3];
                            new_ini_pos_t_duct[(Nc1[tind - 1] + n) * 3 + 1] = t_track[tind * 3 + 1];
                            new_ini_pos_t_duct[(Nc1[tind - 1] + n) * 3 + 2] = t_track[tind * 3 + 2];
                            new_ini_pos_r_duct[(Nc1[tind - 1] + n) * 3] = r_track[tind * 3];
                            new_ini_pos_r_duct[(Nc1[tind - 1] + n) * 3 + 1] = r_track[tind * 3 + 1];
                            new_ini_pos_r_duct[(Nc1[tind - 1] + n) * 3 + 2] = r_track[tind * 3 + 2];

                            // 将结果追加到数组中 给新簇信息
                            for (int i = 0; i < nn; i++) {
                                new_azimuthT_duct[(Nc1[tind - 1] + n) * nn + i] = new_T_a1_duct[i];
                                new_elevationT_duct[(Nc1[tind - 1] + n) * nn + i] = new_T_e1_duct[i];
                                new_rT_duct[(Nc1[tind - 1] + n) * nn + i] = new_T_r1_duct[i];

                                new_azimuthR_duct[(Nc1[tind - 1] + n) * nn + i] = new_R_a1_duct[i];
                                new_elevationR_duct[(Nc1[tind - 1] + n) * nn + i] = new_R_e1_duct[i];
                                new_rR_duct[(Nc1[tind - 1] + n) * nn + i] = new_R_r1_duct[i];
//                                if(tind==2){
//                                  printf("%d,new_azimuthT[%d][%d]=%.6f\n",tind,Nc1[tind - 1] + n,i,new_azimuthT_duct[(Nc1[tind - 1] + n) * nn + i]);
//                                  printf("%d,new_elevationT[%d][%d]=%.6f\n",tind,Nc1[tind - 1] + n,i,new_elevationT_duct[(Nc1[tind - 1] + n) * nn + i]);
//                                  printf("%d,new_rT[%d][%d]=%.6f\n",tind,Nc1[tind - 1] + n,i,new_rT_duct[(Nc1[tind - 1] + n) * nn + i]);
//                                }
                            }

        //                    float rand_delay = rand_gen_number[rand_gen_count++];
                            cnt1++;

                            P1_duct[tind][cnt1]=stage4_pow10_model(-randn_gen_number[randn_gen_count++]*0.1f);

                            // printf("rand_delay = %.16f\n", rand_delay);
                            new_Delay_duct[Nc1[tind - 1] + n] = -para.DS_r * stage4_pow10_model(DS[(yy_t-1)*para.square[1]*para.altitudes + (xx_t-1)*para.altitudes + zz_t - 1]) * logf(rand_gen_number[rand_gen_count++]);//[num_rand_delay]
                            // num_rand_delay = num_rand_delay + 1;
                            for (int i = 0; i < nn; i++) {
                                new_InitPhi_duct[tind][(Nc1[tind - 1] + n) * nn + i] = (rand_gen_number[rand_gen_count++]) * 2 * pi;
                            }
                            //new_tau_tilder[Nc[tind - 1] + n] = 1;
                            new_tau_tilder_duct[Nc1[tind - 1] + n] = Exprnd(para.tau_tilder_mean) + para.tau_tilder_min;
                            //printf("tind %d tau=%f\n",tind,rand_gen_number[rand_gen_count++]);
                         }
                      }
                }


                float DptT_duct[CLUSTER_RAYS];
                float DqtR_duct[CLUSTER_RAYS];
                //float Dpqt[MAX_RAYS];


                //float T_pos_t[MAX_RAYS][3];
                //float R_pos_t[MAX_RAYS][3];
                float T_pos_t_duct[3];
                float R_pos_t_duct[3];

                //printf("Nc[%d] = %d\n", tind, Nc[tind]);

                for (int c = 0; c < Nc1[tind]; c++) {//Nc[tind]MAX_CLUSTERS + 40
                    for (int p = 0; p < MAX_MT; p++) {
                        float deltap = p * para.delta_tx * para.lambda;
                        //printf("deltap = %.16f\n", deltap);
                        for (int q = 0; q < MAX_MR; q++) {
                            float deltaq = q * para.delta_rx * para.lambda;

                            T_pos_t_duct[0] = t_track[tind * 3];
                            T_pos_t_duct[1] = t_track[tind * 3 + 1];
                            T_pos_t_duct[2] = t_track[tind * 3 + 2];
                            R_pos_t_duct[0] = r_track[tind * 3];
                            R_pos_t_duct[1] = r_track[tind * 3 + 1];
                            R_pos_t_duct[2] = r_track[tind * 3 + 2];

                            float new2_ini_pos_t_duct[3];
                            float new2_ini_pos_r_duct[3];
                            new2_ini_pos_t_duct[0] = new_ini_pos_t_duct[c * 3];
                            new2_ini_pos_t_duct[1] = new_ini_pos_t_duct[c * 3 + 1];
                            new2_ini_pos_t_duct[2] = new_ini_pos_t_duct[c * 3 + 2];
                            new2_ini_pos_r_duct[0] = new_ini_pos_r_duct[c * 3];
                            new2_ini_pos_r_duct[1] = new_ini_pos_r_duct[c * 3 + 1];
                            new2_ini_pos_r_duct[2] = new_ini_pos_r_duct[c * 3 + 2];
                            //printf("tind = %d\n", tind);
                            //printf("Nc[%d] = %d\n", tind,Nc[tind]);
                            // 打印 T_pos_t 的所有元素
                            // printf("T_pos_t[0] = %.16f, T_pos_t[1] = %.16f, T_pos_t[2] = %.16f\n", T_pos_t[0], T_pos_t[1], T_pos_t[2]);
                            // // 打印 R_pos_t 的所有元素
                            // printf("R_pos_t[0] = %.16f, R_pos_t[1] = %.16f, R_pos_t[2] = %.16f\n", R_pos_t[0], R_pos_t[1], R_pos_t[2]);
                            // // 打印 new2_ini_pos_t 的所有元素
                            // printf("new2_ini_pos_t[0] = %.16f, new2_ini_pos_t[1] = %.16f, new2_ini_pos_t[2] = %.16f\n", new2_ini_pos_t[0], new2_ini_pos_t[1], new2_ini_pos_t[2]);
                            // // 打印 R_pos_t 的所有元素
                            // printf("new2_ini_pos_r[0] = %.16f, new2_ini_pos_r[1] = %.16f, new2_ini_pos_r[2] = %.16f\n", new2_ini_pos_r[0], new2_ini_pos_r[1], new2_ini_pos_r[2]);
                            float new2_azimuthR_duct[CLUSTER_RAYS];
                            float new2_elevationR_duct[CLUSTER_RAYS];
                            float new2_rR_duct[CLUSTER_RAYS];
                            float new2_azimuthT_duct[CLUSTER_RAYS];
                            float new2_elevationT_duct[CLUSTER_RAYS];
                            float new2_rT_duct[CLUSTER_RAYS];

                            for (int i = 0; i < CLUSTER_RAYS; i++) {
                                new2_azimuthR_duct[i] = new_azimuthR_duct[c * CLUSTER_RAYS + i];
                                new2_elevationR_duct[i] = new_elevationR_duct[c * CLUSTER_RAYS + i];
                                new2_rR_duct[i] = new_rR_duct[c * CLUSTER_RAYS + i];
                                new2_azimuthT_duct[i] = new_azimuthT_duct[c * CLUSTER_RAYS + i];
                                new2_elevationT_duct[i] = new_elevationT_duct[c * CLUSTER_RAYS + i];
                                new2_rT_duct[i] = new_rT_duct[c * CLUSTER_RAYS + i];
                                if(1<tind&&tind<3){
//                                printf("%d new2_azimuthR[%d] = %.16f\n", c,i,new2_azimuthR[i]);
//                                printf("%d new2_elevationR[%d] = %.16f\n",tind,i,new2_elevationR[i]);
//                                printf("%d,new2_rR[%d] = %.16f\n", tind,i,new2_rR[i]);
//                                printf("%d new2_azimuthT[%d] = %.16f\n", c,i,new2_azimuthT_duct[i]);
//                                printf("%d new2_elevationT[%d] = %.16f\n",c, i,new2_elevationT_duct[i]);
//                                printf("%d,new2_rT[%d] = %.16f\n",c, i,new2_rT_duct[i]);
                                }
                            }

                            TravelLength3DSim(new2_ini_pos_t_duct, T_pos_t_duct, para.nu_E_T, para.nu_A_T, deltap, new2_rT_duct, new2_elevationT_duct, new2_azimuthT_duct, DptT_duct, 0, 0, 0, 0);
                            TravelLength3DSim(new2_ini_pos_r_duct, R_pos_t_duct, para.nu_E_R, para.nu_A_R, deltaq, new2_rR_duct, new2_elevationR_duct, new2_azimuthR_duct, DqtR_duct, 0, 0, 0, 0);
                            if(tind==2){
                            //printf("new_ini_pos_t[%d][0] = %.6f, new_ini_pos_t[%d][1] = %.6f, new_ini_pos_t[%d][2] = %.6f\n",c, new2_ini_pos_t_duct[0], c, new2_ini_pos_t_duct[1], c, new2_ini_pos_t_duct[2]);
        //
        //                    printf("new_ini_pos_r[%d][0] = %.6f, new_ini_pos_r[%d][1] = %.6f, new_ini_pos_r[%d][2] = %.6f\n", c, new_ini_pos_r[c][0], c, new_ini_pos_r[c][1], c, new_ini_pos_r[c][2]);

                        }
                            /////////已验证
//                            if(tind==2){
//        						 for (int i = 0; i <10; i++) {
//        							printf("%d,DptT[%d] = %.16f\n",c, i,DptT_duct[i]);
//        						  printf("DqtR[%d] = %.16f\n", i,DqtR_duct[i]);
//        						 }
//                            }
                            TravelLengthLoS(T_pos_t_duct, R_pos_t_duct, para.nu_E_T, para.nu_A_T, deltap, para.nu_E_R, para.nu_A_R, deltaq, &Dpqt_duct);
                            Dlos_duct[tind]=Dpqt_duct;
                            // // 打印 T_pos_t 的所有元素
                            //printf("T_pos_t[0] = %.16f, T_pos_t[1] = %.16f, T_pos_t[2] = %.16f\n", T_pos_t[0], T_pos_t[1], T_pos_t[2]);
                            //// 打印 R_pos_t 的所有元素
                            // printf("R_pos_t[0] = %.16f, R_pos_t[1] = %.16f, R_pos_t[2] = %.16f\n", R_pos_t[0], R_pos_t[1], R_pos_t[2]);
        //                    if(tind==3)
        //                         printf("直线距离 Dpqt: %f\n", Dpqt);
                            if(1<tind&&tind<3){
                            	for(int i=0;i<10;i++){
        //							printf("%d new2_azimuthR[%d] = %.16f\n", tind,i,new2_azimuthR[i]);
        //							printf("%d new2_elevationR[%d] = %.16f\n",tind,i,new2_elevationR[i]);
        //							printf("%d,new2_rR[%d] = %.16f\n", tind,i,new2_rR[i]);
        //                    		printf("%d new2_azimuthT[%d] = %.16f\n", tind,i,new2_azimuthT[i]);
        //							printf("%d new2_elevationT[%d] = %.16f\n",tind, i,new2_elevationT[i]);
        //							printf("%d,new2_rT[%d] = %.16f\n",tind, i,new2_rT[i]);
                            	}
        //                        printf("%d new2_ini_pos_t[0] = %.6f, new2_ini_pos_t[1] = %.6f, new2_ini_pos_t[2] = %.6f\n", tind,new2_ini_pos_t[0], new2_ini_pos_t[1], new2_ini_pos_t[2]);
        //                        printf("%d new2_ini_pos_r[0] = %.6f, new2_ini_pos_r[1] = %.6f, new2_ini_pos_r[2] = %.6f\n", tind,new2_ini_pos_r[0], new2_ini_pos_r[1], new2_ini_pos_r[2]);
                            }
                            float dm_duct = 0.0;
                            LengthCACZ(tind,new2_ini_pos_t_duct[0], new2_ini_pos_t_duct[1], new2_ini_pos_t_duct[2], new2_ini_pos_r_duct[0], new2_ini_pos_r_duct[1], new2_ini_pos_r_duct[2],
                                new2_elevationT_duct, new2_azimuthT_duct, new2_rT_duct, new2_elevationR_duct, new2_azimuthR_duct, new2_rR_duct, &dm_duct);
                            if(tind==2){
                                //printf("%d,dm = %.16f\n",tind, dm);
                            }

                        // float mean_DptT;
                        // float mean_DqtR;
                        calculateMean(DptT_duct, CLUSTER_RAYS,&mean_DptT);
                        calculateMean(DqtR_duct, CLUSTER_RAYS,&mean_DqtR);
                        // printf("mean_DptT = %.16f\n", mean_DptT);
                        // printf("mean_DqtR = %.16f\n", mean_DqtR);
                        D_Sum_duct[p * MAX_MR * MAX_TIME *  (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind *  (MAX_CLUSTERS + ADD_CLUSTERS) + c] = mean_DptT + mean_DqtR + dm_duct;
                        // printf("D_Sum[%d]= %.16f\n", p * MAX_MR * MAX_TIME *  Nc[tind] + q * MAX_TIME * Nc[tind] + tind *  Nc[tind] + c,
                        //        D_Sum[p * MAX_MR * MAX_TIME *  Nc[tind] + q * MAX_TIME * Nc[tind] + tind *  Nc[tind] + c]);
                        // printf("D_Sum[%d]= %.16f\n", p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + 40) + q * MAX_TIME * (MAX_CLUSTERS + 40) + tind * (MAX_CLUSTERS + 40) + c,
                        //        D_Sum[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + 40) + q * MAX_TIME * (MAX_CLUSTERS + 40) + tind * (MAX_CLUSTERS + 40) + c]);
                    // xor_temp=Delay_t(p,q,tind,c)~=zeros(size(Delay_t(p,q,tind,c))); % 判断虚拟链路延迟是否为零
                    // Delay_t(p,q,tind,c)=Delay_t(p,q,tind,c)+~xor_temp; % 如果延迟为零,则进行调整

                        //nlos
                        Delay_t_duct[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] =
                                 new_Delay_duct[c] + D_Sum_duct[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] / para.c;
                                //  printf("para.c= %d\n", para.c);
                                //printf("new_Delay [%d]= %.16f\n",c, new_Delay[c]);
                        // 判断延迟是否为零
                        // float xor_temp;
                        // xor_temp = (Delay_t[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] != 0);
                        // 如果延迟为零,则进行调整
                        if (Delay_t_duct[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] == 0) {
                         Delay_t_duct[p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = 1;
                        }
                        //los
                        int index_Delay_t_los_duct = p * MAX_MR * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + q * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + 0;
                        Delay_t_duct[index_Delay_t_los_duct] = Dpqt_duct / para.c;
                        }
                    }
                }
                        // printf("Delay_t[%d]= %.16f\n", p * MAX_MR * MAX_TIME *  Nc[tind] + q * MAX_TIME * Nc[tind] + tind * Nc[tind] + c,
                        //        Delay_t[p * MAX_MR * MAX_TIME *  Nc[tind] + q * MAX_TIME * Nc[tind] + tind *  Nc[tind] + c]);
    }

    //初始化Delay_t_permuted
for (int q = 0; q < MAX_MR; q++) {
    for (int p = 0; p < MAX_MT; p++) {
        for (int tind = 0; tind < MAX_TIME; tind++) {
            for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
                Delay_t_permuted[q * MAX_MT * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + p * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = 1.0;
            }
        }
    }
}
    for (int q = 0; q < MAX_MR; q++) {
        for (int p = 0; p < MAX_MT; p++) {
            for (int tind = 0; tind < MAX_TIME; tind++) {
                for (int c = 0; c < MAX_CLUSTERS + ADD_CLUSTERS; c++) {
                    Delay_t_permuted[q * MAX_MT * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + p * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] 
                            = Delay_t[q * MAX_MT * MAX_TIME *(MAX_CLUSTERS + ADD_CLUSTERS) + p * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c];

                }
            }
        }
    }

int time_length = para.T_length; // 时间维度的长度


float *d_t = d_t_storage;
            for (int tind = 0; tind < t_num_points; tind++) {
                for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
                    d_t[tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = Delay_t_permuted[0 * MAX_MT * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + 0 * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) 
                             + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c];
    }
}

// 初始化 d_time
int d_time_rows = t_num_points;
int d_time_cols = (MAX_CLUSTERS + ADD_CLUSTERS);
float *d_time = d_time_storage;
for (int i = 0; i < d_time_rows * d_time_cols; i++) {
    d_time[i] = 1.0;
}
// 填充 d_time
for (int i = 0; i < d_time_rows; i++) {
    int v = 0; // 当前行d_time的有效元素计数器
    for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
        // 检查d_t当前元素是否小于1.0
        if (d_t[i * (MAX_CLUSTERS + ADD_CLUSTERS) + c] < 1.0) {
            if (v < d_time_cols) { // 确保不越界
                d_time[i * d_time_cols + v] = d_t[i * (MAX_CLUSTERS + ADD_CLUSTERS) + c];
                 //printf("d_time[%d] = %.16f\n", i * d_time_cols + v, d_time[i * d_time_cols + v]);
                v++; // 计数器递增
            }
        }
        // 若d_t元素≥1.0,直接跳过,v不增加
    }
}

//波导簇延时数据
for (int q = 0; q < MAX_MR; q++) {
    for (int p = 0; p < MAX_MT; p++) {
        for (int tind = 0; tind < MAX_TIME; tind++) {
            for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
                Delay_t_permuted_duct[q * MAX_MT * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + p * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = 1.0;
            }
        }
    }
}
    for (int q = 0; q < MAX_MR; q++) {
        for (int p = 0; p < MAX_MT; p++) {
            for (int tind = 0; tind < MAX_TIME; tind++) {
                for (int c = 0; c < MAX_CLUSTERS + ADD_CLUSTERS; c++) {
                    Delay_t_permuted_duct[q * MAX_MT * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + p * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c]
                            = Delay_t_duct[q * MAX_MT * MAX_TIME *(MAX_CLUSTERS + ADD_CLUSTERS) + p * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c];

                }
            }
        }
    }


float *d_t_duct = d_t_duct_storage;
            for (int tind = 0; tind < t_num_points; tind++) {
                for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
                    d_t_duct[tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c] = Delay_t_permuted_duct[0 * MAX_MT * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS) + 0 * MAX_TIME * (MAX_CLUSTERS + ADD_CLUSTERS)
                             + tind * (MAX_CLUSTERS + ADD_CLUSTERS) + c];
    }
}


float *d_time_duct = d_time_duct_storage;
for (int i = 0; i < d_time_rows * d_time_cols; i++) {
    d_time_duct[i] = 1.0;
}
// 填充 d_time
for (int i = 0; i < d_time_rows; i++) {
    int v = 0; // 当前行d_time的有效元素计数器
    for (int c = 0; c < (MAX_CLUSTERS + ADD_CLUSTERS); c++) {
        // 检查d_t当前元素是否小于1.0
        if (d_t_duct[i * (MAX_CLUSTERS + ADD_CLUSTERS) + c] < 1.0) {
            if (v < d_time_cols) { // 确保不越界
                d_time_duct[i * d_time_cols + v] = d_t_duct[i * (MAX_CLUSTERS + ADD_CLUSTERS) + c];
                 //printf("d_time[%d] = %.16f\n", i * d_time_cols + v, d_time[i * d_time_cols + v]);
                v++; // 计数器递增
            }
        }
        // 若d_t元素≥1.0,直接跳过,v不增加
    }
}




uint32_t stage5_start = read_cycle32();
for (int tind=0;tind<t_num_points-1;tind++){
	int xx_tt = 0, yy_tt = 0, zz_tt = 0;
	float h_tt_track_tind[3] = { t_track[3*tind], t_track[3*tind + 1], t_track[3*tind + 2] };
	xx_tt = ceil(h_tt_track_tind[0] / para.D_g);
	yy_tt = ceil(h_tt_track_tind[1] / para.D_g);
	zz_tt = ceil(h_tt_track_tind[2] / para.delet_altitude);
    for (int i=0; i<(MAX_CLUSTERS + ADD_CLUSTERS);i++){
    	if(i==0){
    		Pow_FPGA[tind][i]=1;
    		Pow_FPGA_duct[tind][i]=1;
    	}
    	else{
    	    Pow_FPGA[tind][i]=exp(-d_time[tind*(MAX_CLUSTERS + ADD_CLUSTERS)+i]*(para.DS_r-1)/(para.DS_r)/pow(10,DS[(yy_tt-1)*200*20+(xx_tt-1)*20+zz_tt-1]));
    	    Pow_FPGA_duct[tind][i]=exp(-d_time_duct[tind*(MAX_CLUSTERS + ADD_CLUSTERS)+i]*(para.DS_r-1)/(para.DS_r)/pow(10,DS[(yy_tt-1)*200*20+(xx_tt-1)*20+zz_tt-1]));
    	}
    }
}//计算路径衰减
printf("[5/7] 时变簇与路径功率计算完成 cycles=%u\n", (unsigned)(read_cycle32() - stage5_start));
uint32_t stage6_start = read_cycle32();
uint32_t stage6_step = stage6_start;
printf("[6A] 普通簇信道响应开始\n");
for (int time_idx = 0; time_idx < t_num_points-1; time_idx++) {
	for (int clear_idx = 0; clear_idx < TOTAL_CLUSTERS; ++clear_idx) {
		H_FPGA[time_idx][clear_idx].real = 0.0f;
		H_FPGA[time_idx][clear_idx].image = 0.0f;
	}
	cto = 0;
	for(int jo=0;jo<Nc[time_idx];jo++)
	{
		if(jo==0)
		{
			cto=cto+1;
		}
		else if(new_InitPhi[time_idx][jo*10]==0){
		}
		else {

			for (int cnt_array=0;cnt_array<10;cnt_array++)
			{
				iniphi[cto][cnt_array]=new_InitPhi[time_idx][jo*10+cnt_array];
			}
			cto++;
		}//把零散的相位变为挨着的，下面同
	}
	pto=0;

	for (int po=0;po<Nc[time_idx];po++)
	{
		if(po==0)
		{
			pto=pto+1;
		}
		else if(P1[time_idx][po]==0){
		}
		else {
			p1[pto]=P1[time_idx][po];
			pto=pto+1;
		}
	}
    int active_clusters = cto;
    if (active_clusters < 1) {
        active_clusters = 1;
    }
    if (active_clusters > TOTAL_CLUSTERS) {
        active_clusters = TOTAL_CLUSTERS;
    }
    active_cluster_count[time_idx] = active_clusters;
    float phase_angles[64];
    for (int c = 0; c < active_clusters; c += 6) {
        int batch_clusters = active_clusters - c;
        if (batch_clusters > 6) {
            batch_clusters = 6;
        }
        for (int bc = 0; bc < batch_clusters; ++bc) {
            int cluster = c + bc;
            float tau = d_time[time_idx * TOTAL_CLUSTERS + cluster];
            float theta = tau * (2.0f * (float)M_PI * para.fc);
            for(int i=0;i<CLUSTER_RAYS;i++){
                phase_angles[bc * CLUSTER_RAYS + i] = iniphi[cluster][i] + theta;
            }
        }
        compute_cluster_responses_batched(phase_angles, batch_clusters, &H_FPGA[time_idx][c]);
        for (int bc = 0; bc < batch_clusters; ++bc) {
            int cluster = c + bc;
            float amp = sqrtf(Pow_FPGA[time_idx][cluster] * p1[cluster]);
            H_FPGA[time_idx][cluster].real *= amp;
            H_FPGA[time_idx][cluster].image *= amp;
        }
    }

    if (time_idx == 0 || ((time_idx + 1) % 16) == 0 || time_idx == (t_num_points - 2)) {
        printf("[6A] 普通簇 time=%d/%d active=%d hw=%d sw=%d\n",
               time_idx + 1, t_num_points - 1, active_clusters, g_hw_used, g_sw_fallback);
    }



}
//波导簇H计算
printf("[6A] 普通簇信道响应完成 cycles=%u\n", (unsigned)(read_cycle32() - stage6_step));
stage6_step = read_cycle32();
printf("[6B] 波导簇信道响应开始\n");
for (int time_idx = 0; time_idx < t_num_points-1; time_idx++) {
	for (int clear_idx = 0; clear_idx < TOTAL_CLUSTERS; ++clear_idx) {
		H_FPGA_duct[time_idx][clear_idx].real = 0.0f;
		H_FPGA_duct[time_idx][clear_idx].image = 0.0f;
	}
	cto_duct = 0;
	for(int jo=0;jo<Nc1[time_idx];jo++)
	{
		if(jo==0)
		{
			cto_duct=cto_duct+1;
		}
		else if(new_InitPhi_duct[time_idx][jo*10]==0){
		}
		else {

			for (int cnt_array=0;cnt_array<10;cnt_array++)
			{
				iniphi_duct[cto_duct][cnt_array]=new_InitPhi_duct[time_idx][jo*10+cnt_array];
			}
			cto_duct++;
		}//把零散的相位变为挨着的，下面同
	}
	pto_duct=0;

	for (int po=0;po<Nc1[time_idx];po++)
	{
		if(po==0)
		{

			pto_duct=pto_duct+1;
		}
		else if(P1_duct[time_idx][po]==0){
		}
		else {
			p1_duct[pto_duct]=P1_duct[time_idx][po];
			pto_duct=pto_duct+1;
		}
	}
    int active_clusters_duct = cto_duct;
    if (active_clusters_duct < 1) {
        active_clusters_duct = 1;
    }
    if (active_clusters_duct > TOTAL_CLUSTERS) {
        active_clusters_duct = TOTAL_CLUSTERS;
    }
    active_cluster_count_duct[time_idx] = active_clusters_duct;
    float phase_angles[64];
    for (int c = 0; c < active_clusters_duct; c += 6) {
        int batch_clusters = active_clusters_duct - c;
        if (batch_clusters > 6) {
            batch_clusters = 6;
        }
        for (int bc = 0; bc < batch_clusters; ++bc) {
            int cluster = c + bc;
            float tau = d_time_duct[time_idx * TOTAL_CLUSTERS + cluster];
            float theta = tau * (2.0f * (float)M_PI * para.fc);
            for(int i=0;i<CLUSTER_RAYS;i++){
                phase_angles[bc * CLUSTER_RAYS + i] = iniphi_duct[cluster][i] + theta;
            }
        }
        compute_cluster_responses_batched(phase_angles, batch_clusters, &H_FPGA_duct[time_idx][c]);
        for (int bc = 0; bc < batch_clusters; ++bc) {
            int cluster = c + bc;
            float amp = sqrtf(Pow_FPGA_duct[time_idx][cluster] * p1_duct[cluster]);
            H_FPGA_duct[time_idx][cluster].real *= amp;
            H_FPGA_duct[time_idx][cluster].image *= amp;
        }
    }

    if (time_idx == 0 || ((time_idx + 1) % 16) == 0 || time_idx == (t_num_points - 2)) {
        printf("[6B] 波导簇 time=%d/%d active=%d hw=%d sw=%d\n",
               time_idx + 1, t_num_points - 1, active_clusters_duct, g_hw_used, g_sw_fallback);
    }
}//波导簇H计算

printf("[6B] 波导簇信道响应完成 cycles=%u\n", (unsigned)(read_cycle32() - stage6_step));
stage6_step = read_cycle32();
printf("[6C] 归一化/LoS合并开始\n");
for(int tind = 0; tind < t_num_points;tind++) {
    	float mean_h=0;
    	float mean_h_duct=0;
    	int x_t = 0, y_t = 0, z_t = 0;
    	float h_t_track_tind[3] = { t_track[3*tind], t_track[3*tind + 1], t_track[3*tind + 2] };
    	x_t = ceil(h_t_track_tind[0] / para.D_g);
    	y_t = ceil(h_t_track_tind[1] / para.D_g);
    	z_t = ceil(h_t_track_tind[2] / para.delet_altitude);
        float K = K_ricean_lin[(y_t-1)*200*20+ (x_t-1)*20+ (z_t-1)];
        const int normal_active = active_cluster_count[tind];
        const int duct_active = active_cluster_count_duct[tind];
        for (int j=1 ;j<normal_active;j++){
        	mean_h +=complex_mod(H_FPGA[tind][j]);
        }
        for (int j=1 ;j<duct_active;j++){
        	mean_h_duct +=complex_mod(H_FPGA_duct[tind][j]);
        }
        //printf("mean_h=%f\n",sqrt(mean_h));
//        printf("sqrt(1/(1+K))=%f\n",sqrt(1/(1+K)));
        //printf("K=%f\n",K);
        const float k_scale = sqrtf(1.0f / (1.0f + K));
        const float normal_scale = (mean_h > 0.0f) ? (k_scale / sqrtf(mean_h)) : 0.0f;
        const float duct_scale = (mean_h_duct > 0.0f) ? (k_scale / sqrtf(mean_h_duct)) : 0.0f;
        for (int c=1;c<normal_active;c++){
        	H_FPGA[tind][c]=complex_mul_x(H_FPGA[tind][c], normal_scale);
        }
        for (int c=1;c<duct_active;c++){
        	H_FPGA_duct[tind][c]=complex_mul_x(H_FPGA_duct[tind][c], duct_scale);
        }
    }


    float los_phase[MAX_TIME];
    float los_sin[MAX_TIME];
    float los_cos[MAX_TIME];
    for (int tind = 0; tind < t_num_points; tind++) {
        los_phase[tind] = 2.0f * (float)pi * para.fc * (Dlos[tind] / para.c);
    }
    cordic_batch_sin_cos(los_phase, t_num_points, los_sin, los_cos);

    for (int tind = 0; tind < t_num_points;tind++) {
    		complex Hlos;
        	int x_t = 0, y_t = 0, z_t = 0;
        	float h_t_track_tind[3] = { t_track[3*tind], t_track[3*tind + 1], t_track[3*tind + 2] };
        	x_t = ceil(h_t_track_tind[0] / para.D_g);
        	y_t = ceil(h_t_track_tind[1] / para.D_g);
        	z_t = ceil(h_t_track_tind[2] / para.delet_altitude);
        	Hlos.real = los_cos[tind];
        	Hlos.image = los_sin[tind];
            float K = K_ricean_lin[(y_t-1)*200*20+ (x_t-1)*20+ (z_t-1)];
            H_FPGA[tind][0]=complex_mul_x(Hlos,sqrtf(K/(1.0f+K)));//这里是直射路径的幅值
           // printf("H_los[%d]=%f\n",tind,cos(2*pi*para.fc*(Dlos[tind]/para.c)));
    }
printf("[6C] 归一化/LoS合并完成 cycles=%u\n", (unsigned)(read_cycle32() - stage6_step));
printf("[6/7] 信道响应计算完成 total_cycles=%u (hw=%d sw=%d mm2s_to=%d s2mm_to=%d)\n",
       (unsigned)(read_cycle32() - stage6_start), g_hw_used, g_sw_fallback,
       g_dma_timeout_mm2s, g_dma_timeout_s2mm);
uint32_t stage7_prep_start = read_cycle32();
float (*D)[TOTAL_CLUSTERS] = D_output_storage;
for (int t=0;t<t_num_points;t++){
	for (int c=0;c<220;c++){
		if(c<39){
			H[t][c]=H_FPGA[t][c];
			D[t][c]=d_time [t*(MAX_CLUSTERS + ADD_CLUSTERS)+c];
		}
		else
		{
			H[t][c]=H_FPGA_duct[t][c-39];
			D[t][c]=d_time_duct [t*(MAX_CLUSTERS + ADD_CLUSTERS)+c-39];
		}

	}
}
    char cmd;
    printf("[7A] H/D整理完成 cycles=%u\n", (unsigned)(read_cycle32() - stage7_prep_start));
    printf("[7/7] 结果已就绪，发送字符 '1' 输出详细数据\n");
    while (1) {
        //为了方便控制接收，所以我这里弄了个接收字符来控制打印
        cmd = getchar();  // 读取一个字符
        if (cmd == '1') {

            printf("H_real\n");
            for (int i = 0; i < t_num_points; i++) {
                for (int j = 0; j <num_col; j++) {
                    printf("H_FPGA[%d][%d]=%f\n", i, j, H[i][j].real);
                }
            }


            printf("H_image\n");
            for (int i = 0; i < t_num_points; i++) {
                for (int j = 0; j <num_col; j++) {
                    printf("H_FPGA[%d][%d]=%f\n", i, j, H[i][j].image);
                }
            }


            printf("delay\n");
            for (int i = 0; i < t_num_points; i++) {
                for (int j = 0; j <num_col; j++) {
//                    printf("Delay_t_permuted[%d][%d]=%.12f\n", i, j,D[i*(MAX_CLUSTERS + ADD_CLUSTERS)+j]);
                	printf("Delay_t_permuted[%d][%d]=%.12f\n", i, j,D[i][j]);
                }
            }

            fflush(stdout);
        } else if (cmd == '3') {
            run_sw_channel_convolution_demo(H, D, t_num_points, num_col, 2.0f * (float)fc);
        } else if (cmd == '4') {
            run_pl_channel_convolution_demo(H, D, t_num_points, num_col, 2.0f * (float)fc);
        } else if (cmd == '5') {
            pl_channel_convolution_selftest();
        } else if (cmd == '6') {
            run_accel_benchmark(H, D, t_num_points, num_col, 2.0f * (float)fc);
        }
    }


       printf("传输完成。\n");

    return 0;
}
