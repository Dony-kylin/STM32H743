#include "G_Export_V3.h"
#include "rtwtypes.h"
#include "G_Export_V3_private.h"
#include <math.h>
#include <string.h>

#define G_Export_V3_Fs                 (8.0E+6)
#define G_Export_V3_NFFT               (16384.0)

#if defined(__GNUC__)
#define TASK0729_AXI_RAM \
  __attribute__((section(".task0729_ram"), aligned(32)))
#else
#define TASK0729_AXI_RAM
#endif

B_G_Export_V3_T G_Export_V3_B TASK0729_AXI_RAM;
DW_G_Export_V3_T G_Export_V3_DW;
ExtU_G_Export_V3_T G_Export_V3_U TASK0729_AXI_RAM;
ExtY_G_Export_V3_T G_Export_V3_Y;
static RT_MODEL_G_Export_V3_T G_Export_V3_M_;
RT_MODEL_G_Export_V3_T *const G_Export_V3_M = &G_Export_V3_M_;
static real32_T G_Export_V3_peak_vertex(const real32_T x[16384], real32_T dc,
  real_T k, real_T N, boolean_T isMaximum);
void MWDSPCG_FFT_Interleave_R2BR_S(const real32_T x[], creal32_T y[], int32_T
  nChans, int32_T nRows)
{
  int32_T bit_fftLen;
  int32_T br_j;
  int32_T j;
  int32_T nChansBy2;
  int32_T uIdx;
  int32_T yIdx;
  br_j = 0;
  yIdx = 0;
  uIdx = 0;
  for (nChansBy2 = nChans >> 1; nChansBy2 != 0; nChansBy2--) {
    for (j = nRows; j - 1 > 0; j--) {
      bit_fftLen = yIdx + br_j;
      y[bit_fftLen].re = x[uIdx];
      y[bit_fftLen].im = x[uIdx + nRows];
      uIdx++;
      bit_fftLen = nRows;
      do {
        bit_fftLen = (int32_T)((uint32_T)bit_fftLen >> 1);
        br_j ^= bit_fftLen;
      } while ((int32_T)((uint32_T)br_j & (uint32_T)bit_fftLen) == 0);
    }

    bit_fftLen = yIdx + br_j;
    y[bit_fftLen].re = x[uIdx];
    br_j = uIdx + nRows;
    y[bit_fftLen].im = x[br_j];
    uIdx = br_j + 1;
    yIdx += nRows << 1;
    br_j = 0;
  }

  if (((uint32_T)nChans & 1U) != 0U) {
    for (j = nRows >> 1; j - 1 > 0; j--) {
      bit_fftLen = yIdx + br_j;
      y[bit_fftLen].re = x[uIdx];
      y[bit_fftLen].im = x[uIdx + 1];
      uIdx += 2;
      bit_fftLen = nRows >> 1;
      do {
        bit_fftLen = (int32_T)((uint32_T)bit_fftLen >> 1);
        br_j ^= bit_fftLen;
      } while ((int32_T)((uint32_T)br_j & (uint32_T)bit_fftLen) == 0);
    }

    bit_fftLen = yIdx + br_j;
    y[bit_fftLen].re = x[uIdx];
    y[bit_fftLen].im = x[uIdx + 1];
  }
}

void MWDSPCG_R2DIT_TBLS_C(creal32_T y[], int32_T nChans, int32_T nRows, int32_T
  fftLen, int32_T offset, const real32_T tablePtr[], int32_T twiddleStep,
  boolean_T isInverse)
{
  int32_T fwdInvFactor;
  int32_T iCh;
  int32_T ix;
  int32_T j;
  int32_T nHalf;
  int32_T nQtr;
  int32_T offsetCh;
  nHalf = (fftLen >> 1) * twiddleStep;
  nQtr = nHalf >> 1;
  fwdInvFactor = isInverse ? -1 : 1;
  offsetCh = offset;
  for (iCh = 0; iCh < nChans; iCh++) {
    int32_T idelta;
    int32_T k;
    int32_T kratio;
    real32_T tmp_re_tmp;
    real32_T twidRe;
    for (ix = offsetCh; ix < (fftLen + offsetCh) - 1; ix += 2) {
      tmp_re_tmp = y[ix + 1].re;
      twidRe = y[ix + 1].im;
      y[ix + 1].re = y[ix].re - tmp_re_tmp;
      y[ix + 1].im = y[ix].im - twidRe;
      y[ix].re += tmp_re_tmp;
      y[ix].im += twidRe;
    }

    idelta = 2;
    k = fftLen >> 2;
    kratio = k * twiddleStep;
    while (k > 0) {
      int32_T i1;
      int32_T i2;
      int32_T istart;
      i1 = offsetCh;
      for (ix = 0; ix < k; ix++) {
        i2 = i1 + idelta;
        tmp_re_tmp = y[i2].re;
        twidRe = y[i2].im;
        y[i2].re = y[i1].re - tmp_re_tmp;
        y[i2].im = y[i1].im - twidRe;
        y[i1].re += tmp_re_tmp;
        y[i1].im += twidRe;
        i1 += idelta << 1;
      }

      istart = offsetCh;
      for (j = kratio; j < nHalf; j += kratio) {
        real32_T twidIm;
        i1 = istart + 1;
        twidRe = tablePtr[j];
        twidIm = tablePtr[j + nQtr] * (real32_T)fwdInvFactor;
        for (ix = 0; ix < k; ix++) {
          real32_T tmp_re;
          real32_T tmp_re_tmp_0;
          i2 = i1 + idelta;
          tmp_re_tmp = y[i2].re;
          tmp_re_tmp_0 = y[i2].im;
          tmp_re = tmp_re_tmp * twidRe - tmp_re_tmp_0 * twidIm;
          tmp_re_tmp_0 = tmp_re_tmp * twidIm + tmp_re_tmp_0 * twidRe;
          y[i2].re = y[i1].re - tmp_re;
          y[i2].im = y[i1].im - tmp_re_tmp_0;
          y[i1].re += tmp_re;
          y[i1].im += tmp_re_tmp_0;
          i1 += idelta << 1;
        }

        istart++;
      }

      idelta <<= 1;
      k >>= 1;
      kratio >>= 1;
    }

    offsetCh += nRows;
  }
}

void MWDSPCG_FFT_DblLen_C(creal32_T y[], int32_T nChans, int32_T nRows, const
  real32_T twiddleTable[], int32_T twiddleStep)
{
  int32_T N2;
  int32_T N4;
  int32_T W4;
  int32_T ix;
  int32_T tempOut0Re_tmp;
  int32_T tmp;
  int32_T yIdx;
  real32_T tempOut0Im;
  real32_T tempOut0Re;
  N2 = nRows >> 1;
  N4 = N2 >> 1;
  W4 = N4 * twiddleStep;
  yIdx = (nChans - 1) * nRows;
  if (nRows > 2) {
    tempOut0Re_tmp = N4 + yIdx;
    tempOut0Re = y[tempOut0Re_tmp].re;
    tempOut0Im = y[tempOut0Re_tmp].im;
    tmp = (N2 + N4) + yIdx;
    y[tmp].re = tempOut0Re;
    y[tmp].im = tempOut0Im;
    y[tempOut0Re_tmp].re = tempOut0Re;
    y[tempOut0Re_tmp].im = -tempOut0Im;
  }

  if (nRows > 1) {
    tmp = N2 + yIdx;
    y[tmp].re = y[yIdx].re - y[yIdx].im;
    y[tmp].im = 0.0F;
  }

  y[yIdx].re += y[yIdx].im;
  y[yIdx].im = 0.0F;
  tempOut0Re_tmp = twiddleStep;
  for (ix = 1; ix < N4; ix++) {
    int32_T temp2Re_tmp;
    int32_T temp2Re_tmp_0;
    real32_T temp2Re_tmp_1;
    real32_T temp2Re_tmp_tmp;
    real32_T tempOut0Im_tmp;
    real32_T tempOut0Im_tmp_tmp;
    temp2Re_tmp = ix + yIdx;
    temp2Re_tmp_0 = (N2 - ix) + yIdx;
    temp2Re_tmp_1 = y[temp2Re_tmp_0].re;
    temp2Re_tmp_tmp = y[temp2Re_tmp].re;
    tempOut0Re = (temp2Re_tmp_1 + temp2Re_tmp_tmp) / 2.0F;
    tempOut0Im_tmp = y[temp2Re_tmp_0].im;
    tempOut0Im_tmp_tmp = y[temp2Re_tmp].im;
    tempOut0Im = (tempOut0Im_tmp_tmp - tempOut0Im_tmp) / 2.0F;
    y[temp2Re_tmp].re = (tempOut0Im_tmp + tempOut0Im_tmp_tmp) / 2.0F;
    y[temp2Re_tmp].im = (temp2Re_tmp_1 - temp2Re_tmp_tmp) / 2.0F;
    temp2Re_tmp_tmp = y[temp2Re_tmp].im;
    tempOut0Im_tmp = -twiddleTable[W4 - tempOut0Re_tmp];
    tempOut0Im_tmp_tmp = y[temp2Re_tmp].re;
    temp2Re_tmp_1 = tempOut0Im_tmp_tmp * twiddleTable[tempOut0Re_tmp] -
      tempOut0Im_tmp * temp2Re_tmp_tmp;
    temp2Re_tmp_tmp = temp2Re_tmp_tmp * twiddleTable[tempOut0Re_tmp] +
      tempOut0Im_tmp * tempOut0Im_tmp_tmp;
    y[temp2Re_tmp].re = tempOut0Re + temp2Re_tmp_1;
    y[temp2Re_tmp].im = tempOut0Im + temp2Re_tmp_tmp;
    tmp = (nRows - ix) + yIdx;
    y[tmp].re = y[temp2Re_tmp].re;
    y[tmp].im = -y[temp2Re_tmp].im;
    tmp = (N2 + ix) + yIdx;
    y[tmp].re = tempOut0Re - temp2Re_tmp_1;
    y[tmp].im = tempOut0Im - temp2Re_tmp_tmp;
    y[temp2Re_tmp_0].re = y[tmp].re;
    y[temp2Re_tmp_0].im = -y[tmp].im;
    tempOut0Re_tmp += twiddleStep;
  }
}

real32_T rt_hypotf(real32_T u0, real32_T u1)
{
  real32_T a;
  real32_T b;
  real32_T y;
  a = fabsf(u0);
  b = fabsf(u1);
  if (a < b) {
    a /= b;
    y = sqrtf(a * a + 1.0F) * b;
  } else if (a > b) {
    b /= a;
    y = sqrtf(b * b + 1.0F) * a;
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

static real32_T G_Export_V3_peak_vertex(const real32_T x[16384], real32_T dc,
  real_T k, real_T N, boolean_T isMaximum)
{
  real_T km;
  real_T kp;
  real32_T den;
  real32_T ym;
  real32_T yp;
  real32_T yv;
  km = k - 1.0;
  if (k - 1.0 < 1.0) {
    km = N;
  }

  kp = k + 1.0;
  if (k + 1.0 > N) {
    kp = 1.0;
  }

  ym = x[(int32_T)km - 1] - dc;
  yv = x[(int32_T)k - 1] - dc;
  yp = x[(int32_T)kp - 1] - dc;
  den = ((yp + ym) - 2.0F * yv) * 8.0F;
  if (fabsf(den) > 1.0E-12F) {
    ym = yp - ym;
    den = yv - ym * ym / den;
    if ((isMaximum && (den >= yv)) || ((!isMaximum) && (den <= yv))) {
      yv = den;
    }
  }

  return yv;
}

void G_Export_V3_step(void)
{
  real_T i0;
  real_T startIndex;
  int32_T d;
  int32_T i;
  int32_T imin;
  int32_T p;
  int32_T q;
  real32_T bestBin[3];
  real32_T bestScore[3];
  real32_T ampApprox;
  real32_T ay;
  real32_T b_y1;
  real32_T c;
  real32_T cw;
  real32_T den;
  real32_T orderValue;
  real32_T r;
  real32_T sn;
  real32_T ss;
  real32_T sw;
  real32_T y;
  real32_T y3;
  uint32_T peakCount;
  uint32_T qY;
  boolean_T exitg1;
  for (i = 0; i < 16384; i++) {
    G_Export_V3_B.Hann[i] = 7.62939453E-5F * (real32_T)G_Export_V3_U.adc_block[i];
  }

  i = 0;
  imin = 0;
  while (imin < 16384) {
    if (G_Export_V3_DW.FIR__PhaseIdx == 0) {
      for (p = 0; p < 64; p++) {
        G_Export_V3_DW.FIR__Sums[p] = G_Export_V3_ConstP.FIR__FILT[p] *
          G_Export_V3_B.Hann[imin];
      }

      imin++;
      G_Export_V3_DW.FIR__PhaseIdx = -1;
    }

    while ((imin < 16384) && (G_Export_V3_DW.FIR__PhaseIdx > -1)) {
      for (p = 0; p < 64; p++) {
        G_Export_V3_DW.FIR__Sums[p] += G_Export_V3_ConstP.FIR__FILT
          [(G_Export_V3_DW.FIR__PhaseIdx << 6) + p] * G_Export_V3_B.Hann[imin];
      }

      imin++;
      G_Export_V3_DW.FIR__PhaseIdx--;
    }

    if (G_Export_V3_DW.FIR__PhaseIdx == -1) {
      G_Export_V3_B.FIR_[i] = G_Export_V3_DW.FIR__StatesBuff[0] +
        G_Export_V3_DW.FIR__Sums[0];
      i++;
      for (p = 0; p < 62; p++) {
        G_Export_V3_DW.FIR__StatesBuff[p] = G_Export_V3_DW.FIR__StatesBuff[p + 1]
          + G_Export_V3_DW.FIR__Sums[p + 1];
      }

      G_Export_V3_DW.FIR__StatesBuff[62] = G_Export_V3_DW.FIR__Sums[63];
      G_Export_V3_DW.FIR__PhaseIdx = 0;
    }
  }

  for (i = 0; i < 16384; i++) {
    G_Export_V3_B.Hann[i] = G_Export_V3_B.FIR_[i] *
      G_Export_V3_ConstP.Hann_WindowSamples[i];
  }

  MWDSPCG_FFT_Interleave_R2BR_S(&G_Export_V3_B.Hann[0U], &G_Export_V3_B.u096FFT
    [0U], 1, 16384);
  MWDSPCG_R2DIT_TBLS_C(&G_Export_V3_B.u096FFT[0U], 1, 16384, 8192, 0,
                       &G_Export_V3_ConstP.u096FFT_TwiddleTable[0U], 2, false);
  MWDSPCG_FFT_DblLen_C(&G_Export_V3_B.u096FFT[0U], 1, 16384,
                       &G_Export_V3_ConstP.u096FFT_TwiddleTable[0U], 1);
  G_Export_V3_Y.component_count = 0U;
  G_Export_V3_Y.frequency_Hz[0] = 0.0F;
  G_Export_V3_Y.amplitude_Vpk[0] = 0.0F;
  G_Export_V3_Y.harmonic_order[0] = 0U;
  G_Export_V3_Y.frequency_Hz[1] = 0.0F;
  G_Export_V3_Y.amplitude_Vpk[1] = 0.0F;
  G_Export_V3_Y.harmonic_order[1] = 0U;
  G_Export_V3_Y.frequency_Hz[2] = 0.0F;
  G_Export_V3_Y.amplitude_Vpk[2] = 0.0F;
  G_Export_V3_Y.harmonic_order[2] = 0U;
  for (i = 0; i < 16384; i++) {
    G_Export_V3_B.Hann[i] = rt_hypotf(G_Export_V3_B.u096FFT[i].re,
      G_Export_V3_B.u096FFT[i].im);
  }

  bestScore[0] = 0.0F;
  bestBin[0] = 0.0F;
  bestScore[1] = 0.0F;
  bestBin[1] = 0.0F;
  bestScore[2] = 0.0F;
  bestBin[2] = 0.0F;
  if (G_Export_V3_U.mode == 1) {
    ampApprox = 200000.0F;
  } else {
    ampApprox = 500000.0F;
  }

  i = (int32_T)ceil(ampApprox * G_Export_V3_NFFT / G_Export_V3_Fs);
  for (imin = 0; imin <= i - 20; imin++) {
    ampApprox = G_Export_V3_B.Hann[imin + 21];
    orderValue = G_Export_V3_B.Hann[imin + 20];
    if (ampApprox >= orderValue) {
      ay = G_Export_V3_B.Hann[imin + 22];
      if (ampApprox > ay) {
        b_y1 = logf(fmaxf(orderValue, 1.0E-30F));
        y3 = logf(fmaxf(ay, 1.0E-30F));
        den = (b_y1 - 2.0F * logf(fmaxf(ampApprox, 1.0E-30F))) + y3;
        cw = 0.0F;
        if (fabsf(den) > 1.0E-20F) {
          cw = (b_y1 - y3) * 0.5F / den;
        }

        b_y1 = fminf(0.5F, fmaxf(-0.5F, cw));
        ampApprox = ((orderValue + ampApprox) + ay) * 2.0F / 16383.0F;
        if (ampApprox >= 0.004F) {
          p = 0;
          exitg1 = false;
          while ((!exitg1) && (p < 3)) {
            if (ampApprox > bestScore[p]) {
              d = 2 - p;
              for (q = 0; q < d; q++) {
                bestScore[2 - q] = bestScore[1 - q];
                bestBin[2 - q] = bestBin[1 - q];
              }

              bestScore[p] = ampApprox;
              bestBin[p] = (((real32_T)imin + 22.0F) - 1.0F) + b_y1;
              exitg1 = true;
            } else {
              p++;
            }
          }
        }
      }
    }
  }

  if (bestScore[0] > 0.0F) {
    G_Export_V3_Y.frequency_Hz[0] = bestBin[0] * 488.28125F;
    G_Export_V3_Y.component_count = 1U;
  }

  if (bestScore[1] > 0.0F) {
    G_Export_V3_Y.frequency_Hz[1] = bestBin[1] * 488.28125F;
    G_Export_V3_Y.component_count++;
  }

  if (bestScore[2] > 0.0F) {
    G_Export_V3_Y.frequency_Hz[2] = bestBin[2] * 488.28125F;
    G_Export_V3_Y.component_count++;
  }

  for (i = 0; i < 2; i++) {
    imin = 2 - i;
    for (p = 0; p < imin; p++) {
      d = (i + p) + 1;
      if ((G_Export_V3_Y.frequency_Hz[d] > 0.0F) &&
          ((G_Export_V3_Y.frequency_Hz[i] == 0.0F) ||
           (G_Export_V3_Y.frequency_Hz[d] < G_Export_V3_Y.frequency_Hz[i]))) {
        ampApprox = G_Export_V3_Y.frequency_Hz[i];
        G_Export_V3_Y.frequency_Hz[i] = G_Export_V3_Y.frequency_Hz[d];
        G_Export_V3_Y.frequency_Hz[d] = ampApprox;
      }
    }
  }

  if (G_Export_V3_Y.frequency_Hz[0] > 0.0F) {
    ampApprox = G_Export_V3_Y.frequency_Hz[0] / G_Export_V3_Y.frequency_Hz[0];
    orderValue = roundf(ampApprox);
    if ((orderValue >= 1.0F) && (orderValue <= 255.0F) && (fabsf(ampApprox -
          orderValue) < 0.15F)) {
      G_Export_V3_Y.harmonic_order[0] = (uint8_T)orderValue;
    }

    if (G_Export_V3_Y.frequency_Hz[1] > 0.0F) {
      ampApprox = G_Export_V3_Y.frequency_Hz[1] / G_Export_V3_Y.frequency_Hz[0];
      orderValue = roundf(ampApprox);
      if ((orderValue >= 1.0F) && (orderValue <= 255.0F) && (fabsf(ampApprox -
            orderValue) < 0.15F)) {
        G_Export_V3_Y.harmonic_order[1] = (uint8_T)orderValue;
      }
    }

    if (G_Export_V3_Y.frequency_Hz[2] > 0.0F) {
      ampApprox = G_Export_V3_Y.frequency_Hz[2] / G_Export_V3_Y.frequency_Hz[0];
      orderValue = roundf(ampApprox);
      if ((orderValue >= 1.0F) && (orderValue <= 255.0F) && (fabsf(ampApprox -
            orderValue) < 0.15F)) {
        G_Export_V3_Y.harmonic_order[2] = (uint8_T)orderValue;
      }
    }
  }

  ampApprox = 0.0F;
  for (i = 0; i < 16384; i++) {
    ampApprox += G_Export_V3_B.FIR_[i];
  }

  ampApprox /= 16384.0F;
  orderValue = 0.0F;
  for (p = 0; p < 16384; p++) {
    ay = fabsf(G_Export_V3_B.FIR_[p] - ampApprox);
    if (ay > orderValue) {
      orderValue = ay;
    }
  }

  b_y1 = orderValue * 0.99F;
  y3 = 0.0F;
  peakCount = 0U;
  for (p = 0; p < 16382; p++) {
    ay = fabsf(G_Export_V3_B.FIR_[p + 1] - ampApprox);
    if ((ay >= b_y1) && (ay >= fabsf(G_Export_V3_B.FIR_[p] - ampApprox)) && (ay >
         fabsf(G_Export_V3_B.FIR_[p + 2] - ampApprox))) {
      y3 += ay;
      qY = peakCount + 1U;
      if (peakCount + 1U < peakCount) {
        qY = MAX_uint32_T;
      }

      peakCount = qY;
    }
  }

  if (peakCount > 0U) {
    orderValue = y3 / (real32_T)peakCount;
  }

  for (i = 0; i < 3; i++) {
    ay = G_Export_V3_Y.frequency_Hz[i];
    if (ay > 0.0F) {
      ay = 6.28318548F * ay / 8.0E+6F;
      cw = cosf(ay);
      sw = sinf(ay);
      c = 1.0F;
      sn = 0.0F;
      ay = 0.0F;
      ss = 0.0F;
      b_y1 = 0.0F;
      y3 = 0.0F;
      den = 0.0F;
      for (p = 0; p < 16384; p++) {
        y = G_Export_V3_B.FIR_[p] - ampApprox;
        ay += c * c;
        ss += sn * sn;
        b_y1 += c * sn;
        y3 += y * c;
        den += y * sn;
        y = c * cw - sn * sw;
        sn = sn * cw + c * sw;
        c = y;
        if (((uint32_T)(p + 1) & 255U) == 0U) {
          r = sqrtf(y * y + sn * sn);
          c = y / r;
          sn /= r;
        }
      }

      cw = ay * ss - b_y1 * b_y1;
      if (fabsf(cw) > 1.0E-20F) {
        ss = (y3 * ss - den * b_y1) / cw;
        ay = (den * ay - y3 * b_y1) / cw;
        G_Export_V3_Y.amplitude_Vpk[i] = sqrtf(ss * ss + ay * ay);
      }
    }
  }

  ampApprox = 1.0F;
  if ((G_Export_V3_Y.amplitude_Vpk[0] > 1.0E-6F) && (orderValue >
       G_Export_V3_Y.amplitude_Vpk[0] * 1.005F)) {
    ampApprox = orderValue / G_Export_V3_Y.amplitude_Vpk[0];
  }

  G_Export_V3_Y.amplitude_SettingVpk[0] = G_Export_V3_Y.amplitude_Vpk[0] *
    ampApprox;
  G_Export_V3_Y.amplitude_SettingVpk[1] = G_Export_V3_Y.amplitude_Vpk[1] *
    ampApprox;
  G_Export_V3_Y.amplitude_SettingVpk[2] = G_Export_V3_Y.amplitude_Vpk[2] *
    ampApprox;
  memset(&G_Export_V3_Y.waveform[0], 0, 600U * sizeof(real32_T));
  G_Export_V3_Y.waveCount = 0U;
  ampApprox = 0.0F;
  for (i = 0; i < 16384; i++) {
    ampApprox += G_Export_V3_B.FIR_[i];
  }

  ampApprox /= 16384.0F;
  orderValue = 0.0F;
  i = 1;
  imin = 1;
  ay = G_Export_V3_B.FIR_[0] - ampApprox;
  b_y1 = ay;
  for (p = 0; p < 16384; p++) {
    y = G_Export_V3_B.FIR_[p] - ampApprox;
    orderValue += y * y;
    if (y > ay) {
      ay = y;
      i = p + 1;
    }

    if (y < b_y1) {
      b_y1 = y;
      imin = p + 1;
    }
  }

  G_Export_V3_Y.fundamental_Hz = G_Export_V3_Y.frequency_Hz[0];
  if (G_Export_V3_Y.frequency_Hz[0] > 0.0F) {
    p = 1;
    if (G_Export_V3_U.periods >= 3) {
      p = 3;
    }

    b_y1 = 8.0E+6F / G_Export_V3_Y.frequency_Hz[0] * (real32_T)p;
    i0 = ceil(b_y1);
    if (i0 + 2.0 < 16384.0) {
      startIndex = 1.0;
      p = 0;
      exitg1 = false;
      while ((!exitg1) && (p <= (int32_T)((16384.0 - (i0 + 2.0)) - 1.0) - 1)) {
        if ((G_Export_V3_B.FIR_[p] - ampApprox <= 0.0F) && (G_Export_V3_B.FIR_[p
             + 1] - ampApprox > 0.0F)) {
          startIndex = (real_T)p + 2.0;
          exitg1 = true;
        } else {
          p++;
        }
      }

      for (p = 0; p < 600; p++) {
        ay = (((real32_T)p + 1.0F) - 1.0F) * b_y1 / 599.0F + (real32_T)
          startIndex;
        i0 = floor(ay);
        y3 = ay - (real32_T)i0;
        if (i0 < 1.0) {
          i0 = 1.0;
          y3 = 0.0F;
        } else if (i0 >= 16384.0) {
          i0 = 16383.0;
          y3 = 1.0F;
        }

        ay = G_Export_V3_B.FIR_[(int32_T)i0 - 1] - ampApprox;
        G_Export_V3_Y.waveform[p] = ((G_Export_V3_B.FIR_[(int32_T)(i0 + 1.0) - 1]
          - ampApprox) - ay) * y3 + ay;
      }

      G_Export_V3_Y.waveCount = 600U;
    }
  }

  for (p = 0; p < 600; p++) {
    G_Export_V3_Y.waveform[p] *= 0.25F;
  }

  G_Export_V3_Y.amplitude_Vpk[0] *= 0.25F;
  G_Export_V3_Y.amplitude_SettingVpk[0] *= 0.25F;
  G_Export_V3_Y.amplitude_Vpk[1] *= 0.25F;
  G_Export_V3_Y.amplitude_SettingVpk[1] *= 0.25F;
  G_Export_V3_Y.amplitude_Vpk[2] *= 0.25F;
  G_Export_V3_Y.amplitude_SettingVpk[2] *= 0.25F;
  G_Export_V3_Y.Vpp = (G_Export_V3_peak_vertex(G_Export_V3_B.FIR_, ampApprox,
    (real_T)i, 16384.0, true) - G_Export_V3_peak_vertex(G_Export_V3_B.FIR_,
    ampApprox, (real_T)imin, 16384.0, false)) * 0.25F;
  G_Export_V3_Y.Vrms = sqrtf(orderValue / 16384.0F) * 0.25F;
}

void G_Export_V3_initialize(void)
{
}

void G_Export_V3_terminate(void)
{
}
