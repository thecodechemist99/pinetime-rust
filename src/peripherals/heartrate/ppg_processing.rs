//! Signal processing for heartrate
//!
//! Implementation from https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/heartrate/Ppg.cpp

use microfft::complex::cfft_64;
use num_complex::Complex;

fn linear_interpolation(x_vals: &[f32], y_vals: &[f32], point_x: f32) -> f32 {
    // Handle edge cases
    if point_x > *x_vals.last().unwrap() {
        return *y_vals.last().unwrap();
    } else if point_x <= *x_vals.first().unwrap() {
        return *y_vals.first().unwrap();
    }

    // Find interval
    let mut point_x0 = 0.0;
    let mut point_x1 = 0.0;
    let mut point_y0 = 0.0;
    let mut point_y1 = 0.0;
    for (i, val) in x_vals.iter().enumerate() {
        if point_x <= *val {
            // Set boundaries
            point_x0 = x_vals[i - 1];
            point_x1 = *val;
            point_y0 = y_vals[i - 1];
            point_y1 = y_vals[i];

            break;
        }
    }

    // Interpolate
    let mu = (point_x - point_x0) / (point_x1 - point_x0);
    point_y0 * (1.0 - mu) + point_y1 * mu
}

fn peak_search(
    x_vals: &[f32],
    y_vals: &[f32],
    threshold: f32,
    width: &mut f32,
    start: f32,
    end: f32,
) -> f32 {
    let mut max_bin;
    let mut min_bin = 0;
    let mut peaks = 0;
    let mut enabled = false;
    let mut peak_center = 0.0;
    let mut prev_val = linear_interpolation(x_vals, y_vals, start - 0.01);
    let mut curr_val = linear_interpolation(x_vals, y_vals, start);

    // defmt::debug!("x values: {}", x_vals);
    // defmt::debug!("y values: {}", y_vals);
    for i in (start * 100.0) as u32..(end * 100.0) as u32 {
        let next_val = linear_interpolation(x_vals, y_vals, i as f32 * 0.01 + 0.01);
        if curr_val < threshold {
            enabled = true;
        }
        if curr_val >= threshold && enabled {
            if prev_val < threshold {
                min_bin = i;
            } else if next_val <= threshold {
                max_bin = i;
                peaks += 1;
                *width = (max_bin - min_bin) as f32 * 0.01;
                peak_center = *width / 2.0 + min_bin as f32 * 0.01;
            }
        }
        prev_val = curr_val;
        curr_val = next_val;
    }
    if peaks != 1 {
        *width = 0.0;
        peak_center = 0.0;
    }
    peak_center
}

fn spectrum_mean(signal: &[f32], start: usize, end: usize) -> f32 {
    let total = end - start;
    if total > 0 {
        return signal[start..end].iter().sum::<f32>() / total as f32;
    }
    0.0
}

fn signal_to_noise(signal: &[f32], start: usize, end: usize, max: f32) -> f32 {
    let mean = spectrum_mean(signal, start, end);
    max / mean
}

/// Simple bandpass filter using exponential moving average
fn filter_30_to_240(signal: &mut [f32]) {
    // From: https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/
    let length = signal.len();
    // 0.268 is ~0.5Hz and 0.816 is ~4Hz cutoff at 10Hz sampling
    let mut exp_avg: f32;
    let mut exp_alpha = 0.816;
    for _ in 0..4 {
        exp_avg = *signal.first().unwrap();
        for i in 0..length {
            exp_avg = (exp_alpha * signal[i]) + ((1.0 - exp_alpha) * exp_avg);
            signal[i] = exp_avg;
        }
    }
    exp_alpha = 0.268;
    for _ in 0..4 {
        exp_avg = *signal.first().unwrap();
        for i in 0..length {
            exp_avg = (exp_alpha * signal[i]) + ((1.0 - exp_alpha) * exp_avg);
            signal[i] -= exp_avg;
        }
    }
}

fn spectrum_max(data: &[f32], start: usize, end: usize) -> f32 {
    let max = data[start..end].iter().max_by(|&a, &b| a.total_cmp(b));

    match max {
        Some(val) => *val,
        None => 0.0,
    }
}

fn detrend(signal: &mut [f32]) {
    let last_index = signal.len() - 1;
    let offset = *signal.first().unwrap();
    let slope = (signal.last().unwrap() - offset) / last_index as f32;

    for (i, mut val) in signal.iter().enumerate().rev() {
        let new_val = val - (slope * i as f32 + offset);
        val = &new_val;
        if i < last_index {
            val = &(signal[i + 1] - val);
        }
    }
}

// Hanning Coefficients from numpy: python -c 'import numpy;print(numpy.hanning(64))'
// Note: Harcoded and must be updated if constexpr dataLength is changed. Prevents the need to
// use cosf() which results in an extra ~5KB in storage.
// This data is symetrical so just using the first half (saves 128B when dataLength is 64).
const HANNING: [f32; 32] = [
    0.00000000_f32,
    0.00248461_f32,
    0.00991376_f32,
    0.02221360_f32,
    0.03926189_f32,
    0.06088921_f32,
    0.08688061_f32,
    0.11697778_f32,
    0.15088159_f32,
    0.18825510_f32,
    0.22872687_f32,
    0.27189467_f32,
    0.31732949_f32,
    0.36457977_f32,
    0.41317591_f32,
    0.46263495_f32,
    0.51246535_f32,
    0.56217185_f32,
    0.61126047_f32,
    0.65924333_f32,
    0.70564355_f32,
    0.75000000_f32,
    0.79187184_f32,
    0.83084292_f32,
    0.86652594_f32,
    0.89856625_f32,
    0.92664544_f32,
    0.95048443_f32,
    0.96984631_f32,
    0.98453864_f32,
    0.99441541_f32,
    0.99937846_f32,
];

// Ppg::Ppg() {
//   dataAverage.fill(0.0f);
//   spectrum.fill(0.0f);
// }

// int8_t Ppg::Preprocess(uint32_t hrs, uint32_t als) {
//   if (dataIndex < dataLength) {
//     dataHRS[dataIndex++] = hrs;
//   }
//   alsValue = als;
//   if (alsValue > alsThreshold) {
//     return 1;
//   }
//   return 0;
// }

// int Ppg::HeartRate() {
//   if (dataIndex < dataLength) {
//     return 0;
//   }
//   int hr = 0;
//   hr = ProcessHeartRate(resetSpectralAvg);
//   resetSpectralAvg = false;
//   // Make room for overlapWindow number of new samples
//   for (int idx = 0; idx < dataLength - overlapWindow; idx++) {
//     dataHRS[idx] = dataHRS[idx + overlapWindow];
//   }
//   dataIndex = dataLength - overlapWindow;
//   return hr;
// }

// void Ppg::Reset(bool resetDaqBuffer) {
//   if (resetDaqBuffer) {
//     dataIndex = 0;
//   }
//   avgIndex = 0;
//   dataAverage.fill(0.0f);
//   lastPeakLocation = 0.0f;
//   alsThreshold = UINT16_MAX;
//   alsValue = 0;
//   resetSpectralAvg = true;
//   spectrum.fill(0.0f);
// }

// // Pass init == true to reset spectral averaging.
// // Returns -1 (Reset Acquisition), 0 (Unable to obtain HR) or HR (BPM).
// int Ppg::ProcessHeartRate(bool init) {
//   std::copy(dataHRS.begin(), dataHRS.end(), vReal.begin());
//   Detrend(vReal);
//   Filter30to240(vReal);
//   vImag.fill(0.0f);
//   // Apply Hanning Window
//   int hannIdx = 0;
//   for (int idx = 0; idx < dataLength; idx++) {
//     if (idx >= dataLength >> 1) {
//       hannIdx--;
//     }
//     vReal[idx] *= hanning[hannIdx];
//     if (idx < dataLength >> 1) {
//       hannIdx++;
//     }
//   }
//   // Compute in place power spectrum
//   ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal.data(), vImag.data(), dataLength, sampleFreq);
//   FFT.compute(FFTDirection::Forward);
//   FFT.complexToMagnitude();
//   FFT.~ArduinoFFT();
//   SpectrumAverage(vReal.data(), spectrum.data(), spectrum.size(), init);
//   peakLocation = 0.0f;
//   float threshold = peakDetectionThreshold;
//   float peakWidth = 0.0f;
//   int specLen = spectrum.size();
//   float max = SpectrumMax(spectrum, hrROIbegin, hrROIend);
//   float signalToNoiseRatio = SignalToNoise(spectrum, hrROIbegin, hrROIend, max);
//   if (signalToNoiseRatio > signalToNoiseThreshold && spectrum.at(0) < dcThreshold) {
//     threshold *= max;
//     // Reuse VImag for interpolation x values passed to PeakSearch
//     for (int idx = 0; idx < dataLength; idx++) {
//       vImag[idx] = idx;
//     }
//     peakLocation = PeakSearch(vImag.data(),
//                               spectrum.data(),
//                               threshold,
//                               peakWidth,
//                               static_cast<float>(hrROIbegin),
//                               static_cast<float>(hrROIend),
//                               specLen);
//     peakLocation *= freqResolution;
//   }
//   // Peak too wide? (broad spectrum noise or large, rapid HR change)
//   if (peakWidth > maxPeakWidth) {
//     peakLocation = 0.0f;
//   }
//   // Check HR limits
//   if (peakLocation < minHR || peakLocation > maxHR) {
//     peakLocation = 0.0f;
//   }
//   // Reset spectral averaging if bad reading
//   if (peakLocation == 0.0f) {
//     resetSpectralAvg = true;
//   }
//   // Set the ambient light threshold and return HR in BPM
//   alsThreshold = static_cast<uint16_t>(alsValue * alsFactor);
//   // Get current average HR. If HR reduced to zero, return -1 (reset) else HR
//   peakLocation = HeartRateAverage(peakLocation);
//   int rtn = -1;
//   if (peakLocation == 0.0f && lastPeakLocation > 0.0f) {
//     lastPeakLocation = 0.0f;
//   } else {
//     lastPeakLocation = peakLocation;
//     rtn = static_cast<int>((peakLocation * 60.0f) + 0.5f);
//   }
//   return rtn;
// }

// void Ppg::SpectrumAverage(const float* data, float* spectrum, int length, bool reset) {
//   if (reset) {
//     spectralAvgCount = 0;
//   }
//   float count = static_cast<float>(spectralAvgCount);
//   for (int idx = 0; idx < length; idx++) {
//     spectrum[idx] = (spectrum[idx] * count + data[idx]) / (count + 1);
//   }
//   if (spectralAvgCount < spectralAvgMax) {
//     spectralAvgCount++;
//   }
// }

// float Ppg::HeartRateAverage(float hr) {
//   avgIndex++;
//   avgIndex %= dataAverage.size();
//   dataAverage[avgIndex] = hr;
//   float avg = 0.0f;
//   float total = 0.0f;
//   float min = 300.0f;
//   float max = 0.0f;
//   for (const float& value : dataAverage) {
//     if (value > 0.0f) {
//       avg += value;
//       if (value < min)
//         min = value;
//       if (value > max)
//         max = value;
//       total++;
//     }
//   }
//   if (total > 0) {
//     avg /= total;
//   } else {
//     avg = 0.0f;
//   }
//   return avg;
// }

#[allow(unused)]
/// Photoplethysmogram
pub struct PPG {
    /// Raw ADC data
    data_hrs: [u16; Self::DATA_LENGTH],
    /// Stores real numbers from FFT
    v_real: [f32; Self::DATA_LENGTH],
    /// Stores imaginary numbers from FFT
    v_imag: [f32; Self::DATA_LENGTH],
    /// Stores power spectrum calculated from FFT complex values
    spectrum: [f32; Self::SPECTRUM_LENGTH],
    /// Stores each new HR value (Hz). Non zero values are averaged for HR output
    data_avg: [f32; 20],
    ///
    avg_index: usize,
    ///
    spectral_avg_count: u16,
    ///
    last_peak_location: f32,
    ///
    als_threshold: u16,
    ///
    als_value: u16,
    ///
    data_index: usize,
    ///
    peak_location: f32,
    ///
    reset_spectral_avg: bool,
}

#[allow(unused)]
impl Default for PPG {
    fn default() -> Self {
        Self {
            data_hrs: [0; Self::DATA_LENGTH],
            v_real: [0.0; Self::DATA_LENGTH],
            v_imag: [0.0; Self::DATA_LENGTH],
            spectrum: [0.0; Self::SPECTRUM_LENGTH],
            data_avg: [0.0; 20],
            avg_index: 0,
            spectral_avg_count: 0,
            last_peak_location: 0.0,
            als_threshold: 0,
            als_value: 0,
            data_index: 0,
            peak_location: 0.0,
            reset_spectral_avg: false,
        }
    }
}

#[allow(unused)]
impl PPG {
    pub const DELTA_T_MS: u16 = 100;
    // Daq data length: Must be power of 2
    pub const DATA_LENGTH: usize = 64;
    pub const SPECTRUM_LENGTH: usize = Self::DATA_LENGTH >> 1;

    // The sampling frequency (Hz) based on sampling time in milliseconds (DeltaTms)
    const SAMPLE_FREQ: f32 = 1000.0 / Self::DELTA_T_MS as f32;
    // The frequency resolution (Hz)
    const FREQ_RES: f32 = Self::SAMPLE_FREQ / Self::DATA_LENGTH as f32;
    // Number of samples before each analysis
    // 0.5 second update rate at 10Hz
    const OVERLAP_WINDOW: usize = 5;
    // Maximum number of spectrum running averages
    // Note: actual number of spectra averaged = spectralAvgMax + 1
    const SPECTRAL_AVG_MAX: u16 = 2;
    // Multiple Peaks above this threshold (% of max) are rejected
    const PEAK_DETECTION_THRESHOLD: f32 = 0.6;
    // Maximum peak width (bins) at threshold for valid peak.
    const MAX_PEAK_WIDTH: f32 = 2.5;
    // Metric for spectrum noise level.
    const SN_THRESHOLD: f32 = 3.0;
    // Heart rate Region Of Interest begin (bins)
    const HR_ROI_BEGIN: f32 = ((30.0 / 60.0) / Self::FREQ_RES + 0.5);
    // Heart rate Region Of Interest end (bins)
    const HR_ROI_END: f32 = ((240.0 / 60.0) / Self::FREQ_RES + 0.5);
    // Minimum HR (Hz)
    const MIN_HR: f32 = 40.0 / 60.0;
    // Maximum HR (Hz)
    const MAX_HR: f32 = 230.0 / 60.0;
    // Threshold for high DC level after filtering
    const DC_THRESHOLD: f32 = 0.5;
    // ALS detection factor
    const ALS_FACTOR: f32 = 2.0;

    /// Create new PPG
    pub fn new() -> Self {
        Self::default()
    }
    /// Pre-process raw sensor data
    pub fn preprocess(&mut self, hrs: u32, als: u32) -> bool {
        // defmt::debug!(
        //     "HRS data: {}, ALS data: {}, ALS threshold: {} ",
        //     hrs,
        //     als,
        //     self.als_threshold
        // );
        if self.data_index < Self::DATA_LENGTH {
            self.data_hrs[self.data_index] = hrs as u16;
            self.data_index += 1;
        }
        self.als_value = als as u16;
        self.als_value > self.als_threshold
    }
    /// Get heart rate
    pub fn get_heart_rate(&mut self) -> Option<u8> {
        // Check if buffer is full
        if self.data_index < Self::DATA_LENGTH {
            return None;
        }
        // defmt::debug!("HRS data: {}", self.data_hrs);

        // Calculate heart rate
        let hr = self.process_heart_rate(self.reset_spectral_avg);
        self.reset_spectral_avg = false;

        // Make room for OVERLAP_WINDOW number of new samples
        self.data_hrs.rotate_left(Self::OVERLAP_WINDOW);
        self.data_index = Self::DATA_LENGTH - Self::OVERLAP_WINDOW;

        hr
    }
    /// Reset PPG
    pub fn reset(&mut self) {
        self.data_index = 0;
        self.avg_index = 0;
        self.data_avg.fill(0.0);
        self.last_peak_location = 0.0;
        self.als_threshold = u16::MAX;
        self.als_value = 0;
        self.reset_spectral_avg = true;
        self.spectrum.fill(0.0);
    }
    fn process_heart_rate(&mut self, init: bool) -> Option<u8> {
        for i in 0..Self::DATA_LENGTH {
            self.v_real[i] = self.data_hrs[i] as f32;
        }
        // defmt::debug!("Real numbers: {}", self.v_real);
        detrend(&mut self.v_real);
        filter_30_to_240(&mut self.v_real);
        self.v_imag.fill(0.0);

        // Apply Hanning Window
        for (i, mut val) in self.v_real.iter().enumerate() {
            let mut i_hann = i;
            if i >= Self::DATA_LENGTH >> 1 {
                i_hann = Self::DATA_LENGTH - (i + 1)
            }
            val = &(val * HANNING[i_hann]);
        }

        // Compute in place power spectrum
        let mut v_complex = [Complex::new(0.0, 0.0); Self::DATA_LENGTH];
        for i in 0..Self::DATA_LENGTH {
            v_complex[i] = Complex::new(self.v_real[i], self.v_imag[i]);
        }
        // defmt::debug!("Complex numbers: {}", defmt::Debug2Format(&v_complex));
        let spectrum = cfft_64(&mut v_complex);
        self.spectrum_avg(init);
        // defmt::debug!("Averaged spectrum: {}", defmt::Debug2Format(&spectrum));
        self.peak_location = 0.0;
        let mut threshold = Self::PEAK_DETECTION_THRESHOLD;
        let mut peak_width = 0.0_f32;
        let max = spectrum_max(
            &self.spectrum,
            Self::HR_ROI_BEGIN as usize,
            Self::HR_ROI_END as usize,
        );
        // defmt::debug!("Spectrum max: {}", max);
        let sn_ratio = signal_to_noise(
            &self.spectrum,
            Self::HR_ROI_BEGIN as usize,
            Self::HR_ROI_END as usize,
            max,
        );
        // defmt::debug!("Signal to noise ratio: {}", sn_ratio);
        if sn_ratio > Self::SN_THRESHOLD && self.spectrum[0] < Self::DC_THRESHOLD {
            threshold *= max;
            // Reuse imaginary parts for interpolation of x values passed to PeakSearch
            for i in 0..Self::DATA_LENGTH {
                self.v_imag[i] = i as f32;
            }
            self.peak_location = peak_search(
                &self.v_imag,
                &self.spectrum,
                threshold,
                &mut peak_width,
                Self::HR_ROI_BEGIN,
                Self::HR_ROI_END,
            );
            self.peak_location *= Self::FREQ_RES;
            // defmt::debug!("Peak location: {}", self.peak_location);
        }

        // Peak too wide? (broad spectrum noise or large, rapid HR change)
        if peak_width > Self::MAX_PEAK_WIDTH {
            self.peak_location = 0.0;
        }

        // Check HR limits
        if self.peak_location < Self::MIN_HR || self.peak_location > Self::MAX_HR {
            self.peak_location = 0.0;
        }

        // Reset spectral averaging if bad reading
        if self.peak_location == 0.0 {
            self.reset_spectral_avg = true;
        }

        // Set the ambient light threshold and return HR in BPM
        self.als_threshold = (self.als_value as f32 * Self::ALS_FACTOR) as u16;

        // Get current average HR. If HR reduced to zero, return None (reset) else HR
        self.peak_location = self.heart_rate_avg(self.peak_location);
        let mut rtn = None;
        if self.peak_location == 0.0 && self.last_peak_location > 0.0 {
            self.last_peak_location = 0.0;
        } else {
            self.last_peak_location = self.peak_location;
            rtn = Some(((self.peak_location as f32 * 60.0) + 0.5) as u8);
        }
        rtn
    }

    pub fn heart_rate_avg(&mut self, heart_rate: f32) -> f32 {
        self.avg_index += 1;
        self.avg_index %= self.data_avg.len();
        self.data_avg[self.avg_index] = heart_rate;
        let mut avg = 0.0_f32;
        let mut total = 0;

        for value in self.data_avg {
            if value > 0.0 {
                avg += value;
                total += 1;
            }
        }
        if total > 0 {
            return avg / total as f32;
        }
        0.0
    }
    fn spectrum_avg(&mut self, reset: bool) {
        if reset {
            self.spectral_avg_count = 0;
        }

        let count = self.spectral_avg_count as f32;
        for i in 0..Self::SPECTRUM_LENGTH {
            self.spectrum[i] = (self.spectrum[i] * count + self.v_real[i]) / (count + 1.0);
        }

        if self.spectral_avg_count < Self::SPECTRAL_AVG_MAX {
            self.spectral_avg_count += 1;
        }
    }
}
