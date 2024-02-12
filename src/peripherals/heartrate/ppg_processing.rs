//! Signal processing for heartrate
//!
//! Implementation from https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/heartrate/Ppg.cpp

use libm::{powf, sqrtf};
use microfft::complex::cfft_64;
use num_complex::Complex;

// Source: https://github.com/kosme/arduinoFFT/blob/master/src/arduinoFFT.cpp
fn complex_to_magnitude(v_complex: &mut [Complex<f32>]) {
    // vM is half the size of vReal and vImag
    for i in 0..v_complex.len() {
        v_complex[i].re = sqrtf(powf(v_complex[i].re, 2.0) + powf(v_complex[i].im, 2.0));
    }
}

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
    let mut peaks = 0;
    let mut enabled = false;
    let mut min_bin = 0;
    let mut max_bin;
    let mut peak_center = 0.0;
    let mut prev_val = linear_interpolation(x_vals, y_vals, start - 0.01);
    let mut curr_val = linear_interpolation(x_vals, y_vals, start);

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
    match total > 0 {
        true => signal[start..end].iter().sum::<f32>() / total as f32,
        false => 0.0,
    }
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
    let exp_alpha = 0.816;
    for _ in 0..4 {
        let mut exp_avg = signal[0];
        for i in 0..length {
            exp_avg = (exp_alpha * signal[i]) + ((1.0 - exp_alpha) * exp_avg);
            signal[i] = exp_avg;
        }
    }
    let exp_alpha = 0.268;
    for _ in 0..4 {
        let mut exp_avg = signal[0];
        for i in 0..length {
            exp_avg = (exp_alpha * signal[i]) + ((1.0 - exp_alpha) * exp_avg);
            signal[i] -= exp_avg;
        }
    }
}

fn spectrum_max(data: &[f32], start: usize, end: usize) -> f32 {
    match data[start..end].iter().max_by(|&a, &b| a.total_cmp(b)) {
        Some(val) => *val,
        None => 0.0,
    }
}

fn detrend(signal: &mut [f32]) {
    let length = signal.len();
    let offset = signal[0];
    let slope = (signal[length - 1] - offset) / (length - 1) as f32;

    for i in 0..length {
        signal[i] -= slope * i as f32 + offset;
    }
    for i in 0..(length - 1) {
        signal[i] = signal[i + 1] - signal[i];
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
    const PEAK_DETECTION_THRESHOLD: f32 = 0.7;
    // Maximum peak width (bins) at threshold for valid peak.
    const MAX_PEAK_WIDTH: f32 = 2.5;
    // Metric for spectrum noise level.
    const SNR_THRESHOLD: f32 = 3.0;
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
    const ALS_FACTOR: u16 = 2;

    /// Create new PPG
    pub fn new() -> Self {
        Self::default()
    }
    /// Pre-process raw sensor data
    pub fn preprocess(&mut self, hrs: u32, als: u32) -> bool {
        // Add data and increase index if index within DATA_LENGTH
        if self.data_index < Self::DATA_LENGTH {
            self.data_hrs[self.data_index] = hrs as u16;
            self.data_index += 1;
        }
        // Save ALS value and check against threshold
        self.als_value = als as u16;
        self.als_value > self.als_threshold
    }
    /// Get heart rate
    pub fn get_heart_rate(&mut self) -> Option<u8> {
        // Make sure that buffer is full before processing data
        if self.data_index < Self::DATA_LENGTH {
            return None;
        }

        // Calculate heart rate
        let hr = self.process_heart_rate(self.reset_spectral_avg);
        self.reset_spectral_avg = false;

        // Make room for OVERLAP_WINDOW number of new samples
        self.data_hrs.rotate_left(Self::OVERLAP_WINDOW);
        self.data_index = Self::DATA_LENGTH - Self::OVERLAP_WINDOW;

        // Return heart rate
        hr
    }
    /// Reset PPG
    pub fn reset(&mut self, reset_daq_buf: bool) {
        if reset_daq_buf {
            self.data_index = 0;
        }
        self.avg_index = 0;
        self.data_avg.fill(0.0);
        self.last_peak_location = 0.0;
        self.als_threshold = u16::MAX;
        self.als_value = 0;
        self.reset_spectral_avg = true;
        self.spectrum.fill(0.0);
    }
    fn process_heart_rate(&mut self, init: bool) -> Option<u8> {
        // Prepare datasets
        for i in 0..Self::DATA_LENGTH {
            self.v_real[i] = self.data_hrs[i] as f32;
        }
        self.v_imag.fill(0.0);

        // Remove slope and DC offset from dataset
        detrend(&mut self.v_real);
        // Apply bandpass filter (30 to 240Hz) to remove low frequency noise
        // and high frequency spikes from detrending step
        filter_30_to_240(&mut self.v_real);

        // Apply Hanning window (make signal seem continuous to fft)
        for i in 0..Self::DATA_LENGTH {
            let mut i_hann = i;
            if i >= Self::DATA_LENGTH >> 1 {
                i_hann = Self::DATA_LENGTH - (i + 1);
            }
            self.v_real[i] *= HANNING[i_hann];
        }

        // Calculate FFT
        let mut v_complex = [Complex::new(0.0, 0.0); Self::DATA_LENGTH];
        for i in 0..Self::DATA_LENGTH {
            v_complex[i] = Complex::new(self.v_real[i], self.v_imag[i]);
        }
        let spectrum = cfft_64(&mut v_complex);

        // Calculate the power spectrum
        complex_to_magnitude(spectrum);

        // Write values from complex numbers back to their parts
        for i in 0..Self::DATA_LENGTH {
            self.v_real[i] = spectrum[i].re;
            self.v_imag[i] = spectrum[i].im;
        }

        // Apply result to spectral average, to enhance the more frequently
        // occurring heart rate frequency while diminishing random noise
        self.spectrum_avg(init);

        // Check the signal to noise ratio
        let max = spectrum_max(
            &self.spectrum,
            Self::HR_ROI_BEGIN as usize,
            Self::HR_ROI_END as usize,
        );

        let sn_ratio = signal_to_noise(
            &self.spectrum,
            Self::HR_ROI_BEGIN as usize,
            Self::HR_ROI_END as usize,
            max,
        );
        // defmt::debug!("Signal to noise ratio: {}", sn_ratio);

        // Search for peaks in the spectrum only if SNR is good.
        // Look for peaks above threshold, if only one peak is above threshold,
        // save the peak position to peak_location.
        self.peak_location = 0.0;
        let mut threshold = Self::PEAK_DETECTION_THRESHOLD;
        let mut peak_width = 0.0_f32;

        if sn_ratio > Self::SNR_THRESHOLD && self.spectrum[0] < Self::DC_THRESHOLD {
            // defmt::debug!("SNR is good.");
            threshold *= max;

            // Reuse imaginary parts for interpolation of x values passed to peak_search
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

        // Update the ambient light threshold
        self.als_threshold = self.als_value * Self::ALS_FACTOR;

        // Get current HR average
        self.peak_location = self.heart_rate_avg(self.peak_location);

        // If HR reduced to zero, return None (reset) else HR
        match self.peak_location == 0.0 && self.last_peak_location > 0.0 {
            true => {
                self.last_peak_location = 0.0;
                None
            }
            false => {
                self.last_peak_location = self.peak_location;
                Some(((self.peak_location as f32 * 60.0) + 0.5) as u8)
            }
        }
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

        match total > 0 {
            true => avg / total as f32,
            false => 0.0,
        }
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
