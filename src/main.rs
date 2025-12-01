use critical_section::Mutex;
use esp_idf_sys::*;
use std::ffi::c_void;
use std::sync::atomic::{AtomicU32};
use std::time::{Duration};
// Log jitter for each task. 
use microfft::real::rfft_1024;
use core::f32::consts::PI;
use num_complex::Complex32;


use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::{config::Config, UartDriver};
use esp_idf_hal::gpio::{Gpio0, Gpio2};

use esp_idf_sys::{esp_timer_get_time, xTaskCreatePinnedToCore, xTaskDelayUntil, xTaskGetTickCount, TickType_t};

use core::cell::RefCell;

extern "C" fn gps_task(_arg: *mut c_void) {
    
    // SAFETY: Peripherals::take() only once per task/thread.
    let peripherals = unsafe { esp_idf_hal::peripherals::Peripherals::take() }.unwrap();

    // UART1 is safest. (UART0 shares USB-serial)
    let uart1 = peripherals.uart1;

    let pins = peripherals.pins;

    // NEO-6M typical pins (adjust as needed)
    let tx = pins.gpio17;   // ESP32 TX → GPS RX
    let rx = pins.gpio16;   // GPS TX → ESP32 RX

    // Explicitly specify pin types for UART
    let tx: esp_idf_hal::gpio::AnyIOPin = esp_idf_hal::gpio::AnyIOPin::from(tx);
    let rx: esp_idf_hal::gpio::AnyIOPin = esp_idf_hal::gpio::AnyIOPin::from(rx);

    // 9600 baud default for NEO-6M
    let config = Config::new().baudrate(Hertz(9600));

    let uart = UartDriver::new(
        uart1,
        tx,
        rx,
        None::<esp_idf_hal::gpio::AnyIOPin>,   // CTS
        None::<esp_idf_hal::gpio::AnyIOPin>,   // RTS
        &config,
    )
    .expect("Failed to init UART for GPS");

    let period_ms: u32 = 200;         // 5Hz read
    let period_ticks = ms_to_ticks(period_ms);
    let period_us: i64 = (period_ms as i64) * 1000;

    unsafe {
        let mut last_wake = xTaskGetTickCount();
        let mut expected_us = esp_timer_get_time();

        let mut buf = [0u8; 128];

        loop {
            xTaskDelayUntil(&mut last_wake as *mut TickType_t, period_ticks);

            println!("Task D GPS read");
            let actual_us = esp_timer_get_time();
            let jitter = actual_us - expected_us;
            expected_us += period_us;

            // Read bytes if available
            match uart.read(&mut buf, 100) {
                Ok(n) if n > 0 => {
                    if let Ok(text) = core::str::from_utf8(&buf[..n]) {
                        // ⚡ Extract first complete NMEA line
                        for line in text.lines() {
                            if line.starts_with("$GP") || line.starts_with("$GN") {
                                println!("GPS task interval: {} us | {}", jitter, line);
                                break;
                            }
                        }
                    }
                }
                _ => {
                    println!("GPS Task interval ={} us | (no data)", jitter);
                }
            }
        }
    }
}


fn extract_first_nmea_line(buf: &[u8]) -> Option<&str> {
    if buf.is_empty() {
        return None;
    }

    if let Ok(s) = core::str::from_utf8(buf) {
        // Take the first line that looks like NMEA
        for line in s.lines() {
            if line.starts_with("$GP") || line.starts_with("$GN") {
                return Some(line);
            }
        }
    }
    None
}


// ==========================================================
// JITTER TRACKING STRUCT
// ==========================================================
pub struct TaskStats {
    pub min_jitter: AtomicU32,
    pub max_jitter: AtomicU32,
    pub bins: Mutex<[u32; 8]>, // histogram with 8 buckets
}


//
// Helper to convert milliseconds → ticks
//
fn ms_to_ticks(ms: u32) -> u32 {
    unsafe {
        // configTICK_RATE_HZ is a FreeRTOS constant exposed by ESP-IDF
        let hz = configTICK_RATE_HZ as u32;
        (ms * hz) / 1000
    }
}

fn get_next_runtime(period_time: u32) -> i64{

    unsafe {
    let now = esp_timer_get_time();
    let to_ticks = ms_to_ticks(period_time);
    let next_runtime = now + to_ticks as i64;

    next_runtime
    }
}

fn fft(){ 

// --- FFT workload for Task B ---
    let mut input: [f32; 1024] = [0.0; 1024];
    let freq: f32 = 5.0;
    let sample_rate: f32 = 100.0;

// Generate a synthetic test waveform (sine wave)
    for (i, sample) in input.iter_mut().enumerate() {
        *sample = (2.0 * PI * freq * (i as f32) / sample_rate).sin();
    }

    // Run FFT (in-place)
    let spectrum: &[Complex32; 512] = rfft_1024(&mut input);

    // Compute max magnitude (⚠ must specify type!)
    let mut max_magnitude: f32 = 0.0;

    for c in spectrum.iter() {
        let mag = c.norm();
        if mag > max_magnitude {
            max_magnitude = mag;
        }
    }
    println!("FFT max magnitude: {}", max_magnitude);

}

// Simple FreeRTOS task pinned to a CPU core
// TASK A 50 ms.
// Performs a simple accumulator workload.
extern "C" fn my_task_a(_arg: *mut c_void) {
    unsafe {
        let period_ms: u32 = 50;
        let period_ticks: TickType_t = ms_to_ticks(period_ms);
        let period_us: i64 = (period_ms as i64) * 1000;

        // --- RTOS tick alignment ---
        let mut last_wake = xTaskGetTickCount();
        xTaskDelayUntil(&mut last_wake, period_ticks);

        let mut last_start_us = esp_timer_get_time();

        // --- Simulation state ---
        let mut phase: f32 = 0.0;
        let mut drift: f32 = 0.0;

        // --- Low-pass filter state variables ---
        let mut ax_f: f32 = 0.0;
        let mut ay_f: f32 = 0.0;
        let mut az_f: f32 = 9.81; // gravity baseline

        let alpha: f32 = 0.2; // filter strength

        loop {
            let expected_us = last_start_us + period_us;

            xTaskDelayUntil(&mut last_wake as *mut TickType_t, period_ticks);

            let actual_us = esp_timer_get_time();
            let jitter_us = actual_us - expected_us;
            last_start_us = actual_us;

            let core = xTaskGetCoreID(core::ptr::null_mut());

            // ----- Accelerometer Simulation -----
            phase += 0.05;
            if phase > 2.0 * core::f32::consts::PI {
                phase -= 2.0 * core::f32::consts::PI;
            }

            drift += 0.0002;
            if drift > 0.2 {
                drift = -0.2;
            }

            let ax = 9.81 * phase.sin() + drift;
            let ay = 9.81 * phase.cos();
            let az = 9.81 + (phase * 0.1).sin() * 0.1;

            // --- PRNG vibration noise ---
            let seed = (actual_us as u32)
                .wrapping_mul(1664525)
                .wrapping_add(1013904223);

            let noise_x = (((seed >> 8) & 0xFF) as f32 / 255.0 - 0.5) * 0.02;
            let noise_y = (((seed >> 16) & 0xFF) as f32 / 255.0 - 0.5) * 0.02;
            let noise_z = (((seed >> 24) & 0xFF) as f32 / 255.0 - 0.5) * 0.02;

            let ax_raw = ax + noise_x;
            let ay_raw = ay + noise_y;
            let az_raw = az + noise_z;

            // ====== Add Low-Pass Filtering ======
            ax_f = alpha * ax_raw + (1.0 - alpha) * ax_f;
            ay_f = alpha * ay_raw + (1.0 - alpha) * ay_f;
            az_f = alpha * az_raw + (1.0 - alpha) * az_f;

            println!(
                "Task A accel (core {}): RAW: ax={:.2} ay={:.2} az={:.2} | FILTERED: ax={:.2} ay={:.2} az={:.2} | jitter={} us",
                core, ax_raw, ay_raw, az_raw, ax_f, ay_f, az_f, jitter_us
            );

            if actual_us > expected_us + period_us {
                println!("Task A MISSED DEADLINE!");
            }
        }
    }
}

// TASK B 100 ms FFT workload
extern "C" fn my_task_b(_arg: *mut c_void) {
    unsafe {
        let period_ms: u32 = 120;
        let period_ticks: TickType_t = ms_to_ticks(period_ms);
        let period_us: i64 = (period_ms as i64) * 1000;

        // Anchor RTOS time
        let mut last_wake_time: TickType_t = xTaskGetTickCount();

        // Align to first wake
        xTaskDelayUntil(&mut last_wake_time as *mut TickType_t, period_ticks);
        let mut expected_us: i64 = esp_timer_get_time();  // this is actual first wake

        loop {
            //------------------------------------------------
            // 1. Compute expected start time for THIS cycle
            //------------------------------------------------
            expected_us += period_us;  // expected start of current period

            //------------------------------------------------
            // 2. Sleep until the next RTOS period boundary
            //------------------------------------------------
            xTaskDelayUntil(&mut last_wake_time as *mut TickType_t, period_ticks);

            //------------------------------------------------
            // 3. Measure actual wake time and jitter
            //------------------------------------------------
            let actual_us: i64 = esp_timer_get_time();
            let jitter_us: i64 = actual_us - expected_us;

            println!("Task B jitter: {} us", jitter_us);

            //------------------------------------------------
            // 4. Do the FFT workload
            //------------------------------------------------
            fft();

            let core = xTaskGetCoreID(core::ptr::null_mut());
            println!("Task B running on core {}", core);

            //------------------------------------------------
            // 5. Deadline check (optional)
            //    Here we define deadline as end of this period
            //------------------------------------------------
            let deadline_us = expected_us + period_us;
            let missed = actual_us > deadline_us;
            if missed {
                println!("Task B MISSED DEADLINE!");
            }
        }
    }
}
// Just simulate a temperature sensor read every 250 ms with filtering
extern "C" fn my_task_c(_arg: *mut c_void) {
    unsafe {
        let period_ms: u32 = 250;
        let period_ticks: TickType_t = ms_to_ticks(period_ms);
        let period_us: i64 = (period_ms as i64) * 1000;

        // 1. Anchor to RTOS tick timing and align first wake
        let mut last_wake_time: TickType_t = xTaskGetTickCount();
        xTaskDelayUntil(&mut last_wake_time as *mut TickType_t, period_ticks);

        // 2. Record the time of this first actual start
        let mut last_start_us: i64 = esp_timer_get_time();

        // 3. Temperature filter state
        let mut filtered_temp: f32 = 25.0;       // start at 25°C
        let alpha: f32 = 0.1;                    // low-pass strength

        // 4. Simple simulation phase for slow drift
        let mut sim_phase: f32 = 0.0;

        loop {
            //--------------------------------------------------------
            // A. Compute expected start time for THIS cycle
            //--------------------------------------------------------
            let expected_us: i64 = last_start_us + period_us;

            //--------------------------------------------------------
            // B. Sleep until the next RTOS period boundary
            //--------------------------------------------------------
            xTaskDelayUntil(&mut last_wake_time as *mut TickType_t, period_ticks);

            //--------------------------------------------------------
            // C. Measure actual wake time + jitter
            //--------------------------------------------------------
            let actual_us: i64 = esp_timer_get_time();
            let jitter_us: i64 = actual_us - expected_us;
            last_start_us = actual_us; // anchor next cycle

            let core = xTaskGetCoreID(core::ptr::null_mut());
            println!("Task C jitter: {} us (core {})", jitter_us, core);

            //--------------------------------------------------------
            // D. Simulate temperature sensor
            //--------------------------------------------------------

            // 1) Slow environmental drift using a very low frequency sine
            let dt_s: f32 = (period_us as f32) / 1_000_000.0; // 0.25s
            let drift_freq_hz: f32 = 0.005;                   // 200 s period
            sim_phase += 2.0 * core::f32::consts::PI * drift_freq_hz * dt_s;
            if sim_phase > 2.0 * core::f32::consts::PI {
                sim_phase -= 2.0 * core::f32::consts::PI;
            }

            let base_temp: f32 = 25.0 + 2.0 * sim_phase.sin(); // 25 ±2 °C over time

            // 2) Pseudo-random noise based on timer value (no rand crate needed)
            let prn_seed: u32 = (actual_us as u32)
                .wrapping_mul(1103515245)
                .wrapping_add(12345);
            let noise_unit: f32 = ((prn_seed >> 16) & 0xFF) as f32 / 255.0; // 0..1
            let noise: f32 = (noise_unit - 0.5) * 0.5; // ~ -0.25..+0.25 °C

            let temp_raw: f32 = base_temp + noise;

            // 3) Low-pass IIR filter
            filtered_temp = alpha * temp_raw + (1.0 - alpha) * filtered_temp;

            println!(
                "Task C temp: raw={:.2} °C filtered={:.2} °C",
                temp_raw, filtered_temp
            );

            //--------------------------------------------------------
            // E. Deadline check (optional)
            //--------------------------------------------------------
            let deadline_us: i64 = expected_us + period_us;
            if actual_us > deadline_us {
                println!("Task C MISSED DEADLINE!");
            }
        }
    }
}



fn main() {
    println!("Starting pinned task example...");


    unsafe {
        xTaskCreatePinnedToCore(
            Some(my_task_a),
            b"my_task\0".as_ptr(),       // name as *const u8
            4096,                        // stack size
            core::ptr::null_mut(),       // no arg
            16,                           // priority
            core::ptr::null_mut(),       // no handle
            1,                           // <--- PIN TASK TO CORE 1
        );

        xTaskCreatePinnedToCore(
         Some(my_task_b),
         b"my_task \0".as_ptr(),
         8192,
         core::ptr::null_mut(),
         12, 
         core::ptr::null_mut(), 
         0);


         xTaskCreatePinnedToCore(
         Some(my_task_c),
         b"my_task \0".as_ptr(),
         4096,
         core::ptr::null_mut(),
         8, 
         core::ptr::null_mut(), 
         0);

        xTaskCreatePinnedToCore(
            Some(gps_task),
            b"gps_task\0".as_ptr(),
            4096,
            core::ptr::null_mut(),
            13,
            core::ptr::null_mut(),
            1);

    
    }


     
    loop {
        std::thread::sleep(Duration::from_secs(5));
        println!("Main task alive...");
    }
}