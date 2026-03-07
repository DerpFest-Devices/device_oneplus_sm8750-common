/*
 * Copyright (C) 2021-2024 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "AlsCorrection.h"

#include <android-base/properties.h>
#include <android/binder_manager.h>
#include <binder/IBinder.h>
#include <binder/IServiceManager.h>
#include <log/log.h>
#include <utils/Timers.h>
#include <cmath>
#include <fstream>

using aidl::vendor::lineage::oplus_als::AreaRgbCaptureResult;
using aidl::vendor::lineage::oplus_als::IAreaCapture;
using android::base::GetBoolProperty;
using android::base::GetIntProperty;
using android::base::GetProperty;

#define ALS_CALI_DIR "/proc/sensor/als_cali/"
#define BRIGHTNESS_DIR "/sys/class/backlight/panel0-backlight/"

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {
namespace qsh_wrapper {

static const std::string rgbw_max_lux_paths[4] = {
        ALS_CALI_DIR "red_max_lux",
        ALS_CALI_DIR "green_max_lux",
        ALS_CALI_DIR "blue_max_lux",
        ALS_CALI_DIR "white_max_lux",
};

struct als_config {
    float rgbw_max_lux[4];
    float grayscale_weights[3];
    float sensor_inverse_gain[4];
    float calib_gain;
    float bias;
    float max_brightness;
    float brightness_breakpoint;
    float brightness_poly0[3];
    float brightness_poly1[3];
    bool use_brightness_curve;
};

static struct {
    float middle;
    float min, max;
} hysteresis_ranges[] = {
        {0, 0, 4},
        {7, 1, 12},
        {15, 5, 30},
        {30, 10, 50},
        {360, 25, 700},
        {1200, 300, 1600},
        {2250, 1000, 2940},
        {4600, 2000, 5900},
        {10000, 4000, 80000},
        {HUGE_VALF, 8000, HUGE_VALF},
};

static struct {
    nsecs_t last_update, last_forced_update;
    bool force_update;
    float hyst_min, hyst_max;
    float last_corrected_value;
} state = {
        .last_update = 0,
        .force_update = true,
        .hyst_min = -1.0,
        .hyst_max = -1.0,
};

static als_config conf;
static std::shared_ptr<IAreaCapture> service;

template <typename T>
static T get(const std::string& path, const T& def) {
    std::ifstream file(path);
    T result;

    file >> result;
    return file.fail() ? def : result;
}

void AlsCorrection::init() {
    std::istringstream is;

    conf.bias = GetIntProperty("vendor.sensors.als_correction.bias", 0);
    is = std::istringstream(GetProperty("vendor.sensors.als_correction.grayscale_weights", ""));
    is >> conf.grayscale_weights[0] >> conf.grayscale_weights[1] >> conf.grayscale_weights[2];
    is = std::istringstream(GetProperty("vendor.sensors.als_correction.sensor_inverse_gain", ""));
    is >> conf.sensor_inverse_gain[0] >> conf.sensor_inverse_gain[1] >>
            conf.sensor_inverse_gain[2] >> conf.sensor_inverse_gain[3];

    // There are two options of mapping a display brightness value to a screen illuminance (lux) for
    // fullwhite content.
    // 1. If the display illuminance measured by ALS is directly proportional to the panel
    // brightness value, only rgbw_max_lux needs to be set in props. The ratio of the current
    // brightness over max brightness will be used to scale the rgbw_max_lux as well.
    is = std::istringstream(GetProperty("vendor.sensors.als_correction.rgbw_max_lux", ""));
    is >> conf.rgbw_max_lux[0] >> conf.rgbw_max_lux[1] >> conf.rgbw_max_lux[2] >>
            conf.rgbw_max_lux[3];
    // 2. If the display illuminance does not increase linearly over the panel brightness, and after
    // a certain brightness, the curve changes abruptly (often caused by the HBM threshold), set
    // the breakpoint value and the two sets of polynomial coefficients that describe the curve
    // before and after the breakpoint. poly0 is used if the current brightness is lower than the
    // breakpoint. Otherwise, poly1 is used. poly1 can be made optional when the breakpoint is
    // higher than the max panel brightness. This is useful when the brightness to lux curve is
    // non-linear, but there is no obvious breakpoint and one curve is enough. The current
    // implementation supports two second-order polynomials. Set the first coefficient to 0.0 if a
    // linear mapping is enough.
    is = std::istringstream(GetProperty("vendor.sensors.als_correction.brightness_breakpoint", ""));
    is >> conf.brightness_breakpoint;
    is = std::istringstream(GetProperty("vendor.sensors.als_correction.brightness_poly0", ""));
    is >> conf.brightness_poly0[0] >> conf.brightness_poly0[1] >> conf.brightness_poly0[2];
    is = std::istringstream(GetProperty("vendor.sensors.als_correction.brightness_poly1", ""));
    is >> conf.brightness_poly1[0] >> conf.brightness_poly1[1] >> conf.brightness_poly1[2];

    conf.use_brightness_curve = conf.brightness_breakpoint != 0.0;

    if (!conf.use_brightness_curve) {
        for (int i = 0; i < 4; i++) {
            float max_lux = get(rgbw_max_lux_paths[i], 0.0);
            if (max_lux != 0.0) {
                conf.rgbw_max_lux[i] = max_lux;
            }
        }
        ALOGI("Display maximums: R=%.0f G=%.0f B=%.0f W=%.0f", conf.rgbw_max_lux[0],
              conf.rgbw_max_lux[1], conf.rgbw_max_lux[2], conf.rgbw_max_lux[3]);
    }

    float row_coe = get(ALS_CALI_DIR "row_coe", 0.0);
    if (row_coe != 0.0) {
        conf.sensor_inverse_gain[0] = row_coe / 1000.0;
    }

    float cali_coe = get(ALS_CALI_DIR "cali_coe", 0.0);
    conf.calib_gain = cali_coe > 0.0 ? cali_coe / 1000.0 : 1.0;
    ALOGI("Calibrated sensor gain: %.2fx", 1.0 / (conf.calib_gain * conf.sensor_inverse_gain[0]));

    conf.max_brightness = get(BRIGHTNESS_DIR "max_brightness", 1023.0);

    for (auto& range : hysteresis_ranges) {
        range.min /= conf.calib_gain * conf.sensor_inverse_gain[0];
        range.max /= conf.calib_gain * conf.sensor_inverse_gain[0];
    }
    hysteresis_ranges[0].min = -1.0;

    const auto instancename = std::string(IAreaCapture::descriptor) + "/default";

    if (AServiceManager_isDeclared(instancename.c_str())) {
        service = IAreaCapture::fromBinder(
                ::ndk::SpAIBinder(AServiceManager_waitForService(instancename.c_str())));
    } else {
        ALOGE("Service is not registered");
    }
}

void AlsCorrection::process(Event& event) {
    static AreaRgbCaptureResult screenshot = {0.0, 0.0, 0.0};

    ALOGV("Raw sensor reading: %.0f", event.u.scalar);
    ALOGV("hyst min: %f, max: %f", state.hyst_min, state.hyst_max);

    if (event.u.scalar > conf.bias) {
        event.u.scalar -= conf.bias;
    }

    const nsecs_t now = systemTime(SYSTEM_TIME_BOOTTIME);
    const float brightness = get(BRIGHTNESS_DIR "brightness", 0.0);

    if (state.last_update == 0) {
        state.last_update = now;
        state.last_forced_update = now;
    } else {
        if (brightness > 0.0 && (now - state.last_forced_update) > s2ns(3)) {
            ALOGV("Forcing screenshot");
            state.last_forced_update = now;
            state.force_update = true;
        }
        if ((now - state.last_update) < ms2ns(100)) {
            ALOGV("Events coming too fast, dropping");
            // TODO figure out a better way to drop events
            event.sensorHandle = 0;
            return;
        }
        state.last_update = now;
    }

    if (state.force_update ||
        ((event.u.scalar < state.hyst_min || event.u.scalar > state.hyst_max))) {
        if (service == nullptr || !service->getAreaBrightness(&screenshot).isOk()) {
            ALOGE("Could not get area above sensor");
            // TODO figure out a better way to drop events
            event.sensorHandle = 0;
            return;
        }
        ALOGV("Screen color above sensor: %f %f %f", screenshot.r, screenshot.g, screenshot.b);

        const float screen_lux = [&]() {
            const float grayscale = screenshot.r * conf.grayscale_weights[0] +
                                    screenshot.g * conf.grayscale_weights[1] +
                                    screenshot.b * conf.grayscale_weights[2];
            const float grayscale_normalized = grayscale / 255.0;

            if (conf.use_brightness_curve) {
                const auto& poly = (brightness < conf.brightness_breakpoint)
                                           ? conf.brightness_poly0
                                           : conf.brightness_poly1;

                float lux_fullwhite = 0.0;
                for (const auto coef : poly) {
                    lux_fullwhite = lux_fullwhite * brightness + coef;
                }
                const float lux = grayscale_normalized * lux_fullwhite;

                return lux;
            } else {
                const float lux_fullwhite = conf.rgbw_max_lux[3] * brightness / conf.max_brightness;
                const float lux_grayscale_gamma =
                        std::pow(grayscale_normalized, 2.2) * lux_fullwhite;

                return lux_grayscale_gamma;
            }
        }();
        ALOGV("Screen brightness: %.0f / %.0f, Estimated illuminance: %.0f", brightness,
              conf.max_brightness, screen_lux);

        const float sensor_raw_corrected = std::max(event.u.scalar - screen_lux, 0.0f);

        ALOGV("sensor_raw_corrected: %f", sensor_raw_corrected);

        const float sensor_raw_calibrated = event.u.scalar * conf.calib_gain;
        if (state.force_update || sensor_raw_calibrated < 10000.0 ||
            screen_lux <= event.u.scalar * 1.35) {
            const float sensor_corrected = sensor_raw_corrected * conf.calib_gain;
            for (const auto& range : hysteresis_ranges) {
                if (sensor_corrected <= range.middle) {
                    state.hyst_min = range.min;
                    state.hyst_max = range.max + screen_lux;
                    break;
                }
            }
            event.u.scalar = sensor_corrected;
            state.last_corrected_value = sensor_corrected;
            ALOGV("Fully corrected sensor value: %.0f lux", sensor_corrected);
        } else {
            event.u.scalar = state.last_corrected_value;
            ALOGV("Reusing cached value: %.0f lux", event.u.scalar);
        }

        state.force_update = false;
    } else {
        event.u.scalar = state.last_corrected_value;
        ALOGV("Reusing cached value: %.0f lux", event.u.scalar);
    }
}

}  // namespace qsh_wrapper
}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
