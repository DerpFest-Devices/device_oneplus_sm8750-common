/*
 * Copyright (C) 2021-2024 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/vendor/lineage/oplus_als/BnAreaCapture.h>
#include <android/hardware/sensors/2.1/types.h>

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {
namespace qsh_wrapper {

class AlsCorrection {
  public:
    static void init();
    static void process(Event& event);
};

}  // namespace qsh_wrapper
}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
