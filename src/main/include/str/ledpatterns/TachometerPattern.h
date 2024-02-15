// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <algorithm>
#include <functional>
#include <vector>

#include "frc/AddressableLED.h"
#include "str/LedPattern.h"

class TachometerPattern : public LedPattern {
 public:
  TachometerPattern(std::function<double()> speed, double maxSpeed,
                    int sectionLength)
      : LedPattern(sectionLength), speedMulti(speed), maxSpeedScale(maxSpeed) {
    int idx = sectionLength + ((0.0 - sectionLength) / (maxSpeedScale - 0)) *
                                  (speedMulti() - 0);
    std::vector<frc::AddressableLED::LEDData> gradient;
    for (size_t i = 0; i < buffer.size(); i++) {
      int hue = 80 + ((0 - 80) / (sectionLength - 0)) * (i - 0);
      frc::AddressableLED::LEDData rgb;
      rgb.SetHSV(hue, 255, 255);
      gradient.push_back(rgb);
    }
    std::copy(gradient.begin(), gradient.end() - idx, buffer.begin());
  }
  ~TachometerPattern() {}
  const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
    return LedPattern::buffer;
  }
  void Periodic() override {
    int idx = size + ((0.0 - size) / (maxSpeedScale - 0)) * (speedMulti() - 0);
    std::vector<frc::AddressableLED::LEDData> gradient;
    for (size_t i = 0; i < buffer.size(); i++) {
      int hue = 0 + ((80 - 0) / (size - 80)) * (i - 80);
      frc::AddressableLED::LEDData rgb;
      rgb.SetHSV(hue, 255, 255);
      gradient.push_back(rgb);
    }
    std::copy(gradient.begin(), gradient.end() - idx, buffer.begin());
  }

 private:
  std::function<double()> speedMulti = 0;
  double maxSpeedScale = 0;
};
