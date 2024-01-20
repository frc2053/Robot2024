// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angular_acceleration.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>

#include "str/Units.h"

namespace constants {

namespace intake {
inline constexpr int INTAKE_CAN_ID = 19;
}  // namespace intake

namespace shooter {
struct ShooterGains {
  units::radial_ka_unit_t kA{0};
  frc::DCMotor::radians_per_second_per_volt_t kV{0};
  units::volt_t kS{0};
  units::radian_volt_kp_unit_t kP{0};
  units::radian_volt_ki_unit_t kI{0};
  units::radian_volt_kd_unit_t kD{0};
};

inline constexpr int LEFT_SHOOTER_CAN_ID = 17;
inline constexpr int RIGHT_SHOOTER_CAN_ID = 18;

inline constexpr units::scalar_t SHOOTER_RATIO = 1.0;
inline constexpr units::kilogram_square_meter_t SHOOTER_MOI = 0.001_kg_sq_m;
inline constexpr units::radians_per_second_t SHOOTER_TOLERANCE = 1_rad_per_s;
}  // namespace shooter

namespace swerve {

struct ModuleDriveGains {
  units::linear_ka_unit_t kA{0};
  units::linear_kv_unit_t kV{0};
  units::volt_t kS{0};
  units::scalar_t kP{0};
  units::scalar_t kI{0};
  units::scalar_t kD{0};

  bool operator==(const ModuleDriveGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const ModuleDriveGains& rhs) const {
    return !operator==(rhs);
  }
};

struct ModuleSteerGains {
  units::radial_ka_unit_t kA{0};
  frc::DCMotor::radians_per_second_per_volt_t kV{0};
  units::volt_t kS{0};
  units::scalar_t kP{0};
  units::scalar_t kI{0};
  units::scalar_t kD{0};

  bool operator==(const ModuleSteerGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const ModuleSteerGains& rhs) const {
    return !operator==(rhs);
  }
};

inline constexpr ModuleDriveGains driveGains{units::linear_ka_unit_t{0.0},
                                             units::linear_kv_unit_t{0.0},
                                             3_V,
                                             10.0,
                                             0.0,
                                             0.05};
inline constexpr ModuleSteerGains steerGains{
    units::radial_ka_unit_t{0},
    frc::DCMotor::radians_per_second_per_volt_t{0.0},
    0.0_V,
    150,
    0.0,
    50.0};

namespace pathplanning {
inline constexpr units::scalar_t TRANSLATION_P = 10.0;
inline constexpr units::scalar_t TRANSLATION_I = 0.0;
inline constexpr units::scalar_t TRANSLATION_D = 0.0;

inline constexpr units::scalar_t ROTATION_P = 5.0;
inline constexpr units::scalar_t ROTATION_I = 0.0;
inline constexpr units::scalar_t ROTATION_D = 0.05;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    GLOBAL_THETA_CONTROLLER_CONSTRAINTS;
}  // namespace pathplanning

namespace can {
inline constexpr int FL_DRIVE = 2;
inline constexpr int FL_STEER = 3;
inline constexpr int FL_ENC = 4;

inline constexpr int FR_DRIVE = 5;
inline constexpr int FR_STEER = 6;
inline constexpr int FR_ENC = 7;

inline constexpr int BL_DRIVE = 8;
inline constexpr int BL_STEER = 9;
inline constexpr int BL_ENC = 10;

inline constexpr int BR_DRIVE = 11;
inline constexpr int BR_STEER = 12;
inline constexpr int BR_ENC = 13;

inline constexpr int IMU = 14;
}  // namespace can

namespace physical {
inline constexpr frc::DCMotor SWERVE_MOTOR{frc::DCMotor::Falcon500(1)};
inline constexpr frc::DCMotor SWERVE_MOTOR_FOC{frc::DCMotor::Falcon500FOC(1)};

inline constexpr units::meter_t WHEELBASE_LENGTH = 23.5_in;
inline constexpr units::meter_t WHEELBASE_WIDTH = 23.5_in;
inline constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 4_in;
inline constexpr units::scalar_t DRIVE_GEARING =
    (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);  // SDS L2 with 16t pinion
inline constexpr units::scalar_t STEER_GEARING = (50.0 / 14.0) * (60.0 / 10.0);
inline constexpr units::scalar_t DRIVE_STEER_COUPLING = (50.0 / 16.0);
inline constexpr units::meters_per_second_t MAX_LINEAR_SPEED =
    units::ConvertAngularVelocityToLinearVelocity(
        SWERVE_MOTOR.freeSpeed / DRIVE_GEARING, DRIVE_WHEEL_DIAMETER / 2);
inline constexpr units::meters_per_second_t MAX_LINEAR_SPEED_FOC =
    units::ConvertAngularVelocityToLinearVelocity(
        SWERVE_MOTOR_FOC.freeSpeed / DRIVE_GEARING, DRIVE_WHEEL_DIAMETER / 2);
inline constexpr units::radians_per_second_t MAX_ROTATION_SPEED = 720_deg_per_s;
inline constexpr units::radians_per_second_squared_t MAX_ROTATION_ACCEL =
    10000_deg_per_s_sq;

inline constexpr double FL_ENCODER_OFFSET = -0.468262;
inline constexpr double FR_ENCODER_OFFSET = 0.479492;
inline constexpr double BL_ENCODER_OFFSET = 0.359375;
inline constexpr double BR_ENCODER_OFFSET = 0.131104;

inline constexpr units::ampere_t SLIP_CURRENT = 400_A;

inline constexpr std::array<frc::Translation2d, 4> moduleLocations{
    frc::Translation2d{WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2}};

inline frc::SwerveDriveKinematics<4> KINEMATICS{
    moduleLocations[0], moduleLocations[1], moduleLocations[2],
    moduleLocations[3]};
}  // namespace physical
}  // namespace swerve
}  // namespace constants
