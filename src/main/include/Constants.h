// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angular_acceleration.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>

#include "str/Units.h"

namespace constants {

namespace climber {
inline constexpr int LEFT_CLIMBER_CAN_ID = 22;
inline constexpr int RIGHT_CLIMBER_CAN_ID = 23;
inline constexpr units::scalar_t CLIMBER_RATIO = 9.0 / 1.0;
inline constexpr units::meter_t CLIMBER_SPOOL_RADIUS = 1.75_in;
inline constexpr units::meter_t CLIMBER_TOLERANCE = .25_in;
inline constexpr units::kilogram_t CLIMBER_MASS = 2_lb;

struct ClimberGains {
  units::unit_t<frc::ElevatorFeedforward::ka_unit> kA{0};
  units::unit_t<frc::ElevatorFeedforward::kv_unit> kV{0};
  units::volt_t kS{0};
  units::volt_t kG{0};
  units::meter_volt_kp_unit_t kP{0};
  units::meter_volt_ki_unit_t kI{0};
  units::meter_volt_kd_unit_t kD{0};

  bool operator==(const ClimberGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kG, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const ClimberGains& rhs) const { return !operator==(rhs); }
};

inline constexpr ClimberGains GAINS{
    units::unit_t<frc::ElevatorFeedforward::ka_unit>{0.01},
    units::unit_t<frc::ElevatorFeedforward::kv_unit>{0.1},
    units::volt_t{0},
    units::volt_t{0.5},
    units::meter_volt_kp_unit_t{10.0},
    units::meter_volt_ki_unit_t{0},
    units::meter_volt_kd_unit_t{0.0}};

}  // namespace climber

namespace intake {
inline constexpr int INTAKE_CAN_ID = 19;
inline constexpr int INTAKE_TOF_SENSOR = 24;
inline constexpr units::meter_t INTAKE_SENSOR_DISTANCE = 5_in;
}  // namespace intake

namespace dunker {
inline constexpr int PIVOT_ENCODER_PORT = 1;
inline constexpr int DUNKER_CAN_ID = 20;
inline constexpr int PIVOT_DUNKER_CAN_ID = 21;

inline constexpr double PIVOT_GEAR_RATIO = 60.0;
inline constexpr units::meter_t PIVOT_ARM_LENGTH = 12_in;
inline constexpr units::kilogram_t PIVOT_MASS = 15_lb;

struct DunkerGains {
  units::unit_t<frc::ArmFeedforward::ka_unit> kA{0};
  units::unit_t<frc::ArmFeedforward::kv_unit> kV{0};
  units::volt_t kS{0};
  units::volt_t kG{0};
  units::radian_volt_kp_unit_t kP{0};
  units::radian_volt_ki_unit_t kI{0};
  units::radian_volt_kd_unit_t kD{0};

  bool operator==(const DunkerGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kG, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const DunkerGains& rhs) const { return !operator==(rhs); }
};

inline constexpr DunkerGains GAINS{
    units::unit_t<frc::ArmFeedforward::ka_unit>{0.38299},
    units::unit_t<frc::ArmFeedforward::kv_unit>{7.2702},
    units::volt_t{0.44012},
    units::volt_t{2.5792},
    units::radian_volt_kp_unit_t{64.147},
    units::radian_volt_ki_unit_t{0.0},
    units::radian_volt_kd_unit_t{0.5}};

inline constexpr units::radians_per_second_t MAX_ROTATION_SPEED = 900_deg_per_s;
inline constexpr units::radians_per_second_squared_t MAX_ROTATION_ACCEL =
    10000_deg_per_s_sq;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    PIVOT_CONTROLLER_CONSTRAINTS;

inline constexpr double DUNKER_MIN_ENCODER = 0.1;
inline constexpr double DUNKER_MAX_ENCODER = 0.7;
inline constexpr units::radian_t DUNKER_MIN_ANGLE = 0_deg;
inline constexpr units::radian_t DUNKER_MAX_ANGLE = 180_deg;

inline constexpr units::radian_t DUNKER_PIVOT_ANGLE_TOLERANCE = 1_deg;
inline constexpr units::radians_per_second_t DUNKER_PIVOT_VEL_TOLERANCE =
    1_deg_per_s;
inline constexpr units::radian_t DUNKER_OUT_ANGLE = 120_deg;
inline constexpr units::radian_t DUNKER_IN_ANGLE = 1_deg;
}  // namespace dunker

namespace shooter {
struct ShooterGains {
  units::unit_t<frc::SimpleMotorFeedforward<units::radians>::ka_unit> kA{0};
  units::unit_t<frc::SimpleMotorFeedforward<units::radians>::kv_unit> kV{0};
  units::volt_t kS{0};
  units::radian_volt_kp_unit_t kP{0};
  units::radian_volt_ki_unit_t kI{0};
  units::radian_volt_kd_unit_t kD{0};

  bool operator==(const ShooterGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const ShooterGains& rhs) const { return !operator==(rhs); }
};

inline constexpr int LEFT_SHOOTER_CAN_ID = 17;
inline constexpr int RIGHT_SHOOTER_CAN_ID = 18;

inline constexpr ShooterGains GAINS{
    units::unit_t<frc::SimpleMotorFeedforward<units::radians>::ka_unit>{
        0.028843},
    units::unit_t<frc::SimpleMotorFeedforward<units::radians>::kv_unit>{
        0.11788},
    units::volt_t{0.012179},
    units::radian_volt_kp_unit_t{0.17882},
    units::radian_volt_ki_unit_t{0},
    units::radian_volt_kd_unit_t{0}};

inline constexpr units::scalar_t SHOOTER_RATIO = 1.0;
inline constexpr units::kilogram_square_meter_t SHOOTER_MOI = 0.001_kg_sq_m;
inline constexpr units::radians_per_second_t SHOOTER_TOLERANCE = 1_rad_per_s;
inline constexpr units::radians_per_second_t SHOOTER_SPEED = 6000_rpm;
inline constexpr units::radians_per_second_t SHOOTER_DUNK_SPEED = 2000_rpm;
}  // namespace shooter

namespace swerve {

struct ModuleDriveGains {
  units::unit_t<frc::SimpleMotorFeedforward<units::meters>::ka_unit> kA{0};
  units::unit_t<frc::SimpleMotorFeedforward<units::meters>::kv_unit> kV{0};
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
  units::unit_t<frc::SimpleMotorFeedforward<units::radians>::ka_unit> kA{0};
  units::unit_t<frc::SimpleMotorFeedforward<units::radians>::kv_unit> kV{0};
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

inline constexpr ModuleDriveGains driveGains{
    units::unit_t<frc::SimpleMotorFeedforward<units::meters>::ka_unit>{0.0},
    units::unit_t<frc::SimpleMotorFeedforward<units::meters>::kv_unit>{0.0},
    3_V,
    10.0,
    0.0,
    0.05};
inline constexpr ModuleSteerGains steerGains{
    units::unit_t<frc::SimpleMotorFeedforward<units::radians>::ka_unit>{0},
    units::unit_t<frc::SimpleMotorFeedforward<units::radians>::kv_unit>{0.0},
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

inline constexpr units::meter_t WHEELBASE_LENGTH = 22.75_in;
inline constexpr units::meter_t WHEELBASE_WIDTH = 22.75_in;
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
inline constexpr units::ampere_t SUPPLY_CURRENT_LIMIT = 60_A;

inline constexpr std::array<frc::Translation2d, 4> moduleLocations{
    frc::Translation2d{WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2}};

inline frc::SwerveDriveKinematics<4> KINEMATICS{
    moduleLocations[0], moduleLocations[1], moduleLocations[2],
    moduleLocations[3]};
}  // namespace physical

namespace automation {
inline constexpr frc::Translation2d BLUE_ALLIANCE_GOAL{0.5403_m, 5.419_m};
inline constexpr frc::Translation2d RED_ALLIANCE_GOAL{16_m, 5.419_m};
inline constexpr units::meter_t GOOD_DISTANCE_FOR_SHOOTER = 70_in;
inline constexpr units::meter_t SAFE_ZONE_LINE_BLUE = 2.196_m;
inline constexpr units::meter_t SAFE_ZONE_LINE_RED = 14.26_m;
}  // namespace automation
}  // namespace swerve

namespace vision {
inline constexpr std::string_view kCameraName{"apriltagcam"};
inline const frc::Transform3d kRobotToCam{
    frc::Translation3d{12.996546_in, -2.25_in, 10.937585_in},
    frc::Rotation3d{0_rad, -45_deg, 0_rad}};
inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};
}  // namespace vision

namespace ascope {
inline const frc::Pose3d kdunkerWheelsOrigin{
    -5.169876_in, 0_in, 19.425060_in, frc::Rotation3d{90_deg, 0_deg, -90_deg}};
inline const frc::Pose3d kdunkerBottomArmOrigin{
    5.519653_in, 0_in, 16.216243_in, frc::Rotation3d{0_deg, 90_deg, 90_deg}};
inline const frc::Pose3d kdunkerTopArmOrigin{
    9.029134_in, 0_in, 19.032899_in, frc::Rotation3d{0_deg, 90_deg, 90_deg}};
}  // namespace ascope
}  // namespace constants
