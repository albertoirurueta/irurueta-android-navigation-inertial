# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Zero-velocity update (ZUPT) support in the pose estimators and processors, backed by new per-sensor measurement
  interfaces (`WithAccelerometerSensorMeasurement`, `WithAttitudeSensorMeasurement`, `WithGravitySensorMeasurement`,
  `WithGyroscopeSensorMeasurement`, `WithMagnetometerSensorMeasurement`) implemented by the synced sensor measurements.
- Pose estimators `EcefAbsolutePoseEstimator`, `LocalPoseEstimator` and `RelativePoseEstimator`, with their family of
  attitude-, accelerometer-fused and double-fused pose processors for ECEF absolute, local and relative poses.
- Common `AttitudeEstimator` base and `SensorType` enum for attitude estimation over synced sensor collectors.
- Kalman filter based `KalmanAbsoluteAttitudeEstimator` and `KalmanRelativeAttitudeEstimator` (with their
  processors, gyroscope predictor, accelerometer/magnetometer correctors, phi matrix estimators, process noise
  covariance integrators and external acceleration covariance estimators).
- Antora documentation site under `docs/`.
- Gitflow GitHub Actions workflows: `develop` (build and tests), `main` release, `sync` and `security` scans, plus
  Dependabot configuration.

### Changed

- Refactored the library: calibrators now use internal accelerometer, gyroscope and magnetometer calibrator builders,
  and measurement generators delegate to dedicated measurement generator processors.
- Reworked attitude estimators (geomagnetic, fused, double-fused, leveling, leveled relative and relative gyroscope)
  and synced sensor collectors on top of the new shared attitude estimation base.
- Updated Gradle wrapper and dependency versions; the release workflow replaces the former `publish` workflow.

### Removed

- Numerical utilities (factorial, matrix exponential, Pade approximant, quaternion step integrators and quadrature
  integrators) that are now provided by the `irurueta-numerical` and `irurueta-navigation-inertial` libraries.
