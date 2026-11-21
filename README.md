# irurueta-android-navigation-inertial
Services and views to support inertial navigation

> **Experimental and under development.** Public APIs may change between versions without notice, and some
> estimators are still being tuned. Do not rely on this library for safety-critical navigation.

[![Develop build](https://github.com/albertoirurueta/irurueta-android-navigation-inertial/actions/workflows/develop.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-android-navigation-inertial/actions/workflows/develop.yml)
[![Release build](https://github.com/albertoirurueta/irurueta-android-navigation-inertial/actions/workflows/main.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-android-navigation-inertial/actions/workflows/main.yml)

[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=code_smells)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=coverage)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)

[![Duplicated lines](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=duplicated_lines_density)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)
[![Lines of code](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=ncloc)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)

[![Maintainability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)
[![Quality gate](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=alert_status)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)
[![Reliability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)

[![Security](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=security_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)
[![Technical debt](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=sqale_index)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)
[![Vulnerabilities](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-android-navigation-inertial&metric=vulnerabilities)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-android-navigation-inertial)

[![Maven Central](https://img.shields.io/maven-central/v/com.irurueta/irurueta-android-navigation-inertial)](https://central.sonatype.com/artifact/com.irurueta/irurueta-android-navigation-inertial)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)

Android library (Kotlin) that provides services and estimators to support inertial navigation using the IMU sensors
of an Android device: accelerometer, gyroscope, magnetometer, and the derived Android gravity and rotation-vector
sensors. It builds on the pure-Java `com.irurueta:irurueta-navigation`, `irurueta-navigation-inertial` and
`irurueta-navigation-inertial-extra` libraries and adds the Android-specific part: reading sensors through
`SensorManager`, synchronizing several sensors, converting from Android's ENU device frame to NED, and running
attitude and pose estimators on the measurements in real time.

## Project Status

| Item | Value |
| --- | --- |
| Status | Experimental, under active development |
| Language | Kotlin 2.3.10 (JVM target 17) |
| Build tool | Gradle (Kotlin DSL), Android Gradle Plugin 9.0.1 |
| Current development version | `1.1.0-SNAPSHOT` |
| Latest release | `none yet` |
| Platform | Android (`minSdk` 26, `compileSdk` 36) |
| License | Apache License 2.0 |
| CI | GitHub Actions: `develop` branch builds (`develop.yml`), release builds and Maven Central publishing (`main.yml`), version sync (`sync.yml`) and security scans (`security.yml`) |
| Quality | SonarCloud, JaCoCo coverage, Android Lint |

No release has been published to Maven Central yet under `com.irurueta:irurueta-android-navigation-inertial`.

## Documentation

- [Documentation site (Antora)](https://albertoirurueta.github.io/irurueta-android-navigation-inertial)
- [API documentation (Dokka)](https://albertoirurueta.github.io/irurueta-android-navigation-inertial/api)
- [SonarCloud dashboard](https://sonarcloud.io/summary/new_code?id=albertoirurueta_irurueta-android-navigation-inertial)
- [Changelog](CHANGELOG.md)

## Installation

The library will be distributed through Maven Central. **No version has been published yet**, so the snippet below
only resolves once the first release is out; until then, build the library from source (see below). Make sure
`mavenCentral()` is among your repositories.

Latest release
```kotlin
implementation("com.irurueta:irurueta-android-navigation-inertial:1.1.0")
```

Because the library is experimental, pin an exact version and expect API changes between versions.

### Requirements

- Android `minSdk` 26.
- Java/Kotlin JVM target 17.
- A device with the sensors required by the estimator you use (at least an accelerometer; gyroscope and
  magnetometer for most attitude and pose estimators).

### Building from source

```bash
git clone https://github.com/albertoirurueta/irurueta-android-navigation-inertial.git
cd irurueta-android-navigation-inertial
./gradlew :lib:assembleRelease      # build the library
./gradlew :lib:testDebugUnitTest    # run unit tests
./gradlew :app:installDebug         # install the sample app on a connected device
```

## How It Works

Every high-level estimator follows the same pipeline: it owns one or more sensor collectors that receive Android
`SensorEvent`s, converts them into library measurements, feeds them to a processor implementing the actual
algorithm, and notifies results to a listener supplied by the application. Estimators are created with an Android
`Context`, started with `start()` and stopped with `stop()`. Attitudes are reported as quaternions (and optionally
Euler angles) respect to NED.

The library offers sensor collectors (single and synced), calibration and noise estimation, attitude estimators
(leveling, relative gyroscope, geomagnetic, fused, double-fused and Kalman-filter based), pose estimators (ECEF,
local and relative), ZUPT (zero velocity update) detection, and helper services (location, permissions, sensor
availability). A sample app in `app/` exercises every estimator on a device.

```kotlin
val estimator = GeomagneticAttitudeEstimator(
    context,
    sensorDelay = SensorDelay.FASTEST,
    attitudeAvailableListener = { _, attitude, timestamp, roll, pitch, yaw, _ ->
        // attitude is a Quaternion respect to NED; roll, pitch and yaw are in radians
    }
)

estimator.start()
// ...
estimator.stop()
```

## Contributing

The repository follows gitflow: day-to-day work happens on `develop`, and `main` only receives releases. Create
feature branches off `develop` (e.g. `feature/<issue>-<short-name>`) and open pull requests back into `develop`.
Pushes to `develop` run the build and test workflow; releases are cut from `develop` and merged into `main`, which
publishes the release.

## License

Licensed under the [Apache License 2.0](LICENSE).
