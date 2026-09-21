# Third-party notices

This project is licensed under the Apache License 2.0 (see `LICENSE`). It incorporates or
depends on the following third-party software.

## WPILib `TrapezoidProfile` (vendored, modified) — BSD 3-Clause

`src/third_party/wpilib/trapezoid_profile.hpp` is derived from the WPILib
`TrapezoidProfile` class (`wpimath/src/main/native/include/wpi/math/trajectory/TrapezoidProfile.hpp`
in https://github.com/wpilibsuite/allwpilib).

Copyright (c) 2009-2026 FIRST and other WPILib contributors. All rights reserved.
Licensed under the WPILib BSD license; the full text is in `src/third_party/wpilib/LICENSE.md`.

Modifications: the `wpi::units` dimensional types are replaced by plain `double`s and the
struct-serialization support and runtime constraint exceptions are removed. The algorithm is
otherwise unchanged. It provides the closed-form trapezoidal motion profile behind the module's
cyclic-synchronous-position (CSP) trajectory generator (`src/viam/lib/trapezoid.hpp`).

## SOEM — Simple Open EtherCAT Master (fetched at build time)

https://github.com/OpenEtherCATsociety/SOEM, pinned by commit in `cmake/soem.cmake`. Licensed
under the GNU General Public License v2 with the SOEM linking exception; see the SOEM
repository's `LICENSE` for the exact terms.

## Viam C++ SDK (fetched at build time)

https://github.com/viamrobotics/viam-cpp-sdk, Apache License 2.0.
