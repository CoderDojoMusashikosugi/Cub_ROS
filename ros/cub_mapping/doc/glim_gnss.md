# Cub4 GLIM GNSS mapping

`cub_mapping_gnss` is a GLIM `ExtensionModuleROS2` shared library. It reads
`/um982_node/fix_secondary` through GLIM's generic subscription API, including
direct `glim_rosbag` playback. It adds position constraints to the **global
submap graph**, not the local LIO estimator.

## Build and run

GLIM, GTSAM (with GPSFactorArm), GeographicLib, nlohmann-json and libmetis-dev
development files are required. The normal `cub_mapping` build enables the
extension. On machines without GLIM, use `-DCUB_MAPPING_BUILD_GNSS=OFF` when
building just the existing mapping tools.

```bash
./docker/run_in_container.sh 'cd ~/colcon_ws && colcon build --packages-select cub_mapping cub4_bringup --symlink-install --cmake-args -DCUB_MAPPING_BUILD_GNSS=ON -DBUILD_TESTING=ON'
./docker/run_in_container.sh 'ros2 launch cub4_bringup glim_velodyne.launch.py rosbag:=/home/cub/rosbag2_2026_10_17-15_09_06_no_images'
```

Launch arguments:

* `enable_gnss:=true` (default): calibrate then map in ENU.
* `enable_gnss:=false`: original single-pass LIO mapping in its local frame.
* `calibration_duration:=60`: initial seconds, starting at `start_offset`, to
  estimate yaw and translation. At least 20 matched fixes and 10 m of horizontal
  extent are required. Calibration failure stops the launch; it never silently
  labels an unaligned map as ENU. Increase the interval if initially stationary.
* `gnss_config:=/path/to/config.json`: replace GNSS settings. Otherwise use
  `config_cub_gnss.json` in `config_path`.
* Existing `rosbag`, `dump_path`, `start_offset`, `playback_duration` arguments
  keep their meanings. The calibration interval is capped by playback duration.

The launch copies the input configurations into the output directory; source
configuration files are not modified. First it processes a short interval
without GNSS factors and fits `T_enu_local` using matched antenna positions.
It then replays the requested interval from the same start with this transform
as its initial global pose and GNSS factors enabled. Alignment uses yaw and
translation, not scale or roll/pitch, with iterative outlier rejection.

The second pass disables **global** IMU edges because their velocity states in
this GLIM implementation assume the original LIO world frame. IMU remains active
in odometry and submapping. A first-pose roll/pitch prior (0.02 rad) preserves
LIO's vertical; its yaw and position priors are weak (10 rad, 1000 m).
Adjacent submaps also receive LIO relative-pose factors
(`create_between_factors=true`, `between_registration_type=NONE`, GLIM's fixed
precision 1e6). These preserve connectivity through GNSS outages when global
IMU edges are absent. Point-cloud matching alone allowed large trajectory seams
in testing and must not be used as the only relative constraint in this setup.
Global first-pose damping is reduced from 1e10 to 0.01 so GNSS can correct drift.
This implementation targets this launch's VGICP submap graph, not arbitrary
GLIM backends or merging multiple independent mapping sessions.

## Coordinates and antenna

The map is ENU (east, north, up), based on WGS84 ellipsoid origin
`[36.082628231, 140.076144739, 67.078]` (degrees, degrees, metres).
GeographicLib `LocalCartesian` performs the conversion.

The current `cub_um982` driver publishes GGA/GGAH altitude directly. For this
recorded bag the configured input is `altitude_reference: msl`. The plugin adds
the user-provided GSI Geoid 2024 value **40.1473 m** (reference correction 0 m)
before converting to ENU. The corresponding origin elevation is 26.9307 m.
This is a constant local-area approximation, not an exact reconstruction of
the receiver's internal geoid model. If a driver already publishes ellipsoid
height, set `altitude_reference: ellipsoid` to avoid double correction.

Right-hand antenna (looking forward from behind the robot) is assumed to be
ANT2. From `cub_description/urdf/cub4.urdf`:

* `p_base_antenna = [-0.12, -0.10, 0.5857] m`
* `p_base_imu = [-0.030, 0, 0.2034] m`, IMU roll = **3.14**, not exact pi
* `p_imu_antenna = R_base_imu.transpose() * (p_base_antenna - p_base_imu)`
  = approximately `[-0.09, 0.100608744, -0.382140250] m`.

Only `gnss_secondary_link` (old bag/driver) and `gnss_sub_link` (URDF) are
accepted. This is an explicit alias, not a lookup in a running TF tree. Change
the lever arm configuration if antenna placement changes. Physical ANT2 wiring
has not been verified by this software change.

## Factor and selection

`STATUS_GBAS_FIX == 2` is accepted **specifically because cub_um982 maps only
GGA quality 4 (RTK FIX) to that value**. Standard NavSatStatus does not generally
guarantee RTK FIX; this assumption must be revisited for another driver.
FLOAT, standalone and invalid fixes never add a factor. No primary antenna
position or heading is fused.

For a GNSS timestamp, interpolate position and orientation between neighbouring
optimized IMU poses within one submap (no extrapolation, maximum gap 0.25 s).
Convert the antenna point into the submap origin coordinates, then constrain:

```
residual = T_enu_submap * p_submap_antenna(t) - p_enu_gnss(t)
```

This is the `GPSFactorArm` residual, using an effective arm that includes both
antenna mounting and motion within the submap. `GnssArmFactor` delegates its
error and Jacobian to GTSAM's GPSFactorArm and adds Boost serialization, missing
from the installed upstream class. The library must remain available when
loading the saved graph; GLIM offline viewer loads extensions from saved config.

Observation handling:

* Header stamp plus `time_offset_s` (default 0). This driver stamps reception;
  the configured LiDAR -1 s correction is independent.
* At most one accepted sample per second; no reuse of old FIX during outages.
* Reject non-finite/out-of-range coordinates, wrong frames, duplicate or
  backward timestamps, invalid variances and excessive uncertainty.
* Diagonal covariance with floors 0.15 m horizontal / 0.30 m vertical. Incoming
  covariance is an HDOP approximation; off-diagonal terms are not used.
* Huber loss k=1.345 and a 20 m initial residual gate. The gate can reject
  reacquired fixes if drift exceeds 20 m; inspect its counter before tuning.
* `information_scale=1000` balances GNSS against this configuration's summed
  VGICP point costs. It multiplies the entire robust cost; the Huber transition
  in physical measurement units stays the same. This is an empirical graph-cost
  balance, not a claim that the GNSS uncertainty is 32 times smaller. Recheck
  this scale when changing submap sampling, registration factors or hardware.
* On optimization updates without new submap states (including the save-time
  update), perform `settle_iterations=20` additional iSAM2 updates. The ENU graph
  uses a 0.01 relinearization threshold to settle after loop corrections.
* Matching is retried as submaps/fixes arrive. The save-time optimization drains
  remaining matched observations, including fixes received after a submap.
* Unmatched observations (e.g. LIO initialization or gaps between submaps) are
  counted. They are not attached to an unrelated pose.

## Outputs and checks

The final GLIM submaps, `traj_imu.txt` and `traj_lidar.txt` are in ENU.
`odom_*.txt` remain the original local odometry. Additional outputs:

* `gnss_report.json`: exact settings, coordinate origin, counts and final residual RMSE.
* `gnss_residuals.csv`: matched measurements, residuals and whether used.
* `calibration/gnss_report.json`: `T_enu_local`, alignment RMSE and inlier count.
* `calibration_config/`, `mapping_config/`, `config/`: reproducibility settings.

```bash
./docker/run_in_container.sh 'cd ~/colcon_ws && colcon test --packages-select cub_mapping --ctest-args -R test_gnss --output-on-failure'
./docker/run_in_container.sh 'ros2 run cub_mapping check_gnss_dump /home/cub/maps/OUTPUT'
```

`check_gnss_dump` restores `graph.bin` and `values.bin`, verifies the GNSS factor
count against the report and evaluates restored residuals. GNSS residual is a
consistency check against the fused observations, not independent map accuracy.
Existing LIO IMU-prediction warnings require separate IMU calibration work.

## Recorded-bag validation (2026-10-30)

Container builds and four GNSS unit/integration tests passed. Checks include
geoid sign, lever-arm Jacobian, serialization, yaw alignment, RTK selection,
wrong frames, duplicate fixes, late fixes and the robust information scale.
GNSS-disabled playback and failure on insufficient alignment data were checked.

The final launch configuration was replayed from scratch for 180 seconds:
164 factors, residual RMSE 0.153 m, all factors restored in another process.
The complete 2109-second bag was also processed. Its saved graph was subsequently
reloaded, given the same adjacent LIO relative-pose factors now enabled in the
launch, and optimized for 40 updates. This full-graph validation is distinct from
a fresh full-bag replay of the final continuity configuration.

Full-graph result: 446 submaps, 20,925 trajectory frames, 8,920,000 saved points,
1,769 restored GNSS factors. Of 21,113 GNSS messages, 2,052 non-FIX messages were
excluded; 39 decimated observations could not be matched to a submap interval.
Residual RMSE to fused RTK was 1.159 m (95th percentile 1.501 m). Existing GNSS-free-map
consistency was about 19.6 m after initial rigid alignment. These comparisons
do not measure independent absolute accuracy.

There are still trajectory discontinuities: the largest 0.1-second step in the
full-graph result is 1.23 m. Initial LIO IMU warnings and end-of-bag time-offset
warnings also persist. The implementation and factor persistence are validated;
the resulting full map still needs LIO timing/extrinsic/IMU calibration and
geometric inspection before precision-map use. A small GNSS residual alone is
not an acceptance criterion for map continuity.

Artifacts on the development machine are under
`/home/cub/glim_velodyne_work/gnss_continuous` (host: `docker/home/…`), including
`map.pcd`, `map.ply`, `gnss_validation.png`, and `validation_summary.json`.
