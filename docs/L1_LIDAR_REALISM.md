# Go2 EDU L1 lidar: real behaviour and how to model it in Gazebo Harmonic

Research date: 2026-10-11. Every claim has one of these labels:

- **VERIFIED**: quoted from a primary source (Unitree manual, Unitree source, URDF, gz source, sdformat spec, ROS package source), with file and line.
- **DERIVED**: computed from VERIFIED facts. It has not been measured on a robot.
- **SECONDARY**: from community code or forums. Confidence is stated.
- **UNVERIFIED**: no source found. It must be measured on the real robot (see the checklist at the end).

Pinned source revisions (all permalinks below use these):

| Repo | Commit |
|---|---|
| gazebosim/gz-sensors, branch `gz-sensors8` (Harmonic) | `b531112370bcfcbd1a8cd012b7b0d38f8d3889b8` |
| gazebosim/gz-rendering, branch `gz-rendering8` (Harmonic) | `ed91ca54262fafaa1e9f1b44cdef3162a73f3034` |
| gazebosim/sdformat, branch `sdf14` (Harmonic) | `0af4bdba653dd90eb6c050216ef7707cb0cb6a8b` |
| ros-perception/pointcloud_to_laserscan, branch `jazzy` | `442b33258b3af50e2981f338e1eac3ba259f8afa` |
| unitreerobotics/unitree_ros | `5994d4faef0a9cadd3287f8de0199a67eeb2a259` |
| unitreerobotics/unitree_ros2 | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` |
| unitreerobotics/unitree_sdk2 | `63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36` |
| unitreerobotics/unitree_mujoco | `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` |
| unitreerobotics/unilidar_sdk (L1) | `1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6` |
| unitreerobotics/point_lio_unilidar | `18ed5976d8fab2bd8a5148c26a40692bd3c0dc91` |

The gz behaviour below comes from the `gz-sensors8` and `gz-rendering8` branch heads. Inside the Docker image, check that `dpkg -l | grep -E 'gz-sensors8|gz-rendering8'` shows a Harmonic 8.x build. These code paths have not changed in years, but the image is the binary that actually runs.

---

## Summary

| Item | Real Go2 EDU value | Confidence | Source |
|---|---|---|---|
| Lidar model in the Go2 | **L1** per Go2 User Manual V1.0. **The current unitree.com/go2 page says L2** (360x96 deg, 64,000 pts/s). Your unit must be checked. | VERIFIED that both documents say this; **UNVERIFIED for your unit** | Go2 User Manual V1.0 sec. "Intelligent Avoidance"; unitree.com/go2 |
| FOV | 360 x 90 deg hemisphere on the sensor's **+Z (dome) side** | VERIFIED | L1 User Manual v1.1 p.6 |
| Scan mechanism | High-speed mirror sweeps 180 deg vertically (180 Hz). Low-speed motor turns the core 360 deg (11 Hz). | VERIFIED | L1 UM v1.1 p.3, p.6, p.14 |
| Point rate | 43,200 samples/s, 21,600 effective points/s | VERIFIED | L1 UM v1.1 p.14 |
| Points per vertical sweep line | 120 (one packet = one line) | VERIFIED (SDK) | unilidar_sdk `HowToParse...md` L84-88, L127 |
| Time between points | 1/43,200 s (about 23 us) | VERIFIED (SDK sample output) | unilidar_sdk `unitree_lidar_sdk/README.md` L92-95 |
| Max range | PM: 20 m @90%, 10 m @10%. RM: 30 m @90%, 15 m @10%. | VERIFIED (sensor) | L1 UM v1.1 p.14 |
| PM or RM inside the Go2 | not documented anywhere I found | UNVERIFIED | none |
| Near blind zone | 0.05 m ("for reference only") | VERIFIED | L1 UM v1.1 p.14, footnote 3 |
| Accuracy / resolution | +-2.0 cm (90%, 15 m, 25 C) / 8 mm | VERIFIED | L1 UM v1.1 p.14 |
| `/utlidar/cloud` frame_id | `utlidar_lidar` | VERIFIED | unitree_ros2 README L353-359 |
| `/utlidar/cloud` per message | height 1, width about 1,395-1,405, 6 fields, point_step 32, `is_dense: true` | VERIFIED (Unitree's own screenshot, firmware of Oct 2023) | unitree_ros2 `docs/image/piFtdF1.png` |
| `/utlidar/cloud` rate | about 15 Hz (consecutive stamps 66.8 and 66.9 ms apart) | DERIVED from Unitree's screenshot; SECONDARY sources agree | piFtdF1.png; forum and repos below |
| Field names | probably x, y, z, intensity (f32), ring (u16), time (f32) | DERIVED (the SDK point type has the same 6 fields and a 32 B layout); UNVERIFIED on the Go2 | unilidar_sdk `unitree_lidar_sdk_pcl.h` L49-62 |
| `/utlidar/cloud_deskewed` | exists. Frame is the Go2's world `odom` (origin = power-on pose). About 14.7-15 Hz. | exists: VERIFIED. Frame and rate: SECONDARY (medium-high, three independent repos) | piFtteJ.png; KrepsD, Tmino1, zhaoyufei122 |
| `/utlidar/imu` | frame `utlidar_imu`. The sensor reports at 250 Hz. | frame: SECONDARY. Sensor rate: VERIFIED. Go2 topic rate: UNVERIFIED | unilidar_sdk issue #34; L1 UM p.14 |
| `/utlidar/robot_odom` | about 150 Hz | SECONDARY (one forum user) | mybotshop forum t/1789 |
| Firmware self-crop of the body and legs | none documented. Community pipelines crop it themselves. | UNVERIFIED (SECONDARY evidence of leg returns) | zhaoyufei122 CropBox; forum t/1789 |
| Mount (`radar_joint`) | parent `base`, xyz (0.28945, 0, -0.046825), rpy (0, 2.8782, 0) | VERIFIED | unitree_ros `go2_description.urdf` L748-753 |
| `radar` link is the same frame as `utlidar_lidar` | assumed by community drivers | SECONDARY / UNVERIFIED | Unitree-Go2-Robot/go2_driver `go2_driver.cpp` L132-137 |
| Sensor +Z (dome axis) in base | (0.260, 0, -0.966): 15.1 deg from straight down, tilted forward | DERIVED from the URDF | below |
| Range origin above the mounting face | 0.0445 m along +Z (`z_bias`) | VERIFIED (SDK) | unilidar_sdk `HowToParse...md` L126, L167 |
| Base height when standing | not published. 0.32-0.36 m from leg kinematics of Unitree stand poses. | UNVERIFIED | unitree_sdk2 stand example; unitree_mujoco go2.xml |
| Lidar range-origin height above ground | about h_base - 0.090, i.e. about 0.23-0.27 m | DERIVED / UNVERIFIED | below |
| Gazebo gpu_lidar near clip | `range/min` is the near clip. Hits closer than min come back as **-inf**. Geometry closer than about min/sqrt(3)..min (depends on direction) is **clipped, so the ray reports whatever is behind it**. | VERIFIED (source) | gz-sensors `GpuLidarSensor.cc` L198-202; gz-rendering `Ogre2GpuRays.cc` L1175-1182; shader L130-133 |
| Live lidar diagnostics on the robot | `/utlidar/lidar_state` carries `cloud_frequency`, `cloud_size`, `cloud_scan_num`, motor speeds, firmware version | VERIFIED (message definition) | unitree_ros2 `unitree_go/msg/LidarState.msg` L1-17 |

---

## 0. Which lidar is in your Go2 EDU? (check this first)

- The Go2 User Manual V1.0 (Unitree-authored; copy hosted by reseller Generation Robots, https://static.generation-robots.com/media/Go2-User-Manual.pdf) says: *"Equipped with Unitree's self-developed 4D LiDAR L1 with 360°×90° hemispherical ultra-wide-angle perception ... the minimum detection distance is as low as 0.05m ... (only support forward obstacle avoidance)"*. **VERIFIED.** It does not say PM or RM.
- The current product page, https://www.unitree.com/go2 (fetched 2026-10-11), says: *"Go2 features with Unitree's self-developed 4D LiDAR L2 with 360°x96°hemispherical ultra-wide recognition"*. **VERIFIED** that the page says this.
- L2 (L2 User Manual v1.1, 2024.10, https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf, p. spec table): 64,000 effective pts/s, 5.55 Hz horizontal, 216 Hz vertical, FOV 360x90 or 360x96 in "NEGA" mode, 30 m @90%, IMU 500 Hz. That is about 3x the L1 point rate, so a model tuned for the L1 would be too sparse for an L2 robot.
- SECONDARY: zhaoyufei122/go2_hokuyo_urg4_navigation (Jazzy, real Go2) talks about an "L2" mounted inverted (README L343) but also "L1 non-repetitive" (L328). Even community repos are unsure which sensor they have.

**Action:** before tuning anything, run `ros2 topic echo --once /utlidar/lidar_state` and `ros2 topic hz /utlidar/cloud` (checklist below). About 1,400 pts per message at about 15 Hz means L1. About 4,000+ pts per message means L2. The rest of this document assumes **L1**.

---

## 1. Real Go2 lidar topics, rates, frames, fields

### 1.1 Topic list (VERIFIED, Unitree's own screenshot)
unitree_ros2 README "Rviz" section ([README.md L343-366](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/README.md#L343-L366)). Its `ros2 topic list` screenshot ([docs/image/piFtteJ.png](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/docs/image/piFtteJ.png)) shows these topics:
`/utlidar/cloud`, `/utlidar/cloud_deskewed`, `/utlidar/foot_position`, `/utlidar/grid_map`, `/utlidar/height_map`, `/utlidar/height_map_array`, `/utlidar/imu`, `/utlidar/lidar_state`, `/utlidar/map_state`, `/utlidar/mapping_cmd`, `/utlidar/range_info`, `/utlidar/range_map`, `/utlidar/robot_odom`, `/utlidar/robot_pose`, `/utlidar/switch`, `/utlidar/voxel_map`, `/utlidar/voxel_map_compressed`.

SECONDARY (mybotshop forum, staff reply, 2026-05-27, https://forum.mybotshop.de/t/unitree-go2-w-lidar-in-chin-orientation/1815): *"default GO2 legged ones have the deskew topics. The Unitree GO2-W doesnt have it"*. Topic availability depends on model and firmware.

### 1.2 `/utlidar/cloud` (VERIFIED from Unitree's screenshot, firmware around 2023-10)
README text: *"where frame_id: utlidar_lidar"* ([README.md L359](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/README.md#L359)).

Echo screenshot ([docs/image/piFtdF1.png](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/docs/image/piFtdF1.png)), three consecutive messages:

| stamp (s) | width | point_step | row_step | fields | is_dense |
|---|---|---|---|---|---|
| 1697807635.048271179 | 1399 | 32 | 44768 | 6 | true |
| 1697807635.115090608 | 1395 | 32 | 44640 | 6 | true |
| 1697807635.182014703 | 1405 | 32 | 44960 | 6 | true |

- DERIVED: the stamp deltas are 66.82 ms and 66.92 ms, which is **about 14.95 Hz**. 1,400 pts x 14.95 Hz is about 20,900 pts/s, consistent with the spec's 21,600 effective pts/s. Unitree's RViz screenshot ([piFtyOe.png](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/docs/image/piFtyOe.png)) uses **Decay Time = 5 s** to show a readable scene. A single message is sparse.
- DERIVED: 21,600 pts/s divided by 180 lines/s is 120 points per line. At about 15 Hz that is **12 lines per message** (1,440 max). The ~1,400 observed means about 97% of samples were valid.
- **Fields:** 6 fields with point_step 32 exactly match the L1 SDK's PCL point type `x,y,z (f32), intensity (f32), ring (u16), time (f32)` with EIGEN_ALIGN16 ([unitree_lidar_sdk_pcl.h L49-62](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_sdk/include/unitree_lidar_sdk_pcl.h#L49-L62)). The standalone ROS 2 driver publishes that type through `pcl::toROSMsg` ([unitree_lidar_ros2.h L156-172](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_ros2/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h#L156-L172)). The names on the Go2 are **UNVERIFIED** (the screenshot hides the array). `time` is *"the time difference of the point relative to the point cloud timestamp"* and `ring` is *"the number of the laser ring to which the point belongs"* (Unilidar SDK User Manual v1.0, 2023.05, p.10, https://oss-global-cdn.unitree.com/Lidar/Unilidar%20SDK%20User%20Manual_v1.0.pdf).
- `is_dense: true`: invalid returns are removed, not padded with NaN. The SDK parser skips them: *"If the distance of a point equals 0, that point is invalid and should be jumped"* ([HowToParse...md L88, L151](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_sdk/HowToParsePointCloudAndIMUDataFromMavLinkMessages.md#L84-L88)). So the width varies from message to message.

### 1.3 `/utlidar/cloud_deskewed` (SECONDARY)
- Frame: Go2 world `odom`. Three independent sources agree:
  - KrepsD/Go2-Nav2-Slam-adaptor `config/pointcloud_to_laserscan.yaml` L3: *"cloud_deskewed is expressed in odom"* (commit `59ed609e`).
  - Tmino1/go2_ros2_sdk `lidar_processor_cpp/src/pointcloud_aggregator_node.cpp` L69-74: *"The native deskewed cloud is in the driver's world-frame "odom", whose origin is wherever the Go2 powered on"* (commit `0c610b4d`).
  - zhaoyufei122/go2_hokuyo_urg4_navigation `config/scan_accumulator.yaml` L5-6: *"Deskewed GO2 clouds are registered in a static frame"* (commit `37098c83`). Its README L14: *"cloud_deskewed is the current motion-deskewed cloud, not an accumulated map"*.
  - Confidence: medium-high. The exact `frame_id` string is UNVERIFIED.
- Rate: about 14.7 Hz (Tmino1 README L11) and about 15 Hz (zhaoyufei122 `scan_accumulator.yaml` L9).
- Anecdotal, single source, low confidence: Tmino1 L178-183 says the deskewed cloud *"pads empty returns with exact (0,0,0) - measured ~10k of ~10.6k points per cloud"*. If true, consumers must drop (0,0,0) points. Measure it.

### 1.4 Other topics
- `/utlidar/imu`: frame `utlidar_imu` (SECONDARY, unilidar_sdk issue #34, https://github.com/unitreerobotics/unilidar_sdk/issues/34). That issue reports corrupted linear acceleration, later attributed to a damaged sensor. The L1 IMU reports at 250 Hz (VERIFIED, L1 UM p.14). Its offset in the lidar frame is [-0.007698, -0.014655, 0.00667] m with axes parallel (VERIFIED, [unilidar_sdk README L22](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/README.md#L14-L22)).
- `/utlidar/robot_odom`: about 150 Hz (SECONDARY, forum t/1789, https://forum.mybotshop.de/t/unitree-go2-l1-saved-map-localization-only-no-slam/1789). Community repos use it for the `odom -> base_link` TF (KrepsD `src/odom_tf_broadcaster.cpp` L16-17).
- `/utlidar/lidar_state` (`unitree_go/msg/LidarState`) has `sys_rotation_speed`, `com_rotation_speed`, `cloud_frequency`, `cloud_packet_loss_rate`, `cloud_size`, `cloud_scan_num`, `imu_frequency`, `firmware_version` (VERIFIED definition, [LidarState.msg](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/cyclonedds_ws/src/unitree/unitree_go/msg/LidarState.msg)). **This one message answers most open questions.**
- SECONDARY: the Go2's clock is not NTP-synced. Tmino1 L146-153 reports *"drifts ~300ppm (~1s/hour) against the Jetson"* and restamps on receive. Unitree-Go2-Robot/go2_driver also restamps with `now()` ([go2_driver.cpp L132-137](https://github.com/Unitree-Go2-Robot/go2_driver/blob/5a921a7df9b84b433cb5f62ab38ae4c553d39249/src/go2_driver/go2_driver.cpp#L132-L137)). This affects TF lookups on the real robot but not in the sim.
- abizovnuralem/go2_ros2_sdk (WebRTC mode) does not use `/utlidar/cloud`. It decodes `rt/utlidar/voxel_map_compressed` ([webrtc_topics.py L17-21](https://github.com/abizovnuralem/go2_ros2_sdk/blob/b440609591a249e7bdd4bbc88e056a3660575447/go2_robot_sdk/go2_robot_sdk/domain/constants/webrtc_topics.py#L17-L21)) and publishes it in `odom`. Its CycloneDDS `/utlidar/cloud` callback is an empty `pass` (`go2_driver_node.py` L331-334). Tmino1 measured that voxel cloud at about 2.4 Hz on a 5 cm grid (README L11). It is a different product from the raw cloud.

---

## 2. Does the firmware crop the robot's body and legs?

- No Unitree document or source mentions a self-filter or crop box for `/utlidar/cloud` or `cloud_deskewed`. **UNVERIFIED.**
- SECONDARY evidence that body and leg returns are present:
  - zhaoyufei122 crops the body before every pipeline. README L5: *"the 2D pipeline first crops GO2's own points"*. It uses `pcl_ros::CropBox` in `base_link`, `negative: true`, x [-0.45, 0.45], y [-0.32, 0.32], z [-0.45, 0.30] ([pointcloud_to_laserscan.yaml L1-14](https://github.com/zhaoyufei122/go2_hokuyo_urg4_navigation/blob/37098c83816c138a9c21d5d266b442d8c5bed522/src/go2_base_nav/config/pointcloud_to_laserscan.yaml#L1-L14); launch `sensors.launch.py` L80-91). Its accumulator comment mentions *"leg ghosts"* (`scan_accumulator.yaml` L16).
  - Forum t/1789 (Go2 EDU + L1): preprocessing includes *"self-return removal"*.
  - KrepsD README L80-83: tune `min_height`/`max_height` so the scan does not hit *"the floor or the robot's legs"*.
- DERIVED geometry: the dome looks forward-down (section 5), so the front legs and the ground under the chin are inside the FOV. Expect leg returns.

---

## 3. PM or RM, and effective range on the robot

- Sensor variants (VERIFIED, L1 UM v1.1 p.14 and QSG v1.0 spec table): `L1 PM (Precision Measurement)` 20 m @90% / 10 m @10%. `L1 RM (Remote Measurement)` 30 m @90% / 15 m @10%. Near blind zone 0.05 m. Accuracy +-2.0 cm. Resolution 8 mm.
- The Go2 manual and product page name only "L1"/"4D LiDAR". **The Go2 variant (PM or RM) is UNVERIFIED.** The firmware may also cap the range.
- The standalone SDK defaults to `range_max 50`, `range_min 0`, `range_scale 0.001` (mm units) ([launch.py L14-25](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py#L14-L25); [unitree_lidar_sdk.h L137-146](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_sdk/include/unitree_lidar_sdk.h#L137-L146)). Ranges are uint16 in mm (`HowToParse...md` L86, L149-154). Unitree's own Point-LIO config for the L1 uses `blind: 0.5` ([unilidar_l1.yaml L15](https://github.com/unitreerobotics/point_lio_unilidar/blob/18ed5976d8fab2bd8a5148c26a40692bd3c0dc91/config/unilidar_l1.yaml#L11-L15)). That is a SLAM choice, not a sensor limit.

---

## 4. Scan pattern

### 4.1 Mechanism (VERIFIED, L1 User Manual v1.1, https://oss-global-cdn.unitree.com/static/52b72f707b304d229d4321eea223738f.pdf)
- p.6: *"The high-speed motor drives the reflector to rotate, realizing vertical measurement within a 180° range. The low-speed motor then drives the measuring core to rotate 360°, thus achieving a hemispherical super-wide-angle scan of 360*90°. It can measure the three-dimensional space above the radar"*.
- p.6: *"the point cloud density of the L1 varies in different FOV areas, with higher density near the center"*. The figure shows straight scan lines radiating from the pole (zenith of the dome).
- p.14: horizontal scanning 11 Hz, vertical scanning 180 Hz, sampling 43,200 pts/s, effective 21,600 pts/s.

### 4.2 Packet and frame structure (VERIFIED, unilidar_sdk)
- One distance packet holds 120 ranges: `point_data[240]` = 120 x uint16 ([HowToParse...md L73-88](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_sdk/HowToParsePointCloudAndIMUDataFromMavLinkMessages.md#L73-L88)). The SDK `ScanUnitree` holds `PointUnitree points[120]` ([unitree_lidar_sdk.h L70-75](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_sdk/include/unitree_lidar_sdk.h#L70-L75)).
- Inside a packet, pitch steps by `sys_vertical_angle_span` per point. Yaw starts at `com_horizontal_angle_start` and steps by `com_horizontal_angle_step/120` ([HowToParse...md L135-146](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_sdk/HowToParsePointCloudAndIMUDataFromMavLinkMessages.md#L124-L170)). Each packet is one sweep of the mirror: an arc through the pole, at almost constant yaw.
- Range origin: `z = ... * range + z_bias` with `z_bias = 0.0445` (L126, L167). The rays originate 44.5 mm above the frame origin, which is the centre of the bottom mounting face (README L17).
- `cloud_scan_num` (default 18): *"The number of laser ring IDs for the point cloud data"* (SDK User Manual p.11). DERIVED: 18 scans x 120 = 2,160 pts per cloud at 180/18 = **10 Hz** in the standalone driver. The Go2's ~1,400 pts at ~15 Hz implies about 12 scans per cloud. Check `cloud_scan_num` in `/utlidar/lidar_state`. Unitree's Point-LIO L1 config also sets `scan_line: 18` and `timestamp_unit: 0` (seconds) ([unilidar_l1.yaml L12-14](https://github.com/unitreerobotics/point_lio_unilidar/blob/18ed5976d8fab2bd8a5148c26a40692bd3c0dc91/config/unilidar_l1.yaml#L11-L15)).
- The SDK sample output shows `time` increasing 0, 23, 46, 69 us... while `ring` stays 0 ([unitree_lidar_sdk/README.md L91-103](https://github.com/unitreerobotics/unilidar_sdk/blob/1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6/unitree_lidar_sdk/README.md#L91-L103)). DERIVED: points come at the 43,200 Hz sampling clock. 120 points take 2.78 ms, half of a 5.56 ms mirror revolution. So half of each revolution (the side facing the base) produces nothing, which explains "43,200 sampling vs 21,600 effective".

### 4.3 Geometry of one message (DERIVED, not measured)
- Each line is a great-circle arc through the pole. It covers pitch 0 to 180 deg, i.e. azimuth psi on one side of the pole and psi+180 on the other. Pitch step is 180/120 = **1.5 deg** (assumes even spacing; UNVERIFIED).
- Yaw advance between lines = 360 x 11 / 180 = **22 deg**. A ~15 Hz message has 12 lines, so 24 half-arcs. Their azimuths are spread over 360 deg with gaps of **4-22 deg**. At 2 m range a 22 deg gap is about 0.77 m with no data.
- Density follows the angular grid. Arcs converge at the pole, so density goes as 1/cos(elevation). That is the "pole-dense" pattern, and a uniform azimuth x elevation grid has the same density law.
- Repetition: 22k mod 360 cycles after k = 180 lines (exactly 1 s) if the two motors hold exactly 11 and 180 Hz. After 1 s the half-arcs sit every 2 deg. If the motor speeds are not phase-locked, the pattern drifts ("non-repetitive"). UNVERIFIED: compare `sys_rotation_speed` and `com_rotation_speed` in `/utlidar/lidar_state`.
- Consequence for 2D: projecting **one** message gives a LaserScan where most 0.5-1 deg bins are `inf`. Accumulating about 0.33 s (5 messages, 60 lines, about 6 deg spacing) or 1 s (about 2 deg) is needed for a dense scan. Community Go2 users do this (section 7). SECONDARY, forum t/1789: NDT on single L1 frames drifted because *"Frame A and Frame B ... hit different parts of the same wall"*. Buffering 5 scans (about 333 ms) gave 10x better fitness.

---

## 5. Mount pose and height above ground

### 5.1 Official mount (VERIFIED)
[unitree_ros go2_description.urdf L741-753](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/go2_description/urdf/go2_description.urdf#L741-L753):
```xml
<joint name="radar_joint" type="fixed">
  <origin xyz="0.28945 0 -0.046825" rpy="0 2.8782 0" />
  <parent link="base" />  <child link="radar" />
```
The same values are in abizovnuralem/go2_ros2_sdk `go2_robot_sdk/urdf/go2.urdf` L1264-1273 and Unitree-Go2-Robot/go2_description. unitree_mujoco `go2.xml` has no lidar site. It has a "head lower" collision sphere r=0.047 at (0.293, 0, -0.06) ([go2.xml L76](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/go2/go2.xml#L76)) that encloses the lidar.

`radar` being the same frame as `utlidar_lidar` (the SDK frame: origin at the centre of the bottom mounting face, +X away from the cable, +Z through the dome) is assumed, not documented. Unitree publishes no TF. SECONDARY: Unitree-Go2-Robot/go2_driver relabels `/utlidar/cloud` as frame `radar` (`go2_driver.cpp` L135) and projects it with `target_frame: radar` (`go2_driver.launch.py` L42-53).

### 5.2 Orientation (DERIVED)
Pitch 2.8782 rad is 164.91 deg. Sensor +Z in base = (sin 164.91, 0, cos 164.91) = **(0.2604, 0, -0.9655)**: the dome looks down, tilted **15.09 deg forward**. Sensor +X in base = (-0.966, 0, -0.260), pointing backward. The range origin (z_bias 0.0445 along +Z) is at **(0.3010, 0, -0.0898) in base**.

Visible region: direction v is in the FOV when it points into the dome hemisphere. With heights z measured upward, a point is visible only if

  z_point - z_L  <=  0.2697 * (forward distance of the point from the lidar)    (0.2697 = tan 15.09 deg)

- Ahead: up to 15 deg above horizontal. At 2 m ahead you see up to h_L + 0.54 m.
- **Sideways (90 deg bearing): only at or below lidar height.**
- Behind: only 15 deg or more below horizontal. The body blocks most of it anyway.
- The densest region (the pole) hits the ground about h_L x 0.27, i.e. about 7 cm in front of the chin. This matches the manual's *"only support forward obstacle avoidance"* and the height-map topics.

### 5.3 Standing height (UNVERIFIED; estimates only)
- The spec gives only "Dimension of standing: 70cm x 31cm x 40cm" (unitree.com/go2). The axes are unlabelled and it is not a base height.
- Leg kinematics: Go2 thigh = calf = 0.213 m, foot sphere r = 0.022, hips at base z = 0 ([go2.xml L31-33, L78-108](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/go2/go2.xml#L31-L108)).
  - The unitree_sdk2 low-level stand pose is thigh 0.67, calf -1.3 ([go2_stand_example.cpp L64](https://github.com/unitreerobotics/unitree_sdk2/blob/63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36/example/go2/go2_stand_example.cpp#L61-L68)). That gives base height 0.213(cos 0.67 + cos 0.63) + 0.022 = **0.361 m**.
  - unitree_rl_gym `go2_config.py` (main branch) default angles: front 0.8/-1.5, rear 1.0/-1.5. That gives **about 0.32-0.33 m**.
  - Sport-mode stand height is not published. `SportModeState` has `body_height` and `foot_position_body` ([SportModeState.msg L9, L14](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg)). Measure it.
- Lidar range-origin height: **h_L = h_base - 0.0898**, so about **0.23-0.27 m** with these estimates.
- Sim comparison: CHAMP `nominal_height: 0.225` (`unitree_go2_sim/config/gait/gait.yaml` L18), with foot frames at the calf tip and foot radius 0.02 (`unitree_go2_description/urdf/leg.xacro` L147, L163). So sim base height is about 0.245 m, and the sim lidar origin is about 0.245 - 0.03 = **0.215 m**. The sim robot may stand **8-11 cm lower** than the real one. Verify both.

---

## 6. Modelling it in Gazebo Harmonic

### 6.1 What gpu_lidar actually does (VERIFIED from source)
- **Angles.** The point cloud is built in `FillPointCloudMsg`. `inclination` runs from vertical `min_angle` to `max_angle` and `azimuth` from horizontal min to max, with `x = d cos(incl) cos(az)`, `y = d cos(incl) sin(az)`, `z = d sin(incl)` in the sensor frame ([GpuLidarSensor.cc L357-430](https://github.com/gazebosim/gz-sensors/blob/b531112370bcfcbd1a8cd012b7b0d38f8d3889b8/src/GpuLidarSensor.cc#L357-L430)). Vertical angles are elevations above the sensor XY plane, so **0..+pi/2 is exactly the L1 hemisphere**.
- **Grid step.** `angleStep = (max-min)/(count-1)` (L364-366). With -pi..pi the first and last columns coincide. Use `max = pi - 2pi/N` for N distinct azimuths.
- **Fields and density.** Fields are `xyz` (f32), `intensity` (f32), `ring` (u16 = row index) (L138-141). There is **no per-point `time`**. The cloud is organised width x height and **keeps invalid points** (`is_dense` false if any are inf/NaN, L394, L430). The real L1 cloud is unorganised and dense.
- **Sampling.** The grid is uniform in azimuth and elevation only. sdformat has `samples`, `resolution`, `min_angle`, `max_angle` per axis and nothing for non-uniform patterns ([sdf 1.11 lidar.sdf L4-46](https://github.com/gazebosim/sdformat/blob/0af4bdba653dd90eb6c050216ef7707cb0cb6a8b/sdf/1.11/lidar.sdf#L4-L46)). `range/resolution` is parsed but **not applied**: in gz-sensors8 it appears only in the `Lidar::RangeResolution()` getter ([Lidar.cc L364-366](https://github.com/gazebosim/gz-sensors/blob/b531112370bcfcbd1a8cd012b7b0d38f8d3889b8/src/Lidar.cc#L364-L366)).
- **Noise.** Noise is independent Gaussian per ray, applied after rendering. Finite values are then clamped to [min, max] (`ApplyNoise` [Lidar.cc L223-238](https://github.com/gazebosim/gz-sensors/blob/b531112370bcfcbd1a8cd012b7b0d38f8d3889b8/src/Lidar.cc#L223-L238), `Clamp` L517-525: NaN becomes max, +-inf passes through).
- **Near and far clipping (the "see-through" hypothesis is confirmed):**
  1. `SetNearClipPlane(RangeMin())`, `SetFarClipPlane(RangeMax())`, `SetClamp(false)` with the comment *"Mask ranges outside of min/max to +/- inf, as per REP 117"* ([GpuLidarSensor.cc L198-202](https://github.com/gazebosim/gz-sensors/blob/b531112370bcfcbd1a8cd012b7b0d38f8d3889b8/src/GpuLidarSensor.cc#L198-L202)).
  2. `SetClamp(false)` sets `dataMinVal = -inf`, `dataMaxVal = +inf` ([BaseGpuRays.hh L235-248](https://github.com/gazebosim/gz-rendering/blob/ed91ca54262fafaa1e9f1b44cdef3162a73f3034/include/gz/rendering/base/BaseGpuRays.hh#L235-L248)).
  3. The Ogre2 cube-map cameras use a near clip **smaller** than min range: `boxSize = NearClipPlane()*2/sqrt(3); nearClipCube = boxSize*0.5`, i.e. **min/sqrt(3) = 0.577 x min** ([Ogre2GpuRays.cc L1175-1182](https://github.com/gazebosim/gz-rendering/blob/ed91ca54262fafaa1e9f1b44cdef3162a73f3034/ogre2/src/Ogre2GpuRays.cc#L1175-L1182), applied at L783 and L1048).
  4. The first-pass shader does `if (l > far) l = max; else if (l < near) l = min;` ([gpu_rays_1st_pass_fs.glsl L130-133](https://github.com/gazebosim/gz-rendering/blob/ed91ca54262fafaa1e9f1b44cdef3162a73f3034/ogre2/src/media/materials/programs/GLSL/gpu_rays_1st_pass_fs.glsl#L130-L133)). With clamping off, min and max are -inf and +inf.
  - **Result:**
    - Range in [min, max] is reported normally.
    - Range > max is **+inf**.
    - A surface at range < min whose depth along the cube-face axis is >= 0.577 x min is reported as **-inf** (a point at infinity, not dropped).
    - A surface whose face-axis depth is < 0.577 x min is **clipped by the GPU, and the ray returns whatever lies behind it** ("seeing through"). The see-through distance is 0.577 x min on a face axis and up to min at a cube corner.
  - With the current `min 0.8`, objects within about 0.46-0.8 m become -inf. Anything closer (front legs, the ground under the chin at about 0.2 m, a wall 0.4 m away) disappears, and the ray reports what is behind it.
  - With `min 0.05` the clip region shrinks to 2.9-5 cm. That matches the L1's 0.05 m blind zone, but the sensor's own housing mesh and the robot body become visible (handled in 6.3).
  - Downstream: `pointcloud_to_laserscan` rejects NaN and points outside the height band ([L183-196](https://github.com/ros-perception/pointcloud_to_laserscan/blob/442b33258b3af50e2981f338e1eac3ba259f8afa/src/pointcloud_to_laserscan_node.cpp#L183-L196)). +-inf points usually fail the height test or become NaN in the TF transform, so they are dropped. The see-through returns are finite and pass as false obstacles or free space.
- **Cube-map resolution.** Each face texture is sized from the samples, clamped to 128..1024 px ([Ogre2GpuRays.cc L741-776](https://github.com/gazebosim/gz-rendering/blob/ed91ca54262fafaa1e9f1b44cdef3162a73f3034/ogre2/src/Ogre2GpuRays.cc#L741-L776)). A 0..90 deg vertical FOV is supported because the sensor samples a cube map.
- **Visibility.** The sensor honours `<visibility_mask>` (`SetVisibilityMask`, GpuLidarSensor.cc L227; [lidar.sdf L74-76](https://github.com/gazebosim/sdformat/blob/0af4bdba653dd90eb6c050216ef7707cb0cb6a8b/sdf/1.11/lidar.sdf#L74-L76)). It is matched against `<visual><visibility_flags>` ([visual.sdf L21-23](https://github.com/gazebosim/sdformat/blob/0af4bdba653dd90eb6c050216ef7707cb0cb6a8b/sdf/1.11/visual.sdf#L21-L23)). gpu_lidar renders **visual** geometry, not collision geometry.

### 6.2 Current sim vs real (DERIVED)
| Property | Current xacro | Real L1 on Go2 | Effect |
|---|---|---|---|
| FOV in the sensor frame | elevation -45..+45 deg | 0..+90 deg (dome side only) | The sim sees **upward** directions (up to about 60 deg above horizontal ahead) that the real sensor cannot. It misses the densest pole region that looks at the ground ahead. |
| Points/message, rate | 600x30 = 18,000 @10 Hz (180k pts/s) | about 1,400 @ about 15 Hz (about 21k pts/s) | The sim is 12.9x denser per message and 8.3x per second. |
| Pattern | fixed grid, 0.6 deg x 3.1 deg | 24 half-arcs per message, 4-22 deg apart, drifting | A single sim message looks far better than a real one. |
| Range min | 0.8 m (near clip 0.46 m) | 0.05 m | The sim has a see-through zone around the chin. The real sensor sees legs and ground. |
| Mount | xyz (0.25, -0.038, -0.03), rpy (2.879, 0, 1.5705) | xyz (0.28945, 0, -0.046825), rpy (0, 2.8782, 0), plus 0.0445 range origin | About 3 cm lower and 4 cm sideways; +X axis rotated 90 deg. The dome direction (15 deg forward tilt) matches. |
| Frame / topic | `lidar_l1_link`, `/unitree_lidar/points` | `utlidar_lidar`, `/utlidar/cloud` | Real pipelines will not attach without remaps. |
| Fields | x, y, z, intensity, ring (organised, inf kept) | x, y, z, intensity, ring, time (dense) | Deskew and LIO code needs `time`. |
| Base height | about 0.245 m | about 0.32-0.36 m (UNVERIFIED) | The lidar sits lower in the sim. |
| p2l band 0.25-1.1 m above `base_footprint` | | The lidar is about 0.23-0.27 m high, so sideways it sees nothing above about 0.27 m | **The current band works in sim only because the sim sees upward. On the real robot the band would hold only the forward 15 deg wedge.** |

### 6.3 Recommended sim model

Goal: publish the same topics, frames, fields, density and blind zone as the real Go2. Then the identical downstream pipeline (crop, accumulate, project) runs in both the sim and on the robot.

**Frames (xacro).** Each choice traces to a cited fact.
```xml
<!-- utlidar_lidar = L1 SDK frame (bottom mounting face). Pose from official radar_joint (unitree_ros L748-753) -->
<joint name="utlidar_lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="utlidar_lidar"/>
  <origin xyz="0.28945 0 -0.046825" rpy="0 2.8782 0"/>
</joint>
<link name="utlidar_lidar"/>
<!-- Range origin is 44.5 mm along +Z (SDK z_bias, HowToParse L126) -->
<joint name="utlidar_optical_joint" type="fixed">
  <parent link="utlidar_lidar"/>
  <child link="utlidar_optical"/>
  <origin xyz="0 0 0.0445" rpy="0 0 0"/>
</joint>
<link name="utlidar_optical"/>
<!-- IMU offset from unilidar_sdk README L22 -->
<joint name="utlidar_imu_joint" type="fixed">
  <parent link="utlidar_lidar"/>
  <child link="utlidar_imu"/>
  <origin xyz="-0.007698 -0.014655 0.00667" rpy="0 0 0"/>
</joint>
<link name="utlidar_imu"/>
```
Massless fixed links get merged into their parent by URDF-to-SDF conversion. That is fine because the sensor keeps its pose and `gz_frame_id`, and robot_state_publisher still publishes the TF. Check with `gz sdf -p` on the generated URDF.

**Option B (recommended): render dense, resample to the L1 pattern in a small node.** This is the same technique as the Gazebo Harmonic Livox port (KangjianPeng/livox_laser_simulation, commit `57b30973`, README: *"uses Ogre2 GpuRays to render regular distance images on the GPU before resampling them with Livox scan patterns"*, https://github.com/KangjianPeng/livox_laser_simulation).
```xml
<gazebo reference="utlidar_optical">
  <sensor name="l1_dense" type="gpu_lidar">
    <topic>l1_dense</topic>                         <!-- gz publishes l1_dense/points -->
    <update_rate>15</update_rate>                   <!-- about 15 Hz /utlidar/cloud (Unitree screenshot) -->
    <lidar>
      <scan>
        <horizontal>
          <samples>720</samples><resolution>1</resolution>
          <min_angle>-3.141593</min_angle>
          <max_angle>3.132866</max_angle>           <!-- pi - 2pi/720: no duplicate column (GpuLidarSensor.cc L364) -->
        </horizontal>
        <vertical>
          <samples>61</samples><resolution>1</resolution>   <!-- 1.5 deg rows = 180 deg / 120 pts (DERIVED) -->
          <min_angle>0</min_angle>
          <max_angle>1.570796</max_angle>           <!-- dome-side hemisphere only (L1 UM p.6) -->
        </vertical>
      </scan>
      <range>
        <min>0.05</min>                             <!-- near blind zone 0.05 m (L1 UM p.14) -->
        <max>20.0</max>                             <!-- PM @90%; 30 only if RM is confirmed -->
        <resolution>0.001</resolution>              <!-- ignored by gz-sensors8; documents SDK mm units -->
      </range>
      <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>  <!-- ASSUMPTION: +-2 cm accuracy taken as about 2 sigma -->
      <visibility_mask>4294967293</visibility_mask> <!-- 0xFFFFFFFD: ignore visuals flagged 0x2 (housing) -->
    </lidar>
    <gz_frame_id>utlidar_optical</gz_frame_id>
    <always_on>1</always_on>
    <visualize>false</visualize>
  </sensor>
</gazebo>
```
- Give the L1 housing mesh `<visibility_flags>2</visibility_flags>` so the lidar does not see its own dome (the real dome is the optical window). Whether a `<gazebo reference>` block can set visual flags on a merged link is **unverified**. Check the generated SDF. If it cannot, drop the housing visual from the Gazebo model. Test: no points closer than 0.06 m.
- Leave the robot body and legs visible. The real sensor sees them, and the crop happens downstream exactly as on the robot.

The `l1_pattern` node is about 150 lines of C++/Python. It subscribes to the bridged `l1_dense/points` and, for each message:
1. Keep `psi` = the yaw of the next line. For each of `L` lines per message (L = 12 at 15 Hz; set it to the measured `cloud_scan_num`), take pitch `a_i = i*1.5 deg` for i = 0..119.
   - For `a <= 90`: azimuth `psi`, elevation `a`.
   - For `a > 90`: azimuth `psi + 180`, elevation `180 - a`.
   - Read the nearest grid cell (column `round((az+pi)/0.5deg)`, row `round(el/1.5deg)`).
   - Then advance `psi += 360*11/180 = 22 deg` (use the measured `sys/com_rotation_speed`). Optionally add a small random phase drift for non-repetition.
2. Drop non-finite ranges. The real cloud is `is_dense: true` and the SDK skips range == 0.
3. Re-express each point in `utlidar_lidar` (z += 0.0445).
4. Fill `intensity` (gz retro, or a constant), `ring` = line index 0..L-1, and `time` = line_index/180 + i/43200 seconds, matching the SDK sample (23 us steps, unit seconds per Point-LIO `timestamp_unit: 0`).
5. Publish `/utlidar/cloud` (frame `utlidar_lidar`, height 1, fields x, y, z, intensity f32, ring u16, time f32, point_step 32).
6. Also transform with TF at the stamp into `odom` and publish `/utlidar/cloud_deskewed`.
   - Limitation: one render per message is skew-free. The sim `/utlidar/cloud` is slightly too good. Real motion skew within 67 ms is not reproduced unless you render at 180 Hz.

This gives about 1,400 pts/msg, pole-dense, 4-22 deg azimuth gaps, filling in when accumulated, with the same blind zone and frames as the real sensor. Optionally publish a 250 Hz gz IMU on `/utlidar/imu` with frame `utlidar_imu`.

**Option A (no extra node, coarse): a sparse grid that matches density only.**
```xml
<horizontal><samples>24</samples><min_angle>-3.141593</min_angle><max_angle>2.879793</max_angle></horizontal> <!-- 15 deg -->
<vertical><samples>61</samples><min_angle>0</min_angle><max_angle>1.570796</max_angle></vertical>
<update_rate>15</update_rate>   <!-- 24 x 61 = 1,464 pts/msg -->
```
Same range, noise, and frames as above, with `gz_frame_id` set and the 4.45 cm range-origin offset accepted. It reproduces per-message sparsity and pole density, but the azimuths **never fill in when accumulated**. It also keeps organised inf points and has no `time` field. Use it only as a worst-case sparsity test.

### 6.4 Existing community simulations (SECONDARY)
- No public Gazebo model of the Go2's L1 pattern was found.
- Livox (non-repetitive) for Harmonic:
  - KangjianPeng/livox_laser_simulation (Humble + Harmonic, GpuRays + CSV pattern resampling).
  - ashduwihch/mid360_gazebo_harmonic (Embree ray tracing, 200k pts/s Mid-360).
  - thunderchanger66/mid360_ws (reported on Jazzy).
  - LASER-Robotics/laser_gazebo_resources (CSV-pattern LivoxPointsPlugin).
  - Their pattern files are Livox-specific. The L1 pattern must be generated analytically (6.3).
- Neomelt/rm_sim_26 (Jazzy + Harmonic): a Mid-360 using either `gpu_lidar` or RGLGazeboPlugin (NVIDIA only).

---

## 7. Turning the real L1 cloud into Nav2 / slam_toolbox input (SECONDARY)

| Repo (ROS 2) | Input | Self-filter | Accumulation | 2D projection |
|---|---|---|---|---|
| zhaoyufei122/go2_hokuyo_urg4_navigation (Jazzy, `37098c83`) | `/utlidar/cloud_deskewed` | CropBox in base_link: x +-0.45, y +-0.32, z [-0.45, 0.30] | optional `scan_accumulator`: 0.35 s window in `odom`, 5 cm voxels, `min_voxel_hits 2` (`scan_accumulator.yaml` L1-19) | p2l `target_frame base_footprint`, `min_height 0.12`, `max_height 0.45`, `range 0.25-6.0`, 0.5 deg bins, `scan_time 1/15` (`pointcloud_to_laserscan.yaml` L16-29) |
| KrepsD/Go2-Nav2-Slam-adaptor (Foxy, `59ed609e`) | `/utlidar/cloud_deskewed`, TF from `/utlidar/robot_odom` | via height band | none | p2l `target_frame base_link`, `min_height -0.20`, `max_height 0.30`, `range 0.30-20` (`config/pointcloud_to_laserscan.yaml` L1-25) |
| Unitree-Go2-Robot/go2_driver (Humble, `5a921a7d`) | `/utlidar/cloud`, relabelled `radar`, restamped | none | none | p2l `target_frame radar`, default heights (`go2_driver.launch.py` L42-53) |
| Tmino1/go2_ros2_sdk fork (Humble, `0c610b4d`) | `/utlidar/cloud_deskewed` (odom) | range and height filters | aggregator (up to 40 clouds) | p2l |
| forum t/1789 (Foxy, L1) | `/utlidar/cloud` + `/utlidar/imu` into Point-LIO (`point_lio_unilidar`) | yes | 5 scans (about 333 ms) for NDT | 2D grid from the Point-LIO map |

- All Nav2 configs found use a `LaserScan` source in the costmap (e.g. go2_ros2_sdk `nav2_params.yaml` VoxelLayer with `data_type: LaserScan`). None of them feeds the 3D cloud directly into a voxel layer or STVL.
- Note on `pointcloud_to_laserscan` (VERIFIED, jazzy source): `range_min/max` use **hypot(x, y) in `target_frame`**, not the 3D sensor range ([L199-212](https://github.com/ros-perception/pointcloud_to_laserscan/blob/442b33258b3af50e2981f338e1eac3ba259f8afa/src/pointcloud_to_laserscan_node.cpp#L199-L212)). The default `min_height` is `numeric_limits<double>::min()` (a tiny positive number), not -inf (L67).

**Recommended pipeline, identical in sim and on the robot (DERIVED from sections 4-5):**
1. Input `/utlidar/cloud_deskewed` (odom).
2. CropBox self-filter in `base_link`. Start from zhaoyufei122's box and tune on a sim bag, where ground truth is exact.
3. Accumulate 0.3-0.35 s in `odom`.
4. p2l in `base_footprint`. **Keep `max_height` near the lidar height (about h_L + 0.05)**, because above h_L only the front 15 deg wedge has data. Use `min_height` of about 0.10-0.15 for ground noise (+-2 cm) and walking pitch, and `range_min` of about 0.25-0.3 (outside the crop box).
5. If needed, add a VoxelLayer or STVL with the accumulated 3D cloud as a `PointCloud2` source for obstacles above h_L in front. That is a design suggestion; no Go2 repo found doing it.

Concrete change to `unitree_go2_navigation/config/pointcloud_to_laserscan.yaml`, once h_L is measured: `min_height` about 0.12, `max_height` about h_L + 0.05 (about 0.30), `range_min` about 0.3, `scan_time` 0.0667. The current 0.25-1.1 band will not transfer.

---

## Must measure on the real robot

Prerequisite: the unitree_ros2 environment sourced, so the `unitree_go` message types (e.g. `LidarState`, `SportModeState`) are available. Robot standing still on flat ground in a room with walls 1-5 m away.

```bash
# 0. Which sensor, firmware, scan counts, motor speeds (answers L1 vs L2, cloud_scan_num, line rate)
ros2 topic list | grep utlidar
ros2 topic echo --once /utlidar/lidar_state

# 1. Rates (use a window; Go2 topics are best-effort)
ros2 topic hz -w 200 /utlidar/cloud
ros2 topic hz -w 200 /utlidar/cloud_deskewed
ros2 topic hz -w 500 /utlidar/imu
ros2 topic hz -w 500 /utlidar/robot_odom
ros2 topic bw /utlidar/cloud
ros2 topic bw /utlidar/cloud_deskewed

# 2. Frames, sizes, field names/offsets/datatypes
ros2 topic echo --once --no-arr /utlidar/cloud
ros2 topic echo --once /utlidar/cloud --field fields
ros2 topic echo --once --no-arr /utlidar/cloud_deskewed
ros2 topic echo --once /utlidar/cloud_deskewed --field fields
ros2 topic echo --once --no-arr /utlidar/imu
ros2 topic echo --once --no-arr /utlidar/robot_odom      # header.frame_id, child_frame_id
ros2 topic echo --once --no-arr /utlidar/robot_pose

# 3. Standing body height (for h_L = h_base - 0.0898)
ros2 topic echo --once /sportmodestate --field body_height
ros2 topic echo --once /sportmodestate --field foot_position_body   # -mean(z) + 0.022 = base height
# also tape-measure ground to the top of the lidar dome / chin as a sanity check

# 4. Clock offset vs your PC
ros2 topic echo --once /utlidar/cloud --field header.stamp ; date +%s.%N

# 5. Bag for offline analysis (60 s standing + 60 s slow walk + a turn in place)
ros2 bag record -o go2_l1_$(date +%F_%H%M) \
  /utlidar/cloud /utlidar/cloud_deskewed /utlidar/imu /utlidar/lidar_state \
  /utlidar/robot_odom /utlidar/robot_pose /sportmodestate /lowstate /tf /tf_static
```

Offline checks on the bag (sensor_msgs_py):
- [ ] Point count per message, and whether the fields are named `x,y,z,intensity,ring,time` with offsets 0, 4, 8, 16, 20, 24.
- [ ] `ring` range (max + 1 = lines per message, expected 12), points per ring (expected up to 120), `time` step (expected about 2.3e-5 s), `time` unit (s?).
- [ ] Elevation step between consecutive points in one ring (expected 1.5 deg) and azimuth advance between rings (expected 22 deg). Compute in the `utlidar_lidar` frame: `atan2(z, hypot(x, y))`, `atan2(y, x)`.
- [ ] Whether the pattern repeats after 1 s (overlay messages k and k+15).
- [ ] Self returns: fraction of points inside the body box in `base_link` (legs, chin, ground under the chin). Tells you whether the firmware crops anything.
- [ ] `cloud_deskewed`: frame_id, fraction of (0,0,0) padding points, and whether it is the same point set as `cloud`, just transformed.
- [ ] `radar` vs `utlidar_lidar`: transform `/utlidar/cloud` with the URDF `radar_joint`. The floor should land at z of about -h_base in `base` and walls should be vertical. If they are tilted or offset, the frames differ.
- [ ] Max range: farthest valid return down a long corridor (about 20 m means PM, about 30 m means RM).
- [ ] Range noise: std of a flat wall patch at 1, 3, 5 m with the robot stationary.

---

## Still unverified (summary)
1. Which lidar your unit actually has (L1 vs L2), and the PM/RM variant.
2. The firmware range limits.
3. Exact field names and types of `/utlidar/cloud`.
4. `cloud_scan_num` on the Go2 (about 12 expected).
5. Per-line pitch step and yaw advance, and whether the scan pattern repeats.
6. `cloud_deskewed`: frame_id string, rate, and whether it is zero-padded.
7. `/utlidar/imu` and `/utlidar/robot_odom` rates.
8. Firmware self-cropping (the evidence says none).
9. Whether `radar` and `utlidar_lidar` are the same frame.
10. Standing base height, and therefore the lidar height.
11. Range noise.
12. The Go2 clock offset and drift.
