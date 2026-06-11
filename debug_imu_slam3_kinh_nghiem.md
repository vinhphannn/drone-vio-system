# Kinh nghiệm Debug: ORB-SLAM3 Mono-Inertial Map Reset
Ngày: 2026-06-11 | Môi trường: Ubuntu 22.04, ROS2 Humble, BMI160 IMU, USB Camera

## 📋 Triệu chứng Ban đầu
Hệ thống ORB-SLAM3 Monocular-Inertial liên tục reset map sau mỗi 30-60 giây với log:

```text
IMU is not or recently initialized. Reseting active map...
SYSTEM-> Reseting active map in monocular case
```
Visual tracking thuần túy (không IMU) chạy hoàn toàn ổn định → Loại trừ lỗi camera/ORB.

## 🔬 Phương pháp Cô lập (Isolation Method)
### Bước 1: Thêm Diagnostic Log vào Tracking.cc
Inject log `[DIAG]` vào hai hàm chính để quan sát real-time:

```cpp
// Trong TrackLocalMap() — theo dõi optimizer nào đang chạy
if (bNeedNewKF) {
    RCLCPP_INFO(rclcpp::get_logger("DIAG"),
        "TrackLocalMap Optimizer: %s",
        mpImuPreintegratedFromLastKF ? "PoseInertialOptimizationLastKeyFrame" : "PoseOptimization");
}
// Trong TrackLocalMap() — theo dõi inliers sau optimizer
RCLCPP_INFO(rclcpp::get_logger("DIAG"),
    "TrackLocalMap: inliers=%d | totalMPs_in_frame=%d | outliers_after_PO=%d | "
    "localMapPts=%d | localKFs=%d | ImuInit=%d | BA2=%d | state=%d | sinceReloc=%d",
    nInliers, nMatchesInliers, nMatchesInliers - nInliers,
    vpLocalMapPoints.size(), vpLocalKeyFrames.size(),
    (int)mpAtlas->isImuInitialized(), (int)mpAtlas->GetCurrentMap()->isImuInitialized(),
    (int)mState, mnLastRelocFrameId);
```

### Bước 2: Chạy Mono-Only để Baseline
```bash
# Sửa tạm: comment out IMU topic subscriber
# Chạy SLAM chỉ với camera
```
Kết quả Mono-only:

- inliers=150-200 (ổn định)
- sinceReloc=1000+ (không reset)
- reason_low_match=OK liên tục
→ Camera và ORB feature matching 100% khoẻ mạnh

### Bước 3: Quan sát thời điểm ImuInit kích hoạt
Với IMU (trước khi fix):

```text
[DIAG] TrackLocalMap: inliers=48 | ImuInit=0    ← ổn
[DIAG] TrackLocalMap Optimizer: PoseInertialOptimizationLastFrame  ← ImuInit=1 kích hoạt!
[DIAG] TrackLocalMap: inliers=5  | ImuInit=1    ← SỤT NGAY 48→5
```
→ Xác định nguyên nhân: PoseInertialOptimization gây outliers bùng nổ khi ImuInit=1

## 🎯 Root Cause: T_b_c1 Sai Toàn Bộ
Lỗi trong `orb_slam3_config.yaml`
```yaml
# ❌ CẤU HÌNH CŨ (SAI - 2 lỗi nghiêm trọng)
IMU.T_b_c1: !!opencv-matrix
   data: [ 0.0542, 0.1600, 0.9856, 0.0,    # Rotation sai chiều
           0.9985,-0.0007,-0.0548, 0.0,    # Translation = 0 (!!!)
          -0.0080, 0.9871,-0.1598, 0.0,
           0.0,    0.0,    0.0,    1.0]
```
Lỗi #1 — Translation = [0, 0, 0]: Coi camera và IMU cùng vị trí vật lý.
Thực tế IMU cách camera 15cm sang phải → inertial prior predict pose sai 15cm → optimizer reject toàn bộ visual matches.

Lỗi #2 — Rotation sai chiều: Ma trận rotation không khớp với convention của IMU driver.
Driver BMI160 (imu_driver.cpp) đã convert sang ROS FLU (X=Forward, Y=Left, Z=Up):

```cpp
// Xác nhận từ imu_driver.cpp lines 225-231:
m.accel_x = -raw_az;  // FLU X (Forward) = chip -Z
m.accel_y = raw_ay;   // FLU Y (Left)    = chip Y
m.accel_z = raw_ax;   // FLU Z (Up)      = chip X
```

## ✅ Fix: Tính Lại T_b_c1 Đúng
Phân tích Frame Convention

| Frame | Axes |
| :--- | :--- |
| Camera (OpenCV) | X=Right, Y=Down, Z=Forward |
| IMU Body (ROS FLU) | X=Forward, Y=Left, Z=Up |
| Physical Layout | IMU cách camera 15cm sang PHẢI |

Tính Rotation R_b_c (Camera → IMU FLU)

| Camera Axis | → IMU FLU |
| :--- | :--- |
| X (Right) | → −Y (Right = ngược Left) |
| Y (Down) | → −Z (Down = ngược Up) |
| Z (Forward) | → +X (Forward) |

```text
R_b_c = |  0  0  1 |
        | -1  0  0 |
        |  0 -1  0 |
```

Tính Translation t_b_c (vị trí camera trong frame IMU)
IMU ở bên PHẢI camera 15cm
→ Camera ở bên TRÁI IMU 15cm
→ Trong FLU frame (Y = Left): t_b_c = [0, +0.15, 0]
Verify: R_b_c × [-0.15, 0, 0]_camera = [0, +0.15, 0]_imu ✓

Config đúng
```yaml
# ✅ CẤU HÌNH MỚI (ĐÚNG)
# IMU (BMI160, ROS FLU) cách camera 15cm sang phải, cùng hướng vật lý
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0, 0.0,  1.0,  0.0,
           -1.0, 0.0,  0.0,  0.15,
            0.0,-1.0,  0.0,  0.0,
            0.0, 0.0,  0.0,  1.0]
```

## 📊 Kết quả Xác nhận (So sánh Log)
Trước Fix
- ImuInit=0 → inliers=48
- ImuInit=1 → inliers=5  (sụt 90%!)
- → RESET sau 30-60s

Sau Fix
- ImuInit=1, sinceReloc=485 → inliers=105, outliers=117 (51%)
- ImuInit=1, sinceReloc=550 → inliers=149, outliers=90  (38%)
- ImuInit=1, sinceReloc=600 → inliers=152, outliers=69  (31%) ← cải thiện!
- → KHÔNG RESET trong 600+ frames liên tiếp ✅

Improvement:
- Inliers khi ImuInit=1: 5 → 130-160 (tăng 30x)
- Outlier rate: 90% → 35-50% (giảm 2x)
- Time without reset: 30s → 600+ frames (>20x)

## ⚠️ Vấn đề Còn Tồn Đọng
mTinit đóng băng tại ~7s (cần >10s để BA2 hoàn thành)
```text
IMU INITIALIZATION PAUSED
Current motion distance between keyframes: 0.0041 m  ← quá nhỏ!
Accumulated valid motion time (mTinit): 6.9944 seconds  ← không tăng
```
Nguyên nhân: Camera di chuyển < 1cm giữa các keyframe → không đủ baseline để VIO init.

Fix: Khi khởi động hệ thống, di chuyển camera theo pattern figure-8 thực sự với biên độ 20-30cm, liên tục trong ≥15 giây.

BA2=0 (Global Inertial BA chưa hoàn thành)
Hệ quả của mTinit chưa đạt ngưỡng. Khi BA2=0, map dễ reset hơn khi tracking yếu tạm thời.

Threshold trong code (LocalMapping.cc):
```cpp
if (mTinit < 5.0f) return;   // <5s: không init
// 5-10s: IMU init nhưng chưa BA2
if (mTinit > 10.0f) {        // >10s: kích hoạt BA2 toàn cục
    mpCurrentMap->setImuInitialized();
}
```

## 🗺️ Timeline Debug
- Session 1: Phát hiện reset pattern → Inject DIAG logs
- Session 2: Xác nhận ImuInit=1 là trigger → so sánh PoseOptimization vs PoseInertialOptimization  
- Session 3: Chạy Mono-only → xác nhận camera OK → IMU là nguyên nhân
- Session 4: Đọc imu_driver.cpp → xác nhận ROS FLU convention
- Session 5: Tính T_b_c1 bằng tay từ physical layout → sửa config
- Session 6: Xác nhận cải thiện 30x qua log

## 💡 Bài học Rút ra
- Luôn verify T_b_c1 translation ≠ [0,0,0] — Đây là lỗi rất phổ biến khi setup mới.
- Kiểm tra IMU driver convention trước khi tính T_b_c1 — FLU vs FRD vs NED cho kết quả rotation matrix hoàn toàn khác nhau.
- Phương pháp Isolation là chìa khóa:
Mono-only stable + Inertial drops inliers = lỗi nằm ở IMU calibration, không phải camera.
- pip install kalibr là SAI — Package đó không phải Kalibr ETH Zurich. Kalibr thật cần build từ source (ROS1/catkin) hoặc dùng Docker.
- mTinit cần chuyển động thực sự — Camera đặt yên trên bàn sẽ không bao giờ vượt qua ngưỡng 5s. Phải di chuyển tay với biên độ ≥20cm.

## 📁 Files Liên quan
- orb_slam3_config.yaml — T_b_c1 đã được fix
- src/ORB_SLAM3/src/Tracking.cc — DIAG logs đã được inject (lines 3004-3028, 3440-3465)
- drone_core/src/drivers/imu_driver.cpp — IMU FLU conversion logic
