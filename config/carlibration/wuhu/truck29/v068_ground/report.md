# wuhu/truck29/v068_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29/v067_axle
- lineage: wuhu/truck29/v055_import -> wuhu/truck29/v056_axle -> wuhu/truck29/v057_ground -> wuhu/truck29/v058_driven -> wuhu/truck29/v059_camtime -> wuhu/truck29/v060_camlidar -> wuhu/truck29/v061_camloop -> wuhu/truck29/v062_manual -> wuhu/truck29/v063_manual -> wuhu/truck29/v065_driven -> wuhu/truck29/v066_driven -> wuhu/truck29/v067_axle -> wuhu/truck29/v068_ground
- created: 2026-10-09T08:34:04+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- stationary: [2, 20]
- moving: [300, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.6
      ],
      "drpy_deg": [
        -0.068,
        0.05,
        0.004
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        8.35
      ],
      "drpy_deg": [
        0.064,
        -0.136,
        0.001
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        2.16
      ],
      "drpy_deg": [
        0.105,
        0.01,
        -0.001
      ]
    }
  }
}
```

## metrics

```
{
  "corrections": {
    "front_right_lidar": {
      "rotvec_deg": [
        -0.06695738118701218,
        0.051337761702952546,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.602875424752369
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.10549214544698383,
        -0.007536242945855132,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.1575419065615518
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.13731628844297863,
        -0.06064644553020404,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        8.340934852229816
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    }
  }
}
```

## summary

```
{
  "lidars": [
    {
      "lidar": "front_right_lidar",
      "key": "transform_matrix_lidar_1",
      "t": [
        2.7559,
        6.0141,
        1.825
      ],
      "rpy_deg": [
        -1.012,
        -2.907,
        -1.439
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0883,
        -0.2263,
        0.7123
      ],
      "rpy_deg": [
        -0.981,
        0.613,
        -91.442
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0911,
        5.7896,
        1.9719
      ],
      "rpy_deg": [
        -3.366,
        -0.713,
        178.597
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1094,
        8.0405,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.532,
        89.924
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0024,
      8.0426,
      1.7517
    ],
    "rpy_deg": [
      -96.512,
      1.62,
      0.291
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.570556640625,
      1855.53173828125,
      971.0083025529602,
      534.2602337509306
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "source": "defaults (wuhu truck29 yaml)"
  },
  "time_offsets": {
    "lidar": -0.1462,
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "operator set (livo_calib.py set): 时间偏移改回 IMU 硬件对齐对（lidar -0.125 / camera -0.115）：axle 解 -0.1459 含 INS 链路延迟，喂给前端后 21ms 错位打花高角速率段的去畸变与颜色采样（灯柱白度 51.5%→79.3%，牌面着色同步改善）；融合精度在该区间平坦（LIVO 120s xy max 7.12→7.08 cm，达标）。证据链见 docs/mcp-color-regression-time-offsets.md",
    "lidar_frontend": -0.125
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v067_axle/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 11.3 cm / P90 35.0 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 12140
back_lidar: stationary aggregated points 2439
front_left_lidar: stationary aggregated points 18523
front_lidar: stationary aggregated points 43551
front_right_lidar: yaw/xy solved=False; matches 3732->3713; |res| median 4.36->4.12 cm; ground 4.34->4.10 cm (n=3640); vertical 4.04->3.48 cm (n=37); correction rot deg (x,y,z)=(-0.067,+0.051,+0.000) trans cm=(+0.0,+0.0,+1.6); iters 16
front_left_lidar: yaw/xy solved=False; matches 4813->4831; |res| median 5.95->4.96 cm; ground 5.95->4.97 cm (n=4723); vertical 5.31->3.97 cm (n=45); correction rot deg (x,y,z)=(-0.105,-0.008,+0.000) trans cm=(+0.0,+0.0,+2.2); iters 20
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36409->36848; |res| median 12.16->7.82 cm; ground 12.05->7.52 cm (n=31558); vertical n=3872; correction rot deg (x,y,z)=(-0.137,-0.061,+0.000) trans cm=(+0.0,+0.0,+8.3); iters 14
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v068_ground/calibration.json
```
