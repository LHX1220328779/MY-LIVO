# wuhu/truck05/v027_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck05/v026_axle
- lineage: wuhu/truck05/v008_import -> wuhu/truck05/v017_axle -> wuhu/truck05/v018_ground -> wuhu/truck05/v019_driven -> wuhu/truck05/v020_manual -> wuhu/truck05/v021_manual -> wuhu/truck05/v022_manual -> wuhu/truck05/v023_manual -> wuhu/truck05/v024_precheck -> wuhu/truck05/v025_driven -> wuhu/truck05/v026_axle -> wuhu/truck05/v027_ground
- created: 2026-09-29T14:46:43+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- stationary: [2, 20]
- moving: [1840, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        2.15
      ],
      "drpy_deg": [
        -0.107,
        -0.14,
        0.004
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.81
      ],
      "drpy_deg": [
        0.166,
        -0.292,
        0.004
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.35
      ],
      "drpy_deg": [
        0.027,
        -0.08,
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
        -0.1092163299316272,
        -0.1372571657112491,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.148189689532705
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.026249553004727092,
        0.080479833254946,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.35107023357528555
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.2925261087002748,
        -0.1653555456976524,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.8033865057801832
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
        2.8233,
        5.5626,
        1.9687
      ],
      "rpy_deg": [
        0.642,
        -2.253,
        -1.149
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0238,
        -0.6768,
        0.8487
      ],
      "rpy_deg": [
        -0.871,
        0.998,
        -90.157
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0785,
        5.4797,
        1.9221
      ],
      "rpy_deg": [
        -0.454,
        -1.944,
        179.568
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0973,
        7.6438,
        0.5356
      ],
      "rpy_deg": [
        0.058,
        -5.285,
        89.869
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0014,
      7.6498,
      2.2531
    ],
    "rpy_deg": [
      -100.4,
      -0.011,
      -0.455
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1849.37,
      1849.37,
      1003.31,
      502.72
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "intrinsics_check": {
      "method": "supplier ground-distance curves (midrange_camera[0].equation_*m)",
      "curves": 10,
      "distances_m": [
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
        60.0,
        70.0,
        80.0,
        90.0,
        100.0
      ],
      "fits": {
        "free": {
          "intrinsics": [
            1849.32,
            1831.26,
            1004.09,
            499.42
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            -0.002,
            7.529
          ],
          "ground_z_rear_m": -1.283
        },
        "square": {
          "intrinsics": [
            1849.37,
            1849.37,
            1003.31,
            502.72
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            0.001,
            7.535
          ],
          "ground_z_rear_m": -1.249
        }
      },
      "model": "square pixels (fx = fy)",
      "rms_px": 0.05,
      "camera_height_m": 3.502,
      "deviation_vs_truck29_defaults": {
        "fx_pct": 12.59,
        "fy_pct": -0.33,
        "cx_px": 32.3,
        "cy_px": -31.5
      }
    },
    "source": "supplier ground-distance curves equation_*m (square pixels (fx = fy), rms 0.05 px)"
  },
  "time_offsets": {
    "lidar": -0.0267,
    "camera": -0.0019,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend": -0.0119,
    "camera_source": "operator set (livo_calib.py set): 相机偏移按同源时钟规则：camera_h = 家族值-0.115 + 打戳差0.1131 = -0.0019 ≈ 0，与 v016 时代 img=0 最优的实测互证"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v026_axle/calibration.json
bag=/home/project/data/haibo/wuhu/05/20260914/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 1840..1880 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 14.9 cm / P90 42.7 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 17419
back_lidar: stationary aggregated points 2965
front_left_lidar: stationary aggregated points 16225
front_lidar: stationary aggregated points 84300
front_right_lidar: yaw/xy solved=False; matches 5010->5062; |res| median 6.69->6.58 cm; ground 6.88->6.55 cm (n=4277); vertical 5.06->5.36 cm (n=258); correction rot deg (x,y,z)=(-0.109,-0.137,+0.000) trans cm=(+0.0,+0.0,+2.1); iters 21
front_left_lidar: yaw/xy solved=False; matches 5107->5128; |res| median 7.13->6.88 cm; ground 7.48->7.20 cm (n=4383); vertical 5.13->5.09 cm (n=258); correction rot deg (x,y,z)=(-0.026,+0.080,+0.000) trans cm=(+0.0,+0.0,-0.4); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 30148->30122; |res| median 7.74->6.14 cm; ground 7.60->6.01 cm (n=28317); vertical n=994; correction rot deg (x,y,z)=(-0.293,-0.165,+0.000) trans cm=(+0.0,+0.0,-0.8); iters 15
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v027_ground/calibration.json
```
