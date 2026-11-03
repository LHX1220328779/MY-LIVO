# wuhu/truck28/v032_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck28/v031_axle
- lineage: wuhu/truck28/v023_import -> wuhu/truck28/v031_axle -> wuhu/truck28/v032_ground
- created: 2026-10-26T18:04:41+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu/28/测试/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/测试/ros2bag', 'kind': 'raw_split', 'segments': 4, 'duration_s': 119.99, 'has_camera': False}
- stationary: [252, 20]
- moving: [990, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        8.42
      ],
      "drpy_deg": [
        -0.391,
        -0.054,
        -0.006
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -9.43
      ],
      "drpy_deg": [
        0.665,
        -1.567,
        0.053
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -2.84
      ],
      "drpy_deg": [
        0.23,
        0.145,
        -0.003
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
        -0.3900244226020497,
        -0.0525585658879712,
        0.010683180537139247
      ],
      "trans_cm": [
        0.0,
        0.0,
        8.423361883762299
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (0.28 deg / +6.5 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 23,
        "matches": 9234,
        "ground_before_cm": 8.31,
        "ground_after_cm": 5.79,
        "rotation_deg": 0.394,
        "translation_cm": 8.4
      },
      "plane_check": {
        "tilt_deg": 0.284,
        "fore_aft_deg": -0.283,
        "lateral_deg": 0.02,
        "dz_cm": 6.5,
        "pairs": 1,
        "outside_basin": true,
        "applied": true
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.23432155539175137,
        -0.1380629692770216,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -2.840846963356992
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 25,
        "matches": 9174,
        "ground_before_cm": 10.04,
        "ground_after_cm": 9.72,
        "rotation_deg": 0.272,
        "translation_cm": 2.8
      },
      "plane_check": {
        "tilt_deg": 0.821,
        "fore_aft_deg": -0.146,
        "lateral_deg": 0.808,
        "dz_cm": 2.3,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        -1.5729001810576981,
        -0.6483886161550155,
        0.023275616863564542
      ],
      "trans_cm": [
        0.0,
        0.0,
        -9.432228122698124
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (1.53 deg / -8.1 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 20,
        "matches": 17801,
        "ground_before_cm": 11.69,
        "ground_after_cm": 8.97,
        "rotation_deg": 1.701,
        "translation_cm": 9.4
      },
      "plane_check": {
        "tilt_deg": 1.526,
        "fore_aft_deg": -0.912,
        "lateral_deg": 0.973,
        "dz_cm": -8.1,
        "pairs": 14,
        "outside_basin": true,
        "applied": true
      }
    }
  },
  "quality": "warn"
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
        2.7306,
        5.8576,
        1.4613
      ],
      "rpy_deg": [
        0.324,
        2.441,
        -0.185
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2169,
        -0.3327,
        0.2309
      ],
      "rpy_deg": [
        -1.781,
        1.782,
        -90.532
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0704,
        5.6266,
        1.5848
      ],
      "rpy_deg": [
        -1.273,
        -0.69,
        178.363
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.144,
        7.9129,
        -0.1568
      ],
      "rpy_deg": [
        2.014,
        -3.526,
        89.96
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1923,
      7.8117,
      1.5926
    ],
    "rpy_deg": [
      -100.296,
      0.152,
      0.901
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1637.05,
      1851.4,
      1029.33,
      495.55
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
            1637.05,
            1851.4,
            1029.33,
            495.55
          ],
          "rms_px": 0.014,
          "circle_centre_rear_m": [
            0.0,
            7.38
          ],
          "ground_z_rear_m": -1.22
        },
        "square": {
          "intrinsics": [
            1639.36,
            1639.36,
            864.27,
            446.63
          ],
          "rms_px": 1.072,
          "circle_centre_rear_m": [
            0.806,
            7.314
          ],
          "ground_z_rear_m": -1.588
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.014,
      "camera_height_m": 2.827,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -0.34,
        "fy_pct": -0.22,
        "cx_px": 58.3,
        "cy_px": -38.7
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.014 px)"
  },
  "time_offsets": {
    "lidar": -0.1609,
    "lidar_frontend": -0.1559,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.1609 + imu/ins clock lag +0.0050 (corr 0.997); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck28/v031_axle/calibration.json
bag=/home/project/data/haibo/wuhu/28/测试/ros2bag
reference=front_lidar
stationary frames 252..272 (aggregated) for side LiDARs; moving frames 990..1030 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 14.1 cm / P90 48.0 cm (limit 50 cm) -> stationary
front_right_lidar: stationary aggregated points 37020
back_lidar: stationary aggregated points 15648
front_left_lidar: stationary aggregated points 41128
front_lidar: stationary aggregated points 91084
front_right_lidar: plane start: own ground vs reference tilt 0.28 deg (fore-aft -0.28, lateral +0.02), height +6.5 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(-0.283,-0.020,+0.011) z +6.5 cm; ground residual 8.31->6.07 cm (n=8845->8923): used
front_right_lidar: yaw/xy solved=False; matches 9121->9234; |res| median 8.17->5.72 cm; ground 8.31->5.79 cm (n=8959); vertical 2.89->3.07 cm (n=168); correction rot deg (x,y,z)=(-0.390,-0.053,+0.011) trans cm=(+0.0,+0.0,+8.4); iters 23  [quality: warn; plane start applied (0.28 deg / +6.5 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
front_left_lidar: plane start: own ground vs reference tilt 0.82 deg (fore-aft -0.15, lateral +0.81), height +2.3 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(-0.146,-0.808,+0.010) z +2.3 cm; ground residual 10.04->15.21 cm (n=8060->8732): not used (does not improve the overlap)
front_left_lidar: yaw/xy solved=False; matches 8425->9174; |res| median 9.85->9.54 cm; ground 10.04->9.72 cm (n=8768); vertical 5.58->4.34 cm (n=207); correction rot deg (x,y,z)=(-0.234,-0.138,+0.000) trans cm=(+0.0,+0.0,-2.8); iters 25  [quality: ok]
back_lidar: plane start: own ground vs reference tilt 1.53 deg (fore-aft -0.91, lateral +0.97), height -8.1 cm over 14 plane pair(s) -> start rot deg (x,y,z)=(-0.910,-0.972,+0.023) z -8.1 cm; ground residual 11.69->10.41 cm (n=5406->5505): used
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 17640->17801; |res| median 9.51->9.04 cm; ground 11.69->8.97 cm (n=5305); vertical 8.27->8.42 cm (n=9772); correction rot deg (x,y,z)=(-1.573,-0.648,+0.023) trans cm=(+0.0,+0.0,-9.4); iters 20  [quality: warn; plane start applied (1.53 deg / -8.1 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
quality: warn  (front_right_lidar: plane start applied (0.28 deg / +6.5 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage; back_lidar: plane start applied (1.53 deg / -8.1 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck28/v032_ground/calibration.json
```
