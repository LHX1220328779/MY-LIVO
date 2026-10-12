# wuhu/truck28/v030_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: wuhu/truck28/v029_driven
- lineage: wuhu/truck28/v023_import -> wuhu/truck28/v027_axle -> wuhu/truck28/v028_ground -> wuhu/truck28/v029_driven -> wuhu/truck28/v030_gcons
- created: 2026-10-12T22:05:33+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/wuhu/28/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/ros2bag', 'kind': 'raw_split', 'segments': 82, 'duration_s': 1229.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck28_rtk1_1012_214300/Log/fusion/1791813445469015205

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -1.2
      ],
      "drpy_deg": [
        0.069,
        0.041,
        0.003
      ]
    }
  }
}
```

## metrics

```
{
  "ground_consensus": {
    "corrections": {
      "back_lidar": {
        "rotvec_deg": [
          0.0401,
          -0.0696,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          -1.21
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.08,
          "z_cm": -1.21,
          "effect_rms_cm": 2.42,
          "halves_rot_deg": 0.053,
          "halves_z_cm": 0.44,
          "median_abs_res_before_cm": 1.88,
          "median_abs_res_after_cm": 1.52
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck28_rtk1_1012_214300/Log/fusion/1791813445469015205",
    "targets": [
      "back_lidar"
    ],
    "applied": [
      "back_lidar"
    ]
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
        2.7419,
        5.7998,
        1.4424
      ],
      "rpy_deg": [
        0.238,
        2.427,
        -0.259
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1685,
        -0.3556,
        0.3188
      ],
      "rpy_deg": [
        -1.693,
        2.605,
        -90.707
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1141,
        5.5781,
        1.5485
      ],
      "rpy_deg": [
        -1.063,
        -0.219,
        178.582
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1426,
        7.8507,
        -0.1568
      ],
      "rpy_deg": [
        2.071,
        -3.4,
        89.82
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1951,
      7.7526,
      1.5925
    ],
    "rpy_deg": [
      -100.421,
      0.211,
      0.764
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
    "lidar": -0.0151,
    "lidar_frontend": -0.0026,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0151 + imu/ins clock lag +0.0125 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck28/v029_driven/calibration.json
run=/home/project/MY-LIVO/results/calib_truck28_rtk1_1012_214300/Log/fusion/1791813445469015205
window 0.0..237.2 s, stride 4, scans front_right_lidar:589, back_lidar:589, front_left_lidar:589, front_lidar:589; ground-band returns 9094245
front_right_lidar: ground returns 2073338, median along-track position +9.7 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 1066393, median along-track position -12.5 m (rear-facing -> solved)
front_left_lidar: ground returns 2358155, median along-track position +8.3 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x +0.040 y -0.070 deg (about the LiDAR, rear-axle frame), z -1.21 cm; own-ground change RMS 2.4 cm; flat-ground |res| median 1.88->1.52 cm (n=401680); signed median by range: 4-10 m +0.2->-0.7, 10-20 m +2.0->+0.5, 20-40 m +1.8->-0.4; halves: 0.053 deg / 0.4 cm  [quality: ok]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/wuhu/truck28/v030_gcons/calibration.json
```
