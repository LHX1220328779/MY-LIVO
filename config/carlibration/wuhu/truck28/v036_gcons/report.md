# wuhu/truck28/v036_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: wuhu/truck28/v035_driven
- lineage: wuhu/truck28/v023_import -> wuhu/truck28/v033_axle -> wuhu/truck28/v034_ground -> wuhu/truck28/v035_driven -> wuhu/truck28/v036_gcons
- created: 2026-10-26T18:12:48+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/wuhu/28/测试/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/测试/ros2bag', 'kind': 'raw_split', 'segments': 4, 'duration_s': 119.99, 'has_camera': False}
- run: /home/project/MY-LIVO/results/calib_truck28_rtk1_1026_180828/Log/fusion/1793009467122516021

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        2.75
      ],
      "drpy_deg": [
        -0.029,
        -0.022,
        -0.001
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
          -0.0215,
          0.029,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          2.75
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "warn",
          "reasons": [
            "halves differ by 0.081 deg / 1.7 cm"
          ],
          "rotation_deg": 0.036,
          "z_cm": 2.75,
          "effect_rms_cm": 3.27,
          "halves_rot_deg": 0.081,
          "halves_z_cm": 1.67,
          "median_abs_res_before_cm": 2.6,
          "median_abs_res_after_cm": 1.56
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck28_rtk1_1026_180828/Log/fusion/1793009467122516021",
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
        2.7569,
        5.8491,
        1.4713
      ],
      "rpy_deg": [
        0.296,
        2.419,
        -0.112
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2089,
        -0.338,
        0.3636
      ],
      "rpy_deg": [
        -1.888,
        2.406,
        -90.632
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1111,
        5.6011,
        1.6864
      ],
      "rpy_deg": [
        -0.922,
        -0.145,
        178.698
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
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck28/v035_driven/calibration.json
run=/home/project/MY-LIVO/results/calib_truck28_rtk1_1026_180828/Log/fusion/1793009467122516021
scan source: round archive scans.bin/sources.bin (mapper deskew)
window 0.0..117.2 s, stride 4, scans front_right_lidar:293, back_lidar:268, front_left_lidar:293, front_lidar:293; ground-band returns 2955734
front_right_lidar: ground returns 679583, median along-track position +10.8 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 180306, median along-track position -9.8 m (rear-facing -> solved)
front_left_lidar: ground returns 716714, median along-track position +11.1 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x -0.022 y +0.029 deg (about the LiDAR, rear-axle frame), z +2.75 cm; own-ground change RMS 3.3 cm; flat-ground |res| median 2.60->1.56 cm (n=101939); signed median by range: 4-10 m -2.9->-0.7, 10-20 m -1.8->+0.7, 20-40 m -3.2->-0.1; halves: 0.081 deg / 1.7 cm  [quality: warn; halves differ by 0.081 deg / 1.7 cm]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/wuhu/truck28/v036_gcons/calibration.json
```
