# xingye/truck08/v007_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: xingye/truck08/v006_rtktime
- lineage: xingye/truck08/v001_import -> xingye/truck08/v002_axle -> xingye/truck08/v003_ground -> xingye/truck08/v004_driven -> xingye/truck08/v006_rtktime -> xingye/truck08/v007_gcons
- created: 2026-10-20T17:04:12+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/xingye/08/20261018/ros2bag
- dataset: {'path': '/home/project/data/haibo/xingye/08/20261018/ros2bag', 'kind': 'raw_split', 'segments': 10, 'duration_s': 299.99, 'has_camera': False}
- run: /home/project/MY-LIVO/results/calib_truck08_rtk1_1020_165417/Log/fusion/1792486745915508034

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.04
      ],
      "drpy_deg": [
        0.175,
        -0.033,
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
          -0.033,
          -0.1756,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          1.04
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "warn",
          "reasons": [
            "halves differ by 0.188 deg / 0.4 cm"
          ],
          "rotation_deg": 0.179,
          "z_cm": 1.04,
          "effect_rms_cm": 3.05,
          "halves_rot_deg": 0.188,
          "halves_z_cm": 0.38,
          "median_abs_res_before_cm": 1.54,
          "median_abs_res_after_cm": 1.66
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck08_rtk1_1020_165417/Log/fusion/1792486745915508034",
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
        1.8775,
        6.9056,
        1.1554
      ],
      "rpy_deg": [
        -1.006,
        -0.069,
        14.373
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2922,
        -1.8916,
        0.6007
      ],
      "rpy_deg": [
        0.024,
        -0.43,
        -89.926
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -2.0233,
        6.9151,
        1.2049
      ],
      "rpy_deg": [
        -0.016,
        -1.358,
        166.661
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0833,
        6.9974,
        0.7941
      ],
      "rpy_deg": [
        1.031,
        3.331,
        90.436
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1729,
      6.9547,
      1.384
    ],
    "rpy_deg": [
      -101.245,
      2.746,
      -1.747
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.35,
      1855.45,
      959.039,
      539.189
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "source": "midrange_camera[0].camera_in_params"
  },
  "time_offsets": {
    "lidar": -0.0054,
    "lidar_frontend": 0.0021,
    "camera": 0.0,
    "rtk_position": 0.0182,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0054 + imu/ins clock lag +0.0075 (corr 0.999); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.99); effective +10.7 ms (se 1.6 ms)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/xingye/truck08/v006_rtktime/calibration.json
run=/home/project/MY-LIVO/results/calib_truck08_rtk1_1020_165417/Log/fusion/1792486745915508034
scan source: round archive scans.bin/sources.bin (mapper deskew)
window 0.0..237.3 s, stride 4, scans front_right_lidar:591, back_lidar:591, front_left_lidar:591, front_lidar:591; ground-band returns 7643735
front_right_lidar: ground returns 1413472, median along-track position +12.2 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 336228, median along-track position -7.2 m (rear-facing -> solved)
front_left_lidar: ground returns 1551064, median along-track position +11.4 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x -0.033 y -0.176 deg (about the LiDAR, rear-axle frame), z +1.04 cm; own-ground change RMS 3.0 cm; flat-ground |res| median 1.54->1.66 cm (n=102002); signed median by range: 4-10 m -0.2->-0.2, 10-20 m -0.1->-0.1, 20-40 m -0.2->-0.2; halves: 0.188 deg / 0.4 cm  [quality: warn; halves differ by 0.188 deg / 0.4 cm]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/xingye/truck08/v007_gcons/calibration.json
```
