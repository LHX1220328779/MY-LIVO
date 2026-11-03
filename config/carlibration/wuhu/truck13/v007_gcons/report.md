# wuhu/truck13/v007_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: wuhu/truck13/v006_rtktime
- lineage: wuhu/truck13/v001_import -> wuhu/truck13/v002_axle -> wuhu/truck13/v003_ground -> wuhu/truck13/v004_driven -> wuhu/truck13/v006_rtktime -> wuhu/truck13/v007_gcons
- created: 2026-10-26T17:26:12+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/wuhu/13/1/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/13/1/ros2bag', 'kind': 'raw_split', 'segments': 2, 'duration_s': 60.0, 'has_camera': False}
- run: /home/project/MY-LIVO/results/calib_truck13_rtk1_1026_172302/Log/fusion/1793006691611828688

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.24
      ],
      "drpy_deg": [
        0.028,
        -0.003,
        0.0
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -1.64
      ],
      "drpy_deg": [
        -0.061,
        -0.026,
        0.002
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
      "front_right_lidar": {
        "rotvec_deg": [
          0.0278,
          -0.0034,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          -0.23
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.028,
          "z_cm": -0.23,
          "effect_rms_cm": 0.59,
          "halves_rot_deg": 0.089,
          "halves_z_cm": 0.76,
          "median_abs_res_before_cm": 1.17,
          "median_abs_res_after_cm": 1.18
        }
      },
      "front_left_lidar": {
        "rotvec_deg": [
          0.0608,
          0.0255,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          -1.65
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "warn",
          "reasons": [
            "halves differ by 0.054 deg / 1.6 cm"
          ],
          "rotation_deg": 0.066,
          "z_cm": -1.65,
          "effect_rms_cm": 1.55,
          "halves_rot_deg": 0.054,
          "halves_z_cm": 1.56,
          "median_abs_res_before_cm": 1.0,
          "median_abs_res_after_cm": 0.92
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck13_rtk1_1026_172302/Log/fusion/1793006691611828688",
    "targets": [
      "front_right_lidar",
      "front_left_lidar"
    ],
    "applied": [
      "front_left_lidar",
      "front_right_lidar"
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
        2.8244,
        5.6877,
        2.2795
      ],
      "rpy_deg": [
        -1.442,
        -0.99,
        -1.201
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0094,
        -0.6493,
        0.8899
      ],
      "rpy_deg": [
        -0.51,
        1.54,
        -91.242
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0863,
        5.6514,
        2.2627
      ],
      "rpy_deg": [
        1.488,
        -1.936,
        179.82
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0242,
        7.7501,
        0.4596
      ],
      "rpy_deg": [
        0.215,
        -1.686,
        89.873
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0755,
      7.8739,
      2.2451
    ],
    "rpy_deg": [
      -102.142,
      1.542,
      -4.24
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1849.47862,
      1848.72893,
      1004.25076,
      503.038474
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
    "lidar": 0.021,
    "lidar_frontend": 0.036,
    "camera": 0.0,
    "rtk_position": -0.0172,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar +0.0210 + imu/ins clock lag +0.0150 (corr 0.996); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.31); effective -32.2 ms (se 21.4 ms)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck13/v006_rtktime/calibration.json
run=/home/project/MY-LIVO/results/calib_truck13_rtk1_1026_172302/Log/fusion/1793006691611828688
scan source: round archive scans.bin/sources.bin (mapper deskew)
window 0.0..35.3 s, stride 4, scans front_right_lidar:88, back_lidar:88, front_left_lidar:88, front_lidar:88; ground-band returns 940698
front_right_lidar: ground returns 239462, median along-track position -8.2 m (rear-facing -> solved)
back_lidar: ground returns 99484, median along-track position +9.3 m (kept (overlaps the reference, driven stage))
front_left_lidar: ground returns 272956, median along-track position -11.7 m (rear-facing -> solved)
front_right_lidar: correction rotvec x +0.028 y -0.003 deg (about the LiDAR, rear-axle frame), z -0.23 cm; own-ground change RMS 0.6 cm; flat-ground |res| median 1.17->1.18 cm (n=167818); signed median by range: 4-10 m -0.1->-0.4, 10-20 m -0.9->-0.8, 20-40 m +0.5->+0.9; halves: 0.089 deg / 0.8 cm  [quality: ok]
front_left_lidar: correction rotvec x +0.061 y +0.026 deg (about the LiDAR, rear-axle frame), z -1.65 cm; own-ground change RMS 1.6 cm; flat-ground |res| median 1.00->0.92 cm (n=199108); signed median by range: 4-10 m +0.7->+0.1, 10-20 m -0.1->-0.3, 20-40 m +0.0->+0.3; halves: 0.054 deg / 1.6 cm  [quality: warn; halves differ by 0.054 deg / 1.6 cm]
result: applied to front_left_lidar, front_right_lidar
written /home/project/MY-LIVO/config/carlibration/wuhu/truck13/v007_gcons/calibration.json
```
