# zongyang/truck02/v007_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: zongyang/truck02/v006_rtktime
- lineage: zongyang/truck02/v001_import -> zongyang/truck02/v002_axle -> zongyang/truck02/v003_ground -> zongyang/truck02/v004_driven -> zongyang/truck02/v006_rtktime -> zongyang/truck02/v007_gcons
- created: 2026-10-26T19:15:47+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/zongyang/02/20261026/ros2bag
- dataset: {'path': '/home/project/data/haibo/zongyang/02/20261026/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 239.98, 'has_camera': False}
- run: /home/project/MY-LIVO/results/calib_truck02_rtk1_1026_190734/Log/fusion/1793013098265520663

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -3.37
      ],
      "drpy_deg": [
        0.075,
        -0.01,
        -0.004
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
          -0.011,
          -0.0755,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          -3.37
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.076,
          "z_cm": -3.37,
          "effect_rms_cm": 3.3,
          "halves_rot_deg": 0.018,
          "halves_z_cm": 0.1,
          "median_abs_res_before_cm": 2.12,
          "median_abs_res_after_cm": 1.28
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck02_rtk1_1026_190734/Log/fusion/1793013098265520663",
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
        2.7119,
        4.9226,
        1.8137
      ],
      "rpy_deg": [
        -0.412,
        -0.124,
        -2.587
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2779,
        -1.0872,
        0.8205
      ],
      "rpy_deg": [
        0.45,
        -3.19,
        -90.684
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1791,
        4.8316,
        1.832
      ],
      "rpy_deg": [
        -1.822,
        -0.399,
        178.252
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.073,
        7.0543,
        2.4626
      ],
      "rpy_deg": [
        1.021,
        1.809,
        91.002
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.108,
      7.0643,
      2.1813
    ],
    "rpy_deg": [
      -90.063,
      2.989,
      -0.013
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      611.054443359375,
      895.9816284179688,
      954.4519112515845,
      541.4709836341353
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
    "lidar": -0.007,
    "lidar_frontend": -0.002,
    "camera": 0.0,
    "rtk_position": 0.0985,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0070 + imu/ins clock lag +0.0050 (corr 1.000); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.44); effective +93.5 ms (se 2.3 ms)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck02/v006_rtktime/calibration.json
run=/home/project/MY-LIVO/results/calib_truck02_rtk1_1026_190734/Log/fusion/1793013098265520663
scan source: round archive scans.bin/sources.bin (mapper deskew)
window 0.0..147.3 s, stride 4, scans front_right_lidar:367, back_lidar:367, front_left_lidar:367, front_lidar:367; ground-band returns 3212227
front_right_lidar: ground returns 806935, median along-track position +8.4 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 636007, median along-track position -8.7 m (rear-facing -> solved)
front_left_lidar: ground returns 743941, median along-track position +7.1 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x -0.011 y -0.076 deg (about the LiDAR, rear-axle frame), z -3.37 cm; own-ground change RMS 3.3 cm; flat-ground |res| median 2.12->1.28 cm (n=582631); signed median by range: 4-10 m +1.9->+0.1, 10-20 m +2.1->+0.1, 20-40 m +2.9->-0.1; halves: 0.018 deg / 0.1 cm  [quality: ok]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/zongyang/truck02/v007_gcons/calibration.json
```
