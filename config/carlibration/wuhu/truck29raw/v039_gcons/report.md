# wuhu/truck29raw/v039_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: wuhu/truck29raw/v038_rtktime
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v034_axle -> wuhu/truck29raw/v035_ground -> wuhu/truck29raw/v036_driven -> wuhu/truck29raw/v038_rtktime -> wuhu/truck29raw/v039_gcons
- created: 2026-10-15T08:26:10+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29raw_rtk1_1015_080428/Log/fusion/1792023444170145977

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        5.18
      ],
      "drpy_deg": [
        -0.01,
        0.179,
        0.0
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
          0.1795,
          0.006,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          5.18
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.18,
          "z_cm": 5.18,
          "effect_rms_cm": 2.56,
          "halves_rot_deg": 0.085,
          "halves_z_cm": 0.22,
          "median_abs_res_before_cm": 2.17,
          "median_abs_res_after_cm": 1.04
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck29raw_rtk1_1015_080428/Log/fusion/1792023444170145977",
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
        2.7755,
        5.9149,
        1.8312
      ],
      "rpy_deg": [
        -1.031,
        -3.057,
        -1.219
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0611,
        -0.2984,
        0.7111
      ],
      "rpy_deg": [
        -1.047,
        0.481,
        -91.365
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0915,
        5.6621,
        2.0104
      ],
      "rpy_deg": [
        3.243,
        -4.537,
        178.24
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.106,
        7.9492,
        0.051
      ],
      "rpy_deg": [
        1.485,
        -1.56,
        89.984
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0067,
      7.9506,
      1.7516
    ],
    "rpy_deg": [
      -96.477,
      1.668,
      0.485
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
    "lidar": -0.0361,
    "camera": 0.0,
    "rtk_position": 0.0151,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.84); effective +10.1 ms (se 2.0 ms)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v038_rtktime/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29raw_rtk1_1015_080428/Log/fusion/1792023444170145977
scan source: round archive scans.bin/sources.bin (mapper deskew)
window 0.0..117.3 s, stride 4, scans front_right_lidar:292, back_lidar:292, front_left_lidar:292, front_lidar:292; ground-band returns 3362080
front_right_lidar: ground returns 636707, median along-track position +10.6 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 302862, median along-track position -12.1 m (rear-facing -> solved)
front_left_lidar: ground returns 781497, median along-track position +10.3 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x +0.179 y +0.006 deg (about the LiDAR, rear-axle frame), z +5.18 cm; own-ground change RMS 2.6 cm; flat-ground |res| median 2.17->1.04 cm (n=138415); signed median by range: 4-10 m -3.4->-0.5, 10-20 m -0.4->+0.6, 20-40 m +1.4->-0.6; halves: 0.085 deg / 0.2 cm  [quality: ok]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v039_gcons/calibration.json
```
