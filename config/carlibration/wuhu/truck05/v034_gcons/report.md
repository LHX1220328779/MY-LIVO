# wuhu/truck05/v034_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: wuhu/truck05/v033_rtktime
- lineage: wuhu/truck05/v001_import -> wuhu/truck05/v029_axle -> wuhu/truck05/v030_ground -> wuhu/truck05/v031_driven -> wuhu/truck05/v033_rtktime -> wuhu/truck05/v034_gcons
- created: 2026-10-14T14:12:47+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260826/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260826/ros2bag', 'kind': 'raw_split', 'segments': 24, 'duration_s': 719.99, 'has_camera': False}
- run: /home/project/MY-LIVO/results/calib_truck05_rtk1_1014_134623/Log/fusion/1791957544629248223

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.51
      ],
      "drpy_deg": [
        -0.004,
        0.26,
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
          0.2598,
          0.0032,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          -0.51
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.26,
          "z_cm": -0.51,
          "effect_rms_cm": 7.01,
          "halves_rot_deg": 0.046,
          "halves_z_cm": 0.23,
          "median_abs_res_before_cm": 4.42,
          "median_abs_res_after_cm": 2.03
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck05_rtk1_1014_134623/Log/fusion/1791957544629248223",
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
        2.833,
        5.4626,
        1.9685
      ],
      "rpy_deg": [
        0.684,
        -2.163,
        -1.131
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.1147,
        -0.756,
        0.8428
      ],
      "rpy_deg": [
        -1.027,
        1.414,
        -90.244
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -2.9937,
        5.3861,
        1.9336
      ],
      "rpy_deg": [
        -0.449,
        -1.921,
        179.68
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0968,
        7.5648,
        0.5356
      ],
      "rpy_deg": [
        0.072,
        -5.301,
        89.874
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0022,
      7.5704,
      2.2531
    ],
    "rpy_deg": [
      -100.385,
      0.003,
      -0.449
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
    "lidar": -0.0368,
    "camera": 0.0,
    "rtk_position": 0.0486,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend": -0.0418,
    "lidar_frontend_source": "axle: lidar -0.0368 + imu/ins clock lag -0.0050 (corr 0.999); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.50); effective +53.6 ms (se 6.0 ms)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v033_rtktime/calibration.json
run=/home/project/MY-LIVO/results/calib_truck05_rtk1_1014_134623/Log/fusion/1791957544629248223
window 0.0..237.4 s, stride 4, scans front_right_lidar:592, back_lidar:592, front_left_lidar:592, front_lidar:592; ground-band returns 8848786
front_right_lidar: ground returns 1747805, median along-track position +9.7 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 914491, median along-track position -13.1 m (rear-facing -> solved)
front_left_lidar: ground returns 2094753, median along-track position +9.5 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x +0.260 y +0.003 deg (about the LiDAR, rear-axle frame), z -0.51 cm; own-ground change RMS 7.0 cm; flat-ground |res| median 4.42->2.03 cm (n=572483); signed median by range: 4-10 m +1.0->-0.6, 10-20 m +5.1->+0.8, 20-40 m +7.8->-0.4; halves: 0.046 deg / 0.2 cm  [quality: ok]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v034_gcons/calibration.json
```
