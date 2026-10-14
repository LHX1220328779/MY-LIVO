# wuhu/truck29raw/v019_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: wuhu/truck29raw/v018_driven
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v015_axle -> wuhu/truck29raw/v017_ground -> wuhu/truck29raw/v018_driven -> wuhu/truck29raw/v019_gcons
- created: 2026-10-14T16:56:11+08:00
- note: pipeline rerun after ground-stage plane-start fix (front_left_lidar moved vs v001_import)

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29raw_rtk1_1014_164046/Log/fusion/1791967887386031336

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        0.73
      ],
      "drpy_deg": [
        0.035,
        0.217,
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
          0.2168,
          -0.0405,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          0.73
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "warn",
          "reasons": [
            "halves differ by 0.108 deg / 0.1 cm"
          ],
          "rotation_deg": 0.221,
          "z_cm": 0.73,
          "effect_rms_cm": 4.9,
          "halves_rot_deg": 0.108,
          "halves_z_cm": 0.13,
          "median_abs_res_before_cm": 2.99,
          "median_abs_res_after_cm": 1.32
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck29raw_rtk1_1014_164046/Log/fusion/1791967887386031336",
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
        2.7567,
        5.916,
        1.7827
      ],
      "rpy_deg": [
        -0.921,
        -2.978,
        -1.239
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0877,
        -0.2857,
        0.6654
      ],
      "rpy_deg": [
        -0.981,
        0.51,
        -91.375
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1248,
        5.6735,
        1.929
      ],
      "rpy_deg": [
        3.024,
        -4.479,
        178.246
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1291,
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
      -0.0164,
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
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v018_driven/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29raw_rtk1_1014_164046/Log/fusion/1791967887386031336
window 0.0..117.3 s, stride 4, scans front_right_lidar:291, back_lidar:291, front_left_lidar:291, front_lidar:291; ground-band returns 4520469
front_right_lidar: ground returns 845656, median along-track position +11.3 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 390953, median along-track position -13.0 m (rear-facing -> solved)
front_left_lidar: ground returns 1049041, median along-track position +11.0 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x +0.217 y -0.040 deg (about the LiDAR, rear-axle frame), z +0.73 cm; own-ground change RMS 4.9 cm; flat-ground |res| median 2.99->1.32 cm (n=181665); signed median by range: 4-10 m -0.0->-0.8, 10-20 m +3.8->+0.6, 20-40 m +5.6->-0.4; halves: 0.108 deg / 0.1 cm  [quality: warn; halves differ by 0.108 deg / 0.1 cm]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v019_gcons/calibration.json
```
