# wuhu/truck29-0930/v016_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: wuhu/truck29-0930/v015_driven
- lineage: wuhu/truck29-0930/v002_import -> wuhu/truck29-0930/v013_axle -> wuhu/truck29-0930/v014_ground -> wuhu/truck29-0930/v015_driven -> wuhu/truck29-0930/v016_gcons
- created: 2026-10-19T17:32:18+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29-0930_rtk1_1019_172507/Log/fusion/1792402202624748882

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        3.73
      ],
      "drpy_deg": [
        0.013,
        0.126,
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
          0.1258,
          -0.0155,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          3.73
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.127,
          "z_cm": 3.73,
          "effect_rms_cm": 1.86,
          "halves_rot_deg": 0.06,
          "halves_z_cm": 0.4,
          "median_abs_res_before_cm": 1.77,
          "median_abs_res_after_cm": 1.09
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck29-0930_rtk1_1019_172507/Log/fusion/1792402202624748882",
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
        2.7574,
        5.9035,
        1.8372
      ],
      "rpy_deg": [
        -1.038,
        -3.039,
        -1.258
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0973,
        -0.3234,
        0.7176
      ],
      "rpy_deg": [
        -1.051,
        0.467,
        -91.387
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1226,
        5.6504,
        2.0188
      ],
      "rpy_deg": [
        3.26,
        -4.535,
        178.187
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1296,
        7.9462,
        0.051
      ],
      "rpy_deg": [
        1.477,
        -1.57,
        89.955
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0171,
      7.9472,
      1.7516
    ],
    "rpy_deg": [
      -96.467,
      1.661,
      0.456
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
    "source": "midrange_camera[0].camera_in_params"
  },
  "time_offsets": {
    "lidar": -0.0372,
    "lidar_frontend": -0.0322,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0372 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v015_driven/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29-0930_rtk1_1019_172507/Log/fusion/1792402202624748882
scan source: round archive scans.bin/sources.bin (mapper deskew)
window 0.0..177.3 s, stride 4, scans front_right_lidar:442, back_lidar:442, front_left_lidar:442, front_lidar:442; ground-band returns 4988957
front_right_lidar: ground returns 1065592, median along-track position +10.5 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 486184, median along-track position -12.6 m (rear-facing -> solved)
front_left_lidar: ground returns 1222042, median along-track position +10.3 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x +0.126 y -0.015 deg (about the LiDAR, rear-axle frame), z +3.73 cm; own-ground change RMS 1.9 cm; flat-ground |res| median 1.77->1.09 cm (n=273512); signed median by range: 4-10 m -2.6->-0.6, 10-20 m -0.2->+0.6, 20-40 m +0.7->-0.4; halves: 0.060 deg / 0.4 cm  [quality: ok]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v016_gcons/calibration.json
```
