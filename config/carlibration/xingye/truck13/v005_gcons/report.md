# xingye/truck13/v005_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: xingye/truck13/v004_driven
- lineage: xingye/truck13/v001_import -> xingye/truck13/v002_axle -> xingye/truck13/v003_ground -> xingye/truck13/v004_driven -> xingye/truck13/v005_gcons
- created: 2026-10-28T11:08:04+08:00
- note: pipeline: gcons

## inputs

- bag: /home/project/data/haibo/xingye/13/20261027/ros2bag
- dataset: {'path': '/home/project/data/haibo/xingye/13/20261027/ros2bag', 'kind': 'raw_split', 'segments': 68, 'duration_s': 1020.12, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck13_rtk1_1028_105834/Log/fusion/1793156702111555679

## changes vs parent

```
{
  "lidars": {
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.13
      ],
      "drpy_deg": [
        -0.052,
        -0.02,
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
      "front_right_lidar": {
        "rotvec_deg": [
          0.0119,
          0.0039,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          -0.49
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.013,
          "z_cm": -0.49,
          "effect_rms_cm": 0.3,
          "halves_rot_deg": 0.021,
          "halves_z_cm": 0.48,
          "median_abs_res_before_cm": 0.67,
          "median_abs_res_after_cm": 0.64
        }
      },
      "front_left_lidar": {
        "rotvec_deg": [
          0.0555,
          0.0086,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          -0.13
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.056,
          "z_cm": -0.13,
          "effect_rms_cm": 1.16,
          "halves_rot_deg": 0.052,
          "halves_z_cm": 0.04,
          "median_abs_res_before_cm": 0.52,
          "median_abs_res_after_cm": 0.52
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck13_rtk1_1028_105834/Log/fusion/1793156702111555679",
    "targets": [
      "front_right_lidar",
      "front_left_lidar"
    ],
    "applied": [
      "front_left_lidar"
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
        1.9464,
        7.0057,
        1.0298
      ],
      "rpy_deg": [
        -1.755,
        -0.435,
        14.043
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1422,
        -1.9111,
        0.4543
      ],
      "rpy_deg": [
        0.043,
        -3.423,
        -90.139
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.9445,
        6.9514,
        1.0499
      ],
      "rpy_deg": [
        0.796,
        -0.59,
        167.928
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0002,
        7.0096,
        0.6541
      ],
      "rpy_deg": [
        0.426,
        2.516,
        90.688
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0123,
      7.0409,
      1.4569
    ],
    "rpy_deg": [
      -98.675,
      1.353,
      -0.512
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.15,
      1855.56,
      963.247,
      544.29
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
    "lidar": -0.0502,
    "lidar_frontend": -0.0427,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0502 + imu/ins clock lag +0.0075 (corr 0.997); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/xingye/truck13/v004_driven/calibration.json
run=/home/project/MY-LIVO/results/calib_truck13_rtk1_1028_105834/Log/fusion/1793156702111555679
scan source: round archive scans.bin/sources.bin (mapper deskew)
window 0.0..237.3 s, stride 4, scans front_right_lidar:593, back_lidar:593, front_left_lidar:593, front_lidar:593; ground-band returns 6195212
front_right_lidar: ground returns 1532769, median along-track position -12.4 m (rear-facing -> solved)
back_lidar: ground returns 261645, median along-track position +6.3 m (kept (overlaps the reference, driven stage))
front_left_lidar: ground returns 1597243, median along-track position -10.3 m (rear-facing -> solved)
front_right_lidar: correction rotvec x +0.012 y +0.004 deg (about the LiDAR, rear-axle frame), z -0.49 cm; own-ground change RMS 0.3 cm; flat-ground |res| median 0.67->0.64 cm (n=814443); signed median by range: 4-10 m -0.1->-0.2, 10-20 m +0.2->+0.0, 20-40 m +0.0->-0.1; halves: 0.021 deg / 0.5 cm  [quality: ok]
front_right_lidar: change below 0.8 cm RMS -> kept
front_left_lidar: correction rotvec x +0.056 y +0.009 deg (about the LiDAR, rear-axle frame), z -0.13 cm; own-ground change RMS 1.2 cm; flat-ground |res| median 0.52->0.52 cm (n=1147516); signed median by range: 4-10 m -0.1->-0.1, 10-20 m -0.1->+0.1, 20-40 m -0.4->-0.1; halves: 0.052 deg / 0.0 cm  [quality: ok]
result: applied to front_left_lidar
written /home/project/MY-LIVO/config/carlibration/xingye/truck13/v005_gcons/calibration.json
```
