# wuhu/truck05/v010_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck05/v009_axle
- lineage: wuhu/truck05/v008_import -> wuhu/truck05/v009_axle -> wuhu/truck05/v010_ground
- created: 2026-09-24T16:05:13+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- stationary: [2, 20]
- moving: [580, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        4.52
      ],
      "drpy_deg": [
        -0.315,
        0.577,
        0.014
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        4.58
      ],
      "drpy_deg": [
        0.937,
        0.696,
        0.007
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -1.02
      ],
      "drpy_deg": [
        0.233,
        0.247,
        -0.008
      ]
    }
  }
}
```

## metrics

```
{
  "corrections": {
    "front_right_lidar": {
      "rotvec_deg": [
        -0.3038116671628296,
        0.5824862358503357,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        4.515085132903587
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.23316749866069172,
        -0.2461681982492946,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -1.017195227904469
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.6904040946514801,
        -0.9408842705242677,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        4.5851102286924
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    }
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
        2.8011,
        5.4403,
        1.9667
      ],
      "rpy_deg": [
        0.652,
        -2.243,
        -1.095
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.1268,
        -0.7633,
        0.815
      ],
      "rpy_deg": [
        -0.733,
        0.8,
        -90.304
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0098,
        5.375,
        1.9321
      ],
      "rpy_deg": [
        -0.451,
        -1.941,
        179.907
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
      1849.37,
      1849.37,
      1003.31,
      502.72
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
            1849.32,
            1831.26,
            1004.09,
            499.42
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            -0.002,
            7.529
          ],
          "ground_z_rear_m": -1.283
        },
        "square": {
          "intrinsics": [
            1849.37,
            1849.37,
            1003.31,
            502.72
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            0.001,
            7.535
          ],
          "ground_z_rear_m": -1.249
        }
      },
      "model": "square pixels (fx = fy)",
      "rms_px": 0.05,
      "camera_height_m": 3.502,
      "deviation_vs_truck29_defaults": {
        "fx_pct": 12.59,
        "fy_pct": -0.33,
        "cx_px": 32.3,
        "cy_px": -31.5
      }
    },
    "source": "supplier ground-distance curves equation_*m (square pixels (fx = fy), rms 0.05 px)"
  },
  "time_offsets": {
    "lidar": -0.0368,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v009_axle/calibration.json
bag=/home/project/data/haibo/wuhu/05/20260914/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 580..620 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 14.9 cm / P90 42.7 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 17393
back_lidar: stationary aggregated points 3021
front_left_lidar: stationary aggregated points 15967
front_lidar: stationary aggregated points 84681
front_right_lidar: yaw/xy solved=False; matches 5118->5081; |res| median 10.71->6.64 cm; ground 11.56->6.73 cm (n=4326); vertical 6.34->6.12 cm (n=283); correction rot deg (x,y,z)=(-0.304,+0.582,+0.000) trans cm=(+0.0,+0.0,+4.5); iters 25
front_left_lidar: yaw/xy solved=False; matches 4907->5104; |res| median 11.77->7.54 cm; ground 12.33->7.44 cm (n=4307); vertical 9.98->7.83 cm (n=255); correction rot deg (x,y,z)=(-0.233,-0.246,+0.000) trans cm=(+0.0,+0.0,-1.0); iters 24
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 60423->62814; |res| median 15.86->11.88 cm; ground 13.63->9.48 cm (n=44021); vertical n=17867; correction rot deg (x,y,z)=(+0.690,-0.941,+0.000) trans cm=(+0.0,+0.0,+4.6); iters 24
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v010_ground/calibration.json
```
