# wuhu/truck28/v003_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck28/v002_axle
- lineage: wuhu/truck28/v001_import -> wuhu/truck28/v002_axle -> wuhu/truck28/v003_ground
- created: 2026-10-11T12:23:39+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu/28/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/ros2bag', 'kind': 'raw_split', 'segments': 82, 'duration_s': 1229.98, 'has_camera': True}
- stationary: [2, 20]
- moving: [11120, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        1.08,
        0.65,
        4.71
      ],
      "drpy_deg": [
        -0.275,
        -0.21,
        0.094
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        0.5
      ],
      "drpy_deg": [
        0.392,
        -0.641,
        0.02
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -2.44,
        -2.87,
        1.48
      ],
      "drpy_deg": [
        0.352,
        0.797,
        0.421
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
        -0.27651506850618773,
        -0.20821578874699911,
        0.10490463652096349
      ],
      "trans_cm": [
        1.0789790468643694,
        0.6566310819041327,
        4.709533939478521
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.37342249707885283,
        -0.7874703233058586,
        0.4239631601655954
      ],
      "trans_cm": [
        -2.4383519123394946,
        -2.873057954513018,
        1.4809266994222092
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.6457236700583545,
        -0.38310355017738124,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        0.5055247196074156
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
        2.7394,
        5.7985,
        1.4258
      ],
      "rpy_deg": [
        0.313,
        2.342,
        -0.227
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2347,
        -0.3936,
        0.3484
      ],
      "rpy_deg": [
        -2.11,
        2.581,
        -90.705
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0971,
        5.5466,
        1.6359
      ],
      "rpy_deg": [
        -1.024,
        -0.091,
        178.649
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1426,
        7.8507,
        -0.1568
      ],
      "rpy_deg": [
        2.071,
        -3.4,
        89.82
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1951,
      7.7526,
      1.5925
    ],
    "rpy_deg": [
      -100.421,
      0.211,
      0.764
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
    "lidar": -0.0151,
    "lidar_frontend": -0.0026,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0151 + imu/ins clock lag +0.0125 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck28/v002_axle/calibration.json
bag=/home/project/data/haibo/wuhu/28/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 11120..11160 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 5.5 cm / P90 15.2 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 17792
back_lidar: stationary aggregated points 8523
front_left_lidar: stationary aggregated points 17130
front_lidar: stationary aggregated points 28308
front_right_lidar: yaw/xy solved=True; matches 3126->3151; |res| median 8.01->3.77 cm; ground 11.09->4.23 cm (n=2233); vertical 2.35->2.36 cm (n=601); correction rot deg (x,y,z)=(-0.277,-0.208,+0.105) trans cm=(+1.1,+0.7,+4.7); iters 15
front_left_lidar: yaw/xy solved=True; matches 2348->2459; |res| median 10.78->3.25 cm; ground 13.78->3.73 cm (n=1697); vertical 7.60->1.91 cm (n=564); correction rot deg (x,y,z)=(-0.373,-0.787,+0.424) trans cm=(-2.4,-2.9,+1.5); iters 11
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 53402->54486; |res| median 15.83->7.96 cm; ground 14.98->8.32 cm (n=43633); vertical n=1702; correction rot deg (x,y,z)=(-0.646,-0.383,+0.000) trans cm=(+0.0,+0.0,+0.5); iters 25
written /home/project/MY-LIVO/config/carlibration/wuhu/truck28/v003_ground/calibration.json
```
