# wuhu/truck05/v018_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck05/v017_axle
- lineage: wuhu/truck05/v008_import -> wuhu/truck05/v017_axle -> wuhu/truck05/v018_ground
- created: 2026-09-26T14:23:28+08:00
- note: mcp-rebuild

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- stationary: [2, 20]
- moving: [1840, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        3.74
      ],
      "drpy_deg": [
        -0.296,
        0.556,
        0.013
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        7.62
      ],
      "drpy_deg": [
        0.694,
        0.875,
        0.006
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.62
      ],
      "drpy_deg": [
        0.252,
        0.245,
        -0.009
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
        -0.2844966981654353,
        0.561672176532908,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        3.7455829832852885
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.25206427117409136,
        -0.24421954531299822,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.6178241240599218
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.8714478099889951,
        -0.6986826223963227,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        7.621104425690888
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
        2.8219,
        5.5326,
        1.9602
      ],
      "rpy_deg": [
        0.656,
        -2.277,
        -1.101
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.1472,
        -0.671,
        0.8477
      ],
      "rpy_deg": [
        -0.963,
        0.964,
        -90.311
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -2.9891,
        5.4679,
        1.936
      ],
      "rpy_deg": [
        -0.417,
        -1.93,
        179.9
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0755,
        7.6571,
        0.5356
      ],
      "rpy_deg": [
        0.058,
        -5.285,
        89.869
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0232,
      7.6631,
      2.2531
    ],
    "rpy_deg": [
      -100.4,
      -0.011,
      -0.455
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
    "lidar": -0.0328,
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
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v017_axle/calibration.json
bag=/home/project/data/haibo/wuhu/05/20260914/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 1840..1880 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 14.9 cm / P90 42.7 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 17346
back_lidar: stationary aggregated points 2940
front_left_lidar: stationary aggregated points 16077
front_lidar: stationary aggregated points 84279
front_right_lidar: yaw/xy solved=False; matches 5175->5109; |res| median 11.08->6.69 cm; ground 11.65->6.69 cm (n=4311); vertical 7.05->6.14 cm (n=268); correction rot deg (x,y,z)=(-0.284,+0.562,+0.000) trans cm=(+0.0,+0.0,+3.7); iters 25
front_left_lidar: yaw/xy solved=False; matches 4836->4998; |res| median 11.94->7.24 cm; ground 12.63->7.21 cm (n=4260); vertical 9.40->6.66 cm (n=266); correction rot deg (x,y,z)=(-0.252,-0.244,+0.000) trans cm=(+0.0,+0.0,-0.6); iters 24
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 27571->30259; |res| median 13.15->6.07 cm; ground 13.23->5.83 cm (n=28642); vertical n=856; correction rot deg (x,y,z)=(+0.871,-0.699,+0.000) trans cm=(+0.0,+0.0,+7.6); iters 14
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v018_ground/calibration.json
```
