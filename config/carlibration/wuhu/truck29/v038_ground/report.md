# wuhu/truck29/v038_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29/v037_import
- lineage: wuhu/truck29/v037_import -> wuhu/truck29/v038_ground
- created: 2026-09-24T11:23:29+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- stationary: [2, 20]
- moving: [300, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.3
      ],
      "drpy_deg": [
        -0.124,
        0.359,
        0.012
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        2.18
      ],
      "drpy_deg": [
        0.119,
        0.225,
        0.002
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.63
      ],
      "drpy_deg": [
        -0.129,
        1.311,
        -0.002
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
        -0.10744892445956421,
        0.3641645903488774,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.302440101525321
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.07086824028530513,
        -1.3161739539696258,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.6273631411864251
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.2191186823533359,
        -0.12846910372261602,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.1737373039709738
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
        2.8317,
        5.4452,
        1.9261
      ],
      "rpy_deg": [
        -0.413,
        -5.349,
        -2.543
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0696,
        -0.7051,
        0.6479
      ],
      "rpy_deg": [
        1.437,
        1.223,
        -92.389
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0379,
        5.3396,
        1.8284
      ],
      "rpy_deg": [
        -3.941,
        1.711,
        177.45
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        0.1069,
        7.6128,
        0.051
      ],
      "rpy_deg": [
        -1.002,
        -2.072,
        88.958
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1455,
      7.5982,
      1.7548
    ],
    "rpy_deg": [
      -95.986,
      -0.821,
      -0.616
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
    "lidar": 0.0,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v037_import/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 11.3 cm / P90 35.0 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 12232
back_lidar: stationary aggregated points 2599
front_left_lidar: stationary aggregated points 18396
front_lidar: stationary aggregated points 44456
front_right_lidar: yaw/xy solved=False; matches 3847->3823; |res| median 6.03->4.09 cm; ground 6.03->4.09 cm (n=3753); vertical 8.68->6.84 cm (n=40); correction rot deg (x,y,z)=(-0.107,+0.364,+0.000) trans cm=(+0.0,+0.0,+1.3); iters 24
front_left_lidar: yaw/xy solved=False; matches 4626->4960; |res| median 16.22->4.87 cm; ground 16.51->4.88 cm (n=4837); vertical 4.73->4.44 cm (n=62); correction rot deg (x,y,z)=(+0.071,-1.316,+0.000) trans cm=(+0.0,+0.0,-0.6); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36768->36928; |res| median 8.16->7.96 cm; ground 7.78->7.60 cm (n=31684); vertical n=3928; correction rot deg (x,y,z)=(+0.219,-0.128,+0.000) trans cm=(+0.0,+0.0,+2.2); iters 16
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v038_ground/calibration.json
```
