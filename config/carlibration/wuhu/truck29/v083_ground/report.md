# wuhu/truck29/v083_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29/v082_axle
- lineage: wuhu/truck29/v001_import -> wuhu/truck29/v082_axle -> wuhu/truck29/v083_ground
- created: 2026-10-12T09:04:29+08:00
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
        0.87
      ],
      "drpy_deg": [
        -0.108,
        0.37,
        0.006
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.58
      ],
      "drpy_deg": [
        0.11,
        0.24,
        0.001
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.98
      ],
      "drpy_deg": [
        -0.143,
        1.305,
        0.003
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
        -0.0999211506009302,
        0.3719192380621204,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        0.8759427706916351
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.11389673096451969,
        -1.308186192471846,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.9840694128692915
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.2370332954650712,
        -0.11502846956173729,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.5740187582225518
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
        2.7364,
        5.9468,
        1.8241
      ],
      "rpy_deg": [
        -0.858,
        -3.048,
        -1.235
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0755,
        -0.2752,
        0.7006
      ],
      "rpy_deg": [
        -0.863,
        0.783,
        -91.143
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1279,
        5.7108,
        1.9642
      ],
      "rpy_deg": [
        -3.494,
        -0.585,
        178.727
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1089,
        8.0403,
        0.051
      ],
      "rpy_deg": [
        1.3,
        -1.67,
        90.16
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0017,
      8.0387,
      1.7519
    ],
    "rpy_deg": [
      -96.438,
      1.487,
      0.699
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
    "lidar": -0.146,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.1335,
    "lidar_frontend_source": "axle: lidar -0.1460 + imu/ins clock lag +0.0125 (corr 0.998); template none replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v082_axle/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 11.3 cm / P90 35.0 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 12229
back_lidar: stationary aggregated points 2409
front_left_lidar: stationary aggregated points 18490
front_lidar: stationary aggregated points 43474
front_right_lidar: yaw/xy solved=False; matches 3783->3764; |res| median 6.13->4.10 cm; ground 6.13->4.09 cm (n=3691); vertical 7.48->5.68 cm (n=40); correction rot deg (x,y,z)=(-0.100,+0.372,+0.000) trans cm=(+0.0,+0.0,+0.9); iters 16
front_left_lidar: yaw/xy solved=False; matches 4507->4781; |res| median 16.48->4.83 cm; ground 16.63->4.83 cm (n=4646); vertical 6.88->4.90 cm (n=67); correction rot deg (x,y,z)=(+0.114,-1.308,+0.000) trans cm=(+0.0,+0.0,-1.0); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36483->36659; |res| median 8.31->7.97 cm; ground 7.93->7.62 cm (n=31378); vertical n=3909; correction rot deg (x,y,z)=(+0.237,-0.115,+0.000) trans cm=(+0.0,+0.0,+1.6); iters 15
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v083_ground/calibration.json
```
