# wuhu/truck29raw/v003_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29raw/v002_axle
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v002_axle -> wuhu/truck29raw/v003_ground
- created: 2026-09-24T10:16:03+08:00
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
        1.37
      ],
      "drpy_deg": [
        -0.123,
        0.363,
        0.007
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.87
      ],
      "drpy_deg": [
        0.13,
        0.231,
        0.001
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -1.02
      ],
      "drpy_deg": [
        -0.144,
        1.304,
        0.004
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
        -0.11278469308617442,
        0.36526479946606655,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.368028399289161
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.10919657257204225,
        -1.307707939697256,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -1.019922841416937
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.227826190854141,
        -0.13576937248149715,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.8661430880986354
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
        2.7333,
        5.9263,
        1.8266
      ],
      "rpy_deg": [
        -1.014,
        -2.907,
        -1.461
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1071,
        -0.287,
        0.7233
      ],
      "rpy_deg": [
        -0.991,
        0.634,
        -91.379
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1314,
        5.7145,
        1.9774
      ],
      "rpy_deg": [
        -3.354,
        -0.734,
        178.488
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.108,
        8.0271,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.533,
        89.921
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0037,
      8.0291,
      1.7517
    ],
    "rpy_deg": [
      -96.505,
      1.635,
      0.421
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
    "lidar": -0.1463,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v002_axle/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 11.3 cm / P90 35.0 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 12266
back_lidar: stationary aggregated points 2421
front_left_lidar: stationary aggregated points 18492
front_lidar: stationary aggregated points 43514
front_right_lidar: yaw/xy solved=False; matches 3807->3783; |res| median 6.04->4.06 cm; ground 6.08->4.04 cm (n=3702); vertical 4.32->4.13 cm (n=31); correction rot deg (x,y,z)=(-0.113,+0.365,+0.000) trans cm=(+0.0,+0.0,+1.4); iters 17
front_left_lidar: yaw/xy solved=False; matches 4524->4844; |res| median 16.30->5.14 cm; ground 16.60->5.15 cm (n=4742); vertical 3.77->3.93 cm (n=37); correction rot deg (x,y,z)=(+0.109,-1.308,+0.000) trans cm=(+0.0,+0.0,-1.0); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36561->36704; |res| median 8.29->7.96 cm; ground 7.88->7.60 cm (n=31328); vertical n=3947; correction rot deg (x,y,z)=(+0.228,-0.136,+0.000) trans cm=(+0.0,+0.0,+1.9); iters 16
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v003_ground/calibration.json
```
