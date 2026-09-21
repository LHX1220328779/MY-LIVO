# wuhu/truck29/v002_ground

- stage: ground
- parent: wuhu/truck29/v001_import
- created: 2026-09-21T17:58:52+08:00
- note: stage1 ground roll/pitch/z (tool chain test)

## inputs

- bag: results/prepared/wuhu_livo
- stationary: [2, 20]
- moving: [300, 40]

## metrics

```
{
  "corrections": {
    "front_right_lidar": {
      "rotvec_deg": [
        -0.11125897970222925,
        0.3682551849600907,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.115475084573922
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.11457588306250073,
        -1.311793031071289,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -1.0242244162236145
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.2273957639016077,
        -0.13440698910248985,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.5640244099732254
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
        2.8123,
        5.6996,
        1.8265
      ],
      "rpy_deg": [
        -0.87,
        -3.052,
        -1.234
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0004,
        -0.5225,
        0.7005
      ],
      "rpy_deg": [
        -0.843,
        0.773,
        -91.143
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.052,
        5.4635,
        1.9638
      ],
      "rpy_deg": [
        -3.495,
        -0.581,
        178.727
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.033,
        7.793,
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
      0.0742,
      7.7914,
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
    "lidar": -0.125,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v001_import/calibration.json
bag=results/prepared/wuhu_livo
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
front_right_lidar: stationary aggregated points 12362
back_lidar: stationary aggregated points 2487
front_left_lidar: stationary aggregated points 18449
front_lidar: stationary aggregated points 44072
front_right_lidar: yaw/xy solved=False; matches 3762->3761; |res| median 6.06->4.15 cm; ground 6.11->4.12 cm (n=3686); vertical 4.45->5.29 cm (n=35); correction rot deg (x,y,z)=(-0.111,+0.368,+0.000) trans cm=(+0.0,+0.0,+1.1); iters 18
front_left_lidar: yaw/xy solved=False; matches 4629->4809; |res| median 15.79->4.76 cm; ground 16.15->4.73 cm (n=4664); vertical 5.26->5.16 cm (n=62); correction rot deg (x,y,z)=(+0.115,-1.312,+0.000) trans cm=(+0.0,+0.0,-1.0); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36824->37010; |res| median 8.41->8.06 cm; ground 8.02->7.74 cm (n=31660); vertical n=3947; correction rot deg (x,y,z)=(+0.227,-0.134,+0.000) trans cm=(+0.0,+0.0,+1.6); iters 18
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v002_ground/calibration.json
```
