# xingye/truck13/v008_camtime

- stage: camtime (相机时间偏移)
- parent: xingye/truck13/v005_gcons
- lineage: xingye/truck13/v001_import -> xingye/truck13/v002_axle -> xingye/truck13/v003_ground -> xingye/truck13/v004_driven -> xingye/truck13/v005_gcons -> xingye/truck13/v008_camtime
- created: 2026-10-28T11:25:27+08:00
- note: pipeline: camera-time

## inputs

- bag: /home/project/data/haibo/xingye/13/20261027/ros2bag
- dataset: {'path': '/home/project/data/haibo/xingye/13/20261027/ros2bag', 'kind': 'raw_split', 'segments': 68, 'duration_s': 1020.12, 'has_camera': True}
- duration: 60.0
- start_s: 337.0

## changes vs parent

```
{
  "time_offsets": {
    "camera": {
      "from": 0.0,
      "to": -0.0109
    }
  }
}
```

## metrics

```
{
  "camera_offset_s": -0.0109,
  "estimated_offset_s": -0.0109,
  "applied": true,
  "quality": {
    "ok": true,
    "problems": [],
    "segment_spread_ms": null,
    "gain": 1.167
  },
  "yaw": {
    "lag_s": -0.01085632086070022,
    "corr": 0.9956793537553037,
    "gain": 1.1673967594655903,
    "n": 1800,
    "sse_ratio_zero_lag": 1.1448494751946483
  },
  "pitch": {
    "lag_s": -0.011648565755421912,
    "corr": 0.8571096471593235,
    "gain": 0.9068716812103546,
    "n": 1800,
    "sse_ratio_zero_lag": 1.0560073205361185
  },
  "segments": [
    {
      "start_s": 0,
      "lag_s": -0.011169202810124774,
      "corr": 0.9948556413619695
    },
    {
      "start_s": 30,
      "note": "insufficient yaw excitation"
    }
  ],
  "implied_fx_from_gain": 1917.0405885564192
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
    "camera": -0.0109,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0502 + imu/ins clock lag +0.0075 (corr 0.997); template -0.1250 replaced",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/xingye/13/20261027/ros2bag (corr 1.00)"
  }
}
```
