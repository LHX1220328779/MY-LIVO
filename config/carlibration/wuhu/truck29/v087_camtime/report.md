# wuhu/truck29/v087_camtime

- stage: camtime (相机时间偏移)
- parent: wuhu/truck29/v086_rtktime
- lineage: wuhu/truck29/v001_import -> wuhu/truck29/v082_axle -> wuhu/truck29/v083_ground -> wuhu/truck29/v084_driven -> wuhu/truck29/v086_rtktime -> wuhu/truck29/v087_camtime
- created: 2026-10-12T09:13:41+08:00
- note: pipeline: camera-time

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- duration: 60.0
- start_s: 0.0

## changes vs parent

```
{
  "time_offsets": {
    "camera": {
      "from": 0.0,
      "to": -0.1211
    }
  }
}
```

## metrics

```
{
  "camera_offset_s": -0.1211,
  "estimated_offset_s": -0.1211,
  "applied": true,
  "quality": {
    "ok": true,
    "problems": [],
    "segment_spread_ms": 3.8249962981878465,
    "gain": 1.254
  },
  "yaw": {
    "lag_s": -0.1211107980166752,
    "corr": 0.9153197022448454,
    "gain": 1.2544744287751537,
    "n": 1800,
    "sse_ratio_zero_lag": 1.3167298285638973
  },
  "pitch": {
    "lag_s": -0.11503434394154183,
    "corr": 0.5941459906592645,
    "gain": 0.9802067009538817,
    "n": 1800,
    "sse_ratio_zero_lag": 1.6253611918686321
  },
  "segments": [
    {
      "start_s": 0,
      "lag_s": -0.11516509529873611,
      "corr": 0.9267262899546919
    },
    {
      "start_s": 30,
      "lag_s": -0.11134009900054827,
      "corr": 0.3697300229933422
    }
  ],
  "implied_fx_from_gain": 2060.5627607646343
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
        2.7476,
        6.0173,
        1.8362
      ],
      "rpy_deg": [
        -0.856,
        -3.163,
        -1.23
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1047,
        -0.1903,
        0.6347
      ],
      "rpy_deg": [
        -0.916,
        0.844,
        -91.046
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1068,
        5.7927,
        1.9582
      ],
      "rpy_deg": [
        -3.587,
        -0.628,
        178.823
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
    "camera": -0.1211,
    "rtk_position": -0.0065,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.1335,
    "lidar_frontend_source": "axle: lidar -0.1460 + imu/ins clock lag +0.0125 (corr 0.998); template none replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.71); effective -19.0 ms (se 6.4 ms)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```
