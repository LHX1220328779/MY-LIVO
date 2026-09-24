# wuhu/truck29raw/v005_camtime

- stage: camtime (相机时间偏移)
- parent: wuhu/truck29raw/v004_driven
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v002_axle -> wuhu/truck29raw/v003_ground -> wuhu/truck29raw/v004_driven -> wuhu/truck29raw/v005_camtime
- created: 2026-09-24T10:19:34+08:00
- note: pipeline: camera-time

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- duration: 60.0

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
        2.756,
        5.9895,
        1.8333
      ],
      "rpy_deg": [
        -1.001,
        -3.026,
        -1.433
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1094,
        -0.311,
        0.6862
      ],
      "rpy_deg": [
        -1.079,
        0.746,
        -91.392
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1269,
        5.7908,
        1.9755
      ],
      "rpy_deg": [
        -3.418,
        -0.801,
        178.584
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
    "camera": -0.1211,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```
