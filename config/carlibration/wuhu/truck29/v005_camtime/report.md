# wuhu/truck29/v005_camtime

- stage: camtime
- parent: wuhu/truck29/v003_driven
- created: 2026-09-21T18:06:57+08:00
- note: camera offset from flow/gyro

## inputs

- bag: results/prepared/wuhu_livo_camera

## metrics

```
{
  "camera_offset_s": -0.115,
  "yaw": {
    "lag_s": -0.11500473391901006,
    "corr": 0.9211172864822589,
    "gain": 1.1453223942592208,
    "n": 3599,
    "sse_ratio_zero_lag": 1.42776102555733
  },
  "pitch": {
    "lag_s": -0.11524333154821957,
    "corr": 0.5729316129177794,
    "gain": 0.9454290445863256,
    "n": 3599,
    "sse_ratio_zero_lag": 1.5107345315937477
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
    },
    {
      "start_s": 60,
      "lag_s": -0.11185331584775143,
      "corr": 0.9722875848582033
    },
    {
      "start_s": 90,
      "lag_s": -0.1178901438345484,
      "corr": 0.9803513941912821
    }
  ],
  "implied_fx_from_gain": 1881.2728426713416
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
        2.8296,
        5.7648,
        1.8091
      ],
      "rpy_deg": [
        -0.804,
        -3.132,
        -1.22
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0042,
        -0.5528,
        0.5995
      ],
      "rpy_deg": [
        -0.988,
        0.76,
        -91.154
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0469,
        5.536,
        1.9515
      ],
      "rpy_deg": [
        -3.574,
        -0.665,
        178.807
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
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "optical flow vs gyro on results/prepared/wuhu_livo_camera (corr 0.92)"
  }
}
```
