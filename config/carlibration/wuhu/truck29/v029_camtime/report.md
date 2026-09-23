# wuhu/truck29/v029_camtime

- stage: camtime (相机时间偏移)
- parent: wuhu/truck29/v028_camloop
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v012_ground -> wuhu/truck29/v013_driven -> wuhu/truck29/v014_camtime -> wuhu/truck29/v018_camtime -> wuhu/truck29/v019_camlidar -> wuhu/truck29/v022_camloop -> wuhu/truck29/v023_camtime -> wuhu/truck29/v024_camlidar -> wuhu/truck29/v027_camloop -> wuhu/truck29/v028_camloop -> wuhu/truck29/v029_camtime
- created: 2026-09-23T08:16:20+08:00
- note: pipeline: camera-time

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}

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
        2.835,
        5.769,
        1.8093
      ],
      "rpy_deg": [
        -0.802,
        -3.13,
        -1.216
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0076,
        -0.5292,
        0.6086
      ],
      "rpy_deg": [
        -0.979,
        0.794,
        -91.174
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.033,
        5.5449,
        1.9503
      ],
      "rpy_deg": [
        -3.582,
        -0.661,
        178.827
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
      -96.347,
      1.471,
      0.559
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
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```
