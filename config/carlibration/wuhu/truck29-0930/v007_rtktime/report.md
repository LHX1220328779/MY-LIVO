# wuhu/truck29-0930/v007_rtktime

- stage: rtktime (RTK 位置关联时间)
- parent: wuhu/truck29-0930/v005_driven
- lineage: wuhu/truck29-0930/v002_import -> wuhu/truck29-0930/v003_axle -> wuhu/truck29-0930/v004_ground -> wuhu/truck29-0930/v005_driven -> wuhu/truck29-0930/v007_rtktime
- created: 2026-10-19T17:08:15+08:00
- note: RTK position time association +0.0250 -> +0.0209 s (secant of both rounds (measured gain 0.74))

## inputs

- run: /home/project/MY-LIVO/results/calib_truck29-0930_rtk1_1019_165333/Log/fusion/1792400581509476815
- run2: /home/project/MY-LIVO/results/calib_truck29-0930_rtk2_1019_165333/Log/fusion/1792400734721316050

## changes vs parent

```
{
  "time_offsets": {
    "rtk_position": {
      "from": 0.025,
      "to": 0.0209
    }
  }
}
```

## metrics

```
{
  "rtk_time": {
    "parent": "wuhu/truck29-0930/v005_driven",
    "parent_rtk_position_s": 0.025,
    "round1": {
      "run": "/home/project/MY-LIVO/results/calib_truck29-0930_rtk1_1019_165333/Log/fusion/1792400581509476815",
      "effective_s": 0.020000000000000004,
      "correction_s": -0.0029993431422225326,
      "se_s": 0.0005797116713083032,
      "observable": true,
      "frames": 887,
      "informative_voxels": 13391
    },
    "round2": {
      "run": "/home/project/MY-LIVO/results/calib_truck29-0930_rtk2_1019_165333/Log/fusion/1792400734721316050",
      "effective_s": 0.015000000000000003,
      "correction_s": 0.000701260411787437,
      "se_s": 0.0007940038952261707,
      "observable": true,
      "frames": 887,
      "informative_voxels": 13351
    },
    "method": "secant of both rounds (measured gain 0.74)",
    "gain": 0.74,
    "best_effective_s": 0.01595,
    "best_se_s": 0.00088,
    "rtk_position_s": {
      "from": 0.025,
      "to": 0.0209
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
        2.7568,
        5.9059,
        1.8332
      ],
      "rpy_deg": [
        -1.026,
        -3.031,
        -1.254
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0977,
        -0.3133,
        0.6723
      ],
      "rpy_deg": [
        -1.067,
        0.344,
        -91.379
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1232,
        5.651,
        2.0183
      ],
      "rpy_deg": [
        3.257,
        -4.535,
        178.187
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1296,
        7.9462,
        0.051
      ],
      "rpy_deg": [
        1.477,
        -1.57,
        89.955
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0171,
      7.9472,
      1.7516
    ],
    "rpy_deg": [
      -96.467,
      1.661,
      0.456
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
    "source": "midrange_camera[0].camera_in_params"
  },
  "time_offsets": {
    "lidar": -0.0372,
    "lidar_frontend": -0.0322,
    "camera": 0.0,
    "rtk_position": 0.0209,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0372 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.74); effective +15.9 ms (se 0.9 ms)"
  }
}
```

## stage report

```
round 1: effective +20.00 ms -> correction -3.00 ms (se 0.58)
round 2: effective +15.00 ms -> correction +0.70 ms (se 0.79)
secant of both rounds (measured gain 0.74): best effective +15.95 ms (se 0.88 ms); rtk_position +0.0250 -> +0.0209 s

RTK time correction -3.00 ms (se 0.58 ms) -> best effective offset +17.00 ms (run +20.00); voxels 17104, informative 13391; observable
RTK time correction +0.70 ms (se 0.79 ms) -> best effective offset +15.70 ms (run +15.00); voxels 17091, informative 13351; observable
```
