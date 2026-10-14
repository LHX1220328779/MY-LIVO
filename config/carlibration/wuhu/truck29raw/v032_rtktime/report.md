# wuhu/truck29raw/v032_rtktime

- stage: rtktime (RTK 位置关联时间)
- parent: wuhu/truck29raw/v030_driven
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v028_axle -> wuhu/truck29raw/v029_ground -> wuhu/truck29raw/v030_driven -> wuhu/truck29raw/v032_rtktime
- created: 2026-10-14T23:45:53+08:00
- note: RTK position time association +0.0250 -> +0.0146 s (secant of both rounds (measured gain 0.80))

## inputs

- run: /home/project/MY-LIVO/results/calib_truck29raw_rtk1_1014_231631/Log/fusion/1791992258110968407
- run2: /home/project/MY-LIVO/results/calib_truck29raw_rtk2_1014_231631/Log/fusion/1791992500540226741

## changes vs parent

```
{
  "time_offsets": {
    "rtk_position": {
      "from": 0.025,
      "to": 0.0146
    }
  }
}
```

## metrics

```
{
  "rtk_time": {
    "parent": "wuhu/truck29raw/v030_driven",
    "parent_rtk_position_s": 0.025,
    "round1": {
      "run": "/home/project/MY-LIVO/results/calib_truck29raw_rtk1_1014_231631/Log/fusion/1791992258110968407",
      "effective_s": 0.02,
      "correction_s": -0.008376444846651957,
      "se_s": 0.0021168978609876076,
      "observable": true,
      "frames": 1176,
      "informative_voxels": 12260
    },
    "round2": {
      "run": "/home/project/MY-LIVO/results/calib_truck29raw_rtk2_1014_231631/Log/fusion/1791992500540226741",
      "effective_s": 0.005999999999999998,
      "correction_s": 0.002889013036270507,
      "se_s": 0.0017845047904972058,
      "observable": true,
      "frames": 1176,
      "informative_voxels": 12214
    },
    "method": "secant of both rounds (measured gain 0.80)",
    "gain": 0.805,
    "best_effective_s": 0.00959,
    "best_se_s": 0.00178,
    "rtk_position_s": {
      "from": 0.025,
      "to": 0.0146
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
        2.7755,
        5.9149,
        1.8312
      ],
      "rpy_deg": [
        -1.031,
        -3.057,
        -1.219
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0611,
        -0.2984,
        0.6593
      ],
      "rpy_deg": [
        -1.037,
        0.302,
        -91.365
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0915,
        5.6621,
        2.0104
      ],
      "rpy_deg": [
        3.243,
        -4.537,
        178.24
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.106,
        7.9492,
        0.051
      ],
      "rpy_deg": [
        1.485,
        -1.56,
        89.984
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0067,
      7.9506,
      1.7516
    ],
    "rpy_deg": [
      -96.477,
      1.668,
      0.485
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
    "lidar": -0.0361,
    "camera": 0.0,
    "rtk_position": 0.0146,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.80); effective +9.6 ms (se 1.8 ms)"
  }
}
```

## stage report

```
round 1: effective +20.00 ms -> correction -8.38 ms (se 2.12)
round 2: effective +6.00 ms -> correction +2.89 ms (se 1.78)
secant of both rounds (measured gain 0.80): best effective +9.59 ms (se 1.78 ms); rtk_position +0.0250 -> +0.0146 s

RTK time correction -8.38 ms (se 2.12 ms) -> best effective offset +11.62 ms (run +20.00); voxels 14808, informative 12260; observable
RTK time correction +2.89 ms (se 1.78 ms) -> best effective offset +8.89 ms (run +6.00); voxels 14791, informative 12214; observable
```
