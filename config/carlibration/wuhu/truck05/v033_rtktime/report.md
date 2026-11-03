# wuhu/truck05/v033_rtktime

- stage: rtktime (RTK 位置关联时间)
- parent: wuhu/truck05/v031_driven
- lineage: wuhu/truck05/v001_import -> wuhu/truck05/v029_axle -> wuhu/truck05/v030_ground -> wuhu/truck05/v031_driven -> wuhu/truck05/v033_rtktime
- created: 2026-10-14T14:11:34+08:00
- note: RTK position time association +0.0250 -> +0.0486 s (secant of both rounds (measured gain 0.50))

## inputs

- run: /home/project/MY-LIVO/results/calib_truck05_rtk1_1014_134623/Log/fusion/1791957544629248223
- run2: /home/project/MY-LIVO/results/calib_truck05_rtk2_1014_134623/Log/fusion/1791957913191211522

## changes vs parent

```
{
  "time_offsets": {
    "rtk_position": {
      "from": 0.025,
      "to": 0.0486
    }
  }
}
```

## metrics

```
{
  "rtk_time": {
    "parent": "wuhu/truck05/v031_driven",
    "parent_rtk_position_s": 0.025,
    "round1": {
      "run": "/home/project/MY-LIVO/results/calib_truck05_rtk1_1014_134623/Log/fusion/1791957544629248223",
      "effective_s": 0.03,
      "correction_s": 0.011829773677902108,
      "se_s": 0.0020362203258130957,
      "observable": true,
      "frames": 1190,
      "informative_voxels": 11885
    },
    "round2": {
      "run": "/home/project/MY-LIVO/results/calib_truck05_rtk2_1014_134623/Log/fusion/1791957913191211522",
      "effective_s": 0.049699999999999994,
      "correction_s": 0.00193746699159065,
      "se_s": 0.0025025332371547957,
      "observable": true,
      "frames": 1190,
      "informative_voxels": 11860
    },
    "method": "secant of both rounds (measured gain 0.50)",
    "gain": 0.502,
    "best_effective_s": 0.05356,
    "best_se_s": 0.00601,
    "rtk_position_s": {
      "from": 0.025,
      "to": 0.0486
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
        2.833,
        5.4626,
        1.9685
      ],
      "rpy_deg": [
        0.684,
        -2.163,
        -1.131
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.1147,
        -0.756,
        0.8479
      ],
      "rpy_deg": [
        -1.023,
        1.154,
        -90.244
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -2.9937,
        5.3861,
        1.9336
      ],
      "rpy_deg": [
        -0.449,
        -1.921,
        179.68
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0968,
        7.5648,
        0.5356
      ],
      "rpy_deg": [
        0.072,
        -5.301,
        89.874
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0022,
      7.5704,
      2.2531
    ],
    "rpy_deg": [
      -100.385,
      0.003,
      -0.449
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
    "lidar": -0.0368,
    "camera": 0.0,
    "rtk_position": 0.0486,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend": -0.0418,
    "lidar_frontend_source": "axle: lidar -0.0368 + imu/ins clock lag -0.0050 (corr 0.999); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.50); effective +53.6 ms (se 6.0 ms)"
  }
}
```

## stage report

```
round 1: effective +30.00 ms -> correction +11.83 ms (se 2.04)
round 2: effective +49.70 ms -> correction +1.94 ms (se 2.50)
secant of both rounds (measured gain 0.50): best effective +53.56 ms (se 6.01 ms); rtk_position +0.0250 -> +0.0486 s

RTK time correction +11.83 ms (se 2.04 ms) -> best effective offset +41.83 ms (run +30.00); voxels 13653, informative 11885; observable
RTK time correction +1.94 ms (se 2.50 ms) -> best effective offset +51.64 ms (run +49.70); voxels 13632, informative 11860; observable
```
