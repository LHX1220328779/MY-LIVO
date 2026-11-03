# wuhu/truck13/v006_rtktime

- stage: rtktime (RTK 位置关联时间)
- parent: wuhu/truck13/v004_driven
- lineage: wuhu/truck13/v001_import -> wuhu/truck13/v002_axle -> wuhu/truck13/v003_ground -> wuhu/truck13/v004_driven -> wuhu/truck13/v006_rtktime
- created: 2026-10-26T17:26:07+08:00
- note: RTK position time association +0.0250 -> -0.0172 s (secant of both rounds (measured gain 0.31))

## inputs

- run: /home/project/MY-LIVO/results/calib_truck13_rtk1_1026_172302/Log/fusion/1793006691611828688
- run2: /home/project/MY-LIVO/results/calib_truck13_rtk2_1026_172302/Log/fusion/1793006728191165078

## changes vs parent

```
{
  "time_offsets": {
    "rtk_position": {
      "from": 0.025,
      "to": -0.0172
    }
  }
}
```

## metrics

```
{
  "rtk_time": {
    "parent": "wuhu/truck13/v004_driven",
    "parent_rtk_position_s": 0.025,
    "round1": {
      "run": "/home/project/MY-LIVO/results/calib_truck13_rtk1_1026_172302/Log/fusion/1793006691611828688",
      "effective_s": 0.010000000000000005,
      "correction_s": -0.0131977083257627,
      "se_s": 0.0022820347076908925,
      "observable": true,
      "frames": 358,
      "informative_voxels": 3571
    },
    "round2": {
      "run": "/home/project/MY-LIVO/results/calib_truck13_rtk2_1026_172302/Log/fusion/1793006728191165078",
      "effective_s": -0.011999999999999997,
      "correction_s": -0.006318600425835237,
      "se_s": 0.0033136149725457005,
      "observable": true,
      "frames": 358,
      "informative_voxels": 3624
    },
    "method": "secant of both rounds (measured gain 0.31)",
    "gain": 0.313,
    "best_effective_s": -0.03221,
    "best_se_s": 0.02141,
    "rtk_position_s": {
      "from": 0.025,
      "to": -0.0172
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
        2.8244,
        5.6877,
        2.2819
      ],
      "rpy_deg": [
        -1.47,
        -0.987,
        -1.201
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0094,
        -0.6493,
        0.8899
      ],
      "rpy_deg": [
        -0.51,
        1.54,
        -91.242
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0863,
        5.6514,
        2.2791
      ],
      "rpy_deg": [
        1.549,
        -1.91,
        179.818
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0242,
        7.7501,
        0.4596
      ],
      "rpy_deg": [
        0.215,
        -1.686,
        89.873
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0755,
      7.8739,
      2.2451
    ],
    "rpy_deg": [
      -102.142,
      1.542,
      -4.24
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1849.47862,
      1848.72893,
      1004.25076,
      503.038474
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
    "lidar": 0.021,
    "lidar_frontend": 0.036,
    "camera": 0.0,
    "rtk_position": -0.0172,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar +0.0210 + imu/ins clock lag +0.0150 (corr 0.996); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.31); effective -32.2 ms (se 21.4 ms)"
  }
}
```

## stage report

```
round 1: effective +10.00 ms -> correction -13.20 ms (se 2.28)
round 2: effective -12.00 ms -> correction -6.32 ms (se 3.31)
secant of both rounds (measured gain 0.31): best effective -32.21 ms (se 21.41 ms); rtk_position +0.0250 -> -0.0172 s

RTK time correction -13.20 ms (se 2.28 ms) -> best effective offset -3.20 ms (run +10.00); voxels 5627, informative 3571; observable
RTK time correction -6.32 ms (se 3.31 ms) -> best effective offset -18.32 ms (run -12.00); voxels 5652, informative 3624; observable
```
