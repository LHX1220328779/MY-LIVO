# wuhu/truck29/v094_rtktime

- stage: rtktime (RTK 位置关联时间)
- parent: wuhu/truck29/v092_driven
- lineage: wuhu/truck29/v001_import -> wuhu/truck29/v090_axle -> wuhu/truck29/v091_ground -> wuhu/truck29/v092_driven -> wuhu/truck29/v094_rtktime
- created: 2026-10-12T10:07:59+08:00
- note: RTK position time association +0.0250 -> -0.0200 s (first + probe rounds with the prior gain 0.6, standard-error weighted (secant gain 0.73 not usable))

## inputs

- run: /home/project/MY-LIVO/results/calib_truck29_rtk1_1012_095506/Log/fusion/1791770507748050306
- run2: /home/project/MY-LIVO/results/calib_truck29_rtk2_1012_095506/Log/fusion/1791770685620202266

## changes vs parent

```
{
  "time_offsets": {
    "rtk_position": {
      "from": 0.025,
      "to": -0.02
    }
  }
}
```

## metrics

```
{
  "rtk_time": {
    "parent": "wuhu/truck29/v092_driven",
    "parent_rtk_position_s": 0.025,
    "round1": {
      "run": "/home/project/MY-LIVO/results/calib_truck29_rtk1_1012_095506/Log/fusion/1791770507748050306",
      "effective_s": 0.0040000000000000105,
      "correction_s": -0.027657637805231117,
      "se_s": 0.0032247895300438257,
      "observable": true,
      "frames": 1179,
      "informative_voxels": 17945
    },
    "round2": {
      "run": "/home/project/MY-LIVO/results/calib_truck29_rtk2_1012_095506/Log/fusion/1791770685620202266",
      "effective_s": -0.04209999999999999,
      "correction_s": 0.0059518379539602855,
      "se_s": 0.0090061287009444,
      "observable": false,
      "frames": 1179,
      "informative_voxels": 17800
    },
    "method": "first + probe rounds with the prior gain 0.6, standard-error weighted (secant gain 0.73 not usable)",
    "gain": 0.729,
    "best_effective_s": -0.04097,
    "best_se_s": 0.00506,
    "rtk_position_s": {
      "from": 0.025,
      "to": -0.02
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
        2.7482,
        6.0106,
        1.8345
      ],
      "rpy_deg": [
        -1.007,
        -3.028,
        -1.45
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1311,
        -0.1709,
        0.6466
      ],
      "rpy_deg": [
        -1.05,
        0.666,
        -91.31
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1091,
        5.8074,
        1.9688
      ],
      "rpy_deg": [
        -3.444,
        -0.775,
        178.578
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
        1.452,
        -1.532,
        89.924
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0028,
      8.0424,
      1.7516
    ],
    "rpy_deg": [
      -96.575,
      1.64,
      0.463
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
    "camera": 0.0,
    "rtk_position": -0.02,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.125,
    "lidar_frontend_source": "axle: import default (parent version had no lidar_frontend)",
    "rtk_position_source": "rtk-time: map consistency, first + probe rounds with the prior gain 0.6, standard-error weighted (secant gain 0.73 not usable); effective -41.0 ms (se 5.1 ms)"
  }
}
```

## stage report

```
round 1: effective +4.00 ms -> correction -27.66 ms (se 3.22)
round 2: effective -42.10 ms -> correction +5.95 ms (se 9.01)
first + probe rounds with the prior gain 0.6, standard-error weighted (secant gain 0.73 not usable): best effective -40.97 ms (se 5.06 ms); rtk_position +0.0250 -> -0.0200 s

RTK time correction -27.66 ms (se 3.22 ms) -> best effective offset -23.66 ms (run +4.00); voxels 20453, informative 17945; observable
RTK time correction +5.95 ms (se 9.01 ms) -> best effective offset -36.15 ms (run -42.10); voxels 20449, informative 17800; NOT observable
```
