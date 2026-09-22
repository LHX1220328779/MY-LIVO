# wuhu/truck29/v011_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29/v010_ground
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v010_ground -> wuhu/truck29/v011_driven
- created: 2026-09-22T14:54:21+08:00
- note: Studio v2 验证：LiDAR 流水线（未激活）

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_lio_0922_145038/Log/fusion/1790059848258417823
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.99,
        0.46,
        -1.57
      ],
      "drpy_deg": [
        0.06,
        -0.085,
        0.01
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.07,
        3.23,
        -8.16
      ],
      "drpy_deg": [
        -0.145,
        0.019,
        0.033
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        1.48,
        0.78,
        -0.81
      ],
      "drpy_deg": [
        -0.069,
        -0.088,
        0.025
      ]
    }
  }
}
```

## metrics

```
{
  "refinement": {
    "refinement": {
      "reference": "front_lidar",
      "corrections": {
        "front_right_lidar": {
          "rotvec_deg": [
            -0.061787652758260106,
            0.08309966002559983,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.7757440047724524
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.05498358832349467,
            -0.09071537045934852,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            0.25823758995354185
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            0.009767132835071034,
            -0.1868638944365991,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            9.050264718778646
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "/home/project/data/haibo/wuhu_livo/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        300,
        40
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              0.05845754435533029,
              -0.08630434477690734,
              0.01270467016378434
            ],
            "trans_cm": [
              0.9898055642372924,
              0.46376849108272805,
              -1.5748967543807186
            ],
            "about": "lidar position in rear-axle frame"
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.07126688752857893,
              0.0865456345168783,
              0.023791214991601
            ],
            "trans_cm": [
              1.4828488331668972,
              0.7786887891096348,
              -0.8140350630268512
            ],
            "about": "lidar position in rear-axle frame"
          },
          "back_lidar": {
            "rotvec_deg": [
              0.02162981679205006,
              0.14449934564681485,
              0.0349468156058435
            ],
            "trans_cm": [
              0.06189760374147413,
              3.222401066567931,
              -8.165218226665509
            ],
            "about": "lidar position in rear-axle frame"
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29_lio_0922_145038/Log/fusion/1790059848258417823",
        "window": [
          8.0,
          40.0
        ],
        "prior": [
          0.5,
          0.1
        ]
      }
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
        2.8398,
        5.7695,
        1.8111
      ],
      "rpy_deg": [
        -0.807,
        -3.135,
        -1.207
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0035,
        -0.5205,
        0.6084
      ],
      "rpy_deg": [
        -0.946,
        0.793,
        -91.118
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0315,
        5.5434,
        1.9459
      ],
      "rpy_deg": [
        -3.591,
        -0.661,
        178.831
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
      -96.369,
      1.482,
      0.659
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
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v010_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29_lio_0922_145038/Log/fusion/1790059848258417823
window 8.0..40.0 s, lidar_time_offset -0.125
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
reference front_lidar: 515651 world points
front_right_lidar: matches 208236->207897 |res| median 3.43->3.04 cm; ground 3.35->2.91 (n=168854); vertical 3.94->3.86 (n=18716); correction rot deg (x,y,z)=(+0.058,-0.086,+0.013) trans cm=(+1.0,+0.5,-1.6); iters 13
front_left_lidar: matches 236260->235837 |res| median 5.22->5.03 cm; ground 5.27->5.07 (n=155104); vertical 3.89->3.86 (n=27793); correction rot deg (x,y,z)=(+0.071,+0.087,+0.024) trans cm=(+1.5,+0.8,-0.8); iters 16
back_lidar: matches 47762->47785 |res| median 7.15->3.57 cm; ground 7.91->3.40 (n=38427); vertical 3.99->4.37 (n=3424); correction rot deg (x,y,z)=(+0.022,+0.144,+0.035) trans cm=(+0.1,+3.2,-8.2); iters 15
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v011_driven/calibration.json
```
