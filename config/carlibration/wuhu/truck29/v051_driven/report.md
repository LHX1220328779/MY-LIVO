# wuhu/truck29/v051_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29/v050_ground
- lineage: wuhu/truck29/v037_import -> wuhu/truck29/v049_axle -> wuhu/truck29/v050_ground -> wuhu/truck29/v051_driven
- created: 2026-09-24T12:53:09+08:00
- note: mcp turn4: raw auto-detect (exact re-pose)

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_lio_0924_124649/Log/fusion/1790225417707757686
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        2.04,
        6.17,
        0.85
      ],
      "drpy_deg": [
        0.014,
        -0.122,
        0.023
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -0.64,
        -2.89,
        -3.2
      ],
      "drpy_deg": [
        -0.075,
        0.126,
        0.016
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.34,
        7.72,
        -0.27
      ],
      "drpy_deg": [
        -0.068,
        -0.067,
        0.099
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
            -0.1256373800802659,
            0.3638653783197681,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.6091803058469154
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            0.10452630628623628,
            -1.3105670674410848,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -0.9120594125419231
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            0.21546247067921592,
            -0.14668205673708554,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            2.0326871115484417
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
              0.011004052116650294,
              -0.12196623786838601,
              0.023760156485786006
            ],
            "trans_cm": [
              2.032317112432538,
              6.168127450367766,
              0.8412414744527743
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 19,
              "matches": 207479,
              "vertical_matches": 17651,
              "median_before_cm": 3.7,
              "median_after_cm": 3.13,
              "vertical_before_cm": 5.13,
              "vertical_after_cm": 4.08,
              "rotation_deg": 0.125,
              "translation_cm": 6.5,
              "match_time_gap_s": 3.27,
              "pass2": {
                "ground_before_cm": 3.86,
                "ground_after_cm": 3.23,
                "ground_matches": 167499,
                "delta_rotvec_deg": [
                  -0.0685,
                  -0.0329,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.87
                ],
                "iterations": 10
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.06985171808262522,
              0.06475680044722192,
              0.09806223072055377
            ],
            "trans_cm": [
              0.33258239005022117,
              7.720289949641507,
              -0.26095860833045775
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 15,
              "matches": 226808,
              "vertical_matches": 26813,
              "median_before_cm": 5.09,
              "median_after_cm": 4.74,
              "vertical_before_cm": 4.65,
              "vertical_after_cm": 4.02,
              "rotation_deg": 0.137,
              "translation_cm": 7.7,
              "match_time_gap_s": 5.58,
              "pass2": {
                "ground_before_cm": 4.9,
                "ground_after_cm": 4.65,
                "ground_matches": 146534,
                "delta_rotvec_deg": [
                  -0.0483,
                  -0.0204,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.19
                ],
                "iterations": 11
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.12817589998776674,
              0.07181575546944297,
              0.016584780081120826
            ],
            "trans_cm": [
              -0.6425142542662439,
              -2.8907857052984336,
              -3.2035018617935944
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 20,
              "matches": 47246,
              "vertical_matches": 3191,
              "median_before_cm": 10.75,
              "median_after_cm": 3.27,
              "vertical_before_cm": 5.22,
              "vertical_after_cm": 4.59,
              "rotation_deg": 0.148,
              "translation_cm": 4.4,
              "match_time_gap_s": 10.61,
              "pass2": {
                "ground_before_cm": 5.67,
                "ground_after_cm": 3.09,
                "ground_matches": 37503,
                "delta_rotvec_deg": [
                  0.047,
                  -0.045,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  6.88
                ],
                "iterations": 8
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29_lio_0924_124649/Log/fusion/1790225417707757686",
        "window": [
          8.0,
          40.0
        ],
        "prior": [
          0.5,
          0.1
        ],
        "poses": {
          "pass1": "native frames.csv",
          "pass2": "RTK-optimized trajectory_optimized.txt",
          "cross_time_s": 2.0
        },
        "quality": "ok"
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
        2.7472,
        5.9783,
        1.8375
      ],
      "rpy_deg": [
        -1.014,
        -3.031,
        -1.448
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1211,
        -0.325,
        0.6932
      ],
      "rpy_deg": [
        -1.055,
        0.746,
        -91.374
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1345,
        5.7832,
        1.9759
      ],
      "rpy_deg": [
        -3.416,
        -0.798,
        178.576
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1141,
        8.0179,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.531,
        89.91
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0024,
      8.02,
      1.7517
    ],
    "rpy_deg": [
      -96.507,
      1.636,
      0.41
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
    "lidar": -0.1539,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v050_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29_lio_0924_124649/Log/fusion/1790225417707757686
window 8.0..40.0 s, lidar_time_offset -0.1539
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
reference front_lidar: 514002 world points
front_right_lidar: pass 1 matches 207908->207479 |res| median 3.70->3.13 cm; ground 3.52->2.98 (n=169834); vertical 5.13->4.08 (n=17651); match time gap median 3.3 s; correction rot deg (x,y,z)=(+0.079,-0.089,+0.024) trans cm=(+2.0,+6.2,-2.0); iters 19
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.86->3.23 cm (n=167499); tilt about x/y adjusted by (-0.068,-0.033) deg, z by +2.9 cm; iters 10
    final correction rot deg (x,y,z)=(+0.011,-0.122,+0.024) trans cm=(+2.0,+6.2,+0.8)  [quality: ok]
front_left_lidar: pass 1 matches 226846->226808 |res| median 5.09->4.74 cm; ground 5.01->4.66 (n=150528); vertical 4.65->4.02 (n=26813); match time gap median 5.6 s; correction rot deg (x,y,z)=(+0.118,+0.085,+0.098) trans cm=(+0.3,+7.7,-2.5); iters 15
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.90->4.65 cm (n=146534); tilt about x/y adjusted by (-0.048,-0.020) deg, z by +2.2 cm; iters 11
    final correction rot deg (x,y,z)=(+0.070,+0.065,+0.098) trans cm=(+0.3,+7.7,-0.3)  [quality: ok]
back_lidar: pass 1 matches 47170->47246 |res| median 10.75->3.27 cm; ground 11.94->3.09 (n=38351); vertical 5.22->4.59 (n=3191); match time gap median 10.6 s; correction rot deg (x,y,z)=(+0.081,+0.117,+0.017) trans cm=(-0.6,-2.9,-10.1); iters 20
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.67->3.09 cm (n=37503); tilt about x/y adjusted by (+0.047,-0.045) deg, z by +6.9 cm; iters 8
    final correction rot deg (x,y,z)=(+0.128,+0.072,+0.017) trans cm=(-0.6,-2.9,-3.2)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v051_driven/calibration.json
```
