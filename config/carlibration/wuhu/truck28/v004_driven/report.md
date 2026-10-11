# wuhu/truck28/v004_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck28/v003_ground
- lineage: wuhu/truck28/v001_import -> wuhu/truck28/v002_axle -> wuhu/truck28/v003_ground -> wuhu/truck28/v004_driven
- created: 2026-10-11T12:29:02+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu/28/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/ros2bag', 'kind': 'raw_split', 'segments': 82, 'duration_s': 1229.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck28_lio_1011_121344/Log/fusion/1791692620927067100
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.21,
        0.22,
        2.31
      ],
      "drpy_deg": [
        -0.086,
        0.095,
        -0.03
      ]
    },
    "back_lidar": {
      "dt_cm": [
        5.83,
        4.24,
        -0.77
      ],
      "drpy_deg": [
        0.336,
        -0.031,
        0.005
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -1.63,
        3.02,
        -7.83
      ],
      "drpy_deg": [
        -0.023,
        -0.137,
        -0.065
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
            -0.27651506850618773,
            -0.20821578874699911,
            0.10490463652096349
          ],
          "trans_cm": [
            1.0789790468643694,
            0.6566310819041327,
            4.709533939478521
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.37342249707885283,
            -0.7874703233058586,
            0.4239631601655954
          ],
          "trans_cm": [
            -2.4383519123394946,
            -2.873057954513018,
            1.4809266994222092
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            -0.6457236700583545,
            -0.38310355017738124,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            0.5055247196074156
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "/home/project/data/haibo/wuhu/28/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        11120,
        40
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              -0.08551406911241465,
              0.09540375280346794,
              -0.025630643047702684
            ],
            "trans_cm": [
              0.21319462077977053,
              0.22264476086699814,
              2.3065558886941075
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 11,
              "matches": 174853,
              "vertical_matches": 34697,
              "median_before_cm": 3.23,
              "median_after_cm": 2.97,
              "vertical_before_cm": 3.13,
              "vertical_after_cm": 2.85,
              "rotation_deg": 0.131,
              "translation_cm": 2.3,
              "match_time_gap_s": 2.35,
              "pass2": {
                "ground_before_cm": 2.99,
                "ground_after_cm": 2.88,
                "ground_matches": 111338,
                "delta_rotvec_deg": [
                  -0.0344,
                  0.0147,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.68
                ],
                "iterations": 7
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.02670211144299093,
              0.13701986072360883,
              -0.06481027262290909
            ],
            "trans_cm": [
              -1.6216903653811665,
              3.014134686584469,
              -7.831146533106102
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 13,
              "matches": 278417,
              "vertical_matches": 93047,
              "median_before_cm": 5.0,
              "median_after_cm": 3.73,
              "vertical_before_cm": 4.51,
              "vertical_after_cm": 2.82,
              "rotation_deg": 0.154,
              "translation_cm": 8.5,
              "match_time_gap_s": 3.0,
              "pass2": {
                "ground_before_cm": 4.76,
                "ground_after_cm": 4.66,
                "ground_matches": 162560,
                "delta_rotvec_deg": [
                  0.038,
                  -0.0441,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.81
                ],
                "iterations": 11
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              -0.035311695097133305,
              -0.3350495323062318,
              -0.009882838829809408
            ],
            "trans_cm": [
              5.830203535504517,
              4.241784084424066,
              -0.7735305344527355
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 24,
              "matches": 50310,
              "vertical_matches": 6552,
              "median_before_cm": 3.76,
              "median_after_cm": 3.09,
              "vertical_before_cm": 4.81,
              "vertical_after_cm": 3.84,
              "rotation_deg": 0.337,
              "translation_cm": 7.3,
              "match_time_gap_s": 8.97,
              "pass2": {
                "ground_before_cm": 3.53,
                "ground_after_cm": 2.83,
                "ground_matches": 39301,
                "delta_rotvec_deg": [
                  0.0224,
                  -0.2052,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.57
                ],
                "iterations": 9
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck28_lio_1011_121344/Log/fusion/1791692620927067100",
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
        2.7415,
        5.8007,
        1.4489
      ],
      "rpy_deg": [
        0.227,
        2.437,
        -0.257
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1764,
        -0.3512,
        0.3407
      ],
      "rpy_deg": [
        -1.774,
        2.55,
        -90.7
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1134,
        5.5768,
        1.5576
      ],
      "rpy_deg": [
        -1.047,
        -0.228,
        178.584
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1426,
        7.8507,
        -0.1568
      ],
      "rpy_deg": [
        2.071,
        -3.4,
        89.82
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1951,
      7.7526,
      1.5925
    ],
    "rpy_deg": [
      -100.421,
      0.211,
      0.764
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1637.05,
      1851.4,
      1029.33,
      495.55
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "intrinsics_check": {
      "method": "supplier ground-distance curves (midrange_camera[0].equation_*m)",
      "curves": 10,
      "distances_m": [
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
        60.0,
        70.0,
        80.0,
        90.0,
        100.0
      ],
      "fits": {
        "free": {
          "intrinsics": [
            1637.05,
            1851.4,
            1029.33,
            495.55
          ],
          "rms_px": 0.014,
          "circle_centre_rear_m": [
            0.0,
            7.38
          ],
          "ground_z_rear_m": -1.22
        },
        "square": {
          "intrinsics": [
            1639.36,
            1639.36,
            864.27,
            446.63
          ],
          "rms_px": 1.072,
          "circle_centre_rear_m": [
            0.806,
            7.314
          ],
          "ground_z_rear_m": -1.588
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.014,
      "camera_height_m": 2.827,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -0.34,
        "fy_pct": -0.22,
        "cx_px": 58.3,
        "cy_px": -38.7
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.014 px)"
  },
  "time_offsets": {
    "lidar": -0.0151,
    "lidar_frontend": -0.0026,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0151 + imu/ins clock lag +0.0125 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck28/v003_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck28_lio_1011_121344/Log/fusion/1791692620927067100
window 8.0..40.0 s, lidar_time_offset -0.0026
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:320, back_lidar:320, front_left_lidar:321, front_lidar:320
matching: viewing-side consistent (oriented normals)
reference front_lidar: 687356 world points
front_right_lidar: pass 1 matches 174666->174853 |res| median 3.23->2.97 cm; ground 3.17->2.85 (n=108111); vertical 3.13->2.85 (n=34697); match time gap median 2.4 s; correction rot deg (x,y,z)=(-0.051,+0.081,-0.026) trans cm=(+0.2,+0.2,+0.6); iters 11
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 2.99->2.88 cm (n=111338); tilt about x/y adjusted by (-0.034,+0.015) deg, z by +1.7 cm; iters 7
    final correction rot deg (x,y,z)=(-0.086,+0.095,-0.026) trans cm=(+0.2,+0.2,+2.3)  [quality: ok]
front_left_lidar: pass 1 matches 277716->278417 |res| median 5.00->3.73 cm; ground 5.43->4.47 (n=160634); vertical 4.51->2.82 (n=93047); match time gap median 3.0 s; correction rot deg (x,y,z)=(-0.011,+0.181,-0.065) trans cm=(-1.6,+3.0,-6.0); iters 13
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.76->4.66 cm (n=162560); tilt about x/y adjusted by (+0.038,-0.044) deg, z by -1.8 cm; iters 11
    final correction rot deg (x,y,z)=(+0.027,+0.137,-0.065) trans cm=(-1.6,+3.0,-7.8)  [quality: ok]
back_lidar: pass 1 matches 50031->50310 |res| median 3.76->3.09 cm; ground 3.47->2.83 (n=38623); vertical 4.81->3.84 (n=6552); match time gap median 9.0 s; correction rot deg (x,y,z)=(-0.058,-0.130,-0.010) trans cm=(+5.8,+4.2,-3.3); iters 24
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.53->2.83 cm (n=39301); tilt about x/y adjusted by (+0.022,-0.205) deg, z by +2.6 cm; iters 9
    final correction rot deg (x,y,z)=(-0.035,-0.335,-0.010) trans cm=(+5.8,+4.2,-0.8)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck28/v004_driven/calibration.json
```
