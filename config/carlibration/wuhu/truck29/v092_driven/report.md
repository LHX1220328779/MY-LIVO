# wuhu/truck29/v092_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29/v091_ground
- lineage: wuhu/truck29/v001_import -> wuhu/truck29/v090_axle -> wuhu/truck29/v091_ground -> wuhu/truck29/v092_driven
- created: 2026-10-12T10:01:46+08:00
- note: mcp: axle dof + frontend fix check (v001 lineage)

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_lio_1012_095506/Log/fusion/1791770346069759987
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        1.56,
        7.1,
        0.83
      ],
      "drpy_deg": [
        0.009,
        -0.116,
        0.008
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -2.36,
        10.28,
        -7.76
      ],
      "drpy_deg": [
        -0.058,
        0.037,
        0.066
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        2.31,
        7.97,
        -0.98
      ],
      "drpy_deg": [
        -0.094,
        -0.044,
        0.088
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
            -0.11411954070860465,
            0.36020524487822614,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.3293588897352007
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            0.10673838038661784,
            -1.3109256433013996,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -0.9022167170256057
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            0.22465376793455455,
            -0.13477833051741434,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.9331865658505758
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
              0.005885522863710775,
              -0.11579551852714255,
              0.00898654974555297
            ],
            "trans_cm": [
              1.5605726068052779,
              7.096184287438547,
              0.8293682937687523
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 11,
              "matches": 183377,
              "vertical_matches": 15604,
              "median_before_cm": 3.47,
              "median_after_cm": 2.9,
              "vertical_before_cm": 5.1,
              "vertical_after_cm": 3.76,
              "rotation_deg": 0.116,
              "translation_cm": 7.3,
              "match_time_gap_s": 3.04,
              "pass2": {
                "ground_before_cm": 3.28,
                "ground_after_cm": 3.09,
                "ground_matches": 146815,
                "delta_rotvec_deg": [
                  -0.0256,
                  -0.0324,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.42
                ],
                "iterations": 6
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.09474097423084875,
              0.041984324782749924,
              0.08674029155522349
            ],
            "trans_cm": [
              2.3157755043561905,
              7.974902385505247,
              -0.9832688821126562
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 12,
              "matches": 218298,
              "vertical_matches": 26029,
              "median_before_cm": 5.15,
              "median_after_cm": 4.93,
              "vertical_before_cm": 4.64,
              "vertical_after_cm": 3.92,
              "rotation_deg": 0.135,
              "translation_cm": 8.4,
              "match_time_gap_s": 5.39,
              "pass2": {
                "ground_before_cm": 5.05,
                "ground_after_cm": 4.94,
                "ground_matches": 138296,
                "delta_rotvec_deg": [
                  0.0403,
                  -0.0172,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -0.42
                ],
                "iterations": 7
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.037622078072659014,
              0.05727552928431282,
              0.06686811317664734
            ],
            "trans_cm": [
              -2.363004192851501,
              10.283246274739161,
              -7.759519116052268
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "vertical residual regressed 4.59->5.12 cm"
              ],
              "converged": true,
              "iterations": 13,
              "matches": 41171,
              "vertical_matches": 2163,
              "median_before_cm": 6.32,
              "median_after_cm": 3.45,
              "vertical_before_cm": 4.59,
              "vertical_after_cm": 5.12,
              "rotation_deg": 0.096,
              "translation_cm": 13.1,
              "match_time_gap_s": 10.55,
              "pass2": {
                "ground_before_cm": 3.66,
                "ground_after_cm": 3.14,
                "ground_matches": 32811,
                "delta_rotvec_deg": [
                  0.119,
                  -0.072,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.31
                ],
                "iterations": 6
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29_lio_1012_095506/Log/fusion/1791770346069759987",
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
        "quality": "warn"
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
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.125,
    "lidar_frontend_source": "axle: import default (parent version had no lidar_frontend)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v091_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29_lio_1012_095506/Log/fusion/1791770346069759987
window 8.0..40.0 s, lidar_time_offset -0.125
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
matching: viewing-side consistent (oriented normals)
reference front_lidar: 514245 world points
front_right_lidar: pass 1 matches 183742->183377 |res| median 3.47->2.90 cm; ground 3.28->2.75 (n=149058); vertical 5.10->3.76 (n=15604); match time gap median 3.0 s; correction rot deg (x,y,z)=(+0.031,-0.083,+0.009) trans cm=(+1.6,+7.1,-0.6); iters 11
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.28->3.09 cm (n=146815); tilt about x/y adjusted by (-0.026,-0.032) deg, z by +1.4 cm; iters 6
    final correction rot deg (x,y,z)=(+0.006,-0.116,+0.009) trans cm=(+1.6,+7.1,+0.8)  [quality: ok]
front_left_lidar: pass 1 matches 218089->218298 |res| median 5.15->4.93 cm; ground 4.98->4.88 (n=140999); vertical 4.64->3.92 (n=26029); match time gap median 5.4 s; correction rot deg (x,y,z)=(+0.054,+0.059,+0.087) trans cm=(+2.3,+8.0,-0.6); iters 12
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.05->4.94 cm (n=138296); tilt about x/y adjusted by (+0.040,-0.017) deg, z by -0.4 cm; iters 7
    final correction rot deg (x,y,z)=(+0.095,+0.042,+0.087) trans cm=(+2.3,+8.0,-1.0)  [quality: ok]
back_lidar: pass 1 matches 40758->41171 |res| median 6.32->3.45 cm; ground 6.74->3.25 (n=33407); vertical 4.59->5.12 (n=2163); match time gap median 10.5 s; correction rot deg (x,y,z)=(-0.081,+0.129,+0.067) trans cm=(-2.4,+10.3,-10.1); iters 13
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.66->3.14 cm (n=32811); tilt about x/y adjusted by (+0.119,-0.072) deg, z by +2.3 cm; iters 6
    final correction rot deg (x,y,z)=(+0.038,+0.057,+0.067) trans cm=(-2.4,+10.3,-7.8)  [quality: warn; vertical residual regressed 4.59->5.12 cm]
quality: warn  (back_lidar: vertical residual regressed 4.59->5.12 cm)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v092_driven/calibration.json
```
