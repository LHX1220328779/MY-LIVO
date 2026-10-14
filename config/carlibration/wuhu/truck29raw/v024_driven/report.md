# wuhu/truck29raw/v024_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29raw/v023_ground
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v022_axle -> wuhu/truck29raw/v023_ground -> wuhu/truck29raw/v024_driven
- created: 2026-10-14T22:12:55+08:00
- note: agent 2026-10-14: same-time driven pairing + whole-recording lateral axle, from v001_import

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29raw_lio_1014_215840/Log/fusion/1791986739022890345
- window: [0.0, 120.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        1.85,
        2.24,
        0.93
      ],
      "drpy_deg": [
        0.001,
        -0.031,
        0.069
      ]
    },
    "back_lidar": {
      "dt_cm": [
        3.61,
        7.29,
        0.77
      ],
      "drpy_deg": [
        -0.048,
        0.273,
        -0.051
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        3.17,
        1.13,
        3.9
      ],
      "drpy_deg": [
        0.06,
        0.045,
        -0.008
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
            -0.12022190690348655,
            0.2898819033493801,
            0.10170351721660748
          ],
          "trans_cm": [
            3.048808744201054,
            4.882253440556311,
            -0.7850459744707222
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 19,
            "matches": 5484,
            "ground_before_cm": 9.53,
            "ground_after_cm": 6.7,
            "rotation_deg": 0.33,
            "translation_cm": 5.8
          },
          "plane_check": {
            "tilt_deg": 0.205,
            "fore_aft_deg": -0.14,
            "lateral_deg": -0.15,
            "dz_cm": -1.1,
            "pairs": 1,
            "outside_basin": false,
            "applied": false
          }
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -6.289624494493789,
            2.6122535196676124,
            0.08747785049395428
          ],
          "trans_cm": [
            1.5596971407001357,
            3.023812515598985,
            -2.9655032522577423
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "warn",
            "reasons": [
              "plane start applied (6.75 deg / -3.9 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
            ],
            "iterations": 25,
            "matches": 6468,
            "ground_before_cm": 15.64,
            "ground_after_cm": 8.65,
            "rotation_deg": 6.811,
            "translation_cm": 4.5
          },
          "plane_check": {
            "tilt_deg": 6.748,
            "fore_aft_deg": -6.232,
            "lateral_deg": -2.597,
            "dz_cm": -3.9,
            "pairs": 1,
            "outside_basin": true,
            "applied": true
          }
        },
        "back_lidar": {
          "rotvec_deg": [
            -0.4215231648414212,
            -0.15497145187959316,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -5.544636914130138
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 21,
            "matches": 32528,
            "ground_before_cm": 10.57,
            "ground_after_cm": 9.45,
            "rotation_deg": 0.449,
            "translation_cm": 5.5
          },
          "plane_check": {
            "tilt_deg": 2.031,
            "fore_aft_deg": -0.126,
            "lateral_deg": 2.028,
            "dz_cm": 10.4,
            "pairs": 40,
            "outside_basin": true,
            "applied": false
          }
        }
      },
      "quality": "warn",
      "source": "/home/project/data/haibo/wuhu/29/20261013/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        810,
        40
      ],
      "coarse_threshold": [
        0.5,
        0.05
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              0.00023627892773780032,
              -0.031068669648152785,
              0.0688802502742118
            ],
            "trans_cm": [
              1.8587941048591874,
              2.2407183993994026,
              0.9294424947713389
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 6,
              "matches": 124173,
              "vertical_matches": 16613,
              "median_before_cm": 2.4,
              "median_after_cm": 2.17,
              "vertical_before_cm": 2.88,
              "vertical_after_cm": 2.27,
              "rotation_deg": 0.076,
              "translation_cm": 3.1,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              -0.060871405079054586,
              -0.04296397254232893,
              -0.0036896227589777696
            ],
            "trans_cm": [
              3.1662411032590216,
              1.1244717997577438,
              3.8997653366483642
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 5,
              "matches": 182276,
              "vertical_matches": 28404,
              "median_before_cm": 3.05,
              "median_after_cm": 2.55,
              "vertical_before_cm": 2.94,
              "vertical_after_cm": 2.06,
              "rotation_deg": 0.075,
              "translation_cm": 5.1,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.27397753238439915,
              0.042033080411267396,
              -0.05054657573899319
            ],
            "trans_cm": [
              3.614148003733961,
              7.2900104008325,
              0.7707622839098427
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 18,
              "matches": 125805,
              "vertical_matches": 15149,
              "median_before_cm": 7.29,
              "median_after_cm": 4.72,
              "vertical_before_cm": 3.59,
              "vertical_after_cm": 3.71,
              "rotation_deg": 0.282,
              "translation_cm": 8.2,
              "match_time_gap_s": 9.0,
              "pass2": {
                "ground_before_cm": 2.78,
                "ground_after_cm": 2.58,
                "ground_matches": 97751,
                "delta_rotvec_deg": [
                  0.0809,
                  -0.0226,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  3.23
                ],
                "iterations": 7
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29raw_lio_1014_215840/Log/fusion/1791986739022890345",
        "window": [
          0.0,
          117.30027866363525
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
        "pairing": {
          "mode": "same-time",
          "block_s": 1.0,
          "pad_s": 0.5
        },
        "scan_source": "archive",
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
        2.7877,
        5.9219,
        1.8117
      ],
      "rpy_deg": [
        -0.991,
        -2.98,
        -1.228
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0593,
        -0.2922,
        0.6529
      ],
      "rpy_deg": [
        -1.039,
        0.284,
        -91.367
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0784,
        5.6739,
        1.9976
      ],
      "rpy_deg": [
        3.193,
        -4.466,
        178.262
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
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
warning: window end 120.0 s exceeds available poses (117.8 s); clamping
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v023_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29raw_lio_1014_215840/Log/fusion/1791986739022890345
window 0.0..117.30027866363525 s, lidar_time_offset -0.0311
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:1172, back_lidar:1172, front_left_lidar:1172, front_lidar:1172
scan source: round archive scans.bin/sources.bin (mapper deskew; returns at scan end, round extrinsics inverted)
matching: viewing-side consistent (oriented normals)
reference front_lidar: 1341008 world points
pairing: same-time for LiDARs overlapping the reference (1.0 s blocks +-0.5 s, 6025425 block points); cross-time LiDARs use the accumulated target (nearest-range voxels)
front_right_lidar: pass 1 matches 124173->124173 |res| median 2.40->2.17 cm; ground 2.28->2.10 (n=102344); vertical 2.88->2.27 (n=16613); match time gap median 0.6 s; correction rot deg (x,y,z)=(+0.000,-0.031,+0.069) trans cm=(+1.9,+2.2,+0.9); iters 6  [quality: ok]
front_left_lidar: pass 1 matches 181876->182276 |res| median 3.05->2.55 cm; ground 3.04->2.63 (n=148331); vertical 2.94->2.06 (n=28404); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.061,-0.043,-0.004) trans cm=(+3.2,+1.1,+3.9); iters 5  [quality: ok]
back_lidar: pass 1 matches 123632->125805 |res| median 7.29->4.72 cm; ground 8.30->4.84 (n=99073); vertical 3.59->3.71 (n=15149); match time gap median 9.0 s; correction rot deg (x,y,z)=(+0.193,+0.065,-0.051) trans cm=(+3.6,+7.3,-2.5); iters 18
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 2.78->2.58 cm (n=97751); tilt about x/y adjusted by (+0.081,-0.023) deg, z by +3.2 cm; iters 7
    final correction rot deg (x,y,z)=(+0.274,+0.042,-0.051) trans cm=(+3.6,+7.3,+0.8)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v024_driven/calibration.json
```
