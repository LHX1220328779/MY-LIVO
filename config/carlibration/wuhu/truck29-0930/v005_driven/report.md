# wuhu/truck29-0930/v005_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29-0930/v004_ground
- lineage: wuhu/truck29-0930/v002_import -> wuhu/truck29-0930/v003_axle -> wuhu/truck29-0930/v004_ground -> wuhu/truck29-0930/v005_driven
- created: 2026-10-19T17:03:00+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29-0930_lio_1019_165333/Log/fusion/1792400249300507330
- window: [0.0, 180.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -0.37,
        1.14,
        2.51
      ],
      "drpy_deg": [
        -0.029,
        -0.093,
        0.035
      ]
    },
    "back_lidar": {
      "dt_cm": [
        2.57,
        5.5,
        1.69
      ],
      "drpy_deg": [
        -0.015,
        0.188,
        -0.033
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -0.24,
        6.04,
        6.38
      ],
      "drpy_deg": [
        0.126,
        -0.045,
        -0.029
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
            -0.00785803578195938,
            0.05617787519412676,
            0.08857064688944878
          ],
          "trans_cm": [
            2.6414711601080687,
            3.652303419883761,
            -1.1774510888201477
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 15,
            "matches": 5365,
            "ground_before_cm": 6.93,
            "ground_after_cm": 6.8,
            "rotation_deg": 0.105,
            "translation_cm": 4.7
          },
          "plane_check": {
            "tilt_deg": 0.101,
            "fore_aft_deg": -0.013,
            "lateral_deg": 0.1,
            "dz_cm": -2.1,
            "pairs": 1,
            "outside_basin": false,
            "applied": false
          }
        },
        "front_left_lidar": {
          "rotvec_deg": [
            0.1695543451226437,
            0.11223300218806169,
            -0.005086584415059723
          ],
          "trans_cm": [
            0.0,
            0.0,
            -5.043290043681474
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "plane start",
          "quality": {
            "level": "warn",
            "reasons": [
              "fine solve regressed the ground residual (8.40->8.50 cm); kept the plane start matrix (8.40 cm)",
              "plane start applied (0.20 deg / -5.0 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
            ],
            "iterations": 17,
            "matches": 6581,
            "ground_before_cm": 8.72,
            "ground_after_cm": 8.4,
            "rotation_deg": 0.203,
            "translation_cm": 5.0
          },
          "plane_check": {
            "tilt_deg": 0.203,
            "fore_aft_deg": 0.17,
            "lateral_deg": -0.112,
            "dz_cm": -5.0,
            "pairs": 1,
            "outside_basin": true,
            "applied": true
          }
        },
        "back_lidar": {
          "rotvec_deg": [
            -0.28580665486582013,
            -0.08944021445002444,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -4.376860148547307
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 19,
            "matches": 23392,
            "ground_before_cm": 9.17,
            "ground_after_cm": 8.46,
            "rotation_deg": 0.299,
            "translation_cm": 4.4
          },
          "plane_check": {
            "tilt_deg": 0.399,
            "fore_aft_deg": -0.128,
            "lateral_deg": 0.183,
            "dz_cm": -8.8,
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
        1260,
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
              -0.031065639639119458,
              -0.09274509540434238,
              0.03403939057157887
            ],
            "trans_cm": [
              -0.3692745416155419,
              1.1382829749877377,
              2.5115953450887347
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 7,
              "matches": 134573,
              "vertical_matches": 3706,
              "median_before_cm": 2.39,
              "median_after_cm": 1.15,
              "vertical_before_cm": 2.61,
              "vertical_after_cm": 2.21,
              "rotation_deg": 0.104,
              "translation_cm": 2.8,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              -0.1234555978367908,
              0.04862642865923614,
              -0.01898134179236021
            ],
            "trans_cm": [
              -0.2448583372283793,
              6.042067973291367,
              6.380670682012826
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 8,
              "matches": 145231,
              "vertical_matches": 2653,
              "median_before_cm": 4.92,
              "median_after_cm": 1.54,
              "vertical_before_cm": 4.33,
              "vertical_after_cm": 2.35,
              "rotation_deg": 0.134,
              "translation_cm": 8.8,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.1881437121837942,
              0.010713854878575669,
              -0.032679457919388484
            ],
            "trans_cm": [
              2.571868391984318,
              5.498983656241879,
              1.6894049249163858
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 12,
              "matches": 144253,
              "vertical_matches": 14432,
              "median_before_cm": 4.24,
              "median_after_cm": 3.73,
              "vertical_before_cm": 3.5,
              "vertical_after_cm": 3.36,
              "rotation_deg": 0.191,
              "translation_cm": 6.3,
              "match_time_gap_s": 9.2,
              "pass2": {
                "ground_before_cm": 3.41,
                "ground_after_cm": 2.38,
                "ground_matches": 128136,
                "delta_rotvec_deg": [
                  0.0529,
                  -0.1213,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  0.05
                ],
                "iterations": 6
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29-0930_lio_1019_165333/Log/fusion/1792400249300507330",
        "window": [
          0.0,
          177.2999393939972
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
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0372 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
warning: window end 180.0 s exceeds available poses (177.8 s); clamping
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v004_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29-0930_lio_1019_165333/Log/fusion/1792400249300507330
window 0.0..177.2999393939972 s, lidar_time_offset -0.0322
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:1770, back_lidar:1770, front_left_lidar:1770, front_lidar:1770
scan source: round archive scans.bin/sources.bin (mapper deskew; returns at scan end, round extrinsics inverted)
matching: viewing-side consistent (oriented normals)
reference front_lidar: 1538686 world points
pairing: same-time for LiDARs overlapping the reference (1.0 s blocks +-0.5 s, 11505134 block points); cross-time LiDARs use the accumulated target (nearest-range voxels)
front_right_lidar: pass 1 matches 148432->134573 |res| median 2.39->1.15 cm; ground 2.37->1.11 (n=128337); vertical 2.61->2.21 (n=3706); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.031,-0.093,+0.034) trans cm=(-0.4,+1.1,+2.5); iters 7  [quality: ok]
front_left_lidar: pass 1 matches 159297->145231 |res| median 4.92->1.54 cm; ground 4.93->1.51 (n=140305); vertical 4.33->2.35 (n=2653); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.123,+0.049,-0.019) trans cm=(-0.2,+6.0,+6.4); iters 8  [quality: ok]
back_lidar: pass 1 matches 143312->144253 |res| median 4.24->3.73 cm; ground 4.29->3.68 (n=121255); vertical 3.50->3.36 (n=14432); match time gap median 9.2 s; correction rot deg (x,y,z)=(+0.135,+0.132,-0.033) trans cm=(+2.6,+5.5,+1.6); iters 12
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.41->2.38 cm (n=128136); tilt about x/y adjusted by (+0.053,-0.121) deg, z by +0.0 cm; iters 6
    final correction rot deg (x,y,z)=(+0.188,+0.011,-0.033) trans cm=(+2.6,+5.5,+1.7)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v005_driven/calibration.json
```
