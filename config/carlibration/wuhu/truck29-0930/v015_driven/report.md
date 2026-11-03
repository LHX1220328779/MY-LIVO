# wuhu/truck29-0930/v015_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29-0930/v014_ground
- lineage: wuhu/truck29-0930/v002_import -> wuhu/truck29-0930/v013_axle -> wuhu/truck29-0930/v014_ground -> wuhu/truck29-0930/v015_driven
- created: 2026-10-19T17:30:01+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29-0930_lio_1019_172507/Log/fusion/1792402026393837050
- window: [0.0, 180.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -0.31,
        0.9,
        2.91
      ],
      "drpy_deg": [
        -0.041,
        -0.101,
        0.031
      ]
    },
    "back_lidar": {
      "dt_cm": [
        2.61,
        4.49,
        2.49
      ],
      "drpy_deg": [
        -0.012,
        0.185,
        -0.041
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -0.18,
        5.98,
        6.43
      ],
      "drpy_deg": [
        0.129,
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
              -0.0432893945230612,
              -0.0996818329536419,
              0.028886012301174283
            ],
            "trans_cm": [
              -0.31089170903100377,
              0.900348689155152,
              2.9145865461399207
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 7,
              "matches": 133749,
              "vertical_matches": 3760,
              "median_before_cm": 2.61,
              "median_after_cm": 1.15,
              "vertical_before_cm": 2.69,
              "vertical_after_cm": 2.25,
              "rotation_deg": 0.112,
              "translation_cm": 3.1,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              -0.12717597301187614,
              0.04864985246374445,
              -0.018244246221840264
            ],
            "trans_cm": [
              -0.17752372660186194,
              5.979595099901179,
              6.435191116713812
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 8,
              "matches": 146089,
              "vertical_matches": 2654,
              "median_before_cm": 4.97,
              "median_after_cm": 1.55,
              "vertical_before_cm": 4.12,
              "vertical_after_cm": 2.26,
              "rotation_deg": 0.137,
              "translation_cm": 8.8,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.18545873210994968,
              0.0072865760064466534,
              -0.04091627582591709
            ],
            "trans_cm": [
              2.6056475561861867,
              4.490226122344589,
              2.488476411843101
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 16,
              "matches": 148627,
              "vertical_matches": 14431,
              "median_before_cm": 3.16,
              "median_after_cm": 2.88,
              "vertical_before_cm": 3.48,
              "vertical_after_cm": 3.39,
              "rotation_deg": 0.19,
              "translation_cm": 5.8,
              "match_time_gap_s": 9.3,
              "pass2": {
                "ground_before_cm": 3.43,
                "ground_after_cm": 2.34,
                "ground_matches": 126679,
                "delta_rotvec_deg": [
                  0.0907,
                  -0.0628,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  0.85
                ],
                "iterations": 6
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29-0930_lio_1019_172507/Log/fusion/1792402026393837050",
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
        2.7574,
        5.9035,
        1.8372
      ],
      "rpy_deg": [
        -1.038,
        -3.039,
        -1.258
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0973,
        -0.3234,
        0.6803
      ],
      "rpy_deg": [
        -1.064,
        0.341,
        -91.387
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1226,
        5.6504,
        2.0188
      ],
      "rpy_deg": [
        3.26,
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
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v014_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29-0930_lio_1019_172507/Log/fusion/1792402026393837050
window 0.0..177.2999393939972 s, lidar_time_offset -0.0322
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:1770, back_lidar:1770, front_left_lidar:1770, front_lidar:1770
scan source: round archive scans.bin/sources.bin (mapper deskew; returns at scan end, round extrinsics inverted)
matching: viewing-side consistent (oriented normals)
reference front_lidar: 1546303 world points
pairing: same-time for LiDARs overlapping the reference (1.0 s blocks +-0.5 s, 11507993 block points); cross-time LiDARs use the accumulated target (nearest-range voxels)
front_right_lidar: pass 1 matches 147725->133749 |res| median 2.61->1.15 cm; ground 2.60->1.12 (n=127420); vertical 2.69->2.25 (n=3760); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.043,-0.100,+0.029) trans cm=(-0.3,+0.9,+2.9); iters 7  [quality: ok]
front_left_lidar: pass 1 matches 160351->146089 |res| median 4.97->1.55 cm; ground 4.99->1.52 (n=141123); vertical 4.12->2.26 (n=2654); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.127,+0.049,-0.018) trans cm=(-0.2,+6.0,+6.4); iters 8  [quality: ok]
back_lidar: pass 1 matches 147428->148627 |res| median 3.16->2.88 cm; ground 3.03->2.72 (n=125414); vertical 3.48->3.39 (n=14431); match time gap median 9.3 s; correction rot deg (x,y,z)=(+0.095,+0.070,-0.041) trans cm=(+2.6,+4.5,+1.6); iters 16
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.43->2.34 cm (n=126679); tilt about x/y adjusted by (+0.091,-0.063) deg, z by +0.9 cm; iters 6
    final correction rot deg (x,y,z)=(+0.185,+0.007,-0.041) trans cm=(+2.6,+4.5,+2.5)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v015_driven/calibration.json
```
