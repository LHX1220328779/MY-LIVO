# wuhu/truck29raw/v018_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29raw/v017_ground
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v015_axle -> wuhu/truck29raw/v017_ground -> wuhu/truck29raw/v018_driven
- created: 2026-10-14T16:51:26+08:00
- note: pipeline rerun after ground-stage plane-start fix (front_left_lidar moved vs v001_import)

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29raw_lio_1014_164046/Log/fusion/1791967267198587947
- window: [0.0, 120.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        1.0,
        1.75,
        -1.92
      ],
      "drpy_deg": [
        0.07,
        -0.036,
        0.055
      ]
    },
    "back_lidar": {
      "dt_cm": [
        3.08,
        7.94,
        1.49
      ],
      "drpy_deg": [
        -0.028,
        0.286,
        -0.059
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.73,
        1.51,
        -2.83
      ],
      "drpy_deg": [
        -0.107,
        0.047,
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
            -0.11855025551452632,
            0.29627186305878495,
            0.10508290040351304
          ],
          "trans_cm": [
            3.1144554778010916,
            4.78302874073508,
            -0.8401158233368909
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 16,
            "matches": 5464,
            "ground_before_cm": 9.6,
            "ground_after_cm": 6.68,
            "rotation_deg": 0.336,
            "translation_cm": 5.8
          },
          "plane_check": {
            "tilt_deg": 0.205,
            "fore_aft_deg": -0.139,
            "lateral_deg": -0.151,
            "dz_cm": -1.1,
            "pairs": 1,
            "outside_basin": false,
            "applied": false
          }
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -6.286666048365164,
            2.6272417795734584,
            0.09347327617663281
          ],
          "trans_cm": [
            1.6690477482442136,
            2.602713903544184,
            -3.0935534141557515
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "warn",
            "reasons": [
              "plane start applied (6.75 deg / -4.0 cm): the sensor moved relative to the parent calibration; yaw/x/y are re-checked by the driven stage"
            ],
            "iterations": 25,
            "matches": 6411,
            "ground_before_cm": 15.47,
            "ground_after_cm": 8.8,
            "rotation_deg": 6.814,
            "translation_cm": 4.4
          },
          "plane_check": {
            "tilt_deg": 6.749,
            "fore_aft_deg": -6.233,
            "lateral_deg": -2.596,
            "dz_cm": -4.0,
            "pairs": 1,
            "outside_basin": true,
            "applied": true
          }
        },
        "back_lidar": {
          "rotvec_deg": [
            -0.42560017532188144,
            -0.1576293613044073,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -5.74774033408779
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 21,
            "matches": 32410,
            "ground_before_cm": 10.71,
            "ground_after_cm": 9.56,
            "rotation_deg": 0.454,
            "translation_cm": 5.7
          },
          "plane_check": {
            "tilt_deg": 1.925,
            "fore_aft_deg": -0.143,
            "lateral_deg": 1.924,
            "dz_cm": 10.6,
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
              0.06832640960513835,
              -0.037173340757605665,
              0.05806870113025694
            ],
            "trans_cm": [
              1.0003602913362184,
              1.7500146660765517,
              -1.9167573676234366
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 9,
              "matches": 312494,
              "vertical_matches": 61178,
              "median_before_cm": 3.79,
              "median_after_cm": 3.71,
              "vertical_before_cm": 3.65,
              "vertical_after_cm": 3.26,
              "rotation_deg": 0.097,
              "translation_cm": 2.8,
              "match_time_gap_s": 5.36,
              "pass2": {
                "ground_before_cm": 3.73,
                "ground_after_cm": 3.72,
                "ground_matches": 220305,
                "delta_rotvec_deg": [
                  0.0034,
                  -0.0073,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  0.29
                ],
                "iterations": 4
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.10498634542142683,
              -0.05102610799922409,
              -0.03803021558145681
            ],
            "trans_cm": [
              0.7291038581726863,
              1.5058523571073736,
              -2.8300932801985184
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 11,
              "matches": 356229,
              "vertical_matches": 80999,
              "median_before_cm": 4.77,
              "median_after_cm": 4.74,
              "vertical_before_cm": 3.55,
              "vertical_after_cm": 3.21,
              "rotation_deg": 0.123,
              "translation_cm": 3.3,
              "match_time_gap_s": 5.3,
              "pass2": {
                "ground_before_cm": 5.49,
                "ground_after_cm": 5.54,
                "ground_matches": 228437,
                "delta_rotvec_deg": [
                  0.0283,
                  -0.0047,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -0.61
                ],
                "iterations": 9
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.28598536543548575,
              0.021863127726418834,
              -0.05870265716515694
            ],
            "trans_cm": [
              3.0824635598916346,
              7.941952592275473,
              1.4899484262236653
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 18,
              "matches": 108744,
              "vertical_matches": 14147,
              "median_before_cm": 8.1,
              "median_after_cm": 5.34,
              "vertical_before_cm": 4.31,
              "vertical_after_cm": 4.42,
              "rotation_deg": 0.293,
              "translation_cm": 8.6,
              "match_time_gap_s": 16.19,
              "pass2": {
                "ground_before_cm": 3.55,
                "ground_after_cm": 2.89,
                "ground_matches": 82985,
                "delta_rotvec_deg": [
                  0.1351,
                  -0.0484,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  5.72
                ],
                "iterations": 8
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29raw_lio_1014_164046/Log/fusion/1791967267198587947",
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
        2.7567,
        5.916,
        1.7827
      ],
      "rpy_deg": [
        -0.921,
        -2.978,
        -1.239
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0877,
        -0.2857,
        0.6581
      ],
      "rpy_deg": [
        -1.016,
        0.293,
        -91.375
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1248,
        5.6735,
        1.929
      ],
      "rpy_deg": [
        3.024,
        -4.479,
        178.246
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1291,
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
      -0.0164,
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
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v017_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29raw_lio_1014_164046/Log/fusion/1791967267198587947
window 0.0..117.30027866363525 s, lidar_time_offset -0.0311
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:1172, back_lidar:1172, front_left_lidar:1172, front_lidar:1173
matching: viewing-side consistent (oriented normals)
reference front_lidar: 1552970 world points
front_right_lidar: pass 1 matches 313202->312494 |res| median 3.79->3.71 cm; ground 3.69->3.73 (n=214115); vertical 3.65->3.26 (n=61178); match time gap median 5.4 s; correction rot deg (x,y,z)=(+0.065,-0.030,+0.058) trans cm=(+1.0,+1.8,-2.2); iters 9
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.73->3.72 cm (n=220305); tilt about x/y adjusted by (+0.003,-0.007) deg, z by +0.3 cm; iters 4
    final correction rot deg (x,y,z)=(+0.068,-0.037,+0.058) trans cm=(+1.0,+1.8,-1.9)  [quality: ok]
front_left_lidar: pass 1 matches 355379->356229 |res| median 4.77->4.74 cm; ground 5.37->5.55 (n=225199); vertical 3.55->3.21 (n=80999); match time gap median 5.3 s; correction rot deg (x,y,z)=(+0.077,-0.046,-0.038) trans cm=(+0.7,+1.5,-2.2); iters 11
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.49->5.54 cm (n=228437); tilt about x/y adjusted by (+0.028,-0.005) deg, z by -0.6 cm; iters 9
    final correction rot deg (x,y,z)=(+0.105,-0.051,-0.038) trans cm=(+0.7,+1.5,-2.8)  [quality: ok]
back_lidar: pass 1 matches 107810->108744 |res| median 8.10->5.34 cm; ground 9.18->5.48 (n=82054); vertical 4.31->4.42 (n=14147); match time gap median 16.2 s; correction rot deg (x,y,z)=(+0.151,+0.070,-0.059) trans cm=(+3.1,+7.9,-4.2); iters 18
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.55->2.89 cm (n=82985); tilt about x/y adjusted by (+0.135,-0.048) deg, z by +5.7 cm; iters 8
    final correction rot deg (x,y,z)=(+0.286,+0.022,-0.059) trans cm=(+3.1,+7.9,+1.5)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v018_driven/calibration.json
```
