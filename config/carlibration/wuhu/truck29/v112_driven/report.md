# wuhu/truck29/v112_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29/v111_ground
- lineage: wuhu/truck29/v098_import -> wuhu/truck29/v110_axle -> wuhu/truck29/v111_ground -> wuhu/truck29/v112_driven
- created: 2026-10-12T21:04:14+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_lio_1012_205712/Log/fusion/1791810084349890307
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        1.39,
        7.23,
        0.71
      ],
      "drpy_deg": [
        0.011,
        -0.117,
        0.006
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -1.89,
        10.41,
        -8.0
      ],
      "drpy_deg": [
        -0.048,
        0.042,
        0.065
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        2.32,
        8.06,
        -1.2
      ],
      "drpy_deg": [
        -0.099,
        -0.043,
        0.09
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
            -0.11375252532893412,
            0.35971172011967334,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.3162870586212532
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            0.10584386170568374,
            -1.311256874923346,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -0.8598019669626147
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            0.22455630770273077,
            -0.13309519679281726,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.9722373004125837
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
              0.007670803537460049,
              -0.11709687224234096,
              0.006919622840164228
            ],
            "trans_cm": [
              1.3899823815768173,
              7.236125330884802,
              0.7126136652515037
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 11,
              "matches": 182715,
              "vertical_matches": 15587,
              "median_before_cm": 3.46,
              "median_after_cm": 2.91,
              "vertical_before_cm": 4.92,
              "vertical_after_cm": 3.66,
              "rotation_deg": 0.118,
              "translation_cm": 7.4,
              "match_time_gap_s": 3.05,
              "pass2": {
                "ground_before_cm": 3.28,
                "ground_after_cm": 3.07,
                "ground_matches": 146339,
                "delta_rotvec_deg": [
                  -0.0261,
                  -0.0341,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.38
                ],
                "iterations": 6
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.10039582512387767,
              0.040260601630419085,
              0.08875587441348858
            ],
            "trans_cm": [
              2.3192608011354254,
              8.05306319431394,
              -1.1939900843441587
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 11,
              "matches": 217225,
              "vertical_matches": 25959,
              "median_before_cm": 5.14,
              "median_after_cm": 4.89,
              "vertical_before_cm": 4.55,
              "vertical_after_cm": 3.84,
              "rotation_deg": 0.14,
              "translation_cm": 8.5,
              "match_time_gap_s": 5.39,
              "pass2": {
                "ground_before_cm": 5.04,
                "ground_after_cm": 4.93,
                "ground_matches": 138938,
                "delta_rotvec_deg": [
                  0.0427,
                  -0.0205,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -0.47
                ],
                "iterations": 7
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.04298986155971083,
              0.047223451709340954,
              0.06582774456561453
            ],
            "trans_cm": [
              -1.8901052813819006,
              10.403611302788104,
              -8.003943285590118
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "vertical residual regressed 4.53->5.15 cm"
              ],
              "converged": true,
              "iterations": 14,
              "matches": 41070,
              "vertical_matches": 2167,
              "median_before_cm": 6.65,
              "median_after_cm": 3.47,
              "vertical_before_cm": 4.53,
              "vertical_after_cm": 5.15,
              "rotation_deg": 0.092,
              "translation_cm": 13.3,
              "match_time_gap_s": 10.55,
              "pass2": {
                "ground_before_cm": 3.7,
                "ground_after_cm": 3.12,
                "ground_matches": 32897,
                "delta_rotvec_deg": [
                  0.121,
                  -0.0814,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.4
                ],
                "iterations": 6
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29_lio_1012_205712/Log/fusion/1791810084349890307",
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
        2.7465,
        6.0118,
        1.8332
      ],
      "rpy_deg": [
        -1.005,
        -3.03,
        -1.452
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1264,
        -0.1699,
        0.6445
      ],
      "rpy_deg": [
        -1.042,
        0.671,
        -91.311
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1089,
        5.808,
        1.9671
      ],
      "rpy_deg": [
        -3.448,
        -0.773,
        178.58
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1088,
        8.0401,
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
      0.0029,
      8.0422,
      1.7517
    ],
    "rpy_deg": [
      -96.506,
      1.636,
      0.424
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
            1642.2,
            1846.33,
            968.83,
            532.82
          ],
          "rms_px": 0.07,
          "circle_centre_rear_m": [
            0.007,
            7.533
          ],
          "ground_z_rear_m": -1.264
        },
        "square": {
          "intrinsics": [
            1642.2,
            1642.2,
            917.71,
            510.63
          ],
          "rms_px": 0.36,
          "circle_centre_rear_m": [
            0.255,
            7.49
          ],
          "ground_z_rear_m": -1.64
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.07,
      "camera_height_m": 3.019,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -0.02,
        "fy_pct": -0.5,
        "cx_px": -2.2,
        "cy_px": -1.4
      }
    },
    "source": "defaults (wuhu truck29 yaml), confirmed by the supplier ground-distance curves (rms 0.07 px)"
  },
  "time_offsets": {
    "lidar": -0.1459,
    "lidar_frontend": -0.125,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v111_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29_lio_1012_205712/Log/fusion/1791810084349890307
window 8.0..40.0 s, lidar_time_offset -0.125
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
matching: viewing-side consistent (oriented normals)
reference front_lidar: 514514 world points
front_right_lidar: pass 1 matches 182751->182715 |res| median 3.46->2.91 cm; ground 3.28->2.77 (n=149609); vertical 4.92->3.66 (n=15587); match time gap median 3.0 s; correction rot deg (x,y,z)=(+0.034,-0.083,+0.007) trans cm=(+1.4,+7.2,-0.7); iters 11
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.28->3.07 cm (n=146339); tilt about x/y adjusted by (-0.026,-0.034) deg, z by +1.4 cm; iters 6
    final correction rot deg (x,y,z)=(+0.008,-0.117,+0.007) trans cm=(+1.4,+7.2,+0.7)  [quality: ok]
front_left_lidar: pass 1 matches 217006->217225 |res| median 5.14->4.89 cm; ground 5.01->4.87 (n=141380); vertical 4.55->3.84 (n=25959); match time gap median 5.4 s; correction rot deg (x,y,z)=(+0.058,+0.061,+0.089) trans cm=(+2.3,+8.1,-0.7); iters 11
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.04->4.93 cm (n=138938); tilt about x/y adjusted by (+0.043,-0.021) deg, z by -0.5 cm; iters 7
    final correction rot deg (x,y,z)=(+0.100,+0.040,+0.089) trans cm=(+2.3,+8.1,-1.2)  [quality: ok]
back_lidar: pass 1 matches 40516->41070 |res| median 6.65->3.47 cm; ground 7.13->3.28 (n=33365); vertical 4.53->5.15 (n=2167); match time gap median 10.5 s; correction rot deg (x,y,z)=(-0.078,+0.129,+0.066) trans cm=(-1.9,+10.4,-10.4); iters 14
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.70->3.12 cm (n=32897); tilt about x/y adjusted by (+0.121,-0.081) deg, z by +2.4 cm; iters 6
    final correction rot deg (x,y,z)=(+0.043,+0.047,+0.066) trans cm=(-1.9,+10.4,-8.0)  [quality: warn; vertical residual regressed 4.53->5.15 cm]
quality: warn  (back_lidar: vertical residual regressed 4.53->5.15 cm)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v112_driven/calibration.json
```
