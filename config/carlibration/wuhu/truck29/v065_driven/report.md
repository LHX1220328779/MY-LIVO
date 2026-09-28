# wuhu/truck29/v065_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29/v063_manual
- lineage: wuhu/truck29/v055_import -> wuhu/truck29/v056_axle -> wuhu/truck29/v057_ground -> wuhu/truck29/v058_driven -> wuhu/truck29/v059_camtime -> wuhu/truck29/v060_camlidar -> wuhu/truck29/v061_camloop -> wuhu/truck29/v062_manual -> wuhu/truck29/v063_manual -> wuhu/truck29/v065_driven
- created: 2026-09-28T22:48:14+08:00
- note: driven yaw/x/y, RTK-optimized pass-1 poses, viewing-side-consistent matches, front-end time base; 8-100 s of fused run mcp_t29_lio_src

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: results/mcp_t29_lio_src/Log/fusion/1790597602894911609
- window: [8.0, 100.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -0.32,
        0.92,
        -1.2
      ],
      "drpy_deg": [
        0.032,
        0.06,
        -0.023
      ]
    },
    "back_lidar": {
      "dt_cm": [
        1.43,
        4.47,
        -4.0
      ],
      "drpy_deg": [
        0.048,
        -0.044,
        -0.065
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        3.55,
        -1.63,
        -1.19
      ],
      "drpy_deg": [
        -0.023,
        0.075,
        0.011
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
              0.007351702975966037,
              -0.11918714112742262,
              0.03299401832314971
            ],
            "trans_cm": [
              2.654772690632984,
              6.195531257217749,
              0.7742568226660305
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 17,
              "matches": 209911,
              "vertical_matches": 18631,
              "median_before_cm": 3.59,
              "median_after_cm": 3.06,
              "vertical_before_cm": 5.13,
              "vertical_after_cm": 3.92,
              "rotation_deg": 0.124,
              "translation_cm": 6.8,
              "match_time_gap_s": 3.33,
              "pass2": {
                "ground_before_cm": 3.83,
                "ground_after_cm": 3.2,
                "ground_matches": 168473,
                "delta_rotvec_deg": [
                  -0.063,
                  -0.0366,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.66
                ],
                "iterations": 10
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.06572967269374715,
              0.07912277561288371,
              0.0970407412297835
            ],
            "trans_cm": [
              0.31648529274322973,
              7.8201882152160955,
              -0.4310941376496209
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 15,
              "matches": 233269,
              "vertical_matches": 28271,
              "median_before_cm": 5.18,
              "median_after_cm": 4.8,
              "vertical_before_cm": 4.65,
              "vertical_after_cm": 3.94,
              "rotation_deg": 0.141,
              "translation_cm": 7.8,
              "match_time_gap_s": 5.59,
              "pass2": {
                "ground_before_cm": 5.05,
                "ground_after_cm": 4.7,
                "ground_matches": 149467,
                "delta_rotvec_deg": [
                  -0.0526,
                  -0.0042,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.16
                ],
                "iterations": 12
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.12242985466784119,
              0.0869519331006743,
              -0.03450009560998213
            ],
            "trans_cm": [
              1.248672104591068,
              -2.5849473635672764,
              -3.709328766446739
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 12,
              "matches": 48282,
              "vertical_matches": 3581,
              "median_before_cm": 10.74,
              "median_after_cm": 3.31,
              "vertical_before_cm": 4.89,
              "vertical_after_cm": 4.66,
              "rotation_deg": 0.154,
              "translation_cm": 4.7,
              "match_time_gap_s": 10.64,
              "pass2": {
                "ground_before_cm": 5.24,
                "ground_after_cm": 2.94,
                "ground_matches": 37765,
                "delta_rotvec_deg": [
                  0.0601,
                  -0.0083,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  6.76
                ],
                "iterations": 8
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29_lio_0924_133530/Log/fusion/1790228347157892405",
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
      },
      "stage3_vertical": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              0.03395404033459419,
              0.05926951591681087,
              -0.022184195897785114
            ],
            "trans_cm": [
              -0.32248397211525276,
              0.9234526420162317,
              -1.2012078700545104
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 12,
              "matches": 24484,
              "vertical_matches": 24484,
              "median_before_cm": 3.85,
              "median_after_cm": 3.77,
              "vertical_before_cm": 3.85,
              "vertical_after_cm": 3.77,
              "rotation_deg": 0.072,
              "translation_cm": 1.5,
              "match_time_gap_s": 3.15,
              "pass2": {
                "ground_before_cm": 3.47,
                "ground_after_cm": 3.47,
                "ground_matches": 232548,
                "delta_rotvec_deg": [
                  0.034,
                  0.0593,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.2
                ],
                "iterations": 10
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.02160100201790605,
              -0.07547816515004539,
              0.010575096389186226
            ],
            "trans_cm": [
              3.5517374757340265,
              -1.6371631712458357,
              -1.1898740471266966
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 16,
              "matches": 31179,
              "vertical_matches": 31179,
              "median_before_cm": 3.93,
              "median_after_cm": 3.73,
              "vertical_before_cm": 3.93,
              "vertical_after_cm": 3.73,
              "rotation_deg": 0.079,
              "translation_cm": 4.1,
              "match_time_gap_s": 4.98,
              "pass2": {
                "ground_before_cm": 6.52,
                "ground_after_cm": 6.15,
                "ground_matches": 236534,
                "delta_rotvec_deg": [
                  0.0216,
                  -0.0755,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.19
                ],
                "iterations": 13
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              -0.045402357063690266,
              -0.04663400511010636,
              -0.06561308023044271
            ],
            "trans_cm": [
              1.4300887363915429,
              4.470454182719456,
              -4.000034520173054
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "overall residual regressed 4.89->4.92 cm"
              ],
              "converged": true,
              "iterations": 20,
              "matches": 3549,
              "vertical_matches": 3549,
              "median_before_cm": 4.89,
              "median_after_cm": 4.92,
              "vertical_before_cm": 4.89,
              "vertical_after_cm": 4.92,
              "rotation_deg": 0.092,
              "translation_cm": 6.2,
              "match_time_gap_s": 10.75,
              "pass2": {
                "ground_before_cm": 3.89,
                "ground_after_cm": 3.19,
                "ground_matches": 56160,
                "delta_rotvec_deg": [
                  -0.0454,
                  -0.0466,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -4.0
                ],
                "iterations": 9
              }
            }
          }
        },
        "run": "results/mcp_t29_lio_src/Log/fusion/1790597602894911609",
        "window": [
          8.0,
          100.0
        ],
        "prior": [
          0.5,
          0.1
        ],
        "poses": {
          "pass1": "RTK-optimized trajectory_optimized.txt",
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
        2.7559,
        6.0106,
        1.8218
      ],
      "rpy_deg": [
        -0.973,
        -2.972,
        -1.449
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0807,
        -0.2551,
        0.6474
      ],
      "rpy_deg": [
        -1.036,
        0.705,
        -91.477
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0934,
        5.7893,
        1.9629
      ],
      "rpy_deg": [
        -3.436,
        -0.736,
        178.599
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
      -96.512,
      1.62,
      0.291
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
    "lidar": -0.1459,
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "operator set (livo_calib.py set): 时间偏移改回 IMU 硬件对齐对（lidar -0.125 / camera -0.115）：axle 解 -0.1459 含 INS 链路延迟，喂给前端后 21ms 错位打花高角速率段的去畸变与颜色采样（灯柱白度 51.5%→79.3%，牌面着色同步改善）；融合精度在该区间平坦（LIVO 120s xy max 7.12→7.08 cm，达标）。证据链见 docs/mcp-color-regression-time-offsets.md",
    "lidar_frontend": -0.125
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v063_manual/calibration.json
run=results/mcp_t29_lio_src/Log/fusion/1790597602894911609
window 8.0..100.0 s, lidar_time_offset -0.125
pass 1 poses: RTK-optimized trajectory_optimized.txt (yaw/x/y, vertical matches only); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:922, back_lidar:922, front_left_lidar:922, front_lidar:922
matching: viewing-side consistent (oriented normals)
reference front_lidar: 1183529 world points
front_right_lidar: pass 1 matches 24526->24484 |res| median 3.85->3.77 cm; ground nan->nan (n=0); vertical 3.85->3.77 (n=24484); match time gap median 3.1 s; correction rot deg (x,y,z)=(+0.000,+0.000,-0.022) trans cm=(-0.3,+0.9,+0.0); iters 12
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.47->3.47 cm (n=232548); tilt about x/y adjusted by (+0.034,+0.059) deg, z by -1.2 cm; iters 10
    final correction rot deg (x,y,z)=(+0.034,+0.059,-0.022) trans cm=(-0.3,+0.9,-1.2)  [quality: ok]
front_left_lidar: pass 1 matches 30968->31179 |res| median 3.93->3.73 cm; ground nan->nan (n=0); vertical 3.93->3.73 (n=31179); match time gap median 5.0 s; correction rot deg (x,y,z)=(+0.000,+0.000,+0.011) trans cm=(+3.6,-1.6,+0.0); iters 16
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 6.52->6.15 cm (n=236534); tilt about x/y adjusted by (+0.022,-0.075) deg, z by -1.2 cm; iters 13
    final correction rot deg (x,y,z)=(+0.022,-0.075,+0.011) trans cm=(+3.6,-1.6,-1.2)  [quality: ok]
back_lidar: pass 1 matches 3468->3549 |res| median 4.89->4.92 cm; ground nan->nan (n=0); vertical 4.89->4.92 (n=3549); match time gap median 10.8 s; correction rot deg (x,y,z)=(+0.000,+0.000,-0.066) trans cm=(+1.4,+4.5,+0.0); iters 20
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.89->3.19 cm (n=56160); tilt about x/y adjusted by (-0.045,-0.047) deg, z by -4.0 cm; iters 9
    final correction rot deg (x,y,z)=(-0.045,-0.047,-0.066) trans cm=(+1.4,+4.5,-4.0)  [quality: warn; overall residual regressed 4.89->4.92 cm]
quality: warn  (back_lidar: overall residual regressed 4.89->4.92 cm)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v065_driven/calibration.json
```
