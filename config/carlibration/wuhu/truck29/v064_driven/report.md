# wuhu/truck29/v064_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29/v063_manual
- lineage: wuhu/truck29/v055_import -> wuhu/truck29/v056_axle -> wuhu/truck29/v057_ground -> wuhu/truck29/v058_driven -> wuhu/truck29/v059_camtime -> wuhu/truck29/v060_camlidar -> wuhu/truck29/v061_camloop -> wuhu/truck29/v062_manual -> wuhu/truck29/v063_manual -> wuhu/truck29/v064_driven
- created: 2026-09-28T22:33:15+08:00
- note: driven yaw/x/y with RTK-optimized pass-1 poses over 8-100 s (fused run mcp_t29_lio_src); motivation: fit-free along-track inter-source test shows back_lidar -4..-7 cm behind the front LiDARs (facing planes, poles)

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
        -1.27,
        -1.15
      ],
      "drpy_deg": [
        0.029,
        0.056,
        -0.049
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -0.73,
        -4.34,
        -3.35
      ],
      "drpy_deg": [
        0.063,
        -0.039,
        0.045
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        3.47,
        -2.97,
        -1.06
      ],
      "drpy_deg": [
        -0.018,
        0.077,
        -0.004
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
              0.03038504191753275,
              0.0544567105127458,
              -0.04809783808224576
            ],
            "trans_cm": [
              -0.32240480133218075,
              -1.272748440466844,
              -1.152752162236512
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "overall residual regressed 4.64->4.69 cm"
              ],
              "converged": true,
              "iterations": 17,
              "matches": 26790,
              "vertical_matches": 26790,
              "median_before_cm": 4.64,
              "median_after_cm": 4.69,
              "vertical_before_cm": 4.64,
              "vertical_after_cm": 4.69,
              "rotation_deg": 0.079,
              "translation_cm": 1.7,
              "match_time_gap_s": 4.03,
              "pass2": {
                "ground_before_cm": 3.85,
                "ground_after_cm": 3.87,
                "ground_matches": 244734,
                "delta_rotvec_deg": [
                  0.0304,
                  0.0545,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.15
                ],
                "iterations": 11
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.016550568472154987,
              -0.07770090010855686,
              -0.004083635124376178
            ],
            "trans_cm": [
              3.468203876021626,
              -2.9727442909989352,
              -1.0584984757330904
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 14,
              "matches": 28826,
              "vertical_matches": 28826,
              "median_before_cm": 4.49,
              "median_after_cm": 4.32,
              "vertical_before_cm": 4.49,
              "vertical_after_cm": 4.32,
              "rotation_deg": 0.08,
              "translation_cm": 4.7,
              "match_time_gap_s": 5.4,
              "pass2": {
                "ground_before_cm": 6.63,
                "ground_after_cm": 6.28,
                "ground_matches": 249860,
                "delta_rotvec_deg": [
                  0.0166,
                  -0.0777,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.06
                ],
                "iterations": 14
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              -0.040649346194585635,
              -0.06243603273446421,
              0.044418922412504384
            ],
            "trans_cm": [
              -0.7320737777769694,
              -4.336612744640509,
              -3.3581006551526342
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 13,
              "matches": 6833,
              "vertical_matches": 6833,
              "median_before_cm": 5.76,
              "median_after_cm": 4.78,
              "vertical_before_cm": 5.76,
              "vertical_after_cm": 4.78,
              "rotation_deg": 0.087,
              "translation_cm": 5.5,
              "match_time_gap_s": 12.16,
              "pass2": {
                "ground_before_cm": 3.88,
                "ground_after_cm": 3.37,
                "ground_matches": 62134,
                "delta_rotvec_deg": [
                  -0.0406,
                  -0.0624,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -3.36
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
        5.9887,
        1.8223
      ],
      "rpy_deg": [
        -0.976,
        -2.976,
        -1.475
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1023,
        -0.3432,
        0.6539
      ],
      "rpy_deg": [
        -1.021,
        0.71,
        -91.367
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0942,
        5.7759,
        1.9642
      ],
      "rpy_deg": [
        -3.431,
        -0.734,
        178.584
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
window 8.0..100.0 s, lidar_time_offset -0.1459
pass 1 poses: RTK-optimized trajectory_optimized.txt (yaw/x/y, vertical matches only); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:922, back_lidar:922, front_left_lidar:922, front_lidar:922
reference front_lidar: 1216151 world points
front_right_lidar: pass 1 matches 26750->26790 |res| median 4.64->4.69 cm; ground nan->nan (n=0); vertical 4.64->4.69 (n=26790); match time gap median 4.0 s; correction rot deg (x,y,z)=(+0.000,+0.000,-0.048) trans cm=(-0.3,-1.3,+0.0); iters 17
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.85->3.87 cm (n=244734); tilt about x/y adjusted by (+0.030,+0.054) deg, z by -1.2 cm; iters 11
    final correction rot deg (x,y,z)=(+0.030,+0.054,-0.048) trans cm=(-0.3,-1.3,-1.2)  [quality: warn; overall residual regressed 4.64->4.69 cm]
front_left_lidar: pass 1 matches 28903->28826 |res| median 4.49->4.32 cm; ground nan->nan (n=0); vertical 4.49->4.32 (n=28826); match time gap median 5.4 s; correction rot deg (x,y,z)=(+0.000,+0.000,-0.004) trans cm=(+3.5,-3.0,+0.0); iters 14
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 6.63->6.28 cm (n=249860); tilt about x/y adjusted by (+0.017,-0.078) deg, z by -1.1 cm; iters 14
    final correction rot deg (x,y,z)=(+0.017,-0.078,-0.004) trans cm=(+3.5,-3.0,-1.1)  [quality: ok]
back_lidar: pass 1 matches 6829->6833 |res| median 5.76->4.78 cm; ground nan->nan (n=0); vertical 5.76->4.78 (n=6833); match time gap median 12.2 s; correction rot deg (x,y,z)=(+0.000,+0.000,+0.044) trans cm=(-0.7,-4.3,+0.0); iters 13
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.88->3.37 cm (n=62134); tilt about x/y adjusted by (-0.041,-0.062) deg, z by -3.4 cm; iters 9
    final correction rot deg (x,y,z)=(-0.041,-0.062,+0.044) trans cm=(-0.7,-4.3,-3.4)  [quality: ok]
quality: warn  (front_right_lidar: overall residual regressed 4.64->4.69 cm)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v064_driven/calibration.json
```
