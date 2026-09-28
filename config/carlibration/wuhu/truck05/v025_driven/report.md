# wuhu/truck05/v025_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck05/v024_precheck
- lineage: wuhu/truck05/v008_import -> wuhu/truck05/v017_axle -> wuhu/truck05/v018_ground -> wuhu/truck05/v019_driven -> wuhu/truck05/v020_manual -> wuhu/truck05/v021_manual -> wuhu/truck05/v022_manual -> wuhu/truck05/v023_manual -> wuhu/truck05/v024_precheck -> wuhu/truck05/v025_driven
- created: 2026-09-29T01:07:28+08:00
- note: same-side driven refinement from mcp_t05_lio_final (fused poses, 40-130 s)

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- run: results/mcp_t05_lio_final/Log/fusion/1790609499997803105
- window: [40.0, 130.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -0.65,
        2.15,
        -4.45
      ],
      "drpy_deg": [
        0.109,
        0.284,
        -0.015
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -13.9,
        0.04,
        6.22
      ],
      "drpy_deg": [
        0.016,
        0.298,
        0.089
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -8.42,
        1.7,
        -3.96
      ],
      "drpy_deg": [
        -0.127,
        0.169,
        -0.104
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
            -0.2844966981654353,
            0.561672176532908,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            3.7455829832852885
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.25206427117409136,
            -0.24421954531299822,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -0.6178241240599218
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            0.8714478099889951,
            -0.6986826223963227,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            7.621104425690888
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "/home/project/data/haibo/wuhu/05/20260914/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        1840,
        40
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              0.040248747933028066,
              0.004899635980104501,
              -0.03779965323245892
            ],
            "trans_cm": [
              2.97950657631345,
              2.1764656227235357,
              -1.420668411093379
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 14,
              "matches": 291548,
              "vertical_matches": 103173,
              "median_before_cm": 4.37,
              "median_after_cm": 4.1,
              "vertical_before_cm": 4.1,
              "vertical_after_cm": 3.73,
              "rotation_deg": 0.055,
              "translation_cm": 4.0,
              "match_time_gap_s": 9.19,
              "pass2": {
                "ground_before_cm": 4.09,
                "ground_after_cm": 3.82,
                "ground_matches": 137154,
                "delta_rotvec_deg": [
                  -0.0153,
                  -0.0708,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -0.31
                ],
                "iterations": 10
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.011501377154353547,
              0.011416728353471623,
              -0.2243656717902158
            ],
            "trans_cm": [
              1.6542828788895552,
              0.8102082192122261,
              -2.5670269913396058
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 16,
              "matches": 302425,
              "vertical_matches": 93359,
              "median_before_cm": 5.02,
              "median_after_cm": 4.2,
              "vertical_before_cm": 5.11,
              "vertical_after_cm": 2.97,
              "rotation_deg": 0.225,
              "translation_cm": 3.2,
              "match_time_gap_s": 7.66,
              "pass2": {
                "ground_before_cm": 4.52,
                "ground_after_cm": 4.18,
                "ground_matches": 148029,
                "delta_rotvec_deg": [
                  -0.0428,
                  0.0652,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.02
                ],
                "iterations": 10
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.028749578505955508,
              0.089798611801484,
              0.06183267521119037
            ],
            "trans_cm": [
              -1.01674808113683,
              0.7131829715679616,
              -5.302653690776885
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 17,
              "matches": 193939,
              "vertical_matches": 106939,
              "median_before_cm": 4.49,
              "median_after_cm": 3.62,
              "vertical_before_cm": 3.23,
              "vertical_after_cm": 3.13,
              "rotation_deg": 0.113,
              "translation_cm": 5.4,
              "match_time_gap_s": 19.67,
              "pass2": {
                "ground_before_cm": 5.33,
                "ground_after_cm": 4.66,
                "ground_matches": 54019,
                "delta_rotvec_deg": [
                  -0.2033,
                  0.0961,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -3.82
                ],
                "iterations": 14
              }
            }
          }
        },
        "run": "results/mcp_t05_v018_lio48/Log/fusion/1790403809522468818",
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
              0.11404529671926389,
              0.28132173285593265,
              -0.010662524923370227
            ],
            "trans_cm": [
              -0.6527492636746427,
              2.1539636331231975,
              -4.4496904785297176
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 12,
              "matches": 200321,
              "vertical_matches": 200321,
              "median_before_cm": 3.19,
              "median_after_cm": 3.02,
              "vertical_before_cm": 3.19,
              "vertical_after_cm": 3.02,
              "rotation_deg": 0.304,
              "translation_cm": 5.0,
              "match_time_gap_s": 21.61,
              "pass2": {
                "ground_before_cm": 7.58,
                "ground_after_cm": 5.34,
                "ground_matches": 233204,
                "delta_rotvec_deg": [
                  0.114,
                  0.2813,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -4.45
                ],
                "iterations": 17
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.12497292337022406,
              -0.16972568481610248,
              -0.10823217943593877
            ],
            "trans_cm": [
              -8.415019042397034,
              1.7022698430195116,
              -3.9622754048347177
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 11,
              "matches": 181358,
              "vertical_matches": 181358,
              "median_before_cm": 5.6,
              "median_after_cm": 3.03,
              "vertical_before_cm": 5.6,
              "vertical_after_cm": 3.03,
              "rotation_deg": 0.237,
              "translation_cm": 9.5,
              "match_time_gap_s": 12.18,
              "pass2": {
                "ground_before_cm": 5.9,
                "ground_after_cm": 5.19,
                "ground_matches": 226634,
                "delta_rotvec_deg": [
                  0.125,
                  -0.1697,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -3.96
                ],
                "iterations": 16
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.29756938451455683,
              -0.016699658545712284,
              0.08923499308211992
            ],
            "trans_cm": [
              -13.902474254929553,
              0.03547544000308748,
              6.211032809493635
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 14,
              "matches": 165980,
              "vertical_matches": 165980,
              "median_before_cm": 5.4,
              "median_after_cm": 3.18,
              "vertical_before_cm": 5.4,
              "vertical_after_cm": 3.18,
              "rotation_deg": 0.311,
              "translation_cm": 15.2,
              "match_time_gap_s": 24.18,
              "pass2": {
                "ground_before_cm": 5.1,
                "ground_after_cm": 3.94,
                "ground_matches": 94354,
                "delta_rotvec_deg": [
                  0.2976,
                  -0.0167,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  6.21
                ],
                "iterations": 18
              }
            }
          }
        },
        "run": "results/mcp_t05_lio_final/Log/fusion/1790609499997803105",
        "window": [
          40.0,
          130.0
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
        2.8452,
        5.5759,
        1.9472
      ],
      "rpy_deg": [
        0.749,
        -2.113,
        -1.153
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.002,
        -0.6635,
        0.8568
      ],
      "rpy_deg": [
        -1.037,
        1.29,
        -90.161
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0567,
        5.493,
        1.9256
      ],
      "rpy_deg": [
        -0.481,
        -1.864,
        179.569
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0755,
        7.6571,
        0.5356
      ],
      "rpy_deg": [
        0.058,
        -5.285,
        89.869
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0232,
      7.6631,
      2.2531
    ],
    "rpy_deg": [
      -100.4,
      -0.011,
      -0.455
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1849.37,
      1849.37,
      1003.31,
      502.72
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
            1849.32,
            1831.26,
            1004.09,
            499.42
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            -0.002,
            7.529
          ],
          "ground_z_rear_m": -1.283
        },
        "square": {
          "intrinsics": [
            1849.37,
            1849.37,
            1003.31,
            502.72
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            0.001,
            7.535
          ],
          "ground_z_rear_m": -1.249
        }
      },
      "model": "square pixels (fx = fy)",
      "rms_px": 0.05,
      "camera_height_m": 3.502,
      "deviation_vs_truck29_defaults": {
        "fx_pct": 12.59,
        "fy_pct": -0.33,
        "cx_px": 32.3,
        "cy_px": -31.5
      }
    },
    "source": "supplier ground-distance curves equation_*m (square pixels (fx = fy), rms 0.05 px)"
  },
  "time_offsets": {
    "lidar": -0.0328,
    "camera": -0.0019,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend": -0.0119,
    "camera_source": "operator set (livo_calib.py set): 相机偏移按同源时钟规则：camera_h = 家族值-0.115 + 打戳差0.1131 = -0.0019 ≈ 0，与 v016 时代 img=0 最优的实测互证"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v024_precheck/calibration.json
run=results/mcp_t05_lio_final/Log/fusion/1790609499997803105
window 40.0..130.0 s, lidar_time_offset -0.0119
pass 1 poses: RTK-optimized trajectory_optimized.txt (yaw/x/y, vertical matches only); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:902, back_lidar:902, front_left_lidar:902, front_lidar:902
matching: viewing-side consistent (oriented normals)
reference front_lidar: 881260 world points
front_right_lidar: pass 1 matches 200747->200321 |res| median 3.19->3.02 cm; ground nan->nan (n=0); vertical 3.19->3.02 (n=200321); match time gap median 21.6 s; correction rot deg (x,y,z)=(+0.000,+0.000,-0.011) trans cm=(-0.7,+2.2,+0.0); iters 12
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 7.58->5.34 cm (n=233204); tilt about x/y adjusted by (+0.114,+0.281) deg, z by -4.4 cm; iters 17
    final correction rot deg (x,y,z)=(+0.114,+0.281,-0.011) trans cm=(-0.7,+2.2,-4.4)  [quality: ok]
front_left_lidar: pass 1 matches 177822->181358 |res| median 5.60->3.03 cm; ground nan->nan (n=0); vertical 5.60->3.03 (n=181358); match time gap median 12.2 s; correction rot deg (x,y,z)=(+0.000,+0.000,-0.108) trans cm=(-8.4,+1.7,+0.0); iters 11
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.90->5.19 cm (n=226634); tilt about x/y adjusted by (+0.125,-0.170) deg, z by -4.0 cm; iters 16
    final correction rot deg (x,y,z)=(+0.125,-0.170,-0.108) trans cm=(-8.4,+1.7,-4.0)  [quality: ok]
back_lidar: pass 1 matches 164907->165980 |res| median 5.40->3.18 cm; ground nan->nan (n=0); vertical 5.40->3.18 (n=165980); match time gap median 24.2 s; correction rot deg (x,y,z)=(+0.000,+0.000,+0.089) trans cm=(-13.9,+0.0,+0.0); iters 14
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.10->3.94 cm (n=94354); tilt about x/y adjusted by (+0.298,-0.017) deg, z by +6.2 cm; iters 18
    final correction rot deg (x,y,z)=(+0.298,-0.017,+0.089) trans cm=(-13.9,+0.0,+6.2)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v025_driven/calibration.json
```
