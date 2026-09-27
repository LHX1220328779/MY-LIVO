# wuhu/truck05/v019_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck05/v018_ground
- lineage: wuhu/truck05/v008_import -> wuhu/truck05/v017_axle -> wuhu/truck05/v018_ground -> wuhu/truck05/v019_driven
- created: 2026-09-26T14:26:30+08:00
- note: mcp-rebuild

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- run: results/mcp_t05_v018_lio48/Log/fusion/1790403809522468818/
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        2.98,
        2.18,
        -1.42
      ],
      "drpy_deg": [
        0.04,
        0.006,
        -0.039
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -1.02,
        0.71,
        -5.31
      ],
      "drpy_deg": [
        -0.09,
        0.028,
        0.061
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        1.66,
        0.81,
        -2.57
      ],
      "drpy_deg": [
        -0.011,
        -0.011,
        -0.224
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
        2.8517,
        5.5544,
        1.946
      ],
      "rpy_deg": [
        0.696,
        -2.271,
        -1.14
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.137,
        -0.6639,
        0.7946
      ],
      "rpy_deg": [
        -1.053,
        0.992,
        -90.25
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -2.9725,
        5.476,
        1.9103
      ],
      "rpy_deg": [
        -0.428,
        -1.941,
        179.676
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
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v018_ground/calibration.json
run=results/mcp_t05_v018_lio48/Log/fusion/1790403809522468818
window 8.0..40.0 s, lidar_time_offset -0.0328
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
reference front_lidar: 582254 world points
front_right_lidar: pass 1 matches 291560->291548 |res| median 4.37->4.10 cm; ground 4.47->4.25 (n=153359); vertical 4.10->3.73 (n=103173); match time gap median 9.2 s; correction rot deg (x,y,z)=(+0.056,+0.076,-0.038) trans cm=(+3.0,+2.2,-1.1); iters 14
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.09->3.82 cm (n=137154); tilt about x/y adjusted by (-0.015,-0.071) deg, z by -0.3 cm; iters 10
    final correction rot deg (x,y,z)=(+0.040,+0.005,-0.038) trans cm=(+3.0,+2.2,-1.4)  [quality: ok]
front_left_lidar: pass 1 matches 302328->302425 |res| median 5.02->4.20 cm; ground 4.88->4.90 (n=171062); vertical 5.11->2.97 (n=93359); match time gap median 7.7 s; correction rot deg (x,y,z)=(+0.054,-0.054,-0.224) trans cm=(+1.7,+0.8,-1.6); iters 16
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.52->4.18 cm (n=148029); tilt about x/y adjusted by (-0.043,+0.065) deg, z by -1.0 cm; iters 10
    final correction rot deg (x,y,z)=(+0.012,+0.011,-0.224) trans cm=(+1.7,+0.8,-2.6)  [quality: ok]
back_lidar: pass 1 matches 194996->193939 |res| median 4.49->3.62 cm; ground 7.70->4.40 (n=65361); vertical 3.23->3.13 (n=106939); match time gap median 19.7 s; correction rot deg (x,y,z)=(+0.232,-0.006,+0.062) trans cm=(-1.0,+0.7,-1.5); iters 17
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.33->4.66 cm (n=54019); tilt about x/y adjusted by (-0.203,+0.096) deg, z by -3.8 cm; iters 14
    final correction rot deg (x,y,z)=(+0.029,+0.090,+0.062) trans cm=(-1.0,+0.7,-5.3)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v019_driven/calibration.json
```
