# wuhu/truck05/v012_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck05/v011_ground
- lineage: wuhu/truck05/v008_import -> wuhu/truck05/v009_axle -> wuhu/truck05/v011_ground -> wuhu/truck05/v012_driven
- created: 2026-09-24T16:15:11+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck05_lio_0924_161043/Log/fusion/1790237459871651002
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        2.93,
        2.34,
        -2.92
      ],
      "drpy_deg": [
        0.076,
        -0.01,
        -0.04
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -1.08,
        0.84,
        -6.04
      ],
      "drpy_deg": [
        -0.135,
        0.08,
        0.061
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        1.69,
        0.89,
        -3.0
      ],
      "drpy_deg": [
        -0.013,
        0.0,
        -0.227
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
            -0.3038116671628296,
            0.5824862358503357,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            4.515085132903587
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.23316749866069172,
            -0.2461681982492946,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -1.017195227904469
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            0.8526130353274589,
            -0.749096222613463,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            7.45142489198036
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
              0.07593193727290418,
              -0.011310824412359316,
              -0.03752892923514896
            ],
            "trans_cm": [
              2.9292624652576764,
              2.3470720182793716,
              -2.916733532593025
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 14,
              "matches": 288000,
              "vertical_matches": 100940,
              "median_before_cm": 4.32,
              "median_after_cm": 4.12,
              "vertical_before_cm": 4.11,
              "vertical_after_cm": 3.8,
              "rotation_deg": 0.085,
              "translation_cm": 4.8,
              "match_time_gap_s": 8.89,
              "pass2": {
                "ground_before_cm": 4.1,
                "ground_after_cm": 3.97,
                "ground_matches": 134789,
                "delta_rotvec_deg": [
                  0.011,
                  -0.0699,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.43
                ],
                "iterations": 10
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.013014826266632413,
              -0.00025136436127947884,
              -0.22756393288669177
            ],
            "trans_cm": [
              1.6944031628255196,
              0.890075083248345,
              -3.001895787419965
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 16,
              "matches": 302469,
              "vertical_matches": 90749,
              "median_before_cm": 5.01,
              "median_after_cm": 4.17,
              "vertical_before_cm": 5.1,
              "vertical_after_cm": 2.92,
              "rotation_deg": 0.228,
              "translation_cm": 3.6,
              "match_time_gap_s": 7.76,
              "pass2": {
                "ground_before_cm": 4.42,
                "ground_after_cm": 4.26,
                "ground_matches": 146774,
                "delta_rotvec_deg": [
                  -0.0205,
                  0.0439,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.83
                ],
                "iterations": 9
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.0805079954045386,
              0.13451297344605653,
              0.06402832869762036
            ],
            "trans_cm": [
              -1.0813258312967282,
              0.841500773194666,
              -6.041502358687083
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 18,
              "matches": 190717,
              "vertical_matches": 105396,
              "median_before_cm": 4.44,
              "median_after_cm": 3.62,
              "vertical_before_cm": 3.21,
              "vertical_after_cm": 3.13,
              "rotation_deg": 0.169,
              "translation_cm": 6.2,
              "match_time_gap_s": 19.54,
              "pass2": {
                "ground_before_cm": 5.16,
                "ground_after_cm": 4.92,
                "ground_matches": 54604,
                "delta_rotvec_deg": [
                  -0.169,
                  0.0878,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -5.22
                ],
                "iterations": 15
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck05_lio_0924_161043/Log/fusion/1790237459871651002",
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
        2.8304,
        5.4637,
        1.9375
      ],
      "rpy_deg": [
        0.728,
        -2.253,
        -1.135
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.116,
        -0.7549,
        0.7833
      ],
      "rpy_deg": [
        -1.061,
        1.041,
        -90.243
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -2.9929,
        5.3839,
        1.9021
      ],
      "rpy_deg": [
        -0.464,
        -1.941,
        179.68
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0968,
        7.5648,
        0.5356
      ],
      "rpy_deg": [
        0.072,
        -5.301,
        89.874
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0022,
      7.5704,
      2.2531
    ],
    "rpy_deg": [
      -100.385,
      0.003,
      -0.449
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
    "lidar": -0.0368,
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
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v011_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck05_lio_0924_161043/Log/fusion/1790237459871651002
window 8.0..40.0 s, lidar_time_offset -0.0368
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
reference front_lidar: 582402 world points
front_right_lidar: pass 1 matches 288192->288000 |res| median 4.32->4.12 cm; ground 4.35->4.26 (n=153044); vertical 4.11->3.80 (n=100940); match time gap median 8.9 s; correction rot deg (x,y,z)=(+0.065,+0.059,-0.038) trans cm=(+2.9,+2.3,-1.5); iters 14
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.10->3.97 cm (n=134789); tilt about x/y adjusted by (+0.011,-0.070) deg, z by -1.4 cm; iters 10
    final correction rot deg (x,y,z)=(+0.076,-0.011,-0.038) trans cm=(+2.9,+2.3,-2.9)  [quality: ok]
front_left_lidar: pass 1 matches 303083->302469 |res| median 5.01->4.17 cm; ground 4.90->4.83 (n=174138); vertical 5.10->2.92 (n=90749); match time gap median 7.8 s; correction rot deg (x,y,z)=(+0.033,-0.044,-0.228) trans cm=(+1.7,+0.9,-1.2); iters 16
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.42->4.26 cm (n=146774); tilt about x/y adjusted by (-0.020,+0.044) deg, z by -1.8 cm; iters 9
    final correction rot deg (x,y,z)=(+0.013,-0.000,-0.228) trans cm=(+1.7,+0.9,-3.0)  [quality: ok]
back_lidar: pass 1 matches 191585->190717 |res| median 4.44->3.62 cm; ground 7.57->4.30 (n=64623); vertical 3.21->3.13 (n=105396); match time gap median 19.5 s; correction rot deg (x,y,z)=(+0.249,+0.047,+0.064) trans cm=(-1.1,+0.8,-0.8); iters 18
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.16->4.92 cm (n=54604); tilt about x/y adjusted by (-0.169,+0.088) deg, z by -5.2 cm; iters 15
    final correction rot deg (x,y,z)=(+0.081,+0.135,+0.064) trans cm=(-1.1,+0.8,-6.0)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v012_driven/calibration.json
```
