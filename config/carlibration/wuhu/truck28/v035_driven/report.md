# wuhu/truck28/v035_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck28/v034_ground
- lineage: wuhu/truck28/v023_import -> wuhu/truck28/v033_axle -> wuhu/truck28/v034_ground -> wuhu/truck28/v035_driven
- created: 2026-10-26T18:11:06+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu/28/测试/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/测试/ros2bag', 'kind': 'raw_split', 'segments': 4, 'duration_s': 119.99, 'has_camera': False}
- run: /home/project/MY-LIVO/results/calib_truck28_lio_1026_180828/Log/fusion/1793009426586304111
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        2.63,
        -0.85,
        1.0
      ],
      "drpy_deg": [
        -0.028,
        -0.022,
        0.073
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.8,
        -0.53,
        10.52
      ],
      "drpy_deg": [
        -0.078,
        0.646,
        -0.099
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -4.07,
        -2.55,
        10.16
      ],
      "drpy_deg": [
        0.351,
        0.545,
        0.335
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
            -0.3900244226020497,
            -0.0525585658879712,
            0.010683180537139247
          ],
          "trans_cm": [
            0.0,
            0.0,
            8.423361883762299
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "warn",
            "reasons": [
              "plane start applied (0.28 deg / +6.5 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
            ],
            "iterations": 23,
            "matches": 9234,
            "ground_before_cm": 8.31,
            "ground_after_cm": 5.79,
            "rotation_deg": 0.394,
            "translation_cm": 8.4
          },
          "plane_check": {
            "tilt_deg": 0.284,
            "fore_aft_deg": -0.283,
            "lateral_deg": 0.02,
            "dz_cm": 6.5,
            "pairs": 1,
            "outside_basin": true,
            "applied": true
          }
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.23432155539175137,
            -0.1380629692770216,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -2.840846963356992
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 25,
            "matches": 9174,
            "ground_before_cm": 10.04,
            "ground_after_cm": 9.72,
            "rotation_deg": 0.272,
            "translation_cm": 2.8
          },
          "plane_check": {
            "tilt_deg": 0.821,
            "fore_aft_deg": -0.146,
            "lateral_deg": 0.808,
            "dz_cm": 2.3,
            "pairs": 1,
            "outside_basin": true,
            "applied": false
          }
        },
        "back_lidar": {
          "rotvec_deg": [
            -1.5729001810576981,
            -0.6483886161550155,
            0.023275616863564542
          ],
          "trans_cm": [
            0.0,
            0.0,
            -9.432228122698124
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "warn",
            "reasons": [
              "plane start applied (1.53 deg / -8.1 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
            ],
            "iterations": 20,
            "matches": 17801,
            "ground_before_cm": 11.69,
            "ground_after_cm": 8.97,
            "rotation_deg": 1.701,
            "translation_cm": 9.4
          },
          "plane_check": {
            "tilt_deg": 1.526,
            "fore_aft_deg": -0.912,
            "lateral_deg": 0.973,
            "dz_cm": -8.1,
            "pairs": 14,
            "outside_basin": true,
            "applied": true
          }
        }
      },
      "quality": "warn",
      "source": "/home/project/data/haibo/wuhu/28/测试/ros2bag",
      "stationary_frames": [
        252,
        20
      ],
      "moving_frames": [
        990,
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
              -0.028214900275307646,
              -0.021613733559433156,
              0.07376557475302116
            ],
            "trans_cm": [
              2.621139802804512,
              -0.8450073962235615,
              0.9969762814127533
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 7,
              "matches": 78195,
              "vertical_matches": 2155,
              "median_before_cm": 1.2,
              "median_after_cm": 1.03,
              "vertical_before_cm": 2.63,
              "vertical_after_cm": 2.61,
              "rotation_deg": 0.082,
              "translation_cm": 2.9,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              -0.36506517341421413,
              -0.5359731807769985,
              0.3379815780064121
            ],
            "trans_cm": [
              -4.062027970962429,
              -2.5488729713238656,
              10.156414493475504
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 9,
              "matches": 77052,
              "vertical_matches": 1665,
              "median_before_cm": 4.74,
              "median_after_cm": 1.15,
              "vertical_before_cm": 6.93,
              "vertical_after_cm": 2.63,
              "rotation_deg": 0.731,
              "translation_cm": 11.2,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.6468587897735003,
              0.07141408511854652,
              -0.09591908447400038
            ],
            "trans_cm": [
              0.7928625350966279,
              -0.524267914313911,
              10.523757351178604
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 13,
              "matches": 15348,
              "vertical_matches": 393,
              "median_before_cm": 8.2,
              "median_after_cm": 1.63,
              "vertical_before_cm": 4.47,
              "vertical_after_cm": 4.61,
              "rotation_deg": 0.658,
              "translation_cm": 10.6,
              "match_time_gap_s": 10.1,
              "pass2": {
                "ground_before_cm": 1.81,
                "ground_after_cm": 1.54,
                "ground_matches": 14607,
                "delta_rotvec_deg": [
                  -0.0138,
                  -0.0119,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  0.68
                ],
                "iterations": 3
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck28_lio_1026_180828/Log/fusion/1793009426586304111",
        "window": [
          8.0,
          27.200008153915405
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
        2.7569,
        5.8491,
        1.4713
      ],
      "rpy_deg": [
        0.296,
        2.419,
        -0.112
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2089,
        -0.338,
        0.3361
      ],
      "rpy_deg": [
        -1.859,
        2.428,
        -90.631
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1111,
        5.6011,
        1.6864
      ],
      "rpy_deg": [
        -0.922,
        -0.145,
        178.698
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.144,
        7.9129,
        -0.1568
      ],
      "rpy_deg": [
        2.014,
        -3.526,
        89.96
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1923,
      7.8117,
      1.5926
    ],
    "rpy_deg": [
      -100.296,
      0.152,
      0.901
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1637.05,
      1851.4,
      1029.33,
      495.55
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
            1637.05,
            1851.4,
            1029.33,
            495.55
          ],
          "rms_px": 0.014,
          "circle_centre_rear_m": [
            0.0,
            7.38
          ],
          "ground_z_rear_m": -1.22
        },
        "square": {
          "intrinsics": [
            1639.36,
            1639.36,
            864.27,
            446.63
          ],
          "rms_px": 1.072,
          "circle_centre_rear_m": [
            0.806,
            7.314
          ],
          "ground_z_rear_m": -1.588
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.014,
      "camera_height_m": 2.827,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -0.34,
        "fy_pct": -0.22,
        "cx_px": 58.3,
        "cy_px": -38.7
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.014 px)"
  },
  "time_offsets": {
    "lidar": -0.1609,
    "lidar_frontend": -0.1559,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.1609 + imu/ins clock lag +0.0050 (corr 0.997); template -0.1250 replaced"
  }
}
```

## stage report

```
warning: window end 40.0 s exceeds available poses (27.7 s); clamping
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck28/v034_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck28_lio_1026_180828/Log/fusion/1793009426586304111
window 8.0..27.200008153915405 s, lidar_time_offset -0.1559
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:194, back_lidar:194, front_left_lidar:194, front_lidar:194
scan source: round archive scans.bin/sources.bin (mapper deskew; returns at scan end, round extrinsics inverted)
matching: viewing-side consistent (oriented normals)
reference front_lidar: 253681 world points
pairing: same-time for LiDARs overlapping the reference (1.0 s blocks +-0.5 s, 1743337 block points); cross-time LiDARs use the accumulated target (nearest-range voxels)
front_right_lidar: pass 1 matches 86477->78195 |res| median 1.20->1.03 cm; ground 1.15->0.98 (n=73895); vertical 2.63->2.61 (n=2155); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.028,-0.022,+0.074) trans cm=(+2.6,-0.8,+1.0); iters 7  [quality: ok]
front_left_lidar: pass 1 matches 86858->77052 |res| median 4.74->1.15 cm; ground 4.67->1.12 (n=73590); vertical 6.93->2.63 (n=1665); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.365,-0.536,+0.338) trans cm=(-4.1,-2.5,+10.2); iters 9  [quality: ok]
back_lidar: pass 1 matches 15280->15348 |res| median 8.20->1.63 cm; ground 8.39->1.57 (n=14593); vertical 4.47->4.61 (n=393); match time gap median 10.1 s; correction rot deg (x,y,z)=(+0.661,+0.083,-0.096) trans cm=(+0.8,-0.5,+9.8); iters 13
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 1.81->1.54 cm (n=14607); tilt about x/y adjusted by (-0.014,-0.012) deg, z by +0.7 cm; iters 3
    final correction rot deg (x,y,z)=(+0.647,+0.071,-0.096) trans cm=(+0.8,-0.5,+10.5)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck28/v035_driven/calibration.json
```
