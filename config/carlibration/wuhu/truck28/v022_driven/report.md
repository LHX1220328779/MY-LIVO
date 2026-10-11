# wuhu/truck28/v022_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck28/v021_ground
- lineage: wuhu/truck28/v019_import -> wuhu/truck28/v020_axle -> wuhu/truck28/v021_ground -> wuhu/truck28/v022_driven
- created: 2026-10-11T23:47:33+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu/28/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/ros2bag', 'kind': 'raw_split', 'segments': 82, 'duration_s': 1229.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck28_lio_1011_233203/Log/fusion/1791733330818938302
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.25,
        0.13,
        1.66
      ],
      "drpy_deg": [
        -0.075,
        0.085,
        -0.032
      ]
    },
    "back_lidar": {
      "dt_cm": [
        6.62,
        3.8,
        -1.76
      ],
      "drpy_deg": [
        0.348,
        -0.017,
        -0.005
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -1.7,
        3.15,
        -8.74
      ],
      "drpy_deg": [
        -0.039,
        -0.128,
        -0.067
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
            -0.27651506850618773,
            -0.20821578874699911,
            0.10490463652096349
          ],
          "trans_cm": [
            1.0789790468643694,
            0.6566310819041327,
            4.709533939478521
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.37342249707885283,
            -0.7874703233058586,
            0.4239631601655954
          ],
          "trans_cm": [
            -2.4383519123394946,
            -2.873057954513018,
            1.4809266994222092
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            -0.6457236700583545,
            -0.38310355017738124,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            0.5055247196074156
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "/home/project/data/haibo/wuhu/28/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        11120,
        40
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              -0.07399153910875121,
              0.08511944256917356,
              -0.028448612063711416
            ],
            "trans_cm": [
              0.2458363228099701,
              0.1310360815570822,
              1.659249315043021
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 7,
              "matches": 172120,
              "vertical_matches": 33869,
              "median_before_cm": 3.19,
              "median_after_cm": 2.95,
              "vertical_before_cm": 3.16,
              "vertical_after_cm": 2.83,
              "rotation_deg": 0.116,
              "translation_cm": 1.7,
              "match_time_gap_s": 2.35,
              "pass2": {
                "ground_before_cm": 2.97,
                "ground_after_cm": 2.87,
                "ground_matches": 110721,
                "delta_rotvec_deg": [
                  -0.0336,
                  0.0119,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.57
                ],
                "iterations": 5
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.042010441720863524,
              0.12680018961114908,
              -0.06650761490344703
            ],
            "trans_cm": [
              -1.6977931641556125,
              3.150902833070563,
              -8.745136283772469
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 10,
              "matches": 275682,
              "vertical_matches": 91133,
              "median_before_cm": 5.05,
              "median_after_cm": 3.76,
              "vertical_before_cm": 4.52,
              "vertical_after_cm": 2.86,
              "rotation_deg": 0.149,
              "translation_cm": 9.4,
              "match_time_gap_s": 3.0,
              "pass2": {
                "ground_before_cm": 4.7,
                "ground_after_cm": 4.67,
                "ground_matches": 162791,
                "delta_rotvec_deg": [
                  0.0145,
                  -0.022,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -0.73
                ],
                "iterations": 6
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              -0.02174163638546741,
              -0.34750745623016077,
              -0.020601604110519096
            ],
            "trans_cm": [
              6.614208716768552,
              3.798617119814577,
              -1.7615132631530368
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 9,
              "matches": 49731,
              "vertical_matches": 6478,
              "median_before_cm": 4.49,
              "median_after_cm": 3.02,
              "vertical_before_cm": 4.91,
              "vertical_after_cm": 3.82,
              "rotation_deg": 0.349,
              "translation_cm": 7.8,
              "match_time_gap_s": 8.98,
              "pass2": {
                "ground_before_cm": 3.36,
                "ground_after_cm": 2.83,
                "ground_matches": 39345,
                "delta_rotvec_deg": [
                  0.0189,
                  -0.1359,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.53
                ],
                "iterations": 7
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck28_lio_1011_233203/Log/fusion/1791733330818938302",
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
        2.7419,
        5.7998,
        1.4424
      ],
      "rpy_deg": [
        0.238,
        2.427,
        -0.259
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1685,
        -0.3556,
        0.3308
      ],
      "rpy_deg": [
        -1.762,
        2.564,
        -90.71
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1141,
        5.5781,
        1.5485
      ],
      "rpy_deg": [
        -1.063,
        -0.219,
        178.582
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1426,
        7.8507,
        -0.1568
      ],
      "rpy_deg": [
        2.071,
        -3.4,
        89.82
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1951,
      7.7526,
      1.5925
    ],
    "rpy_deg": [
      -100.421,
      0.211,
      0.764
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
    "lidar": -0.0151,
    "lidar_frontend": -0.0026,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0151 + imu/ins clock lag +0.0125 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck28/v021_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck28_lio_1011_233203/Log/fusion/1791733330818938302
window 8.0..40.0 s, lidar_time_offset -0.0026
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:320, back_lidar:320, front_left_lidar:321, front_lidar:320
matching: viewing-side consistent (oriented normals)
reference front_lidar: 691383 world points
front_right_lidar: pass 1 matches 172162->172120 |res| median 3.19->2.95 cm; ground 3.10->2.84 (n=106507); vertical 3.16->2.83 (n=33869); match time gap median 2.3 s; correction rot deg (x,y,z)=(-0.040,+0.073,-0.028) trans cm=(+0.2,+0.1,+0.1); iters 7
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 2.97->2.87 cm (n=110721); tilt about x/y adjusted by (-0.034,+0.012) deg, z by +1.6 cm; iters 5
    final correction rot deg (x,y,z)=(-0.074,+0.085,-0.028) trans cm=(+0.2,+0.1,+1.7)  [quality: ok]
front_left_lidar: pass 1 matches 274857->275682 |res| median 5.05->3.76 cm; ground 5.56->4.51 (n=159407); vertical 4.52->2.86 (n=91133); match time gap median 3.0 s; correction rot deg (x,y,z)=(+0.027,+0.149,-0.067) trans cm=(-1.7,+3.2,-8.0); iters 10
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.70->4.67 cm (n=162791); tilt about x/y adjusted by (+0.015,-0.022) deg, z by -0.7 cm; iters 6
    final correction rot deg (x,y,z)=(+0.042,+0.127,-0.067) trans cm=(-1.7,+3.2,-8.7)  [quality: ok]
back_lidar: pass 1 matches 49422->49731 |res| median 4.49->3.02 cm; ground 4.36->2.77 (n=38228); vertical 4.91->3.82 (n=6478); match time gap median 9.0 s; correction rot deg (x,y,z)=(-0.041,-0.212,-0.021) trans cm=(+6.6,+3.8,-4.3); iters 9
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.36->2.83 cm (n=39345); tilt about x/y adjusted by (+0.019,-0.136) deg, z by +2.5 cm; iters 7
    final correction rot deg (x,y,z)=(-0.022,-0.348,-0.021) trans cm=(+6.6,+3.8,-1.8)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck28/v022_driven/calibration.json
```
