# zongyang/truck07/v004_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: zongyang/truck07/v003_ground
- lineage: zongyang/truck07/v001_import -> zongyang/truck07/v002_axle -> zongyang/truck07/v003_ground -> zongyang/truck07/v004_driven
- created: 2026-10-08T17:32:21+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/zongyang/07/20261007/ros2bag
- dataset: {'path': '/home/project/data/haibo/zongyang/07/20261007/ros2bag', 'kind': 'raw_split', 'segments': 12, 'duration_s': 359.99, 'has_camera': False}
- run: /home/project/MY-LIVO/results/calib_truck07_lio_1008_172457/Log/fusion/1791451791012357480
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -4.07,
        -0.51,
        -2.69
      ],
      "drpy_deg": [
        0.06,
        0.089,
        -0.031
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.53,
        0.29,
        4.5
      ],
      "drpy_deg": [
        0.034,
        0.29,
        0.051
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.89,
        2.23,
        -1.07
      ],
      "drpy_deg": [
        -0.079,
        0.053,
        -0.015
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
            0.23576689089201694,
            0.0405002967407208,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -0.0926817027854639
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            0.011434317795864407,
            -0.22578994009563466,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.435170448756437
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            -0.40329563988596606,
            0.49246613539344464,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -1.8276398226083184
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "/home/project/data/haibo/zongyang/07/20261007/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        2750,
        40
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              0.0605089471089759,
              0.08878456686448394,
              -0.03221956100065342
            ],
            "trans_cm": [
              -4.068013385378104,
              -0.5085263352698081,
              -2.6896749115321366
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "only 430 vertical planar matches (yaw/x/y weakly observed)"
              ],
              "converged": true,
              "iterations": 19,
              "matches": 91985,
              "vertical_matches": 430,
              "median_before_cm": 4.06,
              "median_after_cm": 3.92,
              "vertical_before_cm": 5.08,
              "vertical_after_cm": 4.85,
              "rotation_deg": 0.112,
              "translation_cm": 4.9,
              "match_time_gap_s": 4.28,
              "pass2": {
                "ground_before_cm": 3.93,
                "ground_after_cm": 3.97,
                "ground_matches": 85083,
                "delta_rotvec_deg": [
                  0.0272,
                  0.0269,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.39
                ],
                "iterations": 11
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.07812255429646854,
              -0.05370131507570841,
              -0.0152547318907832
            ],
            "trans_cm": [
              0.8909184511282043,
              2.232284891734589,
              -1.065320491638982
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "not converged in 25 iterations",
                "only 387 vertical planar matches (yaw/x/y weakly observed)"
              ],
              "converged": false,
              "iterations": 25,
              "matches": 58113,
              "vertical_matches": 387,
              "median_before_cm": 3.82,
              "median_after_cm": 3.27,
              "vertical_before_cm": 5.19,
              "vertical_after_cm": 4.9,
              "rotation_deg": 0.096,
              "translation_cm": 2.6,
              "match_time_gap_s": 3.22,
              "pass2": {
                "ground_before_cm": 3.22,
                "ground_after_cm": 3.22,
                "ground_matches": 55060,
                "delta_rotvec_deg": [
                  0.0134,
                  0.0163,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  -1.01
                ],
                "iterations": 11
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.2896228737270804,
              -0.03552328685347436,
              0.051038796177638956
            ],
            "trans_cm": [
              0.5263348682758049,
              0.2899925472251973,
              4.4951784557996834
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "only 4 vertical planar matches (yaw/x/y weakly observed)"
              ],
              "converged": true,
              "iterations": 18,
              "matches": 9730,
              "vertical_matches": 4,
              "median_before_cm": 4.65,
              "median_after_cm": 3.44,
              "vertical_before_cm": 9.01,
              "vertical_after_cm": 3.17,
              "rotation_deg": 0.296,
              "translation_cm": 4.5,
              "match_time_gap_s": 15.93,
              "pass2": {
                "ground_before_cm": 3.88,
                "ground_after_cm": 3.52,
                "ground_matches": 8491,
                "delta_rotvec_deg": [
                  0.0579,
                  0.0652,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  0.48
                ],
                "iterations": 9
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck07_lio_1008_172457/Log/fusion/1791451791012357480",
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
        2.7248,
        5.1794,
        1.7214
      ],
      "rpy_deg": [
        -2.083,
        1.115,
        -0.364
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1634,
        -0.8403,
        0.7212
      ],
      "rpy_deg": [
        0.467,
        0.986,
        -90.348
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.2193,
        4.9955,
        1.7915
      ],
      "rpy_deg": [
        2.327,
        0.217,
        179.329
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1092,
        7.2631,
        2.5109
      ],
      "rpy_deg": [
        0.485,
        2.809,
        89.258
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1403,
      7.2764,
      2.4087
    ],
    "rpy_deg": [
      -90.448,
      1.632,
      0.97
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      621.1,
      900.65,
      958.95,
      537.17
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
            621.1,
            900.65,
            958.95,
            537.17
          ],
          "rms_px": 0.309,
          "circle_centre_rear_m": [
            -0.01,
            7.536
          ],
          "ground_z_rear_m": -1.263
        },
        "square": {
          "intrinsics": [
            621.33,
            621.33,
            1050.56,
            537.09
          ],
          "rms_px": 2.404,
          "circle_centre_rear_m": [
            -0.942,
            7.544
          ],
          "ground_z_rear_m": -2.916
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.309,
      "camera_height_m": 3.672,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -62.19,
        "fy_pct": -51.46,
        "cx_px": -12.1,
        "cy_px": 2.9
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.309 px)"
  },
  "time_offsets": {
    "lidar": -0.0041,
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
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck07/v003_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck07_lio_1008_172457/Log/fusion/1791451791012357480
window 8.0..40.0 s, lidar_time_offset -0.125
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:321, back_lidar:321, front_left_lidar:321, front_lidar:320
matching: viewing-side consistent (oriented normals)
reference front_lidar: 629204 world points
front_right_lidar: pass 1 matches 91768->91985 |res| median 4.06->3.92 cm; ground 4.04->3.91 (n=90983); vertical 5.08->4.85 (n=430); match time gap median 4.3 s; correction rot deg (x,y,z)=(+0.033,+0.062,-0.032) trans cm=(-4.1,-0.5,-1.3); iters 19
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.93->3.97 cm (n=85083); tilt about x/y adjusted by (+0.027,+0.027) deg, z by -1.4 cm; iters 11
    final correction rot deg (x,y,z)=(+0.061,+0.089,-0.032) trans cm=(-4.1,-0.5,-2.7)  [quality: warn; only 430 vertical planar matches (yaw/x/y weakly observed)]
front_left_lidar: pass 1 matches 58631->58113 |res| median 3.82->3.27 cm; ground 3.80->3.25 (n=57393); vertical 5.19->4.90 (n=387); match time gap median 3.2 s; correction rot deg (x,y,z)=(+0.065,-0.070,-0.015) trans cm=(+0.9,+2.2,-0.1); iters 25
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.22->3.22 cm (n=55060); tilt about x/y adjusted by (+0.013,+0.016) deg, z by -1.0 cm; iters 11
    final correction rot deg (x,y,z)=(+0.078,-0.054,-0.015) trans cm=(+0.9,+2.2,-1.1)  [quality: warn; not converged in 25 iterations; only 387 vertical planar matches (yaw/x/y weakly observed)]
back_lidar: pass 1 matches 9502->9730 |res| median 4.65->3.44 cm; ground 4.62->3.41 (n=9610); vertical 9.01->3.17 (n=4); match time gap median 15.9 s; correction rot deg (x,y,z)=(+0.232,-0.101,+0.051) trans cm=(+0.5,+0.3,+4.0); iters 18
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.88->3.52 cm (n=8491); tilt about x/y adjusted by (+0.058,+0.065) deg, z by +0.5 cm; iters 9
    final correction rot deg (x,y,z)=(+0.290,-0.036,+0.051) trans cm=(+0.5,+0.3,+4.5)  [quality: warn; only 4 vertical planar matches (yaw/x/y weakly observed)]
quality: warn  (front_right_lidar: only 430 vertical planar matches (yaw/x/y weakly observed); front_left_lidar: not converged in 25 iterations, only 387 vertical planar matches (yaw/x/y weakly observed); back_lidar: only 4 vertical planar matches (yaw/x/y weakly observed))
written /home/project/MY-LIVO/config/carlibration/zongyang/truck07/v004_driven/calibration.json
```
