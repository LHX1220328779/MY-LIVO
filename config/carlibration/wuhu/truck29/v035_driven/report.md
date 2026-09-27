# wuhu/truck29/v035_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29/v033_ground
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v012_ground -> wuhu/truck29/v013_driven -> wuhu/truck29/v014_camtime -> wuhu/truck29/v018_camtime -> wuhu/truck29/v019_camlidar -> wuhu/truck29/v022_camloop -> wuhu/truck29/v023_camtime -> wuhu/truck29/v024_camlidar -> wuhu/truck29/v027_camloop -> wuhu/truck29/v028_camloop -> wuhu/truck29/v029_camtime -> wuhu/truck29/v030_camlidar -> wuhu/truck29/v031_camloop -> wuhu/truck29/v032_camloop -> wuhu/truck29/v033_ground -> wuhu/truck29/v035_driven
- created: 2026-09-23T21:03:35+08:00
- note: driven two-pass: native 6-DoF + RTK-optimized-pose tilt/z for cross-time LiDARs; quality gate

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: results/mcpeff_it_lio48/Log/fusion/1790155569020849680
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.46,
        0.45,
        -0.9
      ],
      "drpy_deg": [
        0.044,
        -0.103,
        0.006
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.71,
        -0.61,
        -4.09
      ],
      "drpy_deg": [
        -0.123,
        0.133,
        0.026
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        1.08,
        0.21,
        0.4
      ],
      "drpy_deg": [
        -0.059,
        -0.034,
        0.013
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
            -0.06279989033418391,
            0.08102088773519586,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.7613630321548095
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.06810100620629238,
            -0.08768849328270907,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            0.5435033678453096
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            -0.024074345309057624,
            -0.16682449694268928,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            8.33551275752048
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
              0.04196031887500078,
              -0.10328628903168084,
              0.009035668135969677
            ],
            "trans_cm": [
              0.460771062300374,
              0.44423406092574297,
              -0.8989334004175826
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 15,
              "matches": 206048,
              "vertical_matches": 18098,
              "median_before_cm": 3.44,
              "median_after_cm": 3.14,
              "vertical_before_cm": 3.93,
              "vertical_after_cm": 3.83,
              "rotation_deg": 0.112,
              "translation_cm": 1.1,
              "match_time_gap_s": 3.25,
              "pass2": {
                "ground_before_cm": 3.46,
                "ground_after_cm": 3.08,
                "ground_matches": 166096,
                "delta_rotvec_deg": [
                  -0.045,
                  -0.0243,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.89
                ],
                "iterations": 9
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.059392146764749305,
              0.033664633765365916,
              0.012485057318525434
            ],
            "trans_cm": [
              1.077237302428994,
              0.2133954847083079,
              0.40538540757357416
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 15,
              "matches": 233002,
              "vertical_matches": 26944,
              "median_before_cm": 5.25,
              "median_after_cm": 4.97,
              "vertical_before_cm": 3.85,
              "vertical_after_cm": 3.82,
              "rotation_deg": 0.069,
              "translation_cm": 1.2,
              "match_time_gap_s": 5.58,
              "pass2": {
                "ground_before_cm": 5.12,
                "ground_after_cm": 5.05,
                "ground_matches": 149974,
                "delta_rotvec_deg": [
                  -0.0208,
                  -0.0471,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.37
                ],
                "iterations": 11
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.13522282785674733,
              0.1204519606270423,
              0.027776560725180994
            ],
            "trans_cm": [
              0.7056156832664023,
              -0.606543362073177,
              -4.091173075558489
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 14,
              "matches": 47725,
              "vertical_matches": 3338,
              "median_before_cm": 8.81,
              "median_after_cm": 3.42,
              "vertical_before_cm": 4.21,
              "vertical_after_cm": 4.03,
              "rotation_deg": 0.183,
              "translation_cm": 4.2,
              "match_time_gap_s": 10.69,
              "pass2": {
                "ground_before_cm": 4.41,
                "ground_after_cm": 3.3,
                "ground_matches": 37746,
                "delta_rotvec_deg": [
                  0.0805,
                  -0.078,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  5.47
                ],
                "iterations": 9
              }
            }
          }
        },
        "run": "results/mcpeff_it_lio48/Log/fusion/1790155569020849680",
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
        2.8396,
        5.7735,
        1.8179
      ],
      "rpy_deg": [
        -0.823,
        -3.153,
        -1.206
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0147,
        -0.5353,
        0.6511
      ],
      "rpy_deg": [
        -0.935,
        0.906,
        -91.146
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0222,
        5.547,
        1.9598
      ],
      "rpy_deg": [
        -3.575,
        -0.606,
        178.839
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.033,
        7.793,
        0.051
      ],
      "rpy_deg": [
        1.3,
        -1.67,
        90.16
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0742,
      7.7914,
      1.7519
    ],
    "rpy_deg": [
      -96.347,
      1.471,
      0.557
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
    "lidar": -0.125,
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v033_ground/calibration.json
run=results/mcpeff_it_lio48/Log/fusion/1790155569020849680
window 8.0..40.0 s, lidar_time_offset -0.125
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
reference front_lidar: 515009 world points
front_right_lidar: pass 1 matches 206544->206048 |res| median 3.44->3.14 cm; ground 3.34->3.02 (n=169335); vertical 3.93->3.83 (n=18098); match time gap median 3.2 s; correction rot deg (x,y,z)=(+0.087,-0.079,+0.009) trans cm=(+0.5,+0.4,-2.8); iters 15
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 3.46->3.08 cm (n=166096); tilt about x/y adjusted by (-0.045,-0.024) deg, z by +1.9 cm; iters 9
    final correction rot deg (x,y,z)=(+0.042,-0.103,+0.009) trans cm=(+0.5,+0.4,-0.9)  [quality: ok]
front_left_lidar: pass 1 matches 232952->233002 |res| median 5.25->4.97 cm; ground 5.28->5.02 (n=155200); vertical 3.85->3.82 (n=26944); match time gap median 5.6 s; correction rot deg (x,y,z)=(+0.080,+0.081,+0.012) trans cm=(+1.1,+0.2,-1.0); iters 15
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 5.12->5.05 cm (n=149974); tilt about x/y adjusted by (-0.021,-0.047) deg, z by +1.4 cm; iters 11
    final correction rot deg (x,y,z)=(+0.059,+0.034,+0.012) trans cm=(+1.1,+0.2,+0.4)  [quality: ok]
back_lidar: pass 1 matches 47460->47725 |res| median 8.81->3.42 cm; ground 10.17->3.25 (n=38635); vertical 4.21->4.03 (n=3338); match time gap median 10.7 s; correction rot deg (x,y,z)=(+0.055,+0.198,+0.028) trans cm=(+0.7,-0.6,-9.6); iters 14
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.41->3.30 cm (n=37746); tilt about x/y adjusted by (+0.081,-0.078) deg, z by +5.5 cm; iters 9
    final correction rot deg (x,y,z)=(+0.135,+0.120,+0.028) trans cm=(+0.7,-0.6,-4.1)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v035_driven/calibration.json
```
