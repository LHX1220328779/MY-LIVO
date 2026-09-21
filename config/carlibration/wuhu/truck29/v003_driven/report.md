# wuhu/truck29/v003_driven

- stage: driven
- parent: wuhu/truck29/v002_ground
- created: 2026-09-21T18:00:03+08:00
- note: stage2 driven 6-DoF (tool chain test)

## inputs

- bag: results/prepared/wuhu_livo
- run: results/wuhu_rtk_98_attitude_motion_gate/Log/fusion/1789900798110589369
- window: [8.0, 40.0]

## metrics

```
{
  "refinement": {
    "refinement": {
      "reference": "front_lidar",
      "corrections": {
        "front_right_lidar": {
          "rotvec_deg": [
            -0.11125897970222925,
            0.3682551849600907,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.115475084573922
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            0.11457588306250073,
            -1.311793031071289,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -1.0242244162236145
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            0.2273957639016077,
            -0.13440698910248985,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.5640244099732254
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "results/prepared/wuhu_livo",
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
              0.06377972820091576,
              -0.08135574838903445,
              0.018343247761372212
            ],
            "trans_cm": [
              1.7288721553684092,
              6.525082741120625,
              -1.7478422951261459
            ],
            "about": "lidar position in rear-axle frame"
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.08101641824106609,
              0.082320537349152,
              0.07982521472540408
            ],
            "trans_cm": [
              0.513715351075462,
              7.251009944562316,
              -1.2348865904857778
            ],
            "about": "lidar position in rear-axle frame"
          },
          "back_lidar": {
            "rotvec_deg": [
              -0.010742856918066934,
              0.14470689430551056,
              -0.009286406817160092
            ],
            "trans_cm": [
              -0.453654701115855,
              -3.0235310495887666,
              -10.095534177945067
            ],
            "about": "lidar position in rear-axle frame"
          }
        },
        "run": "results/wuhu_rtk_98_attitude_motion_gate/Log/fusion/1789900798110589369",
        "window": [
          8.0,
          40.0
        ],
        "prior": [
          0.5,
          0.1
        ]
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
        2.8296,
        5.7648,
        1.8091
      ],
      "rpy_deg": [
        -0.804,
        -3.132,
        -1.22
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0042,
        -0.5528,
        0.5995
      ],
      "rpy_deg": [
        -0.988,
        0.76,
        -91.154
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0469,
        5.536,
        1.9515
      ],
      "rpy_deg": [
        -3.574,
        -0.665,
        178.807
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
      -96.438,
      1.487,
      0.699
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
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v002_ground/calibration.json
run=results/wuhu_rtk_98_attitude_motion_gate/Log/fusion/1789900798110589369
window 8.0..40.0 s, lidar_time_offset -0.125
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
reference front_lidar: 516636 world points
front_right_lidar: matches 204955->204768 |res| median 3.55->3.09 cm; ground 3.37->2.94 (n=168626); vertical 5.06->3.93 (n=17329); correction rot deg (x,y,z)=(+0.064,-0.081,+0.018) trans cm=(+1.7,+6.5,-1.7); iters 17
front_left_lidar: matches 231641->231814 |res| median 5.27->5.00 cm; ground 5.20->5.00 (n=152777); vertical 4.63->4.01 (n=27352); correction rot deg (x,y,z)=(+0.081,+0.082,+0.080) trans cm=(+0.5,+7.3,-1.2); iters 15
back_lidar: matches 47678->47650 |res| median 8.28->3.77 cm; ground 9.21->3.59 (n=38494); vertical 5.19->4.59 (n=3013); correction rot deg (x,y,z)=(-0.011,+0.145,-0.009) trans cm=(-0.5,-3.0,-10.1); iters 16
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v003_driven/calibration.json
```
