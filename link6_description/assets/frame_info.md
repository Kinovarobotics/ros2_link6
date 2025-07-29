|    | name         | link                 | child                |       x |        y |        z |   min |   max |   vel |   torque |
|---:|:-------------|:---------------------|:---------------------|--------:|---------:|---------:|------:|------:|------:|---------:|
|  0 | joint_1      | base_link            | shoulder_link        |   0     |    0     |   53     |  -360 |   360 |   180 |      210 |
|  1 | joint_2      | shoulder_link        | arm_link             | 110.24  |  -69.257 | -137.5   |  -360 |   360 |   180 |      210 |
|  2 | joint_3      | arm_link             | forearm_link         |   0     |  485     |    0     |  -360 |   360 |   180 |      210 |
|  3 | joint_4      | forearm_link         | lower_wrist_link     |   0     | -152.16  |  -91.704 |  -360 |   360 |   360 |      100 |
|  4 | joint_5      | lower_wrist_link     | upper_wrist_link     |   0     |  -62.957 | -222.75  |  -360 |   360 |   360 |      100 |
|  5 | joint_6      | upper_wrist_link     | wrist_interface_link |  87.028 |   86     |  -76.922 |  -360 |   360 |   360 |      100 |
|  6 | end_effector | wrist_interface_link | end_effector_link    |   0     |    0     |  -92     |     0 |     0 |     0 |        0 |