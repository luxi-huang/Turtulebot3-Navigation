# Homework 2
"Luxi Huang, MSR, ME495 HW2"

## overview:
This README answered questions on Homework2

### Robotation:

| | Fv|Av| ET|EY|Ex|OT(rad)|OX(cm)|OY(cm)|FT(degree)|FX(cm)|FY(cm)|GT(degree)|GX(cm)|GY(cm)|DT(degree/distance)|DX(cm/distance)|DY(cm/distance)|
|--| --|--|---|--|--     |--    |---   | --       |--    |---   |--        |--    |--    |--        |--    |--    |--|
|cw  |  1 |  -2.84 | 0  |0   |0   |1.1826522|-0.66507|-0.171022|-0.328272|0|0|2.443460953   | -1.1  | -1.8  |-0.06304043764   | 0.0217465 |0.0814489 |
|cw|0.8 | -2.765486  |  0 |0   |  0 |  3.1297071 | -0.128703  | -0.370157  | 0.317733  | 0  | 0  |3.054326191 |-0.2 | -0.1  | 0.00376904545  | 0.00356485  |-0.01350785 |  
|ccw   | 1  |2.84   |0   | 0  | 0  |-3.1434385   |  0.0382658 | -0.0580493  |  -1.3592756 | 0  |0   | 2.094395102  | -0.3  |-0.5   | -0.2618916801  | 0.01691329  | 0.022097535  |
| ccw  |0.8   |  2.765486 | 0  |0   |0   | -2.1496343  |  0.2342 | -0.2589  | -0.2994052  | 0  |0   | -2.967059728  |0.8   | 0.5  |  0.04087127142 |  -0.02829 | -0.037945  |  


- Encorders improve the odometry over estimating the pose based on the inputs
  - since the feedforward doesn't including sensor values, therefore the feedforward model didn't consider factors such as friction.


- On the clockwise direction, moving slower improve the odometry, but in the counterclockwise direction, moving slower didn't improve much on odometry.

### translation
| | Fv|Av| ET|EY|Ex|OT(rad)|OX(cm)|OY(cm)|FT(degree)|FX(cm)|FY(cm)|GT(degree)|GX(cm)|GY(cm)|DT(degree/distance)|DX(cm/distance)|DY(cm/distance)|
|--| --|--|---|--|--     |--    |---   | --       |--    |---   |--        |--    |--    |--        |--    |--    |--|
| FWD  |  0.8 |0.176   |0   |2   |0   |0.5416186   | 320.05  | 149.584  | 0  | 199.979  |   0 | 0.0174533  |  197.7 | 34  | 0.026208265  | 6.1175  |5.7792|
|FWD | 1  |   0.22| 0  |2   |0   |  0.0840265 | 162.33  |  -4.1058 |  0 | 200  | 0  | 0.1396263402  |  153.8 | 12  | -0.002779992008  | 0.4265  | -0.80529 |  
|  BWD | 0.8  |0.176   | 0  | 2 | 0  | 0.3235244  | -176.93  |  5.8431 |  0 | -200  | 0  | 0.06981317008  | -167.3  |  13 | 0.0126855615  | -0.4815  | -0.357845  |  
|BWD   |1   |0.22   |0   |2   |0   | 0.1159729  | -185.89  | 9.9191  | 0  |  -200.04 | 0  |  0.1047197551 | -180.5  | 28  |  0.000562657244 | -0.2695  | -0.904045  |

- slower speed didn't improve the odometry.

### waypoints

- rviz_waypoints video

  https://github.com/ME495-Navigation/main-assignment-luxi-huang/blob/master/rviz_waypoints.mkv

- world_waypoints video

  https://github.com/ME495-Navigation/main-assignment-luxi-huang/blob/master/world_waypoints.mov
