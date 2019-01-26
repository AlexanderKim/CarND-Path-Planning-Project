# Introduction
The project is aimed to program a model driving car in a simulated environment within conditions outlined in the rubric:
- The car is able to drive at least 4.32 miles without incident..
- The car drives according to the speed limit.
- Max Acceleration and Jerk are not Exceeded.
- Car does not have collisions.
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lanes


Below is the screenshot of the car driven 6.52 miles without accident:
!(./highway.png)

# Implementation approach
The implementation is based on the materials provided by Udacity:
- `main.cpp` scaffold
- Project QA video

Aside of the code provided in the latter sources I have implemented the following:
 - Environment state parameters computation
 - Decision making

# Environment state parameters computation
Based on the sensor fusion data feed from the simulator I calculate the following parameters of environment:
- `check_car_lane`: Current lane the car in
- `too_close`: If there's a car in front is too close
-  `left` `right`: If there's a car left- or right- hand side respectively

```
for (int i = 0; i < sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size*0.02*check_speed);

              int check_car_lane = (int) ceil( d / 4 ) - 1;

              int max_distance = 40;

              too_close = (check_car_lane == lane) & ( (check_car_s > car_s) && ((check_car_s - car_s) < max_distance) ) | too_close;
              right = (check_car_lane - lane == 1) & ( ((car_s - max_distance) < check_car_s) && ((car_s + max_distance) > check_car_s) ) | right;
              left = (lane - check_car_lane == 1) & ( ((car_s - max_distance) < check_car_s) && ((car_s + max_distance) > check_car_s) ) | left;

            }
```

# Decision making
The decision making process is fairly simple:
- If there's no car in front accelerate but not more than speed limit. Else:
- Overtake from the left if there's no car on the left side. Else:
- Overtake from the right if there's no car on the right side. Else:
- Decelerate

```
double acceleration_step = .224;
            double max_speed = 49.5;
            if (too_close) {
              if (!left && lane > 0) {
                lane--;
              }
              else if (!right && lane < 2) {
                lane++;
              }
              else {
                ref_vel -= acceleration_step;
              }
            }
            else {

              if (ref_vel < max_speed) {
                ref_vel += acceleration_step;
              }
            }
```

# Path generation
Path is genereated in main.cpp lines 290-375.

The first two points of the path are initiated with the last points of previous path (or with current position if the car is just starting movement):

```
if (prev_size < 2) {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
              } else {
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];

                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
              }
```

The next three points are on the same lane the car is at though with further distance on it. This in conjunction with the latter 2 points is to set the spline extrapolation up:
```
 vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

```

The rest of the path is generated with respect to the set velocity as far as coordinate X is concerned.
Y is extraploated by spline function set with the points described above and X based on velocity:
```
tk::spline s;
            s.set_points(ptsx, ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
            double x_add_on = 0;

            for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
              double N = (target_dist/(.02*ref_vel/2.24));
              double x_point = x_add_on + (target_x) / N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
```