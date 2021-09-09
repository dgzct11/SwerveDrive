// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //ports
    public static final int left_front_direction_port = 0;
    public static final int left_front_thrust_port = 0;
    public static final int right_back_direction_port = 0;
    public static final int right_back_thrust_port = 0;
    public static final int left_back_direction_port = 0;
    public static final int left_back_thrust_port = 0;
    public static final int right_front_direction_port = 0;
    public static final int right_front_thrust_port = 0;
    public static final int xbox_port = 0;


    //robot spacific constants

    public static final double pos_units_per_degree = 0;
    public static final double left_right_wheel_distance = 0;
    public static final int front_back_wheel_distance = 0;
    public static final double[] center = {0,0};
    public static final double[] leftFrontCenter = {-left_right_wheel_distance/2,front_back_wheel_distance/2};
    public static final double[] leftBackCenter = {left_right_wheel_distance/2,front_back_wheel_distance/2};
    public static final double[] rightFrontCenter = {-left_right_wheel_distance/2,-front_back_wheel_distance/2};
    public static final double[] rightBackCenter = {left_right_wheel_distance/2,-front_back_wheel_distance/2};
    public static final double spin_angle = RobotContainer.to360(Math.toDegrees(Math.atan2(Constants.left_right_wheel_distance/2, Constants.front_back_wheel_distance/2)));
    

    // control modifiers
    public static final double spin_threshold = 0;
   
    public static final double rotation_dampener = 0;
   
   
    




}
