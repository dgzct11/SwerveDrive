// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;

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
    public static final int xbox_port = 0;
    public static final Port mxp_port = Port.kMXP;
    
    public static final int left_front_direction_port = 7;
    public static final int left_front_thrust_port = 8;
    public static final int right_back_direction_port = 3;
    public static final int right_back_thrust_port = 4;
    public static final int left_back_direction_port = 1;
    public static final int left_back_thrust_port = 2;
    public static final int right_front_direction_port = 5;
    public static final int right_front_thrust_port = 6;
   
    

    //xbox bindings
    
    public static final int left_x_axis = 0;
    public static final int left_y_axis = 1;
    public static final int right_x_axis = 4;
    public static final int right_y_axis = 5;

    public static final int a_button_num = 1;
    public static final int b_button_num = 2;
    public static final int x_button_num = 3;
    public static final int y_button_num = 4;

    public static final int left_pad_num = 270;
    public static final int right_pad_num = 90;
    public static final int up_pad_num = 0;
    public static final int down_pad_num = 180;
    public static int start_button_num = 8;
    public static int back_button_num = 7;  
    public static final int lb_button_num = 5;
    public static final int rb_button_num = 6;
    
    //robot spacific constants

    public static final double pos_units_per_degree = 72.857777778;
    public static final double pos_units_per_degree_rf = 72.8171171171;
   
    public static final double pos_units_per_rotation = 16410;
    public static final double pos_units_per_meter = pos_units_per_degree*3.13297132071;
    public static final double left_right_wheel_distance = 0.581;
    public static final double max_pos_units = 6178;
    public static final double front_back_wheel_distance = 0.581;
    public static final double[] center = {0,0};
    public static final double[] leftFrontCenter = {-left_right_wheel_distance/2,front_back_wheel_distance/2};
    public static final double[] leftBackCenter = {left_right_wheel_distance/2,front_back_wheel_distance/2};
    public static final double[] rightFrontCenter = {-left_right_wheel_distance/2,-front_back_wheel_distance/2};
    public static final double[] rightBackCenter = {left_right_wheel_distance/2,-front_back_wheel_distance/2};
    public static final double distance_wheel_center = RobotContainer.distance(center, leftFrontCenter);
    public static final double talon_velocity_per_ms = 0;
  

    // control modifiers
    public static final double spin_threshold = 0.5;
    public static final double rotate_dampaner = 1;
    
    public static double max_motor_percent = 0.5;
    
    // state variables
    
    public static int drive_mode = 0;
    public static boolean in_auto = false;
    
   
   
    




}
