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

    //Ports

    public static final int xbox_p = 0;
    public static final Port mxp_port = Port.kMXP;
    
    public static final int bl_angle = 1;
    public static final int bl_speed = 2;
    public static final int br_angle = 3;
    public static final int br_speed = 4;
    public static final int fr_angle = 5;
    public static final int fr_speed = 6;
    public static final int fl_angle = 7;
    public static final int fl_speed = 8;
    

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

   /*thickness of wheels 41.9mm
    861.25 cm front back
    *61.4 cm
    *85.5 cm
    *9.8552 cm wheel diameter
    *205
    */

    public static final double side_over_radius = 0.84309923278;

    public static final double units_per_degree = 72.857777778;
    public static final double units_per_degree_rf = 72.8171171171;
    public static final double units_per_rotation = 16676.4;
    public static final double units_per_meter = units_per_rotation*(1/(Math.PI*0.098552));
    public static final double talon_velocity_per_ms = units_per_meter/10;
    public static final double distancePP = 0.00007558843;
  

    // control modifiers

    public static final double rotate_dampaner = 1;
    public static final double limeLightHeight = 0;
    
    public static double max_motor_percent = 0.5;
	public static double velocityMax = 0.5;
}
