package frc.robot.functional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class LimeLight {
  /** Creates a new LimeLight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  //horizontal angle offset to target
  NetworkTableEntry tx = table.getEntry("tx");
  //vertical angle offset to target
  NetworkTableEntry ty = table.getEntry("ty");

  //how much of the cameras area is taken up by the object
  //used for distance estimation. 
  NetworkTableEntry ta = table.getEntry("ta");

  NetworkTableEntry tv = table.getEntry("tv");

  double x;
  double y;
  double distance;
  boolean objectInView;
  double area;
  SwerveDrive sd;

  public LimeLight(SwerveDrive sd) {
    this.sd = sd;
    updateValues();
  }
   
  public void updateValues() {
    objectInView = (tv.getDouble(0.0) == 1);
    if (objectInView) {
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      area = ta.getDouble(0.0);
      
      SmartDashboard.putNumber("Lime Area", area);
      SmartDashboard.putNumber("Ball Distance", distance);
      SmartDashboard.putNumber("Ball X", x);
      SmartDashboard.putNumber("Ball Y", y);
    }
  }

  public void trackBall() {
    if(objectInView) {
      distance = Constants.k_length/Math.sqrt(area);
      sd.drive(0, distance, x);
    } else {
      sd.drive(0,0,0.3 * (x<0? -1:1));
    }
  }
}
