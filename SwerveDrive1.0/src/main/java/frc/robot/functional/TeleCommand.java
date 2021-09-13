package frc.robot.functional;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class TeleCommand extends CommandBase{
   
      // Called when the command is initially scheduled.
     
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        if(!Constants.in_auto){
            run();
        }
      }

      public void run(){

      }
    
      // Called once the command ends or is interrupted.
      
    
      // Returns true when the command should end.
     
}
