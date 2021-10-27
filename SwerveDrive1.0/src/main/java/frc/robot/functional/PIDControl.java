package frc.robot.functional;



public class PIDControl {
    double kp;
    double ki;
    double kd;
    double setPoint;
    double previousError = 0;
    double previousTime = 0;
    public PIDControl(double p, double i, double d){
        kp = p;
        ki = i;
        kd = d;
    }
    public void setSetpoint(double s, double currentValue){
        setPoint = s;
        previousError = Math.abs(setPoint - currentValue);
        previousTime = System.currentTimeMillis()/1000;
    }
    public double getOutput(double currentValue){
        double error = Math.abs(setPoint - currentValue);
        double time = System.currentTimeMillis()/1000 - previousTime;
        
        double result =  kp*error + ki*error*time + kd*(error-previousError)/time;
        previousError = error;
        previousTime = time;
        return result;
        
    }
    public double getOutputFromError(double error){
        
        double time = System.currentTimeMillis()/1000 - previousTime;
        
        double result =  kp*error + ki*error*time + kd*(error-previousError)/time;
        previousError = error;
        previousTime = time;
        return result;
        
    }
}
