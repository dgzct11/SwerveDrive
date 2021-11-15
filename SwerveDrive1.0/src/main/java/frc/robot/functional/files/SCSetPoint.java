package frc.robot.functional.files;


import java.util.ArrayList;

public class SCSetPoint {
    public double startDistance;
    public double endDistance;
    public String subsystemIdentifier;
    public ArrayList<Double> inputs = new ArrayList<Double>();
    public SCSetPoint(double sd, double ed, String id){
        startDistance = sd;
        endDistance = ed;
        subsystemIdentifier = id;
    }

    public String toString(){
        String inputString = "";
        for(Double input: inputs)
            inputString += input + ",";
        return String.format("%s:%f,%f:%s\n",subsystemIdentifier, startDistance, endDistance, inputString);
    }
}
