package frc.robot.functional.files;

import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;
public class FileReader {
    private ArrayList<double[]> points, velocity;
    private ArrayList<Double> distances;
    public ArrayList<SCSetPoint> setPoints = new ArrayList<SCSetPoint>();

    public FileReader() {
        points = new ArrayList<double[]>();
        velocity = new ArrayList<double[]>();
        distances = new ArrayList<Double>();
       
        try {
            Scanner sp = new Scanner(new File ("points.txt"));
            while (sp.hasNextLine()) {
                int index = sp.nextLine().indexOf(",");
                String point1 = sp.nextLine().substring(0, index);
                String point2 = sp.nextLine().substring(index + 1);
                points.add(new double[]{Double.parseDouble(point1), Double.parseDouble(point2)});
            }
            Scanner sv = new Scanner(new File("velocity.txt"));
            while (sv.hasNextLine()) {
                int index = sv.nextLine().indexOf(",");
                String point1 = sv.nextLine().substring(0, index);
                String point2 = sv.nextLine().substring(index + 1);
                velocity.add(new double[]{Double.parseDouble(point1), Double.parseDouble(point2)});
            }
            Scanner sd = new Scanner(new File("distance.txt"));
            while (sv.hasNextLine()) {
                distances.add(Double.parseDouble(sd.nextLine()));
            }
            Scanner ss = new Scanner(new File("Subsystems.txt"));
            while(ss.hasNextLine()){
                int indexFirst = ss.nextLine().indexOf(":");
                int indexLast = ss.nextLine().lastIndexOf(":");
                String subsytemId = ss.nextLine().substring(0, ss.nextLine().indexOf(":"));
                String[] startEndPoint = ss.nextLine().substring(indexFirst+1, indexLast).split(",");
                SCSetPoint point = new SCSetPoint(Double.parseDouble(startEndPoint[0]), Double.parseDouble(startEndPoint[1]), subsytemId);
                String[] values = ss.nextLine().substring(indexLast + 1, ss.nextLine().lastIndexOf(",")).split(",");
                for(String value: values){
                    point.inputs.add(Double.parseDouble(value));
                }
                setPoints.add(point);
            }
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }
    
    public double[][] getPoints() {
        double[][] result = new double[points.size()][points.get(0).length];
        for(int i = 0; i<points.size(); i++){
            result[i] = points.get(i);
        }
        return result;
    }
    public double[][] getVelocity() {
        double[][] result = new double[velocity.size()][velocity.get(0).length];
        for(int i = 0; i<velocity.size(); i++){
            result[i] = velocity.get(i);
        }
        return result;
    }

    public double[] getDistances() {
        double[] result = new double[distances.size()];
        for(int i = 0; i<distances.size(); i++){
            result[i] = distances.get(i);
        }
        return result;
    }
    public ArrayList<SCSetPoint>  getSetPoints(){
        return setPoints;
    }
}
