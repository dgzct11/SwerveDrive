package frc.robot.functional.files;

import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;
public class FileReader {
    private ArrayList<double[]> points, velocity;
    private ArrayList<Double> distances, angles;
    private double totalDistance;

    public FileReader() {
        points = new ArrayList<double[]>();
        velocity = new ArrayList<double[]>();
        distances = new ArrayList<Double>();
        angles = new ArrayList<Double>();
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
            totalDistance = Double.parseDouble(sd.nextLine());
            for (int i = 0; i < points.size() - 1; i++) {
                distances.add(Math.sqrt(Math.pow(points.get(i)[0] - points.get(i+1)[0], 2) + Math.pow(points.get(i)[1] - points.get(i+1)[1], 2)));
                angles.add(Math.atan((points.get(i+1)[1] - points.get(i)[1]) - (points.get(i+1)[0] - points.get(i)[0])));
            }
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }
    
    public ArrayList<double[]> getPoints() {
        return points;
    }
    public ArrayList<double[]> getVelocity() {
        return velocity;
    }
    public double getDistance() {
        return totalDistance;
    }
    public ArrayList<Double> getDistances() {
        return distances;
    }

    public ArrayList<Double> getAngles() {
        return angles;
    }
}
