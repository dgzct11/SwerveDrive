package frc.robot.functional.files;

import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;

public class FileReader {
    private ArrayList<double[]> points, velocity;
    private double totalDistance;

    public FileReader() {
        points = new ArrayList<double[]>();
        velocity = new ArrayList<double[]>();
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
}
