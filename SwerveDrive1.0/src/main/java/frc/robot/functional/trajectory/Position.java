package frc.robot.functional.trajectory;


/** Add your docs here. */
public class Position {
    public double x, y, angle;
    String str;
    public double[] point = new double[2];
    public Position(double _x, double _y, double _angle){
        angle = _angle;
        x = _x;
        y = _y;
        point[0] = x;
        point[1] = y;
    }
    public Position(double[] pos, double _angle){
        x = pos[0];
        y = pos[1];
        point = pos;
        angle = _angle;
    }

    public void addAngle(double a){
        angle = (angle+a+360)%360;
    }
    public void add(double dx, double dy){
        x += dx;
        y += dy;
    }
    public boolean equals(Position pos){
        return pos.x == x && pos.y == y && pos.angle == angle;
    }
    public String toString(){
        return String.format("\nPosition\n\tx: %f\n\ty: %f\n\tangle: %f\n", x, y, angle);
    }
}