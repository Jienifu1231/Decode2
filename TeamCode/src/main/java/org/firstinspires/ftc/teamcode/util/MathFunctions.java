package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;

public class MathFunctions {

    public static double angleWrap(double angle){
        while (angle <= -Math.PI){
            angle += 2*Math.PI;
        }
        while (angle > Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }

    public static double linePointDistance(Pose2d line1, Pose2d line2, Pose2d point){
        double a = line1.y - line2.y;
        double b = line2.x - line1.x;
        double c = line1.x*line2.y - line1.y*line2.x;

        double num = Math.abs(a * point.x + b * point.y + c);
        double denom = Math.hypot(a, b);

        return num/denom;
    }

    public static ArrayList<Pose2d> lineCircleIntersection(Pose2d center, double radius, Pose2d line1, Pose2d line2){

        ArrayList<Pose2d> allPoints = new ArrayList<>();

        double vx = line2.x - line1.x;
        double vy = line2.y - line1.y;
        double disx = line1.x - center.x;
        double disy = line1.y - center.y;

        double a = vx*vx + vy*vy;
        double b = 2*vx*disx + 2*vy*disy;
        double c = disx*disx + disy*disy - radius*radius;

        double discriminant = b*b - 4*a*c;

        if(discriminant == 0){
            double t = -b / (2*a);

            allPoints.add(new Pose2d(vx*t + line1.x, vy*t + line1.y, 0));
        } else if (discriminant > 0){
            double t1 = (-b + Math.sqrt(discriminant))/(2*a);
            double t2 = (-b - Math.sqrt(discriminant))/(2*a);

            allPoints.add(new Pose2d(vx*t1 + line1.x, vy*t1 + line1.y, 0));
            allPoints.add(new Pose2d(vx*t2 + line1.x, vy*t2 + line1.y, 0));
        }

        return allPoints;
    }

}