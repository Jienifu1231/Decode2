package org.firstinspires.ftc.teamcode.util;

public class OuttakeRegressions {
    double vel = 0;
    double a;
    double c;
    public double target_vel (double distance){
        vel = a * distance + c;
        //a linear regression that generates a velocity based on the position on the field;
        return vel;
    }

    double target_angle = 0;
    public double angle(double vel){

        //math here to generate the aimed angel based on the pos or vel
        return target_angle;
    }

    double hood = 0;
    public double hood_adjust(double vel, double distance){
        //regression here
        return hood;
    }

    double distance_from_goal = 0;
    public double distanceCalc (Pose2d goal, Pose2d curpos){
        //goal would be either red goal or blue goal -- fixed coordinate
        // get curpos from pinpoint
        double x_dif = goal.getX() - curpos.getX();
        double y_dif = goal.getY() - curpos.getY();
        distance_from_goal = Math.sqrt((x_dif * x_dif) + (y_dif * y_dif));
        //distance fomula

        return distance_from_goal;
    }
}
