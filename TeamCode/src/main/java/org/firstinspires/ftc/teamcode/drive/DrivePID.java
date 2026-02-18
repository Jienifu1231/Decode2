package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Config
public class DrivePID {

    public static double strafe_kP = 0.15; //0.05//y
    public static double strafe_kI = 0.4;//0.3
    public static double strafe_kD = 0;//0.0012d

    public static double forward_kP = 0.04;//0.01//x
    public static double forward_kI = 0.3;//0.6
    public static double forward_kD = 0;//0.003

    double x_kP = forward_kP; //0.095
    double x_kI = forward_kI; //1



    double x_kD = forward_kD;

    double y_kP = strafe_kP; //0.035
    double y_kI = strafe_kI; //0.4
    double y_kD = strafe_kD;

    public static double r_kP = 1.3;//1.5
    public static double r_kI = 0;//0.3
    public static double r_kD = 0;//0.1

    public double x_norm_vel = 10;
    public double y_norm_vel = 10;
    public double r_norm_vel = 2;//2

    public static double x_max = 0.6;
    public static double y_max = 0.6;
    public static double r_max = 0.7;

    double xy_sense = 1;
    double r_sense = 0.05;

    public PID x_pid, y_pid, r_pid;

    public double x, y, r;
    public double rotatedX, rotatedY;

    public boolean pos_reached = false;

    public Pose2d out_power;

    public Pose2d last_target;
    public boolean new_target = false;

    public DrivePID(){
        x_pid = new PID(x_kP, x_kI, x_kD, xy_sense, 10, x_norm_vel, x_max, false);
        y_pid = new PID(y_kP, y_kI, y_kD, xy_sense, 10, y_norm_vel, y_max, false);
        r_pid = new PID(r_kP, r_kI, r_kD, r_sense, 10, r_norm_vel, r_max, true);
    }

    public void setMaxPower(Pose2d newPower){
        x_pid.setMaxPower(newPower.getX());
        y_pid.setMaxPower(newPower.getY());
        r_pid.setMaxPower(newPower.getHeading());
    }

    public void setSensitivity(Pose2d newSensitivity){
        x_pid.setSensitivity(newSensitivity.getX());
        y_pid.setSensitivity(newSensitivity.getY());
        r_pid.setSensitivity(newSensitivity.getHeading());
    }

    public void update(Pose2d target, Pose2d position){

        x_kP = Math.abs(forward_kP * Math.cos(position.getHeading())) + Math.abs(strafe_kP * Math.sin(position.getHeading()));
        x_kI = Math.abs(forward_kI * Math.cos(position.getHeading())) + Math.abs(strafe_kI * Math.sin(position.getHeading()));
        x_kD = Math.abs(forward_kD * Math.cos(position.getHeading())) + Math.abs(strafe_kD * Math.sin(position.getHeading()));

        y_kP = Math.abs(strafe_kP * Math.cos(position.getHeading())) + Math.abs(forward_kP * Math.sin(position.getHeading()));
        y_kI = Math.abs(strafe_kI * Math.cos(position.getHeading())) + Math.abs(forward_kI * Math.sin(position.getHeading()));
        y_kD = Math.abs(strafe_kD * Math.cos(position.getHeading())) + Math.abs(forward_kD * Math.sin(position.getHeading()));

        x_pid.setCoefficients(x_kP, x_kI, x_kD);
        y_pid.setCoefficients(y_kP, y_kI, y_kD);
        r_pid.setCoefficients(r_kP, r_kI, r_kD);

        pos_reached = x_pid.pos_reached && y_pid.pos_reached && r_pid.pos_reached;

        x = x_pid.update(target.getX(), position.getX());
        y = y_pid.update(target.getY(), position.getY());
        r = r_pid.update(target.getHeading(), position.getHeading());

        rotatedX = - x * Math.sin(position.getHeading()) + y * Math.cos(position.getHeading());
        rotatedY = - x * Math.cos(position.getHeading()) - y * Math.sin(position.getHeading());

        out_power = new Pose2d(-rotatedX, -rotatedY, -r);//update 1/18: x, y, -r
        //changed -y to y
        //changed x to -x strafing wrong direction
        //1 /20 all of them are negative RIGHT ONE

        new_target = !target.equals(last_target);

        last_target = target;
    }
}