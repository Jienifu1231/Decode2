package org.firstinspires.ftc.teamcode.util;

import java.text.DecimalFormat;

public final class WayPoint {
    DecimalFormat df = new DecimalFormat("#.#####");

    public double x;
    public double y;
    public double heading;
    public double trans_speed;
    public double rot_speed;
    public double sensitivity;
    public double heading_sensitivity = 0.05;
    public double wait_time = 0;
    public boolean angle_mode = true;
    public double corner_sensitivity = 3;
    public double decelerateDist = 0.01;
    public boolean angleWait = false;

    public WayPoint(double x, double y, double heading, double trans_speed, double rot_speed, double sensitivity) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.trans_speed = trans_speed;
        this.rot_speed = rot_speed;
        this.sensitivity = sensitivity;
        this.wait_time = 0;
        this.angle_mode = true;
        this.decelerateDist = 0.01;
    }

    /*
    public WayPoint(double x, double y, double heading, double trans_speed, double rot_speed, double sensitivity, double wait_time) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.trans_speed = trans_speed;
        this.rot_speed = rot_speed;
        this.sensitivity = sensitivity;
        this.wait_time = wait_time;
        this.angle_mode = true;
        this.decelerateDist = 0;
    }
     */

    public WayPoint(double x, double y, double heading, double trans_speed, double rot_speed, double sensitivity, boolean angle_mode) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.trans_speed = trans_speed;
        this.rot_speed = rot_speed;
        this.sensitivity = sensitivity;
        this.wait_time = 0;
        this.angle_mode = angle_mode;
        this.decelerateDist = 0.01;
    }

    public WayPoint(double x, double y, double heading, double trans_speed, double rot_speed, double sensitivity, double decel_dist) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.trans_speed = trans_speed;
        this.rot_speed = rot_speed;
        this.sensitivity = sensitivity;
        this.wait_time = 0;
        this.angle_mode = true;
        this.decelerateDist = decel_dist;
    }

    public WayPoint(double x, double y, double heading, double trans_speed, double rot_speed, double sensitivity, boolean angle_wait, double decel_dist) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.trans_speed = trans_speed;
        this.rot_speed = rot_speed;
        this.sensitivity = sensitivity;
        this.wait_time = 0;
        this.angle_mode = true;
        this.decelerateDist = decel_dist;
        this.angleWait = angle_wait;
    }

    public WayPoint(double x, double y, double heading, double trans_speed, double rot_speed, double sensitivity, double wait_time, boolean angle_mode) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.trans_speed = trans_speed;
        this.rot_speed = rot_speed;
        this.sensitivity = sensitivity;
        this.wait_time = wait_time;
        this.angle_mode = angle_mode;
        this.decelerateDist = 0.01;
    }

    public WayPoint(double x, double y, double heading, double trans_speed, double rot_speed, double sensitivity, double decel_dist, double wait_time) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.trans_speed = trans_speed;
        this.rot_speed = rot_speed;
        this.sensitivity = sensitivity;
        this.wait_time = wait_time;
        this.angle_mode = true;
        this.decelerateDist = decel_dist;
    }

    public WayPoint(double x, double y, double heading, double trans_speed, double rot_speed, double sensitivity, double decel_dist, double wait_time, boolean angleWait) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.trans_speed = trans_speed;
        this.rot_speed = rot_speed;
        this.sensitivity = sensitivity;
        this.wait_time = wait_time;
        this.angle_mode = true;
        this.decelerateDist = decel_dist;
        this.angleWait = angleWait;
    }

    public double getX() { return this.x; }
    public double getY() { return this.y; }
    public double getHeading() { return this.heading; }
    public double getWaitTime() { return this.wait_time; }
    public double getSensitivity() { return this.sensitivity; }
    public double getTransSpeed() { return this.trans_speed; }
    public boolean getAngleMode() { return this.angle_mode; }
    public double getRotSpeed() { return this.rot_speed; }
    public double getHeadingSensitivity() { return this.heading_sensitivity; }
    public double getCornerSensitivity() { return this.corner_sensitivity; }
    public double getDecelerateDist(){ return this.decelerateDist; }
    public boolean getAngleWait(){ return this.angleWait; }

    public Pose2d toPose(){ return new Pose2d(this.x, this.y, this.heading); }
    public boolean hasWait() { return this.wait_time > 0; }

    public void setDecelerateDist(double new_decelerate){ this.decelerateDist = new_decelerate; }
    public void setHeadingSensitivity(double new_heading_sensitivity){ this.heading_sensitivity = new_heading_sensitivity; }
    public void setCornerSensitivity(double new_corner_sense){ this.corner_sensitivity = new_corner_sense; }
    public void waitAngle(){ this.angleWait = true; }

    public String toString() {
        return "(x: " + df.format(this.x) + ", y: " + df.format(this.y) + ", heading: " + df.format(this.heading) +
                ", translation speed: " + trans_speed + ", rotation speed: " + rot_speed + ", sensitivity: " + sensitivity + ")";
    }

    public WayPoint clone(){ return new WayPoint(this.x, this.y, this.heading, this.trans_speed, this.rot_speed, this.sensitivity); }
}
