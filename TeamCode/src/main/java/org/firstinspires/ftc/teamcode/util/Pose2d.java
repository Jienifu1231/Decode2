package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.text.DecimalFormat;

public final class Pose2d {
    DecimalFormat df = new DecimalFormat("#.#####");

    public double x;
    public double y;
    public double heading;

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getHeading() {
        return this.heading;
    }

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public boolean equals(Object var1) {
        if (this != var1) {
            if (var1 instanceof Pose2d) {
                Pose2d var2 = (Pose2d)var1;
                if (Double.compare(this.x, var2.x) == 0 && Double.compare(this.y, var2.y) == 0 && Double.compare(this.heading, var2.heading) == 0) {
                    return true;
                }
            }

            return false;
        } else {
            return true;
        }
    }

    public String toString() {
        return "(" + df.format(this.x) + ", " + df.format(this.y) + ", " + df.format(this.heading) + ")";
    }

    public Pose2d clone(){ return new Pose2d(this.x, this.y, this.heading); }

    public Pose2d add(Pose2d otherPose){ return new Pose2d(this.x + otherPose.x, this.y + otherPose.y, this.heading + otherPose.heading); }

    public Pose2d minus(Pose2d otherPose){ return new Pose2d(this.x - otherPose.x, this.y - otherPose.y, this.heading - otherPose.heading); }

    public Pose2d scale(double factor){ return new Pose2d(factor * this.x, factor * this.y, factor * this.heading); }

    public Pose2d scaleHeading(double factor){ return new Pose2d(this.x, this.y, factor*this.heading); }

    public Pose2d scaleX(double factor){ return new Pose2d(this.x * factor, this.y, this.heading); }

    public double magnitude(){ return Math.hypot(this.x, this.y); }

    public Pose2d clip(Pose2d maximum){
        return new Pose2d(
                Range.clip(this.x, -maximum.x, maximum.x),
                Range.clip(this.y, -maximum.y, maximum.y),
                Range.clip(this.heading, -maximum.heading, maximum.heading)
        );
    }
    public static Pose2d Dtod (Pose2D pose){
        Pose2d newPos = new Pose2d (
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS));
        return newPos;
    }

    public static Pose2D dtoD (Pose2d pose){
        Pose2D newPose = new Pose2D(
                DistanceUnit.INCH,
                pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getHeading());
        return newPose;
    }

}