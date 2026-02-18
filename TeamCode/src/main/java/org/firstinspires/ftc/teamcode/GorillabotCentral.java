package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Components.Outtake;
//import org.firstinspires.ftc.teamcode.Components.Spindex;
//import org.firstinspires.ftc.teamcode.Components.color;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Angle;
import org.firstinspires.ftc.teamcode.Components.Gate;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.Outtake;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.drive.drive;
import org.firstinspires.ftc.teamcode.drive.OdometryPods;
import org.firstinspires.ftc.teamcode.util.Convolution;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.util.controller.Controller;
import org.firstinspires.ftc.teamcode.util.Pose2d;
//import org.firstinspires.ftc.teamcode.Components.Spindex;
//import org.firstinspires.ftc.teamcode.Components.Outtake;
//import org.firstinspires.ftc.teamcode.Components.color;
//import org.firstinspires.ftc.teamcode.Components.Intake;


public abstract class GorillabotCentral extends LinearOpMode {
    public org.firstinspires.ftc.teamcode.drive.drive drive;
    public OdometryPods pods;
    public Controller g1;
    public Controller g2;

    public Limelight3A limelight;
    public LLResult limeResult;

    public org.firstinspires.ftc.teamcode.Components.Intake Intake;
    public org.firstinspires.ftc.teamcode.Components.Outtake Outtake;

    public org.firstinspires.ftc.teamcode.Components.Turret Turret;
    public org.firstinspires.ftc.teamcode.Components.Lift Lift;
    public org.firstinspires.ftc.teamcode.Components.Gate Gate;
    public org.firstinspires.ftc.teamcode.Components.Angle Angle;
    //public Outtake outtake;
    //public Spindex spindex;
    //public color Colorsensors;

    public ElapsedTime timer;
    public double time_elapsed;
    private double last_time = 0;

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public double LIME_HEIGHT = 0; //tune base on the actual robot
    public double LIME_ANGLE = Math.toRadians(0); //tune base on the robt

    public Convolution txc = new Convolution(5, 4);
    public Convolution tyc = new Convolution(5, 4);
    public Convolution ac = new Convolution(15, 4);
    //might not work, depends

    //final double PI_8 = Math.PI/8;

    public final Pose2d zero = new Pose2d(0, 0, 0);

    public void initializeComponents() {
        drive = new drive(hardwareMap);
        pods = new OdometryPods(hardwareMap);
        g1 = new Controller(gamepad1);
        g2 = new Controller(gamepad2);
        Intake = new Intake(hardwareMap);
        Outtake = new Outtake(hardwareMap);
        Turret = new Turret(hardwareMap);
        Lift = new Lift(hardwareMap);
        Gate = new Gate(hardwareMap);
        Angle = new Angle(hardwareMap);

        timer = new ElapsedTime();
        //change the component base the actual robot later on



        drive.setPose(drive.init_pos);
        drive.resetPath();

    }

    public void updateTime() {
        double time = timer.seconds();
        time_elapsed = time - last_time;
        last_time = time;
    }

    public void updateControllers() {
        g1.update();
        g2.update();
    }
//later on component commands

    public void updateComponents() {
        updateTime();

        drive.updatePose();
        //maybe pinpoint pose update
        Intake.update();
        Outtake.update();
        Turret.update();
        Lift.update();
        Gate.update();
        Angle.update();

        updateControllers();


    }

    public void initLime(int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.setPollRateHz(100);
        limelight.start();
        //change things later on base on the real game??
    }

    public Pose2d getLimeOffset(double angle) {
        limeResult = limelight.getLatestResult();

        if (limeResult != null && limeResult.isValid()) {
            double tx = limeResult.getTx();
            double ty = limeResult.getTy();

            //txc.update(tx);
            //tyc.update(ty);

            double x_offset = Math.tan(Math.toRadians(tx)) * LIME_HEIGHT / Math.cos(LIME_ANGLE);
            double y_offset = LIME_HEIGHT * (Math.tan(LIME_ANGLE + Math.toRadians(ty)) - Math.tan(LIME_ANGLE));

            return new Pose2d(tx, ty, angle);
        } else {
            return null;
        }
    }

    public Pose2d getLimeOffset(double angle, boolean convolute) {
        limeResult = limelight.getLatestResult();

        if (limeResult != null && limeResult.isValid()) {
            double tx = limeResult.getTx();
            double ty = limeResult.getTy();

            if (convolute) {
                txc.update(tx);
                tyc.update(ty);

                return new Pose2d(txc.getAverage(), tyc.getAverage(), angle);
            } else {
                return new Pose2d(tx, ty, angle);
            }
        } else {
            return null;
        }
    }

    // public int getLLTagID(){
    // limeResult = limelight.getLatestResult();

    // if(limeResult != null && limeResult.isValid()){
    // return limeResult.getTid();
    // } else {
    // return - 1;
    // }
    // }


    public double current_angle = 0;
}
//public List<List<Double>> corners; what the heck









