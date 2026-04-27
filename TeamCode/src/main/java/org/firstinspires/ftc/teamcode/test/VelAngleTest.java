package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp
@Config
public class VelAngleTest extends GorillabotCentral {
    public static double vel = 1000;
    public static double angle_pos = 0.2;
    Pose2d curpos = zero;
    public double targetX = 71;
    public double targetY = 71;
    public double LinearDis = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();
        updateControllers();

        Pose2d init_pos = new Pose2d(0,0, Math.toRadians(90)); // change this
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        double output = 0;


        //change this based on calculation

        waitForStart();
        while(!isStopRequested()){
            double curvel = Outtake.RflyWheel.getVelocity();

            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());

            LinearDis = Math.sqrt(Math.pow(targetX - curpos.getX(), 2) + Math.pow(targetY - curpos.getY(), 2));


            if(g2.b.wasJustPressed()){
                Outtake.combined(vel);
            }
            //other test

            if(g1.x.wasJustPressed()){
                vel += 50;
            }
            if(g1.y.wasJustPressed()){
                vel -= 50;
            }

            if(g1.dpadUp.wasJustPressed()){
                vel += 20;
            }
            if(g1.dpadDown.wasJustPressed()){
                vel -= 20;
            }

            if(g1.leftBumper.wasJustPressed()){
               angle_pos += 0.02;
                if(angle_pos >= 1){
                    angle_pos = 1;
                }
            }

            if(g1.rightBumper.wasJustPressed()){
                angle_pos -= 0.02;
                if(angle_pos <= 0){
                    angle_pos = 0;
                }
            }

            if(g1.leftTrigger.moved()){
                Intake.manual(1);
                Gate.open();
            }else{
                Intake.stop();
            }

            if(g1.rightTrigger.moved()){
                Intake.manual(1);
                Gate.close();
            }

            if (g2.a.isPressed()) {
                Turret.pinpointRed(curpos, Turret.turret.getCurrentPosition());
            }else if(g2.b.isPressed()){
                Turret.manual(output);
            }else if(g2.rightBumper.isPressed()){
                Turret.manual(-0.5);
            }else if(g2.leftBumper.isPressed()){
                Turret.manual(0.5);//tune
            }else{
                Turret.stop();
            }

            if(g1.a.isPressed()){
                Angle.manual(angle_pos);
                Outtake.launch_close_test(vel);
            }

            drive.setDrivePower(g1.getDrivePower().scale(1).scaleHeading(1).scaleX(1));
            updateComponents();

            telemetry.addData("cur vel", vel);
            telemetry.addData("actual Rflywheel vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("cur angle", angle_pos);
            telemetry.addData("curret position", curpos);
            telemetry.addData("distance", LinearDis);
            telemetry.addData("wheel vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("target vel", -Outtake.vel);
            telemetry.addData("power", Outtake.RflyWheel.getPower());
            telemetry.addData("ratio", Outtake.far_pid.voltageRatio);
            telemetry.addData("current Voltage", Outtake.voltage);
            telemetry.addData("bang bang", Outtake.bangbang);
            telemetry.addData("if pid is true", Outtake.pid_active);
            telemetry.addData("Turret Power", Turret.turret.getVelocity());
            telemetry.update();

            dashboardTelemetry.addData("Outtake state", Outtake.target_state);
            dashboardTelemetry.addData("wheel vel", Outtake.RflyWheel.getVelocity());
            dashboardTelemetry.addData("target vel", -vel);
            dashboardTelemetry.update();


        }


    }
}
