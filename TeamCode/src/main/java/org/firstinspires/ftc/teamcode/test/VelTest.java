package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.PIDF;

@TeleOp
@Config
public class VelTest extends GorillabotCentral {
    public static double velocity = 1000;
    public static double angle_pos = 0.2;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();
        updateControllers();


        ;//change this based on calculation

        waitForStart();
        while(!isStopRequested()){
            double curvel = Outtake.RflyWheel.getVelocity();
            //double power = far_pid.update(velocity, curvel);

            /*if(g1.a.isPressed()){
                Outtake.manual(power);
                Outtake.manual(power);
            }*/

            if(g2.a.wasJustPressed()){
                Outtake.launch_far();
            }

            if(g2.b.wasJustPressed()){
                Outtake.bang_pid();
            }

            if(g1.b.wasJustPressed()){
                Angle.manual(angle_pos);
            }

            updateComponents();

            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("wheel vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("target vel", -Outtake.vel);
            telemetry.addData("power", Outtake.RflyWheel.getPower());
            telemetry.addData("ratio", Outtake.far_pid.voltageRatio);
            telemetry.addData("current Voltage", Outtake.voltage);
            telemetry.addData("bang bang?", Outtake.bangbang);
            telemetry.addData("if pid is true", Outtake.pid_active);
            telemetry.update();

            dashboardTelemetry.addData("Outtake state", Outtake.target_state);
            dashboardTelemetry.addData("wheel vel", Outtake.RflyWheel.getVelocity());
            dashboardTelemetry.addData("target vel", -Outtake.vel);
            dashboardTelemetry.update();


        }


    }
}
