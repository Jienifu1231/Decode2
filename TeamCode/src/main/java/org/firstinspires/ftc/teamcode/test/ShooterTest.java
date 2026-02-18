package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotCentral;

@TeleOp
public class ShooterTest extends GorillabotCentral {



    @Override
    public void runOpMode() throws InterruptedException {
    updateControllers();
        initializeComponents();

        double output = 0.5;

        waitForStart();

        while (!isStopRequested()) {
            if(g1.a.wasJustPressed()){
                Outtake.launch_far();
            }

            if(g1.b.wasJustPressed()){
                Outtake.launch_close();
            }
            if(g1.x.wasJustPressed()){
                Outtake.stop();
            }



            telemetry.addData("poweer", Outtake.vel);
            telemetry.addData("state", Outtake.target_state);
            telemetry.addData("state reached?", Outtake.state_reached);
            telemetry.update();



        }
    }
}
