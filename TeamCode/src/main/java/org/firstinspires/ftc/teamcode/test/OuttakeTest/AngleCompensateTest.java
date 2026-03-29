package org.firstinspires.ftc.teamcode.test.OuttakeTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;

@TeleOp
public class AngleCompensateTest extends GorillabotCentral {
    //first use bang_pid mode in outtake to figure out how much ticks/sec the shooter drop when rapid firing
    //change the number of velDrop -- this models shooting the second ball when the the shooter is not recovered from shooting the first one
    //adjust the angle of the shooter untill the ball makes into the goal
    //plot the vel and the angle
    //all the numbers for angle in here is the position of the servo is going at -- not the actual angle
    //THE DISTANCE MUST STAY THE SAME WHEN DOING THIS
    // the loop should go like this:
    // 1. generate the target vel based on where we are on the field
    // 2. use the bang_pid mode to get to speed
    // 3. shoot the ball(snesing vel drop)
    // 4. the angle compensation regression computes a angle the hood would use in order to make the ball in


    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();
        updateControllers();

        double angle_pos = 0;
        double velDrop = 300;

        waitForStart();

        while(!isStopRequested()){

            drive.setDrivePower(g1.getDrivePower().scale(1));

            if(g1.a.wasJustPressed()){
                Outtake.launch_far();
                if(g1.leftBumper.isPressed()){
                    Outtake.vel = Outtake.vel - velDrop; //model vel drop
                    //first test out around how many ticks per sec the speed drop
                }
            }else{
                Outtake.stop();
            }

            if(g2.leftBumper.wasJustPressed()){
                angle_pos -= 0.2; //change 0.2 if too small or too big
                if(angle_pos < 0){
                    angle_pos = 0;
                }
            }

            if(g2.rightBumper.wasJustPressed()){
                angle_pos += 0.2;
                if(angle_pos > 0){
                    angle_pos = 1;
                }
            }
            //change this angle to make the ball still shoot in

            Angle.manual(angle_pos);

            updateComponents();
            //intake update in updateComponents()

            telemetry.addData("outtake state", Outtake.target_state);
            telemetry.addData("outtake power right", Outtake.RflyWheel.getPower());
            telemetry.addData("outtake power left", Outtake.LflyWheel.getPower());
            telemetry.addData("outtake power variable", Outtake.power);
            telemetry.addData("target vel", Outtake.vel);
            telemetry.addData("fly wheel vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("fly wheel vel left", Outtake.LflyWheel.getVelocity());
            telemetry.addData("angle servo", Angle.outtake_angle.getPosition());
            telemetry.update();

            dashboardTelemetry.addData("target vel", -Outtake.vel);
            dashboardTelemetry.addData("cur vel right", Outtake.RflyWheel.getVelocity());
            dashboardTelemetry.addData("cur vel left", Outtake.LflyWheel.getVelocity());
            dashboardTelemetry.update();

        }




    }
}
