package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
@Config
public class PinpointAutoTest extends GorillabotCentral {
    enum State{
        INIT,
        INTAKEPATH,
        INTAKE,
        CHECK
    }

    public static Pose2d IntakePath = new Pose2d(38.75,-18.6, Math.toRadians(180));
    public static Pose2d IntakePath1 = new Pose2d(41.75,-62.6,Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        //chaneg the path

        drive.init_pos = new Pose2d(64.75,-18.6,Math.toRadians(180));
        drive.pinpoint.setPosition(Pose2d.dtoD(drive.init_pos)); // set it to the field centric
        drive.pinpoint.recalibrateIMU();
        //test this to see if it mess anything with the angle

        //drive.pinpoint.resetPosAndIMU();

        Pose2d output = zero;
        Pose2d curpos = new Pose2d(64.75,-18.6,Math.toRadians(180));

        boolean intake_reached = false;

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime pathTimer = new ElapsedTime();
        double path_wait = 2;
        //tune path_wait; maybe 3?
        //check use this if got stucked in a path

        ElapsedTime intakeWaitTimer = new ElapsedTime();
        double intake_wait = 2;

        ElapsedTime intakeTimer = new ElapsedTime();
        double intake_time = 5;

        boolean finished = false;


        waitForStart();
        timer.reset();

        while(!isStopRequested()){
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;

            switch(state){
                case INIT:
                    output = zero;
                    Intake.stop();
                    Outtake.stop();
                    if(timer.seconds() > init_wait){
                        pathTimer.reset();
                        state = State.INTAKEPATH;
                    }

                    break;

                case INTAKEPATH:
                   output = drive.goToPosition(IntakePath, 0.4, 1, 0.4, 0.05);
                   if(drive.drivePID.pos_reached == true || pathTimer.seconds() > path_wait){
                       drive.resetPath();
                      // drive.drivePID.pos_reached = false;
                       intakeTimer.reset();
                       state = State.INTAKE;
                   }
                    break;

                case INTAKE:
                    output = drive.goToPosition(IntakePath1, 0.2, 1, 0.4, 0.05);
                    if(intakeTimer.seconds() <= intake_time){
                        Intake.exprelease(false);
                    }
                    if(intakeTimer.seconds() > intake_time){
                        Intake.stop();
                        intake_reached = true;
                        state = State.CHECK;
                        // drive.drivePID.pos_reached
                    }

                    break;
                case CHECK:
                    finished = true;
                    output = zero;
                    break;





            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("current Pos", curpos);
            telemetry.addData("state", state);
            telemetry.addData("power", output);
            telemetry.addData("pos reached?", drive.drivePID.pos_reached);
            telemetry.addData("intake state", Intake.target_state);
            telemetry.addData("intake posh reached", intake_reached);
            telemetry.addData("finished?", finished);
            telemetry.update();

            dashboardTelemetry.addData("current Pos", curpos);
            dashboardTelemetry.addData("pos X", curpos.getX());
            dashboardTelemetry.addData("pos Y", curpos.getY());
            dashboardTelemetry.addData("pos r", curpos.getHeading());
            dashboardTelemetry.addData("finished?", finished);
            dashboardTelemetry.update();






        }


    }
}
