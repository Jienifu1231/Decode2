package org.firstinspires.ftc.teamcode.test.driveTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.WayPoint;

import java.util.ArrayList;

@Autonomous
@Config
public class PurePursuitTest extends GorillabotCentral {
    enum State{
        INIT, 
        PATH,
        SHOOT,
        INTAKEPATH,
        INTAKE,
        GATEOPEN,
        GATEBACK
    }

    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();
        Pose2d init_pos = new Pose2d(-65,-41.375,Math.toRadians(0));
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        Pose2d output = zero;
        Pose2d curpos = zero;

        Pose2d shootPath = new Pose2d (-13.7,-17.5, Math.toRadians(-90));
        Pose2d spike2 = new Pose2d (9.75,-18.6, Math.toRadians(-90));
        Pose2d intake2 = new Pose2d(9.75,-62.6, Math.toRadians(-90));
        Pose2d GateOpen = new Pose2d(0,-54,Math.toRadians(-90));

        ArrayList<WayPoint> GateShootPath = new ArrayList<>();
        GateShootPath.add(new WayPoint(9.75,-62.6, Math.toRadians(-90), 0.95, 0.7, 1));
        GateShootPath.add(new WayPoint(9.75,-62.6, Math.toRadians(-90), 0.95, 0.7, 1, 13)); //0.85, 24
        GateShootPath.add(new WayPoint(-13.7,-17.5, Math.toRadians(-90), 0.6, 0.7, 1.5, 21)); //0.27, 21

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime PathTimer = new ElapsedTime();
        double Path_wait = 2.2;

        ElapsedTime ShootTimer = new ElapsedTime();
        double Shooting = 1.2;

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double IntakePathWait = 1;

        ElapsedTime IntakeTimer = new ElapsedTime();
        double Intaking = 2;

        int IntakeIndex = 0;

        while(!isStopRequested()){
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;

            switch(state){
                case INIT:
                    output = zero;
                    Intake.stop();
                    Outtake.stop();
                    Gate.close();
                    Turret.stop();
                    if(timer.seconds() > init_wait){
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case PATH:
                    output = drive.goToPosition(shootPath, 0.4, 1, 0.4, 0.1);
                    Outtake.launch_close();
                    Gate.close();
                    Intake.exprelease(true);
                    if( PathTimer.seconds() > Path_wait){
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        ShootTimer.reset();
                        Intake.stop();
                        state = State.SHOOT;

                    }
                    break;

                case SHOOT:
                    Turret.limeBlue();
                    output = zero;
                    Outtake.launch_close();
                    Gate.open();
                    //
                    if(ShootTimer.seconds() < Shooting){
                        Angle.close();
                        Outtake.launch_close();
                    }else{
                        Turret.stop();
                        Outtake.stop();
                        Intake.stop();
                        IntakePathTimer.reset();
                    }
                    break;

                case INTAKEPATH:
                    output = drive.goToPosition(spike2, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds()> IntakePathWait){
                        drive.resetPath();
                        IntakeTimer.reset();
                        state = State.INTAKE;
                    }
                    break;

                case INTAKE:
                    output = drive.goToPosition(intake2, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    if(IntakeTimer.seconds() <= Intaking){
                        Intake.exprelease(true);
                    }else{
                        IntakeIndex = 1;
                        Intake.stop();
                        PathTimer.reset();
                        drive.resetPath();
                        state = State.PATH;
                    }
                    break;

                case GATEOPEN:
                    output = drive.goToPosition(GateOpen, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    if(PathTimer.seconds() > Path_wait){
                        drive.resetPath();
                        state = State.GATEBACK;
                    }
                    break;

                case GATEBACK:
                    output = drive.purePursuit(GateShootPath, 5);//tuned number?
                    break;

            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("Intake state", Intake.target_state);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("drivePID pos reached?", drive.drivePID.pos_reached);
            telemetry.update();
        }



    }
}
