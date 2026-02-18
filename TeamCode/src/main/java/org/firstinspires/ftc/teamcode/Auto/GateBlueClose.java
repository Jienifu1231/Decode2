package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

public class GateBlueClose extends GorillabotCentral {
    enum State{
        INIT,
        PREPATH,
        PATH,
        SHOOT,
        INTAKEPATH,
        INTAKE,
        INTAKGATE,
        INTAKEPATH2,
        INTAKE2,
        STOP
    }

    public Pose2d ShootPath = new Pose2d (0,0, Math.toRadians(0));
    public Pose2d IntakePath1 = new Pose2d(0,0,Math.toRadians(0));
    public Pose2d Intake1 = new Pose2d(0,0,Math.toRadians(0));
    public Pose2d GateIntakePath = new Pose2d (0,0, Math.toRadians(0));
    public Pose2d IntakePath2 = new Pose2d (0,0, Math.toRadians(0));
    public Pose2d Intake2 = new Pose2d(0,0, Math.toRadians(0));
    public Pose2d stop = new Pose2d(0,0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(-65,-41.375,Math.toRadians(0));
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        Pose2d output = zero;
        Pose2d curpos = zero;

        int ShootIndex = 0;
        int IntakeIndex =0;

        ElapsedTime TotalTimer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime ShootPathTimer = new ElapsedTime();
        double ShootPathWait = 2.2;

        ElapsedTime ShooterTimer = new ElapsedTime();
        double shooting = 1.2;
        double norm_wait = 0.9;
        ElapsedTime IntakeWait = new ElapsedTime();

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double IntakePathWait = 1;

        ElapsedTime IntakeTimer = new ElapsedTime();
        double Intaking = 2;

        waitForStart();
        TotalTimer.reset();
        //some telemetry here
        while(!isStopRequested()){
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;

            switch (state){
                case INIT:
                    output = zero;
                    Outtake.stop();
                    Turret.stop();
                    Intake.stop();
                    Gate.close();
                    if(TotalTimer.seconds() > init_wait){
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case PATH:
                    output = drive.goToPosition(ShootPath, 0.4, 1, 0.4, 0.1);
                    Outtake.launch_far();
                    Gate.close();
                    Intake.manual(0.5);
                    //
                    Turret.limeBlue();
                    //
                    if(drive.drivePID.pos_reached == true || ShootPathTimer.seconds() > ShootPathWait){
                        //TUNE DRIVE PID
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        IntakeWait.reset();
                        state = State.SHOOT;
                    }

                    break;

                case SHOOT:
                    Turret.limeBlue();
                    output = zero;
                    Outtake.launch_close();
                    Gate.open();
                    if(ShooterTimer.seconds() < shooting){
                        Gate.open();
                        if(IntakeWait.seconds() > norm_wait){
                            Intake.exprelease(true);
                        }else{
                            Intake.stop();
                        }
                    }else{
                        ShootIndex ++;
                        ShooterTimer.reset();
                        IntakeWait.reset();
                    }

                    if(ShootIndex >= 3) {
                        drive.resetPath();
                        Turret.stop();
                        //Outtake.stop();
                        Intake.stop();
                        IntakePathTimer.reset();

                        if(IntakeIndex == 0) {
                            state = State.INTAKEPATH;
                        }else if(IntakeIndex == 1){
                            state = State.INTAKGATE;
                        }else if(IntakeIndex == 2){
                            state = State.INTAKEPATH2;
                        }
                    }

                    break;

                case INTAKEPATH:
                    output = drive.goToPosition(IntakePath1, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds() > IntakePathWait){
                        drive.resetPath();
                        IntakeTimer.reset();
                        state = State.INTAKE;
                    }
                    break;

                case INTAKE:
                    output = drive.goToPosition(Intake1, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    if(IntakeTimer.seconds() <= Intaking){
                        Intake.exprelease(true);
                    }else{
                        Intake.stop();
                        ShootIndex = 0;
                        IntakeIndex = 1;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.PATH;
                    }
                    break;

                case INTAKGATE:
                    output = drive.goToPosition(GateIntakePath, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if(IntakePathTimer.seconds() > IntakePathWait){
                        Intake.stop();
                        ShootIndex = 0;
                        if(TotalTimer.seconds() < 25) {
                            IntakeIndex = 1;
                        }else{
                            IntakeIndex = 2;
                        }
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.PATH;
                    }
                    break;

                case INTAKEPATH2:
                    break;

                case INTAKE2:
                    break;

                case STOP:
                    break;

            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("Intake state", Intake.target_state);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("shoot index", ShootIndex);
            telemetry.addData("drivePID pos reached?", drive.drivePID.pos_reached);
            telemetry.update();
        }

    }
}
