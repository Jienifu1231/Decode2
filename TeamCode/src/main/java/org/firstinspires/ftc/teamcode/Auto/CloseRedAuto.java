package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
@Config
public class CloseRedAuto extends GorillabotCentral {
    enum State{
        INIT,
        PREPATH,
        PATH,
        SHOOT,
        INTAKEPATH,
        INTAKE,
        INTAKEPATH2,
        INTAKE2,
        INTAKEPATH3,
        INTAKE3,
        STOP
    }

    public static Pose2d ShootPath = new Pose2d(-13.7,17.5, Math.toRadians(90));//Position 2 -- angel should be constant
    //-18.5
    public static Pose2d IntakePath1 = new Pose2d(-14,18.6, Math.toRadians(90));//-10
    public static Pose2d Intake1 = new Pose2d(-14,55.5,Math.toRadians(90));//62.6
    public static Pose2d IntakePath2 = new Pose2d (9.75,18.6, Math.toRadians(90));//14.75
    public static Pose2d Intake2 = new Pose2d(9.75,62.6, Math.toRadians(90));
    public static Pose2d IntakePath3 = new Pose2d(32.75,18.6, Math.toRadians(90));//35.75
    public static Pose2d Intake3 = new Pose2d(32.75,62.6,Math.toRadians(90));
    public static Pose2d stop = new Pose2d (3, 18.6, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(-65,41.375,Math.toRadians(0)); // change this
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));
        //drive.pinpoint.recalibrateIMU();

        Pose2d output = zero;
        Pose2d curpos = zero;

        int shootIndex = 0;// how much b// alls we shot
        int intakeIndex = 0;

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime ShootPathTimer = new ElapsedTime();
        double shootPathWait = 2.2;//probably change this one out

        ElapsedTime ShooterTimer  = new ElapsedTime();
        double shooting = 1.2;//1
        double intake_wait = 0.9;//0.7
        double norm_wait = 0.9;//0.7
        ElapsedTime IntakeWait = new ElapsedTime();

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double intakePathWait = 1;//maybe this one as well

        ElapsedTime IntakingTimer = new ElapsedTime();
        double Intaking = 2;

        waitForStart();
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
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }
                    break;
                case PATH:

                    output = drive.goToPosition(ShootPath, 0.4, 1, 0.4, 0.1);//0.4
                    Outtake.launch_far();
                    Gate.close();
                    Intake.exprelease(true);
                    if( ShootPathTimer.seconds() > shootPathWait){
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        IntakeWait.reset();
                        state = State.SHOOT;

                    }
                    break;

                case SHOOT:
                    Turret.limeRed();
                    //updated here
                    output = zero;
                    Outtake.launch_close();
                    Gate.open();
                    //
                    if(ShooterTimer.seconds() < shooting){
                        Angle.close();
                        if(IntakeWait.seconds() > norm_wait){
                            Intake.exprelease(true);
                        }else{
                            Intake.stop();
                        }
                    }else{
                        shootIndex ++;
                        ShooterTimer.reset();
                        IntakeWait.reset();
                    }

                    if(shootIndex >= 3) {
                        //Outtake.stop();
                        Intake.stop();
                        Turret.stop();
                        IntakePathTimer.reset();
                        if (intakeIndex == 0) {
                            drive.resetPath();
                            state = State.INTAKEPATH;
                        }

                        if (intakeIndex == 1) {
                            drive.resetPath();
                            state = State.INTAKEPATH2;
                        }
                        if(intakeIndex == 2){
                            drive.resetPath();
                            state = State.INTAKEPATH3;
                        }
                        if(intakeIndex ==3){
                            drive.resetPath();
                            state = State.INTAKEPATH;
                        }
                    }
                    break;

                case INTAKEPATH:
                    output = drive.goToPosition(IntakePath1, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds() > intakePathWait){
                        //drive.drivePID.pos_reached == true
                        drive.resetPath();
                        IntakingTimer.reset();
                        state = State.INTAKE;
                    }
                    break;
                case INTAKE:
                    output = drive.goToPosition(Intake1, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    if(IntakingTimer.seconds() <= Intaking){
                        Intake.exprelease(true);
                    }else{
                        Intake.stop();
                        shootIndex = 0;
                        intakeIndex = 1;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.PATH;
                    }
                    break;
                case INTAKEPATH2:
                    output  = drive.goToPosition(IntakePath2, 0.5, 1, 0.4, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds() > intakePathWait){
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        IntakingTimer.reset();
                       /* if(intakeIndex == 1) {
                            state = State.INTAKE2;
                        }else if(intakeIndex == 2){
                            drive.resetPath();
                            ShootPathTimer.reset();
                            state = State.PATH;
                        }*/
                        ShootPathTimer.reset();
                        state = State.INTAKE2;
                    }
                    break;
                case INTAKE2:
                    output = drive.goToPosition(Intake2, 0.4,1, 0.4, 0.1);
                    Gate.close();
                    if(IntakingTimer.seconds() <= Intaking){
                        Intake.exprelease(true);
                        IntakePathTimer.reset();
                    }else{
                        output = drive.goToPosition(IntakePath2, 0.4, 1, 0.4, 0.1);
                        if(IntakePathTimer.seconds() > intakePathWait) {
                            Intake.stop();
                            shootIndex = 0;
                            intakeIndex = 2;
                            ShootPathTimer.reset();
                            drive.resetPath();
                            state = State.PATH;
                        }
                    }
                    break;

                case INTAKEPATH3:
                    output  = drive.goToPosition(IntakePath3, 0.4, 1, 0.4, 0.1);
                    Gate.close();
                    if(drive.drivePID.pos_reached == true && IntakePathTimer.seconds() > intakePathWait){
                        drive.resetPath();
                        IntakingTimer.reset();
                        state = State.INTAKE3;
                    }
                    break;
                case INTAKE3:
                    output = drive.goToPosition(Intake3, 0.4,1, 0.4, 0.1);
                    Gate.close();
                    if(IntakingTimer.seconds() <= Intaking){
                        Intake.exprelease(true);
                    }else{
                        Intake.stop();
                        shootIndex = 0;
                        intakeIndex = 3;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.STOP;
                    }
                    break;
                case STOP:
                    output = drive.goToPosition(stop, 0.4, 1, 0.4, 0.1);
                    Intake.exprelease(true);
                    break;



            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("Intake state", Intake.target_state);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("shoot index", shootIndex);
            telemetry.addData("intake index ", intakeIndex );
            telemetry.addData("drivePID pos reached?", drive.drivePID.pos_reached);
            telemetry.update();

        }

    }
}
