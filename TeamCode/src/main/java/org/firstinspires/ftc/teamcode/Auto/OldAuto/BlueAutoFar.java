package org.firstinspires.ftc.teamcode.Auto.OldAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
@Config
public class BlueAutoFar extends GorillabotCentral {

    enum State{
        INIT,
        PATH,
        SHOOT,
        INTAKEPATH,
        INTAKE,
        INTAKEPATH2,
        INTAKE2,
        INTAKEPATH3,
        INTAKE3,
        PATH2,
        SHOOT2,
        STOP,
        CHECK
    }

    public static Pose2d ShootPath = new Pose2d(58.75, -16.6, Math.toRadians(201));
    public static Pose2d ShootPath2 = new Pose2d(-15,-17.5, Math.toRadians(230));
    public static Pose2d IntakePath1 = new Pose2d(35.75,-18.6, Math.toRadians(180));
    public static Pose2d Intake1 = new Pose2d (37.75,-62.6,Math.toRadians(-90));
    public static Pose2d IntakePath2 = new Pose2d(14.75,-18.6, Math.toRadians(-180));
    public static Pose2d Intake2 = new Pose2d (14.75,-62.6, Math.toRadians(-90));
    public static Pose2d IntakePath3 = new Pose2d (-7,-18.6, Math.toRadians(-180));
    public static Pose2d Intake3 = new Pose2d (-7,-62.6,Math.toRadians(-90));
    public static Pose2d Stop = new Pose2d (5, -22, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(64.75,-18.6,Math.toRadians(180)); // change this
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));
        //drive.pinpoint.recalibrateIMU();

        Pose2d output = zero;
        Pose2d curpos = zero;

        double WrapUp = 0;

        int shootIndex = 0;// how much b// alls we shot
        int intakeIndex = 0;

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime ShootPathTimer = new ElapsedTime();
        double shootPathWait = 2;

        ElapsedTime ShooterTimer  = new ElapsedTime();
        double shooting = 1.2;//1
        double intake_wait = 0.9;//0.7
        double norm_wait = 0.9;//0.7
        ElapsedTime IntakeWait = new ElapsedTime();

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double intakePathWait = 1;

        ElapsedTime IntakingTimer = new ElapsedTime();
        double Intaking = 1.5;

        waitForStart();
        while (!isStopRequested()) {
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;

            switch(state){
                case INIT:
                    output = zero;
                    Intake.stop();
                    Outtake.stop();
                    Gate.close();
                    if(timer.seconds() > init_wait){
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }

                    break;
                case PATH:
                    output = drive.goToPosition(ShootPath, 0.6, 1, 0.6, 0.1);
                    Outtake.launch_far();
                    Intake.manual(0.4);
                    if(WrapUp == 0){
                        shootPathWait = shootPathWait + 1.6;
                        WrapUp = 1;
                    }
                    if(drive.drivePID.pos_reached == true && ShootPathTimer.seconds() > shootPathWait){//change stuff here to make sure PID is right -- ki
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        IntakeWait.reset();
                        state = State.SHOOT;
                        //check I term for not being able to reach the state
                    }
                    break;
                case SHOOT:
                    //Turret.limeRed(); -- without turret??
                    Intake.manual(1);
                    //update here
                    output = zero;
                    Outtake.launch_far();
                    Angle.manual(0.228);
                    Gate.open();
                    if(ShooterTimer.seconds() > shooting) {
                        //Outtake.launch_far();
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

                        if (intakeIndex == 2) {
                            drive.resetPath();
                            state = State.INTAKEPATH3;

                        }


                    }
                    break;

                case INTAKEPATH:
                    output = drive.goToPosition(IntakePath1, 0.6, 1, 0.6, 0.1);
                    Gate.close();

                    if(drive.drivePID.pos_reached == true || IntakePathTimer.seconds() > intakePathWait){
                        //drive.drivePID.pos_reached == true &&
                        //if got stucked in the state then take out the pos_reached part
                        drive.resetPath();
                        IntakingTimer.reset();
                        state = State.INTAKE;
                    }
                    break;
                case INTAKE:
                    output = drive.goToPosition(Intake1, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Outtake.launch_far();
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
                    output = drive.goToPosition(IntakePath2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(drive.drivePID.pos_reached == true && IntakePathTimer.seconds() > intakePathWait){
                        //if got stucked in the state then take out the pos_reached part
                        drive.resetPath();
                        IntakingTimer.reset();
                        state = State.INTAKE2;
                    }
                    break;

                case INTAKE2:
                    output = drive.goToPosition(Intake2, 0.6,1, 0.6, 0.1);
                    Gate.close();
                    Outtake.launch_far();
                    if(IntakingTimer.seconds() <= Intaking){
                        Intake.exprelease(true);
                    }else{
                        Intake.stop();
                        shootIndex = 0;
                        intakeIndex = 2;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.PATH;
                    }
                    break;

                case INTAKEPATH3:
                    output = drive.goToPosition(IntakePath3, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(drive.drivePID.pos_reached == true && IntakePathTimer.seconds() > intakePathWait){
                        drive.resetPath();
                        IntakingTimer.reset();
                        state = State.INTAKE3;
                    }
                    break;

                case INTAKE3:
                    output = drive.goToPosition(Intake3, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Outtake.launch_far();
                    if(IntakingTimer.seconds() <= Intaking){
                        Intake.exprelease(true);
                    }else{
                        Intake.stop();
                        shootIndex = 0;
                        intakeIndex =3;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.PATH;
                    }
                    break;


                case PATH2:
                    output = drive.goToPosition(ShootPath2, 0.6, 1, 0.6, 0.1);//0.4
                    Outtake.launch_close();
                    Intake.manual(0.4);
                    if(drive.drivePID.pos_reached == true && ShootPathTimer.seconds() > shootPathWait){//change stuff here to make sure PID is right -- ki
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        IntakeWait.reset();
                        state = State.SHOOT2;
                        //check I term for not being able to reach the state
                    }
                    break;

                case SHOOT2:
                    output = zero;
                    if(ShooterTimer.seconds() < shooting) {
                        Outtake.launch_close();
                        Angle.close();
                        Gate.open();
                        Intake.manual(1);
                    }else{
                        Outtake.launch_close();
                        output = zero;
                        state = State.CHECK;
                    }
                    break;

            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("Intake state", Intake.target_state);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("shoot index", shootIndex);
            telemetry.addData("drivePID pos reached?", drive.drivePID.pos_reached);
            telemetry.update();

        }

    }
}
