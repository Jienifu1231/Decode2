package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Auto.OldAuto.RedAutoFar;
import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;


@Autonomous
@Config
public class BlueGateFar extends GorillabotCentral {
    enum State{
        INIT,
        PATH,
        SHOOT,
        HPINTAKE,
        GATEINTAKE,
        GATEINTAKE2

    }


    public static Pose2d ShootPath = new Pose2d(58.75, -16.6, Math.toRadians(201));
    public static Pose2d HPIntake = new Pose2d (65,-67,Math.toRadians(-90));
    public static Pose2d GateIntake1 = new Pose2d (58,-60, Math.toRadians(-90));
    public static Pose2d GateIntake2 = new Pose2d (55, -60, Math.toRadians(-90));


    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();


        Pose2d init_pos = new Pose2d(64.75,-18.6,Math.toRadians(180)); // change this
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));


        boolean HPIntaked = false;


        Pose2d output = zero;
        Pose2d curpos = zero;
        double WrapUp = 0;


        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;


        ElapsedTime ShootPathTimer = new ElapsedTime();
        double ShootPathWait = 2;


        ElapsedTime ShooterTimer = new ElapsedTime();
        double Shooting = 1.5;


        ElapsedTime HPIntakePathTimer = new ElapsedTime();
        double HPIntakPath = 1.8;


        ElapsedTime HPIntakeTimer = new ElapsedTime();
        double HPIntaking = 1.5;


        ElapsedTime GateIntake = new ElapsedTime();
        double GateIntaking = 1.5;


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
                    if(timer.seconds() > init_wait) {
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }
                    break;


                case PATH:
                    output = drive.goToPosition(ShootPath, 0.6, 2, 0.6, 0.2);
                    Outtake.launch_far();
                    if(WrapUp == 0){
                        ShootPathWait = ShootPathWait + 1.6;
                        WrapUp = 1;
                    }


                    if(drive.drivePID.pos_reached == true && ShootPathTimer.seconds() > ShootPathWait){
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        state = State.SHOOT;
                    }
                    break;


                case SHOOT:
                    Turret.limeRed();
                    output = zero;
                    Outtake.launch_far();
                    Gate.open();
                    Angle.far();
                    Intake.exprelease(true);
                    if (ShooterTimer.seconds() > Shooting) {
                        drive.resetPath();
                        Turret.stop();
                        Gate.close();
                        Intake.stop();
                        HPIntakePathTimer.reset();
                        if(HPIntaked == false) {
                            state = State.HPINTAKE;
                        }else{
                            GateIntake.reset();
                            state = State.GATEINTAKE;
                        }
                    }
                    break;


                case HPINTAKE:
                    output = drive.goToPosition(HPIntake, 0.6, 1, 0.6, 0.1);
                    if(drive.drivePID.pos_reached == true){
                        Intake.exprelease(true);
                        HPIntakeTimer.reset();
                    }


                    if(HPIntakeTimer.seconds() > HPIntaking){
                        HPIntaked = true;
                        drive.resetPath();
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }


                    break;


                case GATEINTAKE:
                    output = drive.goToPosition(GateIntake1, 0.6, 1, 0.6, 0.1);
                    Intake.exprelease(true);
                    Gate.close();
                    if(GateIntake.seconds() > GateIntaking){
                        drive.resetPath();
                        GateIntake.reset();
                        state = State.GATEINTAKE2;
                    }
                    break;


                case GATEINTAKE2:
                    output = drive.goToPosition(GateIntake2, 0.6, 1, 0.6, 0.1);
                    Intake.exprelease(true);
                    Gate.close();
                    if(GateIntake.seconds() > GateIntaking){
                        drive.resetPath();
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }
            }


            drive.setDrivePower(output);
            updateComponents();


            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("pos reached?", drive.drivePID.pos_reached);
            telemetry.addData("Outtake state", Outtake.target_state);
            //telemetry.addData("intake Index", intakeIndex);
            telemetry.addData("outtake vel", Outtake.RflyWheel.getVelocity());
            //telemetry.addData("Gate Intake Index", GateIntakeIndex);
            telemetry.addData("Gate Intake Timer", GateIntake.seconds());
            telemetry.update();


        }
    }
}



