package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.RedCloseSolar.State.INTAKE2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
public class RedCloseSolar extends GorillabotCentral {
    enum State {
        INIT,
        PATH,
        SHOOT,
        ROW2,
        INTAKE2,
        GATEOPEN,
        BACKUP,
        ROW1,
        INTAKE1,
        STOP
    }

    public Pose2d ShootPath = new Pose2d(-13.7, -17.5, Math.toRadians(-90));
    public Pose2d Row2 = new Pose2d(9, -18.6, Math.toRadians(-90));
    public Pose2d intake2 = new Pose2d(8, -59, Math.toRadians(-90));
    public Pose2d GateOpen = new Pose2d(7, -58.5, Math.toRadians(-90));
    public Pose2d Row1 = new Pose2d(-14,18.6, Math.toRadians(-90));
    public Pose2d Intake1 = new Pose2d(-14,-55.5,Math.toRadians(-90));
    public Pose2d Stop = new Pose2d(-14,-18.6, Math.toRadians(-90));


    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(-65, -41.375, Math.toRadians(0)); // change this
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        Pose2d output = zero;
        Pose2d curpos = zero;

        double IntakeIndex = 0;

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime PathTimer = new ElapsedTime();
        double Path_wait = 1;

        ElapsedTime ShooterTimer = new ElapsedTime();
        double Shooting = 1.5;

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double IntakePathWait = 0.8;

        ElapsedTime IntakeTimer = new ElapsedTime();
        double IntakeWait = 1.5;

        ElapsedTime GateOpenTimer = new ElapsedTime();
        double GateOpenWait = 5;

        waitForStart();
        while (!isStopRequested()) {
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;

            switch (state) {
                case INIT:
                    output = zero;
                    Intake.stop();
                    Outtake.stop();
                    Gate.close();
                    Turret.stop();
                    if (timer.seconds() > init_wait) {
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case PATH:
                    output = drive.goToPosition(ShootPath, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if (IntakeIndex == 0) {
                        Outtake.launch_far();
                    } else {
                        Outtake.launch_close();
                    }
                    Gate.close();
                    Intake.exprelease(true);
                    Turret.limeBlue();
                    if (drive.drivePID.pos_reached == true || PathTimer.seconds() > Path_wait) {
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        // IntakeTimer.reset();
                        state = State.SHOOT;
                    }
                    break;

                case SHOOT:
                    Turret.limeRed();
                    output = zero;
                    Outtake.launch_close();
                    Gate.open();
                    if (IntakeIndex == 0) {
                        Angle.manual(0.1735);
                    } else {
                        Angle.manual(0.175);//0.1745
                    }
                    Intake.exprelease(true);
                    if (ShooterTimer.seconds() > Shooting) {
                        drive.resetPath();
                        Turret.stop();
                        Gate.close();
                        Intake.stop();
                        IntakePathTimer.reset();
                        if (IntakeIndex == 0) {
                            IntakePathTimer.reset();
                            state = State.ROW2;
                        }else if (IntakeIndex == 1){
                            IntakePathTimer.reset();
                            state = State.ROW1;
                        }else if(IntakeIndex == 2){
                            IntakePathTimer.reset();
                            state = State.ROW2;
                        }
                    }
                    break;

                case ROW2:
                    output = drive.goToPosition(Row2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if (IntakePathTimer.seconds() > IntakePathWait && drive.drivePID.pos_reached == true) {
                        drive.resetPath();
                        IntakeTimer.reset();
                        state = State.INTAKE2;
                    }

                    break;

                case INTAKE2:
                    output = drive.goToPosition(intake2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    IntakeIndex = 1;
                    if(IntakeTimer.seconds() > IntakeWait){
                        drive.resetPath();
                        GateOpenTimer.reset();
                        state = State.GATEOPEN;
                    }
                    break;

                case GATEOPEN:
                    output = drive.goToPosition(GateOpen, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(GateOpenTimer.seconds() > GateOpenWait){
                        drive.resetPath();
                        IntakePathTimer.reset();
                        state = State.BACKUP;
                    }
                    break;

                case BACKUP:
                    output = drive.goToPosition(Row2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds() > IntakePathWait){
                        drive.resetPath();
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case ROW1:
                    output = drive.goToPosition(Row1, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    IntakeIndex = 2;
                    if(IntakePathTimer.seconds() > IntakePathWait){
                        drive.resetPath();
                        IntakeTimer.reset();
                        state = State.INTAKE1;

            }
                    break;

                case INTAKE1:
                    output = drive.goToPosition(Intake1, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if(IntakeTimer.seconds() > IntakeWait){
                        drive.resetPath();
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case STOP:
                    output = drive.goToPosition(Stop, 0.6, 1, 0.6, 0.1);
                    break;

            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("pos reached?", drive.drivePID.pos_reached);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("intake Index", IntakeIndex);
            telemetry.addData("outtake vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("Gate Open Timer", GateOpenTimer.seconds());
            telemetry.update();




        }
    }
}



