package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp
public class PinpointTest2 extends GorillabotCentral {
    double drive_factor = 1;
    double heading_factor = 1;
    double strafe_factor = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();

      //  drive.pinpoint.resetPosAndIMU();
        Pose2d init_pos = new Pose2d(64.75,-18.6, Math.toRadians(180));

        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));
        Pose2d curpos = new Pose2d(64.75,-18.6, Math.toRadians(180));

        waitForStart();
        while(!isStopRequested()){
            //curpos = new Pose2d( curpos.getX() + pinpoint.getPosX(DistanceUnit.INCH),  curpos.getY() + pinpoint.getPosY(DistanceUnit.INCH), curpos.getHeading() + pinpoint.getHeading(AngleUnit.RADIANS));

            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());

            drive.setDrivePower(g1.getDrivePower().scale(drive_factor).scaleHeading(heading_factor).scaleX(strafe_factor));
            drive.pinpoint.update();

            updateComponents();
            updateControllers();

            telemetry.addData("current Position", curpos);
            telemetry.addData("Heading", drive.pinpoint.getHeading(AngleUnit.RADIANS));
            telemetry.update();


        }


    }
}
