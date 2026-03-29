package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Colors;

import org.firstinspires.ftc.teamcode.GorillabotCentral;


@TeleOp
@Config
public class colortest extends GorillabotCentral {

    @Override
    public void runOpMode() throws InterruptedException {

        double artifact_detected = 0;
        Colors.DetectedColor prev1 = Colors.DetectedColor.EMPTY;
        Colors.DetectedColor prev2 = Colors.DetectedColor.EMPTY;
        Colors.DetectedColor prev3 = Colors.DetectedColor.EMPTY;


        waitForStart();
        initializeComponents();
        while (!isStopRequested()){

            Colors.DetectedColor slot1 = colors.getColor1();
            Colors.DetectedColor slot2 = colors.getColor2();
            Colors.DetectedColor slot3 = colors.getColor3();


            if(prev1 == Colors.DetectedColor.EMPTY && slot1 == Colors.DetectedColor.BALL) {
                artifact_detected++;
                prev1 = slot1;
            }

            if(prev2 == Colors.DetectedColor.EMPTY && slot2 == Colors.DetectedColor.BALL){
                artifact_detected ++;
                prev2 = slot2;
            }

            if(prev3 == Colors.DetectedColor.EMPTY && slot3 == Colors.DetectedColor.BALL){
                artifact_detected ++;
                prev3 = slot3;
            }

            if(prev1 == Colors.DetectedColor.BALL && slot1 == Colors.DetectedColor.EMPTY) {
                artifact_detected--;
                prev1 = slot1;
            }

            if(prev2 == Colors.DetectedColor.BALL && slot2 == Colors.DetectedColor.EMPTY){
                artifact_detected --;
                prev2 = slot2;
            }

            if(prev3 == Colors.DetectedColor.BALL && slot3 == Colors.DetectedColor.EMPTY){
                artifact_detected --;
                prev3 = slot3;
            }




            telemetry.addData("artifacts", artifact_detected);
            telemetry.addData("sensor 1", colors.getColor1());
            telemetry.addData("sensor 2", colors.getColor2());
            telemetry.addData("sensor 3", colors.getColor3());
            telemetry.addData("red", colors.color1.red());
            telemetry.addData("blue", colors.color1.blue());
            telemetry.addData("green", colors.color1.green());
            telemetry.update();

        }






    }
}