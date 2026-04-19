package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.Components.Colors;

import org.firstinspires.ftc.teamcode.GorillabotCentral;



@TeleOp
@Config
public class colortest extends GorillabotCentral {


    @Override
    public void runOpMode() throws InterruptedException {


        DigitalChannel led;
        led = hardwareMap.get(DigitalChannel.class, "led");
        led.setMode(DigitalChannel.Mode.OUTPUT);



        waitForStart();
        initializeComponents();
        while (!isStopRequested()){

            Colors.DetectedColor slot1 = colors.getColor1();
            Colors.DetectedColor slot2 = colors.getColor2();
            Colors.DetectedColor slot3 = colors.getColor3();

            if(slot1 == Colors.DetectedColor.BALL && slot2 == Colors.DetectedColor.BALL && slot3 == Colors.DetectedColor.BALL) {
                led.setState(false);
            }
            else led.setState(true);




            telemetry.addData("sensor 1", colors.getColor1());
            telemetry.addData("sensor 2", colors.getColor2());
            telemetry.addData("sensor 3", colors.getColor3());
            telemetry.addData("red", colors.color1.red());
            telemetry.update();

        }



    }
}
