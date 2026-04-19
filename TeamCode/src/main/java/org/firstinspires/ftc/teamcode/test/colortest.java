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


        DigitalChannel led1;
        led1 = hardwareMap.get(DigitalChannel.class, "led1");
        led1.setMode(DigitalChannel.Mode.OUTPUT);

        DigitalChannel led2;
        led2 = hardwareMap.get(DigitalChannel.class, "led2");
        led2.setMode(DigitalChannel.Mode.OUTPUT);

        DigitalChannel led3;
        led3 = hardwareMap.get(DigitalChannel.class, "led3");
        led3.setMode(DigitalChannel.Mode.OUTPUT);

        waitForStart();
        initializeComponents();
        while (!isStopRequested()){

            Colors.DetectedColor slot1 = colors.getColor1();
            Colors.DetectedColor slot2 = colors.getColor2();
            Colors.DetectedColor slot3 = colors.getColor3();

            if(slot1 == Colors.DetectedColor.BALL){
                led1.setState(false);
            }
            else if(slot1 == Colors.DetectedColor.EMPTY){
                led1.setState(true);
            }

            if(slot2 == Colors.DetectedColor.BALL){
                led2.setState(false);
            }
            else if(slot2 == Colors.DetectedColor.EMPTY){
                led2.setState(true);
            }

            if(slot3 == Colors.DetectedColor.BALL){
                led3.setState(false);
            }
            else if(slot3 == Colors.DetectedColor.EMPTY){
                led3.setState(true);
            }





            telemetry.addData("sensor 1", colors.getColor1());
            telemetry.addData("sensor 2", colors.getColor2());
            telemetry.addData("sensor 3", colors.getColor3());
            telemetry.addData("red", colors.color1.red());
            telemetry.update();

        }



    }
}
