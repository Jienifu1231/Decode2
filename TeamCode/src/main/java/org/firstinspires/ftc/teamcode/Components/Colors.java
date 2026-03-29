package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Colors {

   public ColorSensor color1, color2, color3;


    public enum DetectedColor {
        BALL,
        EMPTY

    } //each value range is stored, this is marker

    public void init(HardwareMap hardwareMap){
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color3 = hardwareMap.get(ColorSensor.class, "color3");
    }

    float[]color_values = new float[3]; //holds hue, saturation, value

    public DetectedColor detect(ColorSensor sensor) {

        int red = sensor.red();
        int blue = sensor.blue();
        int green = sensor.green();


        // Colors.RGBToHSV(red, green, blue, color_values);

        //float hue = color_values[0];
        //float saturation = color_values[1];


        if((0 < red && red < 40) || (0 < blue && blue < 48) || (0 < green && green < 70)){
            return DetectedColor.EMPTY;
        }

        else return DetectedColor.BALL;

    }


    public DetectedColor getColor1(){
        return detect(color1);
    }

    public DetectedColor getColor2(){
        return detect(color2);
    }

    public DetectedColor getColor3(){
        return detect(color3);
    }
}


//double total = red + green + blue;
//double normRed = red / total;
//double normGreen = green / total;
//double normBlue = blue / total; --- may be a more robust system