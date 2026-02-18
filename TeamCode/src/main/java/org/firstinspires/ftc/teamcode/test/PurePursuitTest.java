package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GorillabotCentral;

import java.util.ArrayList;

@Autonomous
@Config
public class PurePursuitTest extends GorillabotCentral {
    enum State{
        INIT, 
        PATH,
    }
    
    ArrayList Path = new ArrayList( )//chaneg this
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        
    }
}
