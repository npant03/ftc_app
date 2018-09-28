package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by nehapant on 6/24/18.
 */

@Autonomous(name="RedFar85Autonomous", group="Team5214")

public class RedFar85Autonomous extends RedAutonomous{

    @Override
    protected void getToLeftDumpPosition(){
        straightWithEncoder(.5, -24);
        leftDump.setPosition(.61);
        strafeWithEncoder(.5, 4);
        turnLeftDegrees(38, parameters);
    }


    @Override
    protected void getToCenterDumpPosition(){
        straightWithEncoder(.55, -24);
        leftDump.setPosition(.61);
        turnLeftDegrees(35, parameters);
    }

    @Override
    protected void getToRightDumpPosition(){
        straightWithEncoder(.5, -24);
        leftDump.setPosition(.61);
        turnLeftDegrees(12, parameters);
    }

    @Override
    protected void getToDefaultDumpPosition(){
        getToLeftDumpPosition();
    }

    @Override

    public void runOpMode(){
        initialize();
        while(opModeIsActive()){
            Run();
        }
    }
}