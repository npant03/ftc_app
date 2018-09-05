package org.firstinspires.ftc.teamcode;

/**
 * Created by nehapant on 6/24/18.
 */



abstract public class RedAutonomous extends RelicRecoveryAutonomous{

    @Override
    protected void knockBall(){
        knockBlueBall();

    }

    protected void knockBlueBall(){
        arm(.15); // put arm down

        UtilityFunctions.sleep(1000);
        colorid = redOrBlue(colorFront, currentRatio);

        printOnPhone(colorid);

        if (colorid == "RED") {
            FLICKSERVO(0.2);
        } else if (redOrBlue(colorFront, .4) == "BLUE") {
            FLICKSERVO(.8);
        }

        UtilityFunctions.sleep(300);
        FLICKSERVO.setPosition(.5);

        arm(.68); // put arm up

    }

}
