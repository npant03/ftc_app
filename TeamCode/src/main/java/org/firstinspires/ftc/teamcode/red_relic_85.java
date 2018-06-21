package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@Autonomous(name = "red_relic_85", group = "Team5214")
//@Disabled
public class red_relic_85 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Autonomous auto = new Autonomous();
        auto.Run();
    }

    private class Autonomous {
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftBack;
        private DcMotor rightBack;
        private DcMotor leftFront;
        private DcMotor rightFront;

        private DcMotor liftMotor;
        private DcMotor relicMotor;

        private DcMotor lBelt;
        private DcMotor rBelt;

        private Servo colorServo;
        private Servo FLICKSERVO;


        private String colorid;
        // declare color sensor
        private ColorSensor colorFront;

        private Servo rightDump;
        private Servo leftDump;
        private Servo centerDump;
        private Servo wrist;
        private Servo finger;

        private Servo leftPush;
        private Servo rightPush;

        private int ticks;
        private int position2move2;
        // The IMU sensor object
        BNO055IMU imu;
        VuforiaLocalizer vuforia;
        VuforiaTrackables relicTrackables;
        VuforiaTrackable relicTemplate;
        BNO055IMU.Parameters parameters;

        //use the two variables in two color sensors situation
//    ColorSensor colorFront;
//    ColorSensor colorBack;

        final double currentRatio = 1.3; //ratio set for red/blue, for color id function

        // State used for updating telemetry
        Orientation angles;
        Orientation angles2;
        Acceleration gravity;

        private void printInitStatus() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

        }

        private void initCamera(){
            //using phone camera for image recognition
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            //License key, do not change
            parametersVu.vuforiaLicenseKey = "AVOFXuz/////AAAAGf45mZPfQEQBt1NyBSqlPuYQkVhLXgkSQpOqQqWb/FoWqJ" +
                    "WqG7KKeaIVeJzCSsLJ58FGWwE0Z/vvzSHrZBeZN9jN7c+gru1h0T3k0wLaoN1b6bFIHn93evRQ0DcFcgy4uMHZ1" +
                    "T87fT4WrKldfG6XT7PyThP2Fk5C8SbASqna7IKl26eb+zdOFXRKG+U1pZyV9yGgMsmBVZCxDZiT/G6JUpg4DMGrZVT" +
                    "8BXdVvs+mpDLQ4tH/XL5ikYp1++1fbYhJtA3naS5/laHiPiHONGAdLbHkE4s8EOxpB8+lqpJN6hlcqtMegarTOuwWYXXP" +
                    "jSnNnkUWBKuW6nWqtF3k1CIUoSTBuFpbwAvf+T6i1CkL6IoB";
            //using back camera for recognizing
            parametersVu.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

            //importing the three image asset and hook up to vuforia engine
            this.vuforia = ClassFactory.createVuforiaLocalizer(parametersVu);

            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        }

        private void hardwareMapping(){
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).

            leftBack = hardwareMap.get(DcMotor.class, "LB");
            rightBack = hardwareMap.get(DcMotor.class, "RB");
            leftFront = hardwareMap.get(DcMotor.class, "LF");
            rightFront = hardwareMap.get(DcMotor.class, "RF");

            lBelt = hardwareMap.dcMotor.get("LBELT");
            rBelt = hardwareMap.dcMotor.get("RBELT");

            liftMotor = hardwareMap.dcMotor.get("LIFT");
            relicMotor = hardwareMap.dcMotor.get("RELICMOTOR");

            centerDump = hardwareMap.servo.get("CD");
            rightDump = hardwareMap.servo.get("RD");
            leftDump = hardwareMap.servo.get("LD");
            colorServo = hardwareMap.servo.get("COLORSERVO");
            FLICKSERVO = hardwareMap.servo.get("FLICKSERVO");
            wrist = hardwareMap.servo.get("WRIST");
            finger = hardwareMap.servo.get("FINGER");

            leftPush = hardwareMap.servo.get("LPUSH");
            rightPush = hardwareMap.servo.get("RPUSH");

            parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


            imu = hardwareMap.get(BNO055IMU.class, "GYRO");
            colorFront = hardwareMap.get(ColorSensor.class, "CSF");
            imu.initialize(parameters);
        }

        public void Run() {
            printInitStatus();
            initCamera();
            hardwareMapping();



            //drive motor directions
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FLICKSERVO.setPosition(.5);
            centerDump.setPosition(.33);
            colorServo.setPosition(.68);
            // leftDump.setPosition(.61);

            leftPush.setPosition(.5);
            rightPush.setPosition(.5);

            composeTelemetry();

            //testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Wait for the game to start (driver presses PLAY)

            waitForStart();
            double start = System.currentTimeMillis();
            relicTrackables.activate();
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double end = System.currentTimeMillis();
            Double result = (end - start) / 1000;
            telemetry.addData("here is the time guysssssss: ", result);

            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                arm(.15); // put arm down

                UtilityFunctions.sleep(1000);
                colorid = checkColor(colorFront, currentRatio);

                telemetry.addLine(colorid);
                telemetry.update();

                if (colorid == "RED") {
                    FLICKSERVO(0.2);
                } else if (checkColor(colorFront, .4) == "BLUE") {
                    FLICKSERVO(.8);
                }

                UtilityFunctions.sleep(300);
                FLICKSERVO.setPosition(.5);

                arm(.68); // put arm up
                wrist.setPosition(1);
                leftPush.setPosition(.55);
                rightPush.setPosition(.55);

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                UtilityFunctions.sleep(1400);

                telemetry.addLine(vuMark.toString());
                telemetry.update();

                String keyResult = vuMark.toString();


                if (keyResult == "LEFT") {

                    telemetry.addLine("I'm going left");
                    telemetry.update();

                    straightWithEncoder(.55, -31);
                    leftDump.setPosition(.61);
                    turnRightDegrees(53, parameters);

                    //DROP THE INTAKE RAMP

                    leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //POSITIONS THE ROBOT AND SERVO TO DUMP AND RETRACT THE DUMPER AFTER
                    leftPush.setPosition(.5);
                    rightPush.setPosition(.5);

                    straightWithEncoder(.5, -1);

                    UtilityFunctions.sleep(200);

                    centerDump.setPosition(.8);
                    leftDump.setPosition(.18);

                    UtilityFunctions.sleep(700);

                    leftDump.setPosition(0.71);

                    //PUSHES THE CUBE AND PARKS

                    straightWithEncoder(.5, -10);

                    straightWithEncoder(.45, 5);

                    straightWithEncoder(.45, -4);

                    straightWithEncoder(.45, 3);


                } else if (keyResult == "CENTER") {

                    telemetry.addLine("I'm going in the middle");
                    telemetry.update();


                    straightWithEncoder(.5, -24);
                    leftDump.setPosition(.61);
                    turnRightDegrees(55, parameters);

                    //DROP THE INTAKE RAMP

                    leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //POSITIONS THE ROBOT AND SERVO TO DUMP AND RETRACT THE DUMPER AFTER
                    leftPush.setPosition(.5);
                    rightPush.setPosition(.5);

                    straightWithEncoder(.5, -1);

                    UtilityFunctions.sleep(200);

                    centerDump.setPosition(.8);
                    leftDump.setPosition(.18);

                    UtilityFunctions.sleep(700);

                    leftDump.setPosition(0.71);

                    //PUSHES THE CUBE AND PARKS

                    straightWithEncoder(.5, -10);

                    straightWithEncoder(.45, 3);

                    straightWithEncoder(.45, -4);

                    straightWithEncoder(.45, 3);


                } else if (keyResult == "RIGHT") {

                    telemetry.addLine("I'm going right");
                    telemetry.update();


                    straightWithEncoder(.5, -35);
                    leftDump.setPosition(.61);
                    turnRightDegrees(105, parameters);

                    //DROP THE INTAKE RAMP

                    leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //POSITIONS THE ROBOT AND SERVO TO DUMP AND RETRACT THE DUMPER AFTER
                    leftPush.setPosition(.5);
                    rightPush.setPosition(.5);

                    straightWithEncoder(.5, -1);

                    UtilityFunctions.sleep(200);

                    centerDump.setPosition(.8);
                    leftDump.setPosition(.18);

                    UtilityFunctions.sleep(700);

                    leftDump.setPosition(0.71);

                    //PUSHES THE CUBE AND PARKS

                    straightWithEncoder(.5, -10);

                    straightWithEncoder(.45, 3);

                    straightWithEncoder(.45, -4);

                    straightWithEncoder(.45, 3);


                } else

                {
                    telemetry.addLine("I'm defaulting to middle");
                    telemetry.update();


                    straightWithEncoder(.5, -24);
                    leftDump.setPosition(.61);
                    turnRightDegrees(55, parameters);

                    //DROP THE INTAKE RAMP

                    leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //POSITIONS THE ROBOT AND SERVO TO DUMP AND RETRACT THE DUMPER AFTER
                    leftPush.setPosition(.5);
                    rightPush.setPosition(.5);

                    straightWithEncoder(.5, -1);

                    UtilityFunctions.sleep(200);

                    centerDump.setPosition(.8);
                    leftDump.setPosition(.18);

                    UtilityFunctions.sleep(700);

                    leftDump.setPosition(0.71);

                    //PUSHES THE CUBE AND PARKS

                    straightWithEncoder(.5, -10);

                    straightWithEncoder(.45, 3);

                    straightWithEncoder(.45, -4);

                    straightWithEncoder(.45, 3);

                }


                telemetry.update();

                leftBack.setPower(0);
                rightBack.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);

                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                idle();
                break;
            }
        }


        //always keep strength positive, use negative inches to go backwards
        private void straightWithEncoder(double strength, int straightInches) {


            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            UtilityFunctions.motorWithEncoder(leftBack, strength, straightInches);
            UtilityFunctions.motorWithEncoder(leftFront, strength, straightInches);
            UtilityFunctions.motorWithEncoder(rightBack, strength, straightInches);
            UtilityFunctions.motorWithEncoder(rightFront, strength, straightInches);

            while (leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {
            }

            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            //reset encoder values
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //put the motors back into a mode where they can run
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        //defined so that power is positive always, right is positive inches, left is negative inches
        private void strafeWithEncoder(double unlimitedpower, int strafeInches) {

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            UtilityFunctions.motorWithEncoder(leftBack, unlimitedpower, -strafeInches);
            UtilityFunctions.motorWithEncoder(leftFront, unlimitedpower, strafeInches);
            UtilityFunctions.motorWithEncoder(rightBack, unlimitedpower, strafeInches);
            UtilityFunctions.motorWithEncoder(rightFront, unlimitedpower, -strafeInches);

            while (leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {
            }

            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        //----------------------------------------------------------------------------------------------
        // Telemetry Configuration
        //----------------------------------------------------------------------------------------------

        void composeTelemetry() {

            // At the beginning of each telemetry update, grab a bunch of data
            // from the IMU that we will then display in separate lines.
            telemetry.addAction(new Runnable() {
                @Override
                public void run() {
                    // Acquiring the angles is relatively expensive; we don't want
                    // to do that in each of the three items that need that info, as that's
                    // three times the necessary expense.
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                }
            });

            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override
                        public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override
                        public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel * gravity.xAccel
                                            + gravity.yAccel * gravity.yAccel
                                            + gravity.zAccel * gravity.zAccel));
                        }
                    });
        }


        private void turn(double power) {
            //left turn is positive power
            leftBack.setPower(-power); //sets left wheels to move backward
            leftFront.setPower(-power);
            rightBack.setPower(power); // makes right hand wheels to move forward
            rightFront.setPower(power);

            //makes the robot go forward for an indefinite amount of time

        }

        private void turnLeftDegress(double deg, BNO055IMU.Parameters parametersMeth) {


            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Orientation agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double curent = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
            double start = curent;
            double stDeg = curent + deg;

            //this loop runs until the robot has turned the correct amount
            while (((curent) < (stDeg - 1.5)) || (curent > (stDeg + 1.5))) {
                telemetry.update();

                //prints all the variables
                telemetry.addLine("IM IN THE WHILE");
                telemetry.addLine("start: " + Double.toString(start));
                telemetry.addLine("stDeg: " + Double.toString(stDeg));
                telemetry.addLine("deg: " + Double.toString(deg));
                telemetry.addLine("current: " + Double.toString(curent));

                turn(.28);

                agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                curent = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
                telemetry.update();
            }

            telemetry.addLine("I LEFT THE WHILE");
            telemetry.update();

            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            imu.initialize(parametersMeth);

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        private void turnRightDegrees(double deg, BNO055IMU.Parameters parametersMeth) {
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Orientation agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double curent = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
            double start = curent;
            double stDeg = curent + deg;

            //this loop runs until the robot has turned the correct amount
            while (((-curent) < (stDeg - 1.5)) || (-curent > (stDeg + 1.5))) {
                telemetry.update();

                //prints all the variables
                telemetry.addLine("IM IN THE WHILE");
                telemetry.addLine("start: " + Double.toString(start));
                telemetry.addLine("stDeg: " + Double.toString(stDeg));
                telemetry.addLine("deg: " + Double.toString(deg));
                telemetry.addLine("current: " + Double.toString(curent));

                turn(-.28);

                agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                curent = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
                telemetry.update();
            }

            telemetry.addLine("I LEFT THE WHILE");
            telemetry.update();

            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            imu.initialize(parametersMeth);

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        private void dump(double left, double right) {
            //setting the two dump servo to an input value
            leftDump.setPosition(left);
            // rightDump.setPosition(right);
        }

        private void FLICKSERVO(double position) {
            //setting the FLICKSERVO servo to an input value
            FLICKSERVO.setPosition(position);
            sleep(2000);
            FLICKSERVO.setPosition(0.5);

        }

        private void FullDump() {


        }


        private void arm(double position) {
            //setting the color servo to an input value
            colorServo.setPosition(position);
        }

        private void rampDown() {
            leftDump.setPosition(.5);
            rightDump.setPosition(.5);

            leftDump.setPosition(.8);
            rightDump.setPosition(.4);

            sleep(3000);

            leftDump.setPosition(.5);
            rightDump.setPosition(.5);
        }



        private void intake(DcMotor leftIntake, DcMotor rightIntake, String status) {
            leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            switch (status) {
                case "IN":
                    leftIntake.setPower(1);
                    rightIntake.setPower(-1);
                    break;
                case "OUT":
                    leftIntake.setPower(-1);
                    rightIntake.setPower(1);
                    break;
                case "OFF":
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    break;
                default:
                    telemetry.addLine("ehhhhh i think you didnt write the correct status");
                    break;
            }
        }

        private String checkColor(ColorSensor sensor, double ratio) {
            double redOverBlue = (sensor.red() + 1) / (sensor.blue() + 1);
            if (redOverBlue >= ratio) {
                //if it is greater than ratio, it is red
                return "RED";
            } else if (redOverBlue <= ratio) {
                //if it is less than ratio, it is blue
                return "BLUE";
            } else {
                //if nothing is detected, return not defined
                return "UNDEF";
            }
        }

        public String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        public String formatDegrees(double degrees) {
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }

        public String valueHead() {
            return formatAngle(angles.angleUnit, angles.firstAngle);
        }
    }
}