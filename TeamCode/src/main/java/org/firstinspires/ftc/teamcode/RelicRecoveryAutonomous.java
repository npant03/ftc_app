package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

//@Autonomous(name = "RelicRecoveryAutonomous", group = "Team5214")

abstract public class RelicRecoveryAutonomous extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftBack;
    protected DcMotor rightBack;
    protected DcMotor leftFront;
    protected DcMotor rightFront;

    protected DcMotor liftMotor;
    protected DcMotor relicMotor;

    protected DcMotor lBelt;
    protected DcMotor rBelt;

    protected Servo colorServo;
    protected Servo FLICKSERVO;

    protected String colorid;
    // declare color sensor
    //protected ColorSensor colorFront;

    protected Servo rightDump;
    protected Servo leftDump;
    protected Servo centerDump;
    protected Servo wrist;
    protected Servo finger;

    protected Servo leftPush;
    protected Servo rightPush;

    protected int ticks;
    protected int position2move2;
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


    @Override
    public void runOpMode() {
        this.Run();
    }

    protected void printInitStatus() {
        printOnPhone("Status Initialized");

    }

    protected void initCamera() {
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

    protected void hardwareMapping() {
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

    protected void setMotorDirections() {
        //drive motor directions
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runToPosition();
    }

    protected void physicalStartingSequence() {
        FLICKSERVO.setPosition(.5);
        centerDump.setPosition(.33);
        colorServo.setPosition(.68);

        leftPush.setPosition(.5);
        rightPush.setPosition(.5);
    }

    protected void activateCamera() {
        relicTrackables.activate();

    }

    protected void gyroSetup() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    abstract protected void knockBall();

    protected void deployIntake() {
        wrist.setPosition(1);
        leftPush.setPosition(.55);
        rightPush.setPosition(.55);
    }

    protected String readPicture() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        UtilityFunctions.sleep(1400);

        printOnPhone(vuMark.toString());
        String keyResult = vuMark.toString();
        return keyResult;
    }

    protected void dumpSequence() {
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

    protected void runWithoutEncoder() {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void retractIntakePushers() {
        leftPush.setPosition(.5);
        rightPush.setPosition(.5);
    }

    protected void printOnPhone(String caption) {
        telemetry.addLine(caption);
        telemetry.update();
    }

    protected void initialize() {
        printInitStatus();
        initCamera();
        hardwareMapping();
        setMotorDirections();
        physicalStartingSequence();

        composeTelemetry();
        waitForStart();
        activateCamera();
        gyroSetup();
        runtime.reset();
    }

    protected void killDriveTrainPower() {
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    protected void resetEncoders() {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    abstract protected void getToLeftDumpPosition();

    abstract protected void getToCenterDumpPosition();

    abstract protected void getToRightDumpPosition();

    abstract protected void getToDefaultDumpPosition();

    public void Run() {

        initialize();

        while (opModeIsActive()) {

            knockBall();
            deployIntake();
            String keyResult = readPicture();

            switch (keyResult) {

                case "LEFT":

                    printOnPhone("Im going left");

                    getToLeftDumpPosition();

                case "CENTER":

                    printOnPhone("Im going in the middle");

                    getToCenterDumpPosition();

                case "RIGHT":

                    printOnPhone("Im going right");

                    getToRightDumpPosition();

                default:

                    printOnPhone("Im defaulting to center");

                    getToDefaultDumpPosition();

            }

            runWithoutEncoder();

            retractIntakePushers();

            dumpSequence();

            telemetry.update();

            killDriveTrainPower();

            resetEncoders();


            idle();
            break;

        }
    }


    //always keep strength positive, use negative inches to go backwards
    protected void straightWithEncoder(double strength, int straightInches) {


        runToPosition();

        UtilityFunctions.motorWithEncoder(leftBack, strength, straightInches);
        UtilityFunctions.motorWithEncoder(leftFront, strength, straightInches);
        UtilityFunctions.motorWithEncoder(rightBack, strength, straightInches);
        UtilityFunctions.motorWithEncoder(rightFront, strength, straightInches);

        while (leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {
        }

        killDriveTrainPower();

        //reset encoder values
        resetEncoders();

        //put the motors back into a mode where they can run
        runToPosition();
    }


    //defined so that power is positive always, right is positive inches, left is negative inches
    protected void strafeWithEncoder(double unlimitedpower, int strafeInches) {

        runToPosition();

        UtilityFunctions.motorWithEncoder(leftBack, unlimitedpower, -strafeInches);
        UtilityFunctions.motorWithEncoder(leftFront, unlimitedpower, strafeInches);
        UtilityFunctions.motorWithEncoder(rightBack, unlimitedpower, strafeInches);
        UtilityFunctions.motorWithEncoder(rightFront, unlimitedpower, -strafeInches);

        while (leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {
        }

        killDriveTrainPower();

        resetEncoders();

        runToPosition();
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


    protected void turn(double power) {
        //left turn is positive power
        leftBack.setPower(-power); //sets left wheels to move backward
        leftFront.setPower(-power);
        rightBack.setPower(power); // makes right hand wheels to move forward
        rightFront.setPower(power);

        //makes the robot go forward for an indefinite amount of time

    }

    protected void turnLeftDegrees(double deg, BNO055IMU.Parameters parametersMeth) {


        runWithoutEncoder();

        Orientation agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
        double start = current;
        double stDeg = current + deg;

        //this loop runs until the robot has turned the correct amount
        while (((current) < (stDeg - 1.5)) || (current > (stDeg + 1.5))) {
            telemetry.update();

            //prints all the variables
            telemetry.addLine("IM IN THE WHILE");
            telemetry.addLine("start: " + Double.toString(start));
            telemetry.addLine("stDeg: " + Double.toString(stDeg));
            telemetry.addLine("deg: " + Double.toString(deg));
            telemetry.addLine("current: " + Double.toString(current));

            turn(.28);

            agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
            telemetry.update();
        }

        telemetry.addLine("I LEFT THE WHILE");
        telemetry.update();

        killDriveTrainPower();

        resetEncoders();

        imu.initialize(parametersMeth);

        runToPosition();
    }

    protected void turnRightDegrees(double deg, BNO055IMU.Parameters parametersMeth) {

        runWithoutEncoder();

        Orientation agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
        double start = current;
        double stDeg = current + deg;

        //this loop runs until the robot has turned the correct amount
        while (((-current) < (stDeg - 1.5)) || (-current > (stDeg + 1.5))) {
            telemetry.update();

            //prints all the variables
            printOnPhone("IM IN THE WHILE");
            printOnPhone("start: " + Double.toString(start));
            printOnPhone("stDeg: " + Double.toString(stDeg));
            printOnPhone("deg: " + Double.toString(deg));
            printOnPhone("current: " + Double.toString(current));

            turn(-.28);

            agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
            telemetry.update();
        }

        printOnPhone("I LEFT THE WHILE");

        killDriveTrainPower();

        resetEncoders();

        imu.initialize(parametersMeth);

        runToPosition();
    }

    protected void runToPosition() {
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void dump(double left, double right) {
        //setting the two dump servo to an input value
        leftDump.setPosition(left);
        // rightDump.setPosition(right);
    }

    protected void FLICKSERVO(double position) {
        //setting the FLICKSERVO servo to an input value
        FLICKSERVO.setPosition(position);
        sleep(2000);
        FLICKSERVO.setPosition(0.5);

    }


    protected void arm(double position) {
        //setting the color servo to an input value
        colorServo.setPosition(position);
    }

    protected void rampDown() {
        leftDump.setPosition(.5);
        rightDump.setPosition(.5);

        leftDump.setPosition(.8);
        rightDump.setPosition(.4);

        sleep(3000);

        leftDump.setPosition(.5);
        rightDump.setPosition(.5);
    }


    protected void intake(DcMotor leftIntake, DcMotor rightIntake, String status) {
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