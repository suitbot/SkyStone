package org.firstinspires.ftc.suitbots;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name= "Test Auto")
public class NewAuto extends LinearOpMode {
    public DcMotor lf, rf, lr, rr;
    public DcMotor lintake, rintake, lift;
    public ColorSensor sensorColor;
    public RangeSensor backRange, leftRange, rightRange;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;


    //for drive convertLiftHeightToEncoderTicks strafing
    public static boolean FORWARD = true;
    public static boolean BACKWARD = false;
    public static boolean turnLeft = true;
    public static boolean turnRight = false;
    //for hook
    public static boolean UP = true;
    public static boolean DOWN = false;
    //for intake
    public static boolean IN = true;
    public static boolean OUT = false;
    public static boolean colourIsRed = true;



    //for drive convertLiftHeightToEncoderTicks strafing
    public final static int driveLeft = 0;
    public final static int driveRight = 1;
    public final static int forward = 2;
    public final static int back = 3;

    public static int position = 5;
    public final static int quarrySide = 0;
    public final static int buildingSiteSide = 1;
    public final static int dumb = 2;
    //initial pause
    public static long pause = 0;

    public final static double PI = Math.PI;
    public final static double TICKS_PER_ROT = 1480.59;
    public final static double GEAR_RATIO = 1;
    public final static double WHEEL_DIAMETER = 8;
    public int inches(int input){
        return (int) (input * TICKS_PER_ROT * GEAR_RATIO/ (WHEEL_DIAMETER * PI));
    }


    public int debugStep = 0;

    //button once
    public boolean buttonOnce(String button){
        switch (button){
            case "dpad_up":
                while(gamepad1.dpad_up) {
                    idle();
                }
                break;
            case "dpad_down":
                while(gamepad1.dpad_down) {
                    idle();
                }
                break;
            case "start":
                while(gamepad1.start) {
                    idle();
                }
                break;

            default:
                break;
        }
        return true;
    }

    public void driveSeconds(double lfpwr, double lrpwr, double rfpwr, double rrpwr, long time){
        setMotorPower(lfpwr, lrpwr, rfpwr, rrpwr);
        long intiTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < time + intiTime && opModeIsActive()){
            idle();
        }
        setMotorPower(0);
        runUsingEnc();
    }
    public void harvesterDeposit(){
        intake(OUT,500);
    }
    public enum Direction {
        LEFT,
        RIGHT,
        FWD,
        BACK
    }
    public void setMotorPower(double lfpwr, double lrpwr, double rfpwr, double rrpwr){
        lf.setPower(lfpwr);
        rf.setPower(rfpwr);
        lr.setPower(lrpwr);
        rr.setPower(rrpwr);
    }
    public void setMotorPower(double pwr){
        setMotorPower(pwr,pwr,pwr,pwr);
    }
    public void setMotorPower(double power, DcMotor ...motors){
        for(DcMotor i : motors){
            i.setPower(power);
        }
    }
    public void setMotorPower(Direction direction, double power){
        switch(direction){
            case LEFT:
                setMotorPower(-power, power, power, -power);
                break;
            case RIGHT:
                setMotorPower(power, -power, -power, power);
                break;
            case FWD:
                setMotorPower(power);
                break;
            case BACK:
                setMotorPower(-power);
                break;
        }
    }
    public void stpResetEncoders(){
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runToPosition(){
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runUsingEnc(){
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setTarPos(int lfpos, int rfpos, int lrpos, int rrpos){
        lf.setTargetPosition(lfpos);
        rf.setTargetPosition(rfpos);
        lr.setTargetPosition(lrpos);
        rr.setTargetPosition(rrpos);
    }
    public void setTarPos(int pos){
        setTarPos(pos, pos, pos, pos);
    }
    public void driveInches(int dist, int dir){
        telemetry.addData("Doing: ", "driveInches");
        telemetry.update();
        int actDist = (dir == forward) ? inches(dist) : (int)(inches(dist) * (1.333333));
        stpResetEncoders();
        switch (dir){
            case driveLeft:
                setTarPos(-actDist, actDist, actDist, -actDist);
                runToPosition();
                setMotorPower(0.5);
                break;
            case driveRight:
                setTarPos(actDist, -actDist, -actDist, actDist);
                runToPosition();
                setMotorPower(0.5);
                break;
            case forward:
                setTarPos(actDist);
                runToPosition();
                setMotorPower(0.5);
                break;
        }
        while(lf.isBusy() && lr.isBusy() && rf.isBusy() && rr.isBusy() && opModeIsActive()){
            idle();
        }
        runUsingEnc();
        setMotorPower(0);
        sleep(500);
    }
    public void driveInches(int dist){
        this.driveInches(dist, forward);
    }
    public void driveInchesHarvesting(int dist, boolean willPassiveIntake) {
        intake(IN);
        stpResetEncoders();
        setTarPos(inches(dist));
        runToPosition();
        setMotorPower(0.3);
        while(lf.isBusy() && lr.isBusy() && rf.isBusy() && rr.isBusy() && opModeIsActive()){
            idle();
        }
        runUsingEnc();
        setMotorPower(0);
        passiveIntake(willPassiveIntake);
        sleep(500);
    }

    public void driveInchesKeepAngle(int dist, int dir){
        double startAngle = getAngle();
        telemetry.addData("Doing: ", "driveInches");
        telemetry.update();
        int actDist = (dir == forward) ? inches(dist) : (int)(inches(dist) * (1.333333));
        stpResetEncoders();
        switch (dir){
            case driveLeft:
                setTarPos(-actDist, actDist, actDist, -actDist);
                runToPosition();
                setMotorPower(0.5);
                break;
            case driveRight:
                setTarPos(actDist, -actDist, -actDist, actDist);
                runToPosition();
                setMotorPower(0.5);
                break;
            case forward:
                setTarPos(actDist);
                runToPosition();
                setMotorPower(0.8);
                break;
        }
        while(lf.isBusy() && lr.isBusy() && rf.isBusy() && rr.isBusy() && opModeIsActive()){
            if((getAngle() != startAngle) && opModeIsActive()) {
                if(getAngle() > startAngle) {
                    turnDeg(Direction.LEFT, -5);
                }
                else if(getAngle() < startAngle) {
                    turnDeg(Direction.RIGHT, -5);
                }
            }
            idle();
        }
        runUsingEnc();
        setMotorPower(0);
        sleep(500);

    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public void turnDeg(Direction direction, int deg) {
        telemetry.addData("Doing", "turnDeg");
        telemetry.update();
        resetAngle();

        double lpwr = direction == Direction.LEFT ? -0.35 : 0.35;
        double rpwr = -lpwr;
        setMotorPower(lpwr, lpwr, rpwr, rpwr);

        while((Math.abs(getAngle()) < Math.abs(deg) - 13) && opModeIsActive()){
            telemetry.addData("Angle:", getAngle());
            telemetry.update();
            idle();
        }
        lpwr /= 1.5;
        rpwr /= 1.5;
        setMotorPower(lpwr, lpwr, rpwr, rpwr);
        while(Math.abs(getAngle()) < 15 && opModeIsActive()){
            telemetry.addData("Angle 2:", getAngle());
            telemetry.update();
            idle();
        }

        runUsingEnc();
        setMotorPower(0);
        sleep(500);
    }


    public void intake(boolean in){
        lintake.setPower(in ? -0.8 : 0.4);
        rintake.setPower(in ? -0.8 : 0.4);
    }
    public void intake(boolean inOut, long time){
        this.intake(inOut);
        sleep(time);
        lintake.setPower(0);
        rintake.setPower(0);
    }
    public void passiveIntake(boolean on){
        lintake.setPower(on ? -0.1 : 0);
        rintake.setPower(on ? -0.1 : 0);
    }

    double getAlpha() {return sensorColor.alpha();}


    public void debugStop(){
        debugStep +=1;
        telemetry.addData("Debug step", debugStep);
        telemetry.update();
        while(!gamepad1.a&& opModeIsActive()){

        }
    }
    private double last = 0.0;
    private boolean lastInf = false;
    public double getReading(ModernRoboticsI2cRangeSensor raw) {
        double val = raw.getDistance(DistanceUnit.CM);
        if (val < 240.0) {
            last = val;
            lastInf = false;
        } else {
            lastInf = true;
        }
        return last;
    }

    public void driveTillCM(double distance, Direction direction, RangeSensor range) {
        /*RangeSensor range;
        switch (direction){
            case LEFT:
                range = leftRange;
                break;
            case RIGHT:
                range = rightRange;
                break;
            default:
                range = backRange;
                break;
        }*/
        setMotorPower(direction, 0.2);
        telemetry.addData("Range", range.getReading());
        telemetry.update();
        sleep(50);
        telemetry.addData("Range", range.getReading());
        telemetry.update();
        sleep(50);
        telemetry.addData("Range", range.getReading());
        telemetry.update();
        sleep(750);
        while (opModeIsActive() && range.getReading() > distance) {
            telemetry.addData("Range", range.getReading());
            telemetry.update();
            idle();
        }
        setMotorPower(0.0);
        sleep(700);
    }

    public void alignWithBlock(int setUp, int whichBlock, boolean isRed){
        switch (setUp){
            case 1:
                switch (whichBlock){
                    case 1:
                        driveInches(3);
                        //driveTillCM(50, isRed ? Direction.LEFT : Direction.RIGHT);
                        break;
                    case 2:
                        //driveTillCM(100, isRed ? Direction.LEFT : Direction.RIGHT);
                        break;
                }
        }
    }
    public void driveToLine(double avg,boolean direction){
        if(direction==FORWARD)
        setMotorPower(.2);
        else
            setMotorPower(-.2);
       while(sensorColor.alpha()<=avg*1.23)
           idle();
       setMotorPower(0.0);
    }
    public int convertLiftHeightToEncoderTicks(double liftHeightInInches){
        return (int) (liftHeightInInches * TICKS_PER_ROT * GEAR_RATIO / (WHEEL_DIAMETER * PI));
    }
    public void liftToIndex(int index){
        double[] heights = {0, 1.65, 2.8, 7, 10.8};
        int level = 0;
        lift.setTargetPosition(convertLiftHeightToEncoderTicks(heights[level]));
        lift.setPower(0.5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && lf.isBusy() || lr.isBusy() || rf.isBusy() || rr.isBusy()){
            idle();
        }
      //  sleep(420);
    }
    //Here is the problem ^

    @Override
    public void runOpMode(){

        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");

        lintake = hardwareMap.dcMotor.get("lintake");
        rintake = hardwareMap.dcMotor.get("rintake");
        lift = hardwareMap.dcMotor.get("lift");


        backRange = new RangeSensor(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backRange"));
        leftRange = new RangeSensor(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange"));
        rightRange = new RangeSensor(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange"));


        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        rintake.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        sensorColor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "sensorColor");

        imu.initialize(parameters);


        /*
        Stupid small brAOIN haredwaere developers ahsve no idea whats thapeening to the robot twhikle the software is w0orking comkpletely correc tly like
        who the frick fdoesnt know how the sytstem works, it sos stupid sand smallbrain vut at lwasrt we know that there is no possible way that it
        could have been a software issure. myt liovecrearfttanb story worked completely well so theres that goijng for me which is nice and rfriendly and
        good. They must remember that the robot CANNOT, under any circumstances, staryt tilted.
         */

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        double avg = 0.0;
        int count = 0;

        for(int i=0; i<300; i++) {
            final double a = getAlpha();
            avg += a;

            telemetry.addData("Alpha", a);
            telemetry.addData("Total", avg);
            telemetry.addData("Count", i);
            telemetry.addData("Average", avg / i);
            telemetry.update();
            count = i;
        }
        double avgClr = avg/((double) count);

        /*
        while(!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }*/
        telemetry.addData("We aight", "");
        telemetry.update();

        telemetry.addData("Color ","(x blue, b red)");
        telemetry.update();
        while(!(gamepad1.x || gamepad1.b) && !opModeIsActive()){
            if(gamepad1.b){
                colourIsRed = true;
            }
            else if(gamepad1.x){
                colourIsRed = false;
            }
        }
        telemetry.addData("Position", "(y build site, a quarry, dpad_left dumb)");
        telemetry.update();
        while(!(gamepad1.y || gamepad1.a || gamepad1.dpad_left) && !opModeIsActive()){
            if (gamepad1.y){
                position = buildingSiteSide;
            }
            else if(gamepad1.a){
                position = quarrySide;
            }
            else if(gamepad1.dpad_left){
                position = dumb;
            }
        }
        while (!(gamepad1.b) && !opModeIsActive()){
            if (!buttonOnce("dpad_up")){
                pause += 500;
            } else if(!buttonOnce("dpad_down")){
                pause -= 500;
            }
            telemetry.addData("Pause", "up +0.5 sec, down -0.5 sec");
            telemetry.addData("Exit", "b");
            telemetry.addData("Pause", pause);
            telemetry.update();
        }



        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();//-------------------------------------------------------
        telemetry.addData("Mode", "running");
        telemetry.update();

        //sleep(pause);

        ///*
        if(position == buildingSiteSide){
            //lift
            liftToIndex(1);
            //drive to platform
            driveInches(24);
            //hook onto platform
            liftToIndex(0);
            //drive back with platform
            driveInches(-25);
            //unhook
            liftToIndex(1);
            //start strafing over to the bridge
            driveInches(15, colourIsRed ? driveLeft : driveRight);
            //drop lift to fit under bridge
            liftToIndex(0);
            //drive under the bridge
            driveInches(15, colourIsRed ? driveLeft : driveRight);

            //stop all motors
            setMotorPower(0, lf, lr, rf, rr, lintake, rintake, lift);
        }
        else if(position == quarrySide){
            ///*
            int setUp = 1;
            //pick up the first stone
            driveInchesHarvesting(45, true);
            //back up to get under the bridge
            driveInches(-20);
                //driveTillCM(30, Direction.BACK, backRange);
            //turn to face bridge
            turnDeg(colourIsRed ? Direction.RIGHT : Direction.LEFT, 90);
            //drive under bridge and deposit
            driveInches(35);
            //driveToLine(avgClr,FORWARD);
            passiveIntake(false);
            harvesterDeposit();
            //drive back to align with second block
            driveInches(-30);
            driveInches(-15);
                //driveTillCM(55, Direction.BACK, backRange);
            //face block
            turnDeg(colourIsRed ? Direction.LEFT : Direction.RIGHT, 90);
            //pick up second block
            driveInchesHarvesting(25, true);
            //back up to drive under bridge
            driveInches(-20);
                //driveTillCM(30, Direction.BACK, backRange);
            //turn to face bridge
            turnDeg(colourIsRed ? Direction.RIGHT : Direction.LEFT, 90);
            //drive under bridge and deposit brick
            driveInches(55);
                //driveToLine(avgClr,FORWARD);
            harvesterDeposit();
            //park on line
            driveInches(-20);
            //driveToLine(avgClr,BACKWARD);

            //stop all motors
            setMotorPower(0, lf, lr, rf, rr, lintake, rintake, lift);
            passiveIntake(false);
        }
        else if(position == dumb){
            liftToIndex(0);
            liftToIndex(1);
            //driveInches(30);
        }
    }
}