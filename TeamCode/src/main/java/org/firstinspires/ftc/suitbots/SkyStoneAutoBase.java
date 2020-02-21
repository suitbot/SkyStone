package org.firstinspires.ftc.suitbots;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name= "Normal Auto")
public class SkyStoneAutoBase extends LinearOpMode {
    public double avg;
    public DcMotor lf, rf, lr, rr;
    public DcMotor lintake, rintake, lift;
    public CRServo rHook, lHook;
    public ColorSensor sensorColor;
    public ModernRoboticsI2cRangeSensor ultrasonicBack, ultrasonicLeft, ultrasonicRight;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle = .30;


    //for drive convertLiftHeightToEncoderTicks strafing
    public static boolean LEFT = true;
    public static boolean RIGHT = false;
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

    public final static double RHOOK_UP_POSITION = -1.5;//0.1
    public final static double RHOOK_DOWN_POSITION = 0.1;//-1.5

    public final static double LHOOK_UP_POSITION = 1.9;//0.1
    public final static double LHOOK_DOWN_POSITION = -.1;//1.9

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
    public void harvesterDown(){
        lift.setPower(-.3);
        sleep(300);
        lift.setPower(0);
    }
    public void setMotorPower(double lfpwr, double lrpwr, double rfpwr, double rrpwr){
        lf.setPower(lfpwr);
        rf.setPower(rfpwr);
        lr.setPower(lrpwr);
        rr.setPower(rrpwr);
    }
    public void setMotorPower(double pwr){
        this.setMotorPower(pwr,pwr,pwr,pwr);
    }
    public void turnSecond(boolean direction, long time){
        telemetry.addData("Second turn,", "");
        telemetry.update();
        if(direction == LEFT){
            driveSeconds(-1,-1, 1, 1, time);
        }
        else{
            driveSeconds(0.8, 0.8, -0.8, -0.8, time);
        }
        runUsingEnc();
        setMotorPower(0);

    }
    boolean FORWARD = true;
    boolean BACKWARD = false;
    public void driveToLine(boolean direction, double floorAlpha){
        if(direction==FORWARD)
            setMotorPower(.2);
        else
            setMotorPower(-.2);
        while (opModeIsActive() && getAlpha() <= (floorAlpha * 1.25)) {
            idle();
        }

        setMotorPower(.0);
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
    public void setTarPos(int pos){ setTarPos(pos, pos, pos, pos); }
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
                setMotorPower(0.8);
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
    public void driveInchesHarvesting(int dist) {
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
        lintake.setPower(0);
        rintake.setPower(0);
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
                    turnDeg(LEFT, -5);
                }
                else if(getAngle() < startAngle) {
                    turnDeg(RIGHT, -5);
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
    public void turnDeg(boolean dir, int deg){
        telemetry.addData("Doing", "turnDeg");
        telemetry.update();
        resetAngle();

        double lpwr = dir ? -0.4 : 0.4;
        double rpwr = -lpwr;
        setMotorPower(lpwr, lpwr, rpwr, rpwr);

        while((Math.abs(getAngle()) < Math.abs(deg) - 20) && opModeIsActive()){
            telemetry.addData("Angle:", getAngle());
            telemetry.update();
            idle();
        }
        lpwr /= 2;
        rpwr /= 2;
        setMotorPower(lpwr, lpwr, rpwr, rpwr);
        while(Math.abs(getAngle()) < 20 && opModeIsActive()){
            telemetry.addData("Angle 2:", getAngle());
            telemetry.update();
            idle();
        }

        runUsingEnc();
        setMotorPower(0);
        sleep(500);
    }
    public void driveUntilLine(double avg){

        while (opModeIsActive() && getAlpha() <= (avg * 1.25)) {
            telemetry.addData("average", avg);
            telemetry.addData("current", getAlpha());
            telemetry.update();
            idle();
        }
    }

    public void hook(boolean position){
        rHook.setPower(position ? RHOOK_UP_POSITION : RHOOK_DOWN_POSITION);
        lHook.setPower(position ? LHOOK_UP_POSITION : LHOOK_DOWN_POSITION);
        sleep(800);
    }

    public void intake(boolean inOut){
        //in is true
        lintake.setPower(!inOut ? -0.5 : .5);
        rintake.setPower(!inOut ? -0.5 : .5);
    }
    public void intake(boolean inOut, long time){
        this.intake(inOut);
        sleep(time);
        lintake.setPower(0);
        rintake.setPower(0);
    }

    double getAlpha() {return sensorColor.alpha();}
    public void preciseLocation(double desiredLeft, double desiredBack) {
        double deadZone = 1;
        //if this breaks, switch around the + and - signs
        while((desiredLeft+deadZone< ultrasonicLeft.rawUltrasonic() || ultrasonicLeft.rawUltrasonic() < desiredLeft-deadZone) || (desiredBack+deadZone > ultrasonicBack.rawUltrasonic() || (desiredBack-deadZone < ultrasonicBack.rawUltrasonic()))) {
            if (ultrasonicLeft.rawUltrasonic() > desiredLeft) {
                driveInches(1,driveLeft);
            }
            if (ultrasonicLeft.rawUltrasonic() < desiredLeft) {
                driveInches(1, driveRight);
            }
            if (ultrasonicBack.rawUltrasonic() > desiredBack) {
                driveInches(-1);
            }
            if (ultrasonicBack.rawUltrasonic() < desiredBack) {
                driveInches(1);
            }
        }
    }


    public void ultrasonicDriveUntilCM(int dist){
        double distt= ultrasonicBack.rawUltrasonic();
        setMotorPower(0.5);
        while (opModeIsActive() && !isStopRequested() && (distt)>(dist)) {
            double temp=0;
            for (int i = 1; i <= 7; i++){
                temp+= ultrasonicBack.rawUltrasonic();
            }
            distt+=temp/7;
            telemetry.addData("raw ultrasonicBack", ultrasonicBack.rawUltrasonic());
            telemetry.addData("raw optical", ultrasonicBack.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", ultrasonicBack.cmOptical());
            telemetry.addData("cm", "%.2f cm", ultrasonicBack.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        setMotorPower(-.6);
        sleep(100);
        setMotorPower(0);
    }

    public void debugStop(){
        debugStep +=1;
        telemetry.addData("Debug step", debugStep);
        telemetry.update();
        while(!gamepad1.a&& opModeIsActive()){

        }
    }

    @Override
    public void runOpMode(){

        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");

        lintake = hardwareMap.dcMotor.get("lintake");
        rintake = hardwareMap.dcMotor.get("rintake");
        lift = hardwareMap.dcMotor.get("lift");

        rHook = hardwareMap.crservo.get("rHook");
        lHook = hardwareMap.crservo.get("lHook");

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

        hook(DOWN);

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


        for(int i=0; i<700; i++) {
            final double a = getAlpha();
            avg += a;

            telemetry.addData("Alpha", a);
            telemetry.addData("Total", avg);
            telemetry.addData("Count", i);
            telemetry.addData("Average", avg / i);
            telemetry.update();
            count = i;
        }
        avg = avg/((double) count);


        while(!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("We aight", "");
        telemetry.update();

        telemetry.addData("Colour ","(x blue, b red)");
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
            //drive to the foundation
            driveInches(45);
            //hook the platform
            hook(DOWN);
            //drive back to wall
            driveInches(-44);
            //un hook platform
            hook(UP);
            //drive away from zone
            driveInches(17, colourIsRed ? driveLeft : driveRight);
            //drive past platform
            driveInches(20);
            //turn around the platform
            turnDeg(colourIsRed ? RIGHT : LEFT,180);
            //drive to center of platform
            driveInches(17, colourIsRed ? driveLeft : driveRight);
            //hook the platform
            hook(DOWN);
            //push platform to the zone
            driveInches(20);
            //unhook platform
            hook(UP);
            //drive under bridge
            driveInches(-1);
            driveInches(30, colourIsRed ? driveLeft : driveRight);
        }
        else if(position == quarrySide){
            //
            //harvesterDown();
            //driving towards a stone while flailing around its lovecraftian monster wheels
            driveInchesHarvesting(42);
            //surprised that this hunting mechanism worked, the monster jumps back
            driveInches(-21);
            //the monster turns towards its nest
            turnDeg((colourIsRed) ? RIGHT : LEFT,90);
            //it nears its home nest, preparing to drop its still screaming prey, the stone
            driveInches(40);
            //the monster releases the stone from its mighty jaws, dropping it to the ground below
            intake(OUT, 800);
            //it leaves the nest, keeping its eye on its prey all the way
            driveInches(-40);
            //just for good measure, the monster backs up a bit more.
            driveInches(-12);
            //finally feeling like the prey is too petrified to move, the monster turns to get more food
            turnDeg((colourIsRed) ? LEFT : RIGHT, 90);
            //having spotted its next meal, the monster once again barrels towards it with its horrible arms flinging about
            driveInchesHarvesting(23);
            //with its prey in its grasp, it backs up to enter its nest once more
            driveInches(-23);
            //the monster turns, preparing to enter its nest
            turnDeg((colourIsRed) ? RIGHT : LEFT, 90);
            //It enters the nest, already hungering for its meal
            driveInches(52);
            //the second course drops limp to the cold nest floor, and the monster prepares for dining
            intake(OUT,800);
            //It then guards the entrance to prevent either of its courses from leaving.
            //driveToLine(BACKWARD,avg);
            driveInches(-18);
            /*
            while (opModeIsActive() && getAlpha() <= (avg * 1.25)) {
                telemetry.addData("average", avg);
                telemetry.addData("current", getAlpha());
                telemetry.update();
                idle();
            }
            */

            setMotorPower(0);

        }
        else if(position == dumb){

            /*
            setMotorPower(.2);

            while (opModeIsActive() && getAlpha() <= (avg * 1.25)) {
                telemetry.addData("average", avg);
                telemetry.addData("current", getAlpha());
                telemetry.update();
                idle();
            }
            setMotorPower(0);
            */
            driveInches(24);

        }

        setMotorPower(0);
    }
}
