package org.firstinspires.ftc.suitbots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * TODO: intake mover at angles
 * outtake at slower speed
 */
@TeleOp(name = "Normal TeleOp")
public class SkyStoneTeleOp extends OpMode {
    public DcMotor lf, rf, lr, rr;
    public DcMotor lintake, rintake, lift;
    public ModernRoboticsI2cRangeSensor backRange, leftRange, rightRange;

    public boolean isSlow = false;
    public float slownesss = 3;
    public boolean isStopped = false;

    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");

        lintake = hardwareMap.dcMotor.get("lintake");
        rintake = hardwareMap.dcMotor.get("rintake");
        lift = hardwareMap.dcMotor.get("lift");

        backRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backRange");
        leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        rightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        lintake.setDirection(DcMotorSimple.Direction.REVERSE);


        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Waiting for start");
        telemetry.update();
    }


    public final static double PI = Math.PI;
    public final static double TICKS_PER_ROT = 383.6;
    public final static double GEAR_RATIO = 1;
    public final static double WHEEL_DIAMETER = 1.5;
    public int convertLiftHeightToEncoderTicks(double liftHeightInInches){
        return (int) (liftHeightInInches * TICKS_PER_ROT * GEAR_RATIO / (WHEEL_DIAMETER * PI));
    }


    @Override
    public void start() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //TODO: change heights so they work
    double[] heights = {0, 1.65, 2.8, 7, 10.8};
    int level = 0;
    boolean amILiftingPresently = false;

    @Override
    public void loop() {

        isStopped = (gamepad1.a) ? !isStopped : isStopped;
        isSlow = (gamepad1.start) ? !isSlow : isSlow;
        /*
        if(gamepad1.start){
            while(gamepad1.start) {
                telemetry.addData("waiting", System.currentTimeMillis());
            }
            isSlow = !isSlow;
        }*/


        if (gamepad1.right_bumper) {
            if (! amILiftingPresently) {
                level = Math.min(level + 1, heights.length - 1);
                amILiftingPresently = true;
                lift.setTargetPosition(convertLiftHeightToEncoderTicks(heights[level]));
                lift.setPower(0.5);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else if (gamepad1.left_bumper) {
            if (! amILiftingPresently) {
                level = Math.max(level - 1, 0);
                amILiftingPresently = true;
                lift.setTargetPosition(convertLiftHeightToEncoderTicks(heights[level]));
                lift.setPower(-0.1);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else {
            amILiftingPresently = false;
        }



        /*
        if(isSlow)
        lift.setPower(0.2);
        else{
            lift.setPower(0.5);
        }
        */


        if (gamepad1.right_trigger > 0.7) {
            lintake.setPower(0.8 / ((isSlow) ? 3 : 1));
            rintake.setPower(0.87 / ((isSlow) ? 3 : 1));
        } else if (gamepad1.left_trigger > 0.7) {
            lintake.setPower(-0.27);
            rintake.setPower(-0.33);
            lf.setPower(0.3 / ((isSlow) ? 3 : 1));
            lr.setPower(0.3 / ((isSlow) ? 3 : 1));
            rf.setPower(0.3 / ((isSlow) ? 3 : 1));
            rr.setPower(0.3 / ((isSlow) ? 3 : 1));
        } else if(isStopped){
            lintake.setPower(0);
            rintake.setPower(0);
        } else{
            lintake.setPower(0.1);
            rintake.setPower(0.1);
        }

        double rotation = -gamepad1.right_stick_x / ((isSlow) ? slownesss : 1);
        double strafe = -gamepad1.left_stick_x / ((isSlow) ? 4 : 1);
        double drive = gamepad1.left_stick_y / ((isSlow) ? slownesss : 1);


        double[] speeds = {
                (drive + strafe + rotation),
                (drive - strafe - rotation),
                (drive - strafe + rotation),
                (drive + strafe - rotation)
        };


        telemetry.addData("Lift Level", level);
        telemetry.addData("Lift Current", lift.getCurrentPosition());
        telemetry.addData("Is Slow : ", isSlow);
        telemetry.addData("LFpower : ", speeds[0]);
        telemetry.addData("RFpower : ", speeds[0]);
        telemetry.addData("LRpower : ", speeds[0]);
        telemetry.addData("RRpower : ", speeds[0]);
        telemetry.update();

        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        lf.setPower(speeds[0]);
        rf.setPower(speeds[1]);
        lr.setPower(speeds[2]);
        rr.setPower(speeds[3]);

        //telemetry.addData("drive angle", Math.toDegrees(robotAngle));
        //telemetry.addData("Motors", String.format("lf %.2f rf %.2f lr %.2f rr %.2f", lfPower, rfPower, lrPower, rrPower));
    }
}
