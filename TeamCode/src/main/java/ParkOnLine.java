import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Drive To Line")
public class ParkOnLine extends LinearOpMode {
    private DcMotor lf, lb, rf, rb;
    private ColorSensor sensorColor;

    double getAlpha() {
        return sensorColor.alpha();
    }

    void setDriveSpeed(double speed) {
        lf.setPower(speed);
        lb.setPower(speed);
        rf.setPower(speed);
        rb.setPower(speed);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        double avg = 0.0;
        int count = 0;

        while (! (isStarted() || isStopRequested())) {
            final double a = getAlpha();
            avg += a;
            ++count;

            telemetry.addData("Alpha", a);
            telemetry.addData("Total", avg);
            telemetry.addData("Count", count);
            telemetry.addData("Average", avg / count);
            telemetry.update();
        }

        avg /= ((double) count);

        setDriveSpeed(.2);

        while (opModeIsActive() && getAlpha() <= (avg * 1.25)) {
            telemetry.addData("average", avg);
            telemetry.addData("current", getAlpha());
            telemetry.update();
            idle();
        }

        setDriveSpeed(.0);
    }
}
