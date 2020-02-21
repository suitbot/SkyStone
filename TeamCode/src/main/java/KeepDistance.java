import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Keep Distance")
public class KeepDistance extends OpMode {
    public DcMotor lf, rf, lr, rr;
    public DcMotor lintake, rintake, intakeMove;
    public CRServo rHook, lHook;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public ColorSensor sensorColor;
    public ModernRoboticsI2cRangeSensor ultrasonic;

    @Override
    public void init(){

        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");

        lintake = hardwareMap.dcMotor.get("lintake");
        rintake = hardwareMap.dcMotor.get("rintake");
        intakeMove = hardwareMap.dcMotor.get("lift");

        rHook = hardwareMap.crservo.get("rHook");
        lHook = hardwareMap.crservo.get("lHook");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rintake.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        sensorColor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "sensorColor");
        ultrasonic = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonicBack");
    }

    @Override
    public void loop(){
        while(ultrasonic.cmUltrasonic() < 50){
            lf.setPower(-0.4);
            lr.setPower(-0.4);
            rf.setPower(-0.4);
            rr.setPower(-0.4);
            telemetry.addData("----NEAR----","");
            telemetry.addData("raw ultrasonicBack", ultrasonic.rawUltrasonic());
            telemetry.addData("raw optical", ultrasonic.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", ultrasonic.cmOptical());
            telemetry.addData("cm", "%.2f cm", ultrasonic.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
        telemetry.addData("FAR","");
        telemetry.addData("raw ultrasonicBack", ultrasonic.rawUltrasonic());
        telemetry.addData("raw optical", ultrasonic.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", ultrasonic.cmOptical());
        telemetry.addData("cm", "%.2f cm", ultrasonic.getDistance(DistanceUnit.CM));
        telemetry.update();
        telemetry.update();
    }
}
