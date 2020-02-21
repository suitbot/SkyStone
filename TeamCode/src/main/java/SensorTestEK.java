import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Sensor Test")
public class SensorTestEK extends OpMode {
    ColorSensor color;
    DistanceSensor lidar;
    ModernRoboticsI2cRangeSensor range1;
    ModernRoboticsI2cRangeSensor range2;
    ModernRoboticsI2cRangeSensor range3;

    @Override
    public void init() {
        lidar = hardwareMap.get(DistanceSensor.class, "lidar");
        range1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range1");
        range2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range2");
        range3 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range3");
        color = hardwareMap.get(ColorSensor.class,"color");
    }

    @Override
    public void loop() {
        telemetry.addData("Lidar: ", lidar.getDistance(DistanceUnit.INCH));
        telemetry.addData("Range1: ", range1.getDistance(DistanceUnit.INCH));
        telemetry.addData("Range2: ", range2.getDistance(DistanceUnit.INCH));
        telemetry.addData("Range3: ", range3.getDistance(DistanceUnit.INCH));
        telemetry.addData("Color Alpha : ",color.alpha());
        telemetry.update();
    }
}
