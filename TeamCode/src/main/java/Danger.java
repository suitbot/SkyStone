import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Danger extends OpMode {
    private DcMotor fl, fr, bl, br;

    @Override
    public void loop() {
        fl.setPower(gamepad1.left_stick_y);
        bl.setPower(gamepad1.left_stick_y);
        fr.setPower(gamepad1.right_stick_y);
        br.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
