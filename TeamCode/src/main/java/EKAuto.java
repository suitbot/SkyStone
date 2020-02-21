import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name= "Big Brian")
public class EKAuto extends LinearOpMode {
    public DcMotor lf, rf, lr, rr;


    public void setMotorPowers(double power, DcMotor ...motors){
        for(DcMotor i : motors){
            i.setPower(power);
        }
    }
    public void setMotorPowers(double power) {
        this.setMotorPowers(power, lf, rf, lr, rr);
    }
    public void driveMiliseconds(double power, long time) {
        setMotorPowers(power);
        sleep(time);
        setMotorPowers(0);
        sleep(500);
    }
    public void stopResetEncoders(){
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
    public void setTargetPosition(int lfPosition, int rfPosition, int lrPosition, int rrPosition){
        lf.setTargetPosition(lfPosition);
        rf.setTargetPosition(rfPosition);
        lr.setTargetPosition(lrPosition);
        rr.setTargetPosition(rrPosition);
    }
    public void setTargetPosition(int position){ setTargetPosition(position, position, position, position); }

    public void driveInches(int distance) {
        stopResetEncoders();
        setTargetPosition(distance);
        runToPosition();
        setMotorPowers(0.5);
        while(lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy()){
            idle();
        }
        setMotorPowers(0);
        runUsingEnc();
        sleep(500);
    }

    @Override
    public void runOpMode() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();



    }
}
