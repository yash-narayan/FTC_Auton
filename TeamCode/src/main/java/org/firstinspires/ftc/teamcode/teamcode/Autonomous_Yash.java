package org.firstinspires.ftc.teamcode.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


        import javax.lang.model.element.ElementVisitor;
@Disabled
@Autonomous(name = "Autonomous_Yash", group = "Tutorials")

public class Autonomous_Yash extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    ColorSensor colorSensor;

    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor Elevator = null;

    Servo leftClaw = null;
    Servo rightClaw = null;
    Servo armServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        Elevator = hardwareMap.dcMotor.get("up");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //CHECK: IF RIGHT DRIVE SHOULD ALSO BE REVERSE?
        //
        leftClaw = hardwareMap.servo.get("left");
        rightClaw = hardwareMap.servo.get("right");
        armServo = hardwareMap.servo.get("armServo");

        leftClaw.setPosition(.8);
        rightClaw.setPosition(.8);
      //WRITE ALL CODE AFTER THIS
        waitForStart();
        armDown(2.0);
        jewel(0.5);
        //FORWARD
//        ServoIn(2.0);
//        sleep(100);
        DriveForward(0.1);
        sleep(1);
        //STOP
        StopDriving();
        //LEFT
        TurnLeft((0.2));
        sleep(15);
        //Elevator Down
        DriveForward(0.3);
        sleep(20);
        ElevatorDown(1);
        sleep(1000);
        //Arm Out
        ArmOut(0.2);
        sleep(1000);
        //Arm In
        ArmIn(0.4);
        sleep(1000);
        //Elevator Up
        ElevatorUp(0.5);
        sleep(3000);
        stop();

        //DriveForwardDistance(0.5, 5);
    }
    public void DriveForward(double power)
    {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }
    public void DriveOut(double power)
    {
        leftDrive.setPower(-power);
        rightDrive.setPower(-power);
    }
    public void StopDriving ()
    {
        DriveForward(0);
    }
    public void TurnRight (double power)
    {
        rightDrive.setPower(-power);
        leftDrive.setPower(power);
    }
    public void TurnLeft (double power)
    {
        rightDrive.setPower(power);
        leftDrive.setPower(-power);
    }
    public void ArmIn (double power)
    {
        leftClaw.setPosition(0.8);
        rightClaw.setPosition(0.8);
    }
    public void ArmOut (double power)
    {
        leftClaw.setPosition(0.2);
        rightClaw.setPosition(0.2);
    }
    public void ElevatorUp (double power)
    {
        Elevator.setPower(power);
    }

    public void ElevatorDown (double power)
    {
        Elevator.setPower(-power);
    }

    public void jewel(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (colorSensor.blue() > 3) {
                DriveForward(0.2);
                sleep(200);
                robot.armServo.setPosition(0.0);
            } else {
                DriveOut(0.2);
                sleep(200);
                robot.armServo.setPosition(0.0);
            }
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
        }
    }

    public void armDown (double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            robot.armServo.setPosition(1.0);
        }
    }
}
