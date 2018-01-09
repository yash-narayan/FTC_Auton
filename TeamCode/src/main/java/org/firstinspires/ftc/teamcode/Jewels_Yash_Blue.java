package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Jewels_Yash_Blue", group="Pushbot")
public class Jewels_Yash_Blue extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime(); // starting a timer once it is run
    ColorSensor colorSensor; // declaring color sensor

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.8;
    public static final double MID_SERVO       =  0.5 ;
    public static final double OUT_SERVO       = 0.2;
    Servo leftClaw = null;
    Servo rightClaw = null;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClaw = hardwareMap.servo.get("left");
        rightClaw = hardwareMap.servo.get("right");

        //TEST
        leftClaw.setPosition(0.6);
        rightClaw.setPosition(0.6);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();
        armDown(2.0);
        jewel(0.5);
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);
//        encoderDrive(DRIVE_SPEED, 0, 0, 2.0);
//        encoderDrive(DRIVE_SPEED, -10, -10, 2.0);  // S1: Forward 15 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED, 1.5, -1.5, 1.25);  // S2: Turn Right 6 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -1.0, -1.0, 0.5);  // S3: Reverse 3 Inches with 4 Sec timeout
//        leftClaw.setPosition(OUT_SERVO);
//        rightClaw.setPosition(OUT_SERVO);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //CODE FOR IF WE ARE ON RED SIDE
    public void jewel(double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (colorSensor.red() > colorSensor.blue()) {
                //TODO: TEST if arm to long, so bring it up before turning back
                encoderDrive(TURN_SPEED, 8, -8, 1.0); // TURN BACKWARDS
                armUp(1.0); // ARM GOES UP
                encoderDrive(TURN_SPEED, -5.0, 5.0, 1.0); // GO FORWARD
                telemetry.addData("BLUE", "%s visible");
                //robot.armServo.setPosition(0.0);
            } else {
                encoderDrive(TURN_SPEED, -3.0, 3.0, 0.5);// GO FORWARD
                //armUp(0.5);\\
                armUp(1.0);
                encoderDrive(TURN_SPEED, 3.0, -3.0, 0.4); // TURN BACKS
                telemetry.addData("RED", "%s visible");

                //robot.armServo.setPosition(0.0);
            }
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
        }
    }
    public void armDown(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            robot.armServo.setPosition(0.0);
        }
    }
    public void armUp(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            robot.armServo.setPosition(1.0);
        }
    }
}

