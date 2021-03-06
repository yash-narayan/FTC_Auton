package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Encoder_Auton_Yash_Test", group="Pushbot")
public class Encoder_Auton_Yash_Test extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime(); // starting a timer once it is run
    //ColorSensor colorSensor; // declaring color sensor

    static final double COUNTS_PER_MOTOR_REV_1 = 1120;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_2 = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH_1 = (COUNTS_PER_MOTOR_REV_1 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_2 = (COUNTS_PER_MOTOR_REV_2 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED_1 = 1.0;
    static final double DRIVE_SPEED_2 = 1.0;
    static final double DRIVE_SPEED = 0.3; // Drive speed is turn speed
    static final double TURN_SPEED = 0.6;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
//        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
//        colorSensor.enableLed(true);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();
        //armDown(2.0);
        //jewel(0.5);
        //encoderDrive(DRIVE_SPEED, 0, 0, 2.0);
        encoderDrive(DRIVE_SPEED, -30, -30, 30.0);  // S1: Forward 15 Inches with 5 Sec timeout
        sleep(1000);
        encoderDrive(TURN_SPEED, 1.3, -1.3, 1.2);  // S2: Turn Right 6 Inches with 4 Sec timeout

        //BREAK

        //encoderDrive(DRIVE_SPEED, -0.8, -0.8, 0.5);  // S3: Reverse 3 Inches with 4 Sec timeout
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
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_1);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_2);
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
                telemetry.addData("Path2", "Running at %7d :%7d", newLeftTarget, newRightTarget,
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
//    public void jewel(double holdTime){
//        ElapsedTime holdTimer = new ElapsedTime();
//        holdTimer.reset();
//        while (opModeIsActive() && holdTimer.time() < holdTime) {
//            if (colorSensor.blue() > 3) {
//                //TODO: TEST if arm to long, so bring it up before turning back
//                encoderDrive(TURN_SPEED, 0.3, -0.3, 1.0);
//                encoderDrive(TURN_SPEED, -0.3, 0.3, 1.0);
//                robot.armServo.setPosition(0.0);
//            } else {
//                encoderDrive(TURN_SPEED, -0.3, 0.3, 1.0);
//                encoderDrive(TURN_SPEED, 0.3, -0.3, 2.0);
//                robot.armServo.setPosition(0.0);
//
//            }
//            robot.leftDrive.setPower(0);
//            robot.rightDrive.setPower(0);
//        }
//    }
//    public void armDown(double holdTime) {
//        ElapsedTime holdTimer = new ElapsedTime();
//        holdTimer.reset();
//        while (opModeIsActive() && holdTimer.time() < holdTime) {
//            robot.armServo.setPosition(1.0);
//        }
//    }

}

