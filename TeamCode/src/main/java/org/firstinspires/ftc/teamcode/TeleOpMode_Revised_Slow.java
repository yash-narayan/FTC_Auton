package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode_Revised_Slow", group = "Tutorials")

public class TeleOpMode_Revised_Slow extends LinearOpMode
{
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor up;
    private Servo   leftArm;
    private Servo   rightArm;
    // private Servo leftArm;
    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;
 // add stop to elevator!!!!!

    @Override
    public void runOpMode() throws InterruptedException //
    {
        //TODO: Find the name of the motors

        leftDrive = hardwareMap.dcMotor.get("left_drive"); // setting the name of the motor so we  can call it throughout the program
        rightDrive = hardwareMap.dcMotor.get("right_drive"); // setting the name of the motor so we  can call it throughout the program
        up = hardwareMap.dcMotor.get("up"); // setting the name of the motor so we  can call it throughout the program
        leftArm = hardwareMap.servo.get("left"); // setting the name of the motor so we  can call it throughout the program
        rightArm = hardwareMap.servo.get("right"); // setting the name of the motor so we  can call it throughout the program
        //grabbyArm = hardwareMap.dcMotor.get("");
        //chomper = hardwareMap.servo.get("");
        //wrist = hardwareMap.servo.get("");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // setting the direction of the left and right arm
        //leftArm.setPosition(0.4); // setting the position (how much the motor is turned) of the motor
        //rightArm.setPosition(0.4); // setting the position (how much the motor is turned) of the motor

        waitForStart();


        while(opModeIsActive())
        {

            //Wheels
            leftDrive.setPower(-0.4 * (gamepad1.left_stick_y)); // setting the left motor power to the position of the joystick
            rightDrive.setPower(-0.4 * (gamepad1.right_stick_y)); // setting the right motor power to the position of the joystick
            up.setPower(gamepad2.right_stick_y); // setting the elevator motor to the joystick position

/*
            //rightArm.setPosition(gamepad2.left_stick_x);
            //leftArm.setPosition((gamepad2.right_stick_x) + 1);

//            leftArm.setPosition(0.4);
//            rightArm.setPosition(0.4);

*/
            leftArm.setPosition(0.5 - (0.5 * (gamepad2.left_stick_x))); //setting the arm position to the joystick position
            rightArm.setPosition(0.5 + (0.5 * (gamepad2.left_stick_x))); //setting the arm position to the joystick position

            //if (gamepad2.a == true){
                //chomper.setPosition(5);
            //}
            //else {
                //chomper.setPosition(0);
            //}

            //if (gamepad2.b == true){
                //wrist.setPosition(5);
            //}
            //else {
                //wrist.setPosition(0);
            //}

            //if (gamepad2.left_trigger > 0){
                //grabbyArm.setPower(-1);
            //}

            //else if (gamepad2.right_trigger > 0){
                //grabbyArm.setPower(1);
            //}


            idle();
        }
    }
}

