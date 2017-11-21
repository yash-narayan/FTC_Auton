package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import  com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by yasnara on 10/13/17.
 */

@TeleOp(name = "TeleOpMode_Yash", group = "Tutorials")

public class TeleOpMode_Yash extends LinearOpMode
{
    // Declaring/Calling the variable, Initialize
    private DcMotor lefDrive;
    private DcMotor rightDrive;
    private DcMotor  middleDrive;
    private DcMotor upDrive;
    private DcMotor downDrive;

    public Servo    leftClaw;
    public Servo    rightClaw;
    //public Servo    rightClaw;
    // private Servo middleDrive;
    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //TODO: Find the name of the motors
        //On the phone there are names set, and the program is calling this, so the phone understand what these variables mean
        lefDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        middleDrive = hardwareMap.dcMotor.get("left_arm");
        upDrive = hardwareMap.dcMotor.get("up");
        //Servo
        leftClaw = hardwareMap.servo.get("left");
        rightClaw = hardwareMap.servo.get("right");
        // Set Direction
        lefDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
            // These are setting the which button the controller will use
            //Motor Power
            lefDrive.setPower(+gamepad1.left_stick_y);
            rightDrive.setPower(+gamepad1.right_stick_y);
            upDrive.setPower(gamepad2.left_stick_y);
            leftClaw.setPosition(gamepad2.left_trigger);
            rightClaw.setPosition(gamepad2.right_trigger);
           //Servo Position
            // leftClaw.setPosition(gamepad1.left_trigger);

           // TEST if the motors increase in term soft

            if(leftClaw.getPosition() <= 0.2) {
                //leftClaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //leftClaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if(gamepad2.left_stick_x > 0.0)  {
                leftClaw.setPosition(0.5);
            }
            if(gamepad2.a)
            {
                upDrive.setPower(1);
            }

            if(gamepad1.a)
            {
                lefDrive.setPower(1);
            }

            if (gamepad1.b) {
                rightDrive.setPower(1);
            }
            idle();
        }
    }
}

