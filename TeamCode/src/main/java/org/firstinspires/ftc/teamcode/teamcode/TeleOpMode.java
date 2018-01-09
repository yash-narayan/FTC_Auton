package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import  com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by yasnara on 10/13/17.
 */
@Disabled
@TeleOp(name = "TeleOpMode", group = "Tutorials")

public class TeleOpMode extends LinearOpMode
{
    private DcMotor lefDrive;
    private DcMotor rightDrive;
    private DcMotor  leftArm;
    // private Servo leftArm;
    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //TODO: Find the name of the motors
        lefDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftArm = hardwareMap.dcMotor.get("left_arm");

        lefDrive.setDirection(DcMotor.Direction.REVERSE);

        //leftArm = hardwareMap.servo.get("armServo");

        waitForStart();

        while(opModeIsActive())
        {
            lefDrive.setPower(gamepad1.left_stick_y);
            rightDrive.setPower(gamepad1.right_stick_y);
            leftArm.setPower(gamepad2.left_stick_y);

            if(gamepad2.a)
        {
            //leftArm.setPosition(ARM_EXTENDED_POSITION);
        }

            if(gamepad2.b)
            {
                //leftArm.setPosition(ARM_RETRACTED_POSITION );
            }

            idle();
        }
    }
}
