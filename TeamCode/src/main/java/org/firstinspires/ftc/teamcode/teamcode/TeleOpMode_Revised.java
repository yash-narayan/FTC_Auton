package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode_Revised", group = "Tutorials")

public class TeleOpMode_Revised extends LinearOpMode
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
    public void runOpMode() throws InterruptedException
    {
        //TODO: Find the name of the motors
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        up = hardwareMap.dcMotor.get("up");
        leftArm = hardwareMap.servo.get("left");
        rightArm = hardwareMap.servo.get("right");
        //grabbyArm = hardwareMap.dcMotor.get("");
        //chomper = hardwareMap.servo.get("");
        //wrist = hardwareMap.servo.get("");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();


        while(opModeIsActive())
        {

            //Wheels
            leftDrive.setPower(-(gamepad1.left_stick_y));
            rightDrive.setPower(-(gamepad1.right_stick_y));
            up.setPower(gamepad2.right_stick_y);


            //rightArm.setPosition(gamepad2.left_stick_x);
            //leftArm.setPosition((gamepad2.right_stick_x) + 1);

            leftArm.setPosition(0.5 - (0.5 * (gamepad2.left_stick_x))); //original right
            rightArm.setPosition(0.4 + (0.4 * (gamepad2.left_stick_x)));

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