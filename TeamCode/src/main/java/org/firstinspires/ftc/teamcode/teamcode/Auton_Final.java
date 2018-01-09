package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
@Disabled
@Autonomous(name="Auton_Final", group ="Concept")
public class Auton_Final extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor Elevator;
    private Servo leftClaw;
    private Servo rightClaw;
    private ElapsedTime runtime = new ElapsedTime();
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    /**

     * {@ link #vuforia} is the variable we will use to store our instance of the Vuforia

     * localization engine.

     */
    VuforiaLocalizer vuforia;
    @Override public void runOpMode() {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        Elevator = hardwareMap.dcMotor.get("up");
        leftClaw = hardwareMap.servo.get("left");
        rightClaw = hardwareMap.servo.get("right");
        leftClaw.setPosition(.8);
        rightClaw.setPosition(.8);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWFBXwH/////AAAAGSPa9Rn7uUfRu/pKnx8SI8FoKmVjnGUBXNQqbjjmrpzqmY87S1XOBcISudQgjC842h8oHi7QABJzSGibRH+KFuYZS7XwJfJvwYXaOUy6gnjQulFeXLbpXGWA41zuPr5WZjDAgUSifvIovuYB7ToFnVKixDSgw1ATJChRxUWyqrzj5kYw0ZeL7glsmVEKsk4dgJ0BtzoW5/WP12pfblYMUqqPhy04bEfQQMNKjK/LThQSPKjNwbey7mhCgNvgs65OSzkHOy2WO/58WdXCv+3veHDaWr0HxXBotFzxA3Zvj420GSC5xYQLwLol/lV6PwsQjte2FvcWnVh7rYwBET0Ef8INTYPcCASwJoFh8m5niPgN";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable

         * in this data set: all three of the VuMarks in the game were created from this one template,

         * but differ in their instance id information.

         * @see VuMarkInstanceId

         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        relicTrackables.activate();
        while (opModeIsActive()) {
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                if (vuMark == RelicRecoveryVuMark.LEFT)  {
                    //FORWARD, For left go only 77 because we are putting glyph in left column
                    DriveForward(0.4);
                    sleep(77);
                    //RIGHT
                    TurnRight((0.2));
                    sleep(50);
                    //ELEVATOR DOWN
                    ElevatorDown(1);
                    sleep(1000);
                    //FORWARD
                    DriveForward(0.3);
                    sleep(40);
                    //ARM OUT
                    ArmOut(0.2);
                    sleep(1000);
                    //Arm In
                    ArmIn(0.4);
                    sleep(1000);
                    //Elevator Up
                    ElevatorUp(0.5);
                    sleep(3000);
                    stop();
                }
                else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    //FORWARD, For right go only 93 because we are putting glyph in right column
                    DriveForward(0.4);
                    sleep(93);
                    //RIGHT
                    TurnRight((0.2));
                    sleep(50);
                    //ELEVATOR DOWN
                    ElevatorDown(1);
                    sleep(1000);
                    //FORWARD
                    DriveForward(0.3);
                    sleep(40);
                    //ARM OUT
                    ArmOut(0.2);
                    sleep(1000);
                    //Arm In
                    ArmIn(0.4);
                    sleep(1000);
                    //Elevator Up
                    ElevatorUp(0.5);
                    sleep(3000);
                    stop();
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    //FORWARD, For center/middle go only 84 because we are putting glyph in middle column
                    DriveForward(0.4);
                    sleep(84);
                    //RIGHT
                    TurnRight((0.2));
                    sleep(50);
                    //ELEVATOR DOWN
                    ElevatorDown(1);
                    sleep(1000);
                    //FORWARD
                    DriveForward(0.3);
                    sleep(40);
                    //ARM OUT
                    ArmOut(0.2);
                    sleep(1000);
                    //Arm In
                    ArmIn(0.4);
                    sleep(1000);
                    //Elevator Up
                    ElevatorUp(0.5);
                    sleep(3000);
                    stop();
                }
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));
                /* We further illustrate how to decompose the pose into useful rotational and

                 * translational components */

                if (pose != null) {

                    VectorF trans = pose.getTranslation();

                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else if (vuMark == RelicRecoveryVuMark.UNKNOWN){
                telemetry.addData("VuMark", "not visible");
                //FORWARD, For left go only 77 because we are putting glyph in left column
                DriveForward(0.4);
                sleep(77);
                //RIGHT
                TurnRight((0.2));
                sleep(50);
                //ELEVATOR DOWN
                ElevatorDown(1);
                sleep(1000);
                //FORWARD
                DriveForward(0.3);
                sleep(40);
                //ARM OUT
                ArmOut(0.2);
                sleep(1000);
                //Arm In
                ArmIn(0.4);
                sleep(1000);
                //Elevator Up
                ElevatorUp(0.5);
                sleep(3000);
                stop();
            }
            telemetry.update();
        }
    }
    String format(OpenGLMatrix transformationMatrix) {

        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";

    }
    public void DriveForward(double power)
    {
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
    }
    public void StopDriving ()
    {
        DriveForward(0);
    }
    public void TurnRight (double power)
    {
        rightDrive.setPower(power);
        leftDrive.setPower(-power);
    }
    public void TurnLeft (double power)
    {
        rightDrive.setPower(-power);
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
}