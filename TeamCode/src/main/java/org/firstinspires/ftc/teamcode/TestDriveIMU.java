package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp public class TestDriveIMU extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lF = null;  //c0
    private DcMotor rF = null;  //c1
    private DcMotor lB = null;  //c2
    private DcMotor rB = null;  //c3
    private DcMotor liftG = null;  //e0
    private DcMotor liftY = null;  //e1
    private DcMotor intake = null;  //e2
    private DcMotor hanger = null;  //e3
    private Servo intakeServo = null; //es0
    private Servo pivot = null; //es1
    private Servo claw = null; //es2
    private Servo droneServo = null; //es3
    private Servo clawauto = null; //cs0
    private Servo clawautop = null; //cs1
    private Servo spoiler = null; //cs2

    @Override
    public void runOpMode() {
        lF = hardwareMap.get(DcMotor.class, "lF");
        rF = hardwareMap.get(DcMotor.class, "rF");
        lB = hardwareMap.get(DcMotor.class, "lB");
        rB = hardwareMap.get(DcMotor.class, "rB");
        liftG = hardwareMap.get(DcMotor.class, "liftG");
        liftY = hardwareMap.get(DcMotor.class, "liftY");
        intake = hardwareMap.get(DcMotor.class, "intake");
        hanger = hardwareMap.get(DcMotor.class, "hanger");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        pivot = hardwareMap.get(Servo.class, "pivot");
        claw = hardwareMap.get(Servo.class, "claw");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        clawauto = hardwareMap.get(Servo.class, "clawauto");
        clawautop = hardwareMap.get(Servo.class, "clawautop");
        spoiler = hardwareMap.get(Servo.class, "spoiler");
        spoiler.setPosition(0.35);
        intakeServo.setPosition(0);
        pivot.setPosition(.5);//1
        claw.setPosition(.5);//0
        //droneServo.setPosition(0);
        //clawauto.setPosition(0);
        //clawautop.setPosition(0);
        lF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.REVERSE);
        //rF.setDirection(DcMotor.Direction.REVERSE);
        //rB.setDirection(DcMotor.Direction.REVERSE);
        //intake.setDirection(DcMotor.Direction.REVERSE);
        liftG.setDirection(DcMotor.Direction.REVERSE);
        //liftY.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //----------------------------Main-Code----------------------------\\

        while (opModeIsActive()) {

            //----------------------------Mecanum-Drive-Code----------------------------\\

            double max;
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double lFPower = axial + lateral + yaw;
            double rFPower = axial - lateral - yaw;
            double lBPower = axial - lateral + yaw;
            double rBPower = axial + lateral - yaw;
            max = Math.max(Math.abs(lFPower), Math.abs(rFPower));
            max = Math.max(max, Math.abs(lBPower));
            max = Math.max(max, Math.abs(rBPower));
            if (max > 1.0) {
                lFPower /= max;
                rFPower /= max;
                lBPower /= max;
                rBPower /= max;
            }

            // Retrieve the IMU from the hardware map
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                lF.setPower(frontLeftPower);
                lB.setPower(backLeftPower);
                rF.setPower(frontRightPower);
                rB.setPower(backRightPower);

                //----------------------------Drive-Speed----------------------------\\

                if (gamepad1.left_bumper) {//speed for going across field
                    lF.setPower(lFPower);
                    rF.setPower(rFPower);
                    lB.setPower(lBPower);
                    rB.setPower(rBPower);
                }
                if (gamepad1.right_bumper) {//slow for corrections
                    lF.setPower(0.1 * lFPower);
                    rF.setPower(0.1 * rFPower);
                    lB.setPower(0.1 * lBPower);
                    rB.setPower(0.1 * rBPower);
                }

                //----------------------------intake----------------------------\\

                if (gamepad2.right_bumper) {
                    intake.setPower(-0.8);      //in
                    spoiler.setPosition(0);
                }  //up
                else if (gamepad2.left_bumper) {
                    intake.setPower(1);         //out
                    spoiler.setPosition(0);
                }  //up
                else {
                    intake.setPower(0);       //stop
                    spoiler.setPosition(0.35);
                }   //down

                if (gamepad2.y) {
                    intakeServo.setPosition(0.59);
                } else if (gamepad2.x) {
                    intakeServo.setPosition(0.43);
                } else {
                    intakeServo.setPosition(0);
                }

                //----------------------------outtake----------------------------\\

                if (gamepad2.a) {
                    pivot.setPosition(.375);    //0
                    claw.setPosition(.625);
                }  //1
                else if (gamepad2.b) {
                    pivot.setPosition(.2);    //0
                    claw.setPosition(.8);
                }  //1
                else {
                    pivot.setPosition(.5);   //0
                    claw.setPosition(.5);
                } //1

                //----------------------------droneServo----------------------------\\

                if (gamepad2.dpad_left) {
                    droneServo.setPosition(1);
                } else {
                    droneServo.setPosition(0);
                }

                //----------------------------hanger----------------------------\\

                if (gamepad2.dpad_up) {
                    hanger.setPower(1);
                } else if (gamepad2.dpad_down) {
                    hanger.setPower(-1);
                } else {
                    hanger.setPower(0);
                }

                //----------------------------liftGY----------------------------\\

                liftG.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                liftY.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

                //----------------------------Telemetry----------------------------\\

                telemetry.addData("liftG", liftG.getCurrentPosition());
                telemetry.addData("liftY", liftY.getCurrentPosition());
            /*telemetry.addData("intakeServo", intakeServo.getPosition());
            telemetry.addData("pivot", pivot.getPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", lFPower, rFPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", lBPower, rBPower);*/
                telemetry.update();
            }
        }
    }
}