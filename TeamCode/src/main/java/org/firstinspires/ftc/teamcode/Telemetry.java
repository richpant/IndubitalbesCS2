package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Telemetry")
public class Telemetry extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lF        = null; //c0
    private DcMotor rF        = null; //c1
    private DcMotor lB        = null; //c2
    private DcMotor rB        = null; //c3
    private DcMotor lift      = null; //e0
    private DcMotor gear      = null; //e1
    private DcMotor hanger    = null; //e2
    private Servo pivot       = null; //es0
    private Servo clawL       = null; //es1
    private Servo clawR       = null; //es2
    private Servo hangerServo = null; //es4
    private Servo droneServo  = null; //es5

    @Override
    public void runOpMode() {
        lF = hardwareMap.get(DcMotor.class, "lF");
        rF = hardwareMap.get(DcMotor.class, "rF");
        lB = hardwareMap.get(DcMotor.class, "lB");
        rB = hardwareMap.get(DcMotor.class, "rB");
        lift = hardwareMap.get(DcMotor.class, "lift");
        gear = hardwareMap.get(DcMotor.class, "gear");
        hanger = hardwareMap.get(DcMotor.class, "hanger");
        hangerServo = hardwareMap.get(Servo.class, "hangerServo");
        pivot = hardwareMap.get(Servo.class, "pivot");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        lF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.REVERSE);
        //rF.setDirection(DcMotor.Direction.REVERSE);
        //rB.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        gear.setDirection(DcMotor.Direction.REVERSE);
        //hanger.setDirection(DcMotor.Direction.REVERSE);

        gear.setTargetPosition(0);
        lift.setTargetPosition(0);
        pivot.setPosition(.8);
        clawL.setPosition(0.45);
        clawR.setPosition(0.25);
        hangerServo.setPosition(0);
        droneServo.setPosition(0.5);

        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        boolean clawState = false;
        boolean oldArmButton = true;

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
                rBPower /= max; }

            lF.setPower(0.6 * lFPower);
            rF.setPower(0.6 * rFPower);
            lB.setPower(0.6 * lBPower);
            rB.setPower(0.6 * rBPower);

            /*----------------------------code----------------------------\\

            if (gamepad1.dpad_up) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(0.2); }
            else if (gamepad1.dpad_down) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(-0.2); }
            else if (gamepad1.dpad_right) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(0.3); }
            else if (gamepad1.dpad_left) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(-0.3); }
            else {gear.setMode(DcMotor.RunMode.RUN_USING_ENCODER); lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }
            if (gamepad1.x) {gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }*/



            //----------------------------Telemetry----------------------------\\

            telemetry.addData("hanger encoder", hanger.getCurrentPosition());
            telemetry.addData("hanger target", lift.getTargetPosition());
            telemetry.addData("lift encoder", lift.getCurrentPosition());
            telemetry.addData("lift target", lift.getTargetPosition());
            telemetry.addData("gear encoder", gear.getCurrentPosition());
            telemetry.addData("gear target", gear.getTargetPosition());
            telemetry.addData("pivot position", pivot.getPosition());
            telemetry.addData("clawL position", clawL.getPosition());
            telemetry.addData("clawR position", clawR.getPosition());
            telemetry.addData("droneServo position", droneServo.getPosition());
            telemetry.update();

        }
    }
}