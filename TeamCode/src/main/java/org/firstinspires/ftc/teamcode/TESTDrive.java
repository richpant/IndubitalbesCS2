package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TESTDrive")
public class TESTDrive extends LinearOpMode {

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

        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gear.setTargetPosition(0);
        lift.setTargetPosition(0);
        pivot.setPosition(.8);
        clawL.setPosition(0.45);
        clawR.setPosition(0.25);
        hangerServo.setPosition(0);
        droneServo.setPosition(0.5);


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

            //----------------------------Drive-Speed----------------------------\\

            lF.setPower(0.6 * lFPower);
            rF.setPower(0.6 * rFPower);
            lB.setPower(0.6 * lBPower);
            rB.setPower(0.6 * rBPower);
            if (gamepad1.left_bumper) { //speed for going across field
                lF.setPower(lFPower);
                rF.setPower(rFPower);
                lB.setPower(lBPower);
                rB.setPower(rBPower); }
            if (gamepad1.right_bumper) { //slow for corrections
                lF.setPower(0.2 * lFPower);
                rF.setPower(0.2 * rFPower);
                lB.setPower(0.2 * lBPower);
                rB.setPower(0.2 * rBPower); }

            //----------------------------pivot----------------------------\\

            if (gamepad2.dpad_right) {pivot.setPosition(0.14); }

            /*boolean armButton = gamepad2.a;
            if (armButton != oldArmButton) {
                telemetry.addData("button presss confirmed", 0);
                oldArmButton = armButton;
                if (!armButton) {
                    clawState = !clawState;
                    if (clawState) {
                        pivot.setPosition(0.75);
                        telemetry.addData("srm up",1); }
                    else {pivot.setPosition(0.35);
                        telemetry.addData("srm dn",0); } } }

            telemetry.update();*/

            //----------------------------claw----------------------------\\

            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                clawL.setPosition(0.33); //close
                clawR.setPosition(0.37); }
            else {clawL.setPosition(.45); //open
                clawR.setPosition(.25); }

            //----------------------------hanger----------------------------\\

            if (gamepad1.dpad_up) {
                hanger.setPower(1);
                hangerServo.setPosition(0.163); }
            else if (gamepad1.dpad_down) {
                hanger.setPower(-1);
                hangerServo.setPosition(0.163); }
            else {hanger.setPower(0); }

            if (gamepad1.dpad_left) {
                hangerServo.setPosition(0); }

            //----------------------------droneServo----------------------------\\

            if (gamepad1.a) {
                droneServo.setPosition(0); }
            else {droneServo.setPosition(0.5); }

            //----------------------------lift/gear----------------------------\\

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //lift.setPower(1);
            gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //gear.setPower(0.5);

            /*if (gear.getCurrentPosition() < 0 || gear.getTargetPosition() < 0) {
                gear.setTargetPosition(5); }
            if (gear.getCurrentPosition() > 1050 || gear.getTargetPosition() > 1050) {
                gear.setTargetPosition(1050); }
            if (lift.getCurrentPosition() < 10 || lift.getTargetPosition() < 10) {
                lift.setTargetPosition(15); }
            if (lift.getCurrentPosition() > 3000 || lift.getTargetPosition() > 3000) {
                lift.setTargetPosition(3000); }*/

            if (gamepad2.dpad_up) {
                raeg(1); }
            if (gamepad2.dpad_down) {
                raeg(-1); }
            if (gamepad2.left_trigger > .1) {
                tfil(-1); }
            if (gamepad2.right_trigger > .1) {
                tfil(1); }

            if (gamepad1.x) {gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

            //---------------High---------------\\
            if (gamepad2.x) {
                raegPosition(845, 1);//740
                tfilPosition(3500, 0.777);//2850
                pivot.setPosition(0.14); }
            //---------------Mid---------------\\
            if (gamepad2.y) {
                raegPosition(875, 1);//850
                tfilPosition(2650, 0.777);//2400
                pivot.setPosition(0.18); }
            //---------------Low---------------\\
            if (gamepad2.b) {
                raegPosition(935, 1);
                tfilPosition(1850, 0.5);
                pivot.setPosition(0.23); }
            //---------------Reset---------------\\
            if (gamepad2.a) {
                tfilPosition(25, 1);
                raegPosition(25, 0.2);
                pivot.setPosition(0.8); }

            //----------Driver1-Reset----------\\
            if (gamepad1.b) {
                raegPosition(200, 0.2);
                tfilPosition(25, 1);
                pivot.setPosition(0.10); }

            /*if (gamepad1.dpad_up) {gear.setPower(0.2); }
            if (gamepad1.dpad_down) {gear.setPower(-0.2); }
            if (gamepad1.dpad_right) {gear.setPower(0.3); }
            if (gamepad1.dpad_left) {gear.setPower(-0.3); }
            if (gamepad1.x) {gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

            if (gamepad1.dpad_up) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(0.2); }
            else if (gamepad1.dpad_down) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(-0.2); }
            else if (gamepad1.dpad_right) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(0.3); }
            else if (gamepad1.dpad_left) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(-0.3); }
            else {gear.setMode(DcMotor.RunMode.RUN_USING_ENCODER); lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }
            if (gamepad1.x) {gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

            if (gamepad1.dpad_up) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(0.2); }
            if (gamepad1.dpad_down) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(-0.2); }
            if (gamepad1.dpad_right) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(0.3); }
            if (gamepad1.dpad_left) {gear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); gear.setPower(-0.3); }
            if (gamepad1.x) {
                gear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }*/

            /*
            if (gamepad2.x && !gamepad2.y) {
                Tilt(.003); }
             else if (gamepad2.y && !gamepad2.x) {
                Tilt(-.003); }
            else if (gamepad2.x && gamepad2.y) {
                Tilt(0); }
            else {pivot.setPosition(.8 - (gear.getCurrentPosition() * 0.00048)); }
            */

            //pivot.setPosition(0.5 - (gear.getCurrentPosition() * 0.00025324));

            //----------------------------Telemetry----------------------------\\

            telemetry.addData("lift encoder", lift.getCurrentPosition());
            telemetry.addData("lift target", lift.getTargetPosition());
            telemetry.addData("gear encoder", gear.getCurrentPosition());
            telemetry.addData("gear target", gear.getTargetPosition());
            telemetry.addData("pivot position" , pivot.getPosition());
            telemetry.update();
        }
    }

    public void raegPosition (int h, double H) {gear.setTargetPosition(h); gear.setPower(H); }
    public void tfilPosition (int e, double E) {lift.setTargetPosition(e); lift.setPower(E); }

    public void raeg (int s) {
        gear.setPower(0.333);
        gear.setTargetPosition(gear.getCurrentPosition() + 50 * s);
        /*if (gear.getTargetPosition() < 10) {
            gear.setTargetPosition(10); }
        if (gear.getTargetPosition() > 1000) {
            gear.setTargetPosition(1000); }*/
    }
    public void tfil (int s) {
        lift.setPower(0.777);
        lift.setTargetPosition(lift.getCurrentPosition() + 50 * s);
        /*if (lift.getTargetPosition() < 25) {
            lift.setTargetPosition(25); }
        if (lift.getTargetPosition() > 3000) {
            lift.setTargetPosition(3000); }*/
    }
    //public void Tilt (double p) {pivot.setPosition(pivot.getPosition() + p); }
}