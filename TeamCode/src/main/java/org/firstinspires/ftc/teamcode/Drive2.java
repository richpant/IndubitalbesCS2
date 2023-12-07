package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled           //this is the previous code for the robot before Thanksgiving break
@TeleOp(name="Drive2")
public class Drive2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lF      = null;  //c0
    private DcMotor rF      = null;  //c1
    private DcMotor lB      = null;  //c2
    private DcMotor rB      = null;  //c3
    private DcMotor liftG   = null;  //e0
    private DcMotor liftY   = null;  //e1
    private DcMotor intake  = null;  //e2
    private DcMotor hanger  = null;  //e3
    private Servo intakeServo   = null; //es0
    private Servo pivot = null; //es1
    private Servo claw = null; //es2
    private Servo hangerServo   = null; //es3
    private Servo droneServo    = null; //es4
    private Servo clawauto      = null; //cs0
    private Servo clawautop     = null; //cs1
    private Servo spoiler       = null; //cs2

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
        hangerServo = hardwareMap.get(Servo.class, "hangerServo");
        pivot = hardwareMap.get(Servo.class, "pivot");
        claw = hardwareMap.get(Servo.class, "claw");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        clawauto = hardwareMap.get(Servo.class, "clawauto");
        clawautop = hardwareMap.get(Servo.class, "clawautop");
        spoiler = hardwareMap.get(Servo.class, "spoiler");
        //spoiler.setPosition(0.35);
        intakeServo.setPosition(0);
        //pivot.setPosition(.4);//1
        //claw.setPosition(.753);//0   .753
        pivot.setPosition(.25);
        hangerServo.setPosition(0);
        droneServo.setPosition(0);
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
                rBPower /= max;
            }
            /*lF.setPower(lFPower);
            rF.setPower(rFPower);
            lB.setPower(lBPower);
            rB.setPower(rBPower);*/

            //----------------------------Drive-Speed----------------------------\\

            lF.setPower(0.6 * lFPower);
            rF.setPower(0.6 * rFPower);
            lB.setPower(0.6 * lBPower);
            rB.setPower(0.6 * rBPower);
            if (gamepad1.left_bumper) {//speed for going across field
                lF.setPower(lFPower);
                rF.setPower(rFPower);
                lB.setPower(lBPower);
                rB.setPower(rBPower);}
            if (gamepad1.right_bumper) {//slow for corrections
                lF.setPower(0.1 * lFPower);
                rF.setPower(0.1 * rFPower);
                lB.setPower(0.1 * lBPower);
                rB.setPower(0.1 * rBPower);}

            //----------------------------intake----------------------------\\

            if (gamepad2.right_bumper) {
                intake.setPower(-0.5);          //in
                spoiler.setPosition(0); }       //up
            else if (gamepad2.left_bumper) {
                intake.setPower(1);             //out
                spoiler.setPosition(0); }       //up
            else {intake.setPower(0);           //stop
                spoiler.setPosition(0.35); }    //down

            if (gamepad2.y) {
                intakeServo.setPosition(0.555);
            } else if (gamepad2.x) {
                intakeServo.setPosition(0.43);
            } else {
                intakeServo.setPosition(0); }

            //----------------------------pivot----------------------------\\

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

            /*

            if (gamepad2.a) {
                pivot.setPosition(0.35); }
                claw.setPosition(0.35); }//close 0
            else {
                pivot.setPosition(0.75); }

            */
            //swing arm
            if (gamepad2.a) {
                pivot.setPosition(0.33); }//.25 swing out
            else {
                pivot.setPosition(0.65); }//.7 static

            //----------------------------claw----------------------------\\

            /*while(!gamepad2.a){
              pivot.setPosition(0.85); }

            if (gamepad2.a) {
                pivot.setPosition(.375);    //0 //Swing
                claw.setPosition(.75); }  //1
                */
            //claw
            if (gamepad2.b) {            //if using gobilda servos
                claw.setPosition(0.4); }//.33 close
            else {
                claw.setPosition(.48); }//.48 open

            /*if (gamepad2.a) {
                pivot.setPosition(.375);    //0
                claw.setPosition(.625); }  //1
            else if (gamepad2.b) {
                pivot.setPosition(.2);      //0
                claw.setPosition(.8); }    //1
            else {
                pivot.setPosition(.5);      //0
                claw.setPosition(.5); }    //1*/

            //----------------------------hanger----------------------------\\

            if (gamepad2.dpad_up) {
                hanger.setPower(1);
                hangerServo.setPosition(0.163);
            } else if (gamepad2.dpad_down) {
                hanger.setPower(-1);
                hangerServo.setPosition(0.163);
            } else {
                hanger.setPower(0); }

            if (gamepad2.dpad_left) {
                hangerServo.setPosition(0); }

            //----------------------------droneServo----------------------------\\

            if (gamepad1.a) {
                droneServo.setPosition(0.7);
            } else {
                droneServo.setPosition(0); }

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
    }}
