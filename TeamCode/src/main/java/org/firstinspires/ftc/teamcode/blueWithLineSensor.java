/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="blueWithLine")

public class blueWithLineSensor extends LinearOpMode{

    private final int READ_PERIOD = 1;


    public int pixelspot = 2;


    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private DcMotor liftG   = null;  //e0
    private DcMotor liftY   = null;  //e1
    private DcMotor intake  = null;  //e2
    private DcMotor hanger  = null;  //e3
    private Servo intakeServo   = null; //es0
    private Servo pivot = null; //es1
    private Servo claw = null; //es2
    private Servo droneServo    = null; //es3
    private Servo clawauto      = null; //cs0
    private Servo clawautop     = null; //cs1
    private Servo spoiler    = null; //cs2
    private HuskyLens huskyLens;
    //TODO add your other motors and sensors here
    /** The variable to store a reference to our color sensor hardware object */
    NormalizedColorSensor leftColor;
    NormalizedColorSensor rightColor;


    static final double     WHITE_THRESHOLD = 0.5;  // spans between 0.0 - 1.0 from dark to light
    static final double     APPROACH_SPEED  = 0.25;



    @Override public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rB");
        liftG       = hardwareMap.get(DcMotor.class, "liftG");
        liftY       = hardwareMap.get(DcMotor.class, "liftY");
        intake      = hardwareMap.get(DcMotor.class, "intake");
        hanger      = hardwareMap.get(DcMotor.class, "hanger");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        pivot = hardwareMap.get(Servo.class, "pivot");
        claw = hardwareMap.get(Servo.class, "claw");
        droneServo  = hardwareMap.get(Servo.class, "droneServo");
        clawauto    = hardwareMap.get(Servo.class, "clawauto");
        clawautop   = hardwareMap.get(Servo.class, "clawautop");
        spoiler     = hardwareMap.get(Servo.class, "spoiler");
        spoiler.setPosition(0);
        intakeServo.setPosition(0);
        pivot.setPosition(.25);//1
        claw.setPosition(.77);//0
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        //TODO initialize the sensors and motors you added
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        liftG.setDirection(DcMotor.Direction.FORWARD);
        liftY.setDirection(DcMotor.Direction.REVERSE);
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS); //from huskylens example
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }//makes sure the huskylens is talking to the control hub
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);// can change to other algorithms
        //Color sensor for line
        leftColor = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");
        // If necessary, turn ON the white LED (if there is no LED switch on the sensor)
        if (leftColor instanceof SwitchableLight) {
            ((SwitchableLight)leftColor).enableLight(true);
        }
        if (rightColor instanceof SwitchableLight) {
            ((SwitchableLight)rightColor).enableLight(true);
        }
        leftColor.setGain(15);
        rightColor.setGain(15);

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        while (opModeInInit()) {

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to drive to white line.");    //

            // Display the light level while we are waiting to start
            getBrightnessR();
            getBrightnessL();
        }
        waitForStart();

        while (opModeIsActive()) {


            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();// from huskylens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());// this gives you the data
                telemetry.addData("location?", blocks[i].x);// this gives you just x
                //TODO ensure your x values of the husky lens are appropriate to the desired areas
                //----------------------------1----------------------------\\
                if (blocks[i].x < 100) {
                    telemetry.addLine("Hooray!!! Area 1");
                    //move to area one using encoder tics
                    move(670, 670, 670, 670);//straight to spike area
                    sleep(2000);
                    turn(-700, -700, 700, 700);// turn to the left
                    sleep(1000);
                    move(50, 50, 50, 50);//straight to spike area
                    sleep(1000);
                    intake.setPower(-0.4);//score the purple pixel
                    sleep(1000);
                    intake.setPower(0);
                    sleep(1000);
                    move(-50, -50, -50, -50);
                    move(-700, -700, 700, 700);//turn back
                    sleep(1000);
                    move(350, 350, 350, 350);//back away from purple pixel
                    move(-750, -750, 750, 750);//turn back
                    sleep(1000);
                    move(-1000, -1000, -1000, -1000);//back up to board
                    intake.setPower(0);
                    sleep(1200);
                    lift(2550);//raise lift
                    sleep(1000);
                    drop(); //drop pixel
                    sleep(1000);
                    lift(-200);//1st lower of lift
                    up();//plWRK// | box up
                    sleep(1000);
                    lift(-2350);
                    sleep(400000);

                    //should be facing the backdrop looking at april tags
                    pixelspot = 1;
                } else {

                }
                //----------------------------2----------------------------\\
                if (blocks[i].x > 100 && blocks[i].x < 200) {
                    telemetry.addLine("Hooray!!! Area 2");
                    move(635, 635, 635, 635);//straight to spike area
                    sleep(1500);
                    move(0, 0, 0, 0);
                    sleep(1000);
                    intake.setPower(-0.4);//score the purple pixel
                    sleep(1000);
                    //intake.setPower(0);
                   // sleep(1000);
                    intake.setPower(0);
                    move(-70, -70, -70, -70);//back up
                    sleep(1000);
                    turn(750, 750, -750, -750);//turn to the right | WAS 730, 750 untested
                    sleep(1000);
                    move(-1000, -1000, -1000, -1000);//back up to board
                    sleep(1200);
                    line();
                    lift(1000);//raise lift
                    sleep(1000);
                    drop(); //drop pixel
                    sleep(1000);
                    lift(-200);//lower just a little
                    lift(200);
                   // up();//plWRK// | box up
                    sleep(1000);
                    lift(-2000);//lower lift
                    sleep(400000);
                    pixelspot = 2;
                } else {

                }
                //----------------------------3----------------------------\\
                if (blocks[i].x > 210) {
                    telemetry.addLine("Hooray!!! Area 3");
                    move(650, 650, 650, 650);//straight to spike area
                    sleep(2000);
                    turn(700, 700, -700, -700);// turn to the right
                    sleep(2000);
                    //clawauto.setPosition(2);
                    intake.setPower(-0.5);
                    sleep(1050);
                    intake.setPower(0);
                    //move(50, 50, 50, 50); // straight to drop of pixel
                    sleep(1000);
                    move(-100, -100, -100, -100);//backp up
                    sleep(500);
                    turn(150, 150, -150, -150);// turn to 3
                    sleep(1000);
                    move(-900, -900, -900, -900);//back up to board
                    line();
                    turn(-150, -150, 150, 150);// turn to 3
                    move(-50,-50,-50,50);
                    intake.setPower(0);
                    sleep(1200);
                    lift(2550);//raise lift
                    sleep(1000);
                    drop(); //drop pixel
                    sleep(1000);
                    lift(-200);//lower just a little
                    up();//plWRK// | box up
                    sleep(1000);
                    lift(-2350);//lower lift
                    sleep(400000);
                    pixelspot = 3;
                }

            }
        }
    }

        public void intake(double in) {
        intake.setPower(-in);
        sleep(100);
        intake.setPower(0);
    }

    public void lift(int LGY) {
        liftG.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftG.setTargetPosition(LGY);
        liftY.setTargetPosition(LGY);

        liftG.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spoiler.setPosition(0.35);

        liftG.setPower(0.7);
        liftY.setPower(0.7);

        while (liftG.isBusy() && liftY.isBusy()) {
            sleep(25);
        }

        liftG.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftG.setPower(0);
        liftY.setPower(0);

        spoiler.setPosition(0);
    }

    public void drop() {
        pivot.setPosition(.45);    //
        sleep(500);
        claw.setPosition(.65);    //1
        sleep(1000);
        claw.setPosition(0.77);
        pivot.setPosition(.25);
    }

    public void up() {
        pivot.setPosition(.5);      //0
        claw.setPosition(.5);      //1

        sleep(1000);
    }

    public void move(int lf, int lb, int rf, int rb) {
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setTargetPosition(rf);
        rightBackDrive.setTargetPosition(rb);
        leftFrontDrive.setTargetPosition(lf);
        leftBackDrive.setTargetPosition(lb);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setPower(0.3);
        rightFrontDrive.setPower(0.3);
        leftFrontDrive.setPower(0.3);
        leftBackDrive.setPower(0.3);

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            sleep(25);

        }
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }
    public void turn(int lf, int lb, int rf, int rb) {
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setTargetPosition(rf);
        rightBackDrive.setTargetPosition(rb);
        leftFrontDrive.setTargetPosition(lf);
        leftBackDrive.setTargetPosition(lb);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setPower(0.3);
        rightFrontDrive.setPower(0.3);
        leftFrontDrive.setPower(0.3);
        leftBackDrive.setPower(0.3);

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            sleep(25);

        }
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }
    // to obtain reflected light, read the normalized values from the color sensor.  Return the Alpha channel.
    double getBrightnessR() {
        NormalizedRGBA colors = rightColor.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
        telemetry.update();

        return colors.alpha;
    }
    double getBrightnessL() {
        NormalizedRGBA colors = leftColor.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
        telemetry.update();

        return colors.alpha;
    }
    public void line(){
        // Start the robot moving forward, and then begin looking for a white line.
        rightBackDrive.setPower(APPROACH_SPEED);
        leftFrontDrive.setPower(APPROACH_SPEED);
        rightFrontDrive.setPower(APPROACH_SPEED);
        leftBackDrive.setPower(APPROACH_SPEED);

        //
        if(getBrightnessL() <WHITE_THRESHOLD){
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
        }
        if(getBrightnessR() < WHITE_THRESHOLD){
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }
        // Stop all motors


    }
}