/* Copyright (c) 2019 FIRST. All rights reserved.
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

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Autonomous(name = "taro_tensorflow_experimentation", group = "Concept")
//@Disabled
public class taro_tensorflow_experimentation extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fldrive, frdrive, bldrive, brdrive, fly_Wheel, back_Slide, left_Slide, top_Slide; // initialize all motors
    Servo arm_servo, hand_servo, head_servo, hair1_servo, hair2_servo;

    //Servo servo;

    private static final String VUFORIA_KEY = "ARZSVjv/////AAABmbFVYMvEBEg1i14c/gYXU1QSxd7JrRov3nzCufUBnZaW+W4Brt9ma44NYvvCTPjEI6yKKvf+tHXE21sApVu52FWz2R7IXb3dUcnL8omF+I3/kH2tmCp+Ps+dygy16NJo2ZPwJe5RNTui0EJ9x6AvNz8z9UdZjMpCwTCWwZ57NW3iXWjFYRRU8sEb0Yb31uQUeN7612t8inQXvPVzTeJAW2A5u48T2alsmVw/Z9vKDmUATi1LgVNwuKa8krGlGBa2joNnt81JqOSuWotWTICmC6W6ZEqiYedlAcCTAZWSDmekIXa8m+espguXv16FzkcJAiq62FxMUFGw+aNKjnyCsubOoIRRwq2k1RZ8K9FveEUF";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.

        initVuforia();

        fldrive  = hardwareMap.get(DcMotor.class, "fl_drive");
        frdrive  = hardwareMap.get(DcMotor.class, "fr_drive");
        brdrive = hardwareMap.get(DcMotor.class, "br_drive");
        bldrive = hardwareMap.get(DcMotor.class, "bl_drive");

        //flywheels and slide initializations
        fly_Wheel = hardwareMap.get(DcMotor.class, "fly_Wheel");
        back_Slide = hardwareMap.get(DcMotor.class, "back_Slide");
        left_Slide = hardwareMap.get(DcMotor.class, "left_Slide");
        top_Slide = hardwareMap.get(DcMotor.class, "top_Slide");

        //servo initialization
        arm_servo = hardwareMap.get(Servo.class, "arm");
        hand_servo = hardwareMap.get(Servo.class, "hand");
        head_servo = hardwareMap.get(Servo.class, "head");
        hair1_servo = hardwareMap.get(Servo.class, "hair1");
        hair2_servo = hardwareMap.get(Servo.class, "hair2");

        //drive train motor directions
        fldrive.setDirection(DcMotor.Direction.FORWARD);
        frdrive.setDirection(DcMotor.Direction.REVERSE);
        brdrive.setDirection(DcMotor.Direction.REVERSE);
        bldrive.setDirection(DcMotor.Direction.FORWARD);

        //other DcMotor directions
        fly_Wheel.setDirection(DcMotor.Direction.FORWARD);// fly wheel motor
        back_Slide.setDirection(DcMotor.Direction.FORWARD);
        left_Slide.setDirection(DcMotor.Direction.FORWARD);
        top_Slide.setDirection(DcMotor.Direction.FORWARD);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD.");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                straferight(0.05, 1000);
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("% Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                      int i = 0;

                      for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)){
                              straferight(0.05, 1);
                          }
                          if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
                              //when SkyStone detected
                              forward(0.1, 2);
                              arm(0);
                              backward(0.1, 2);
                              right(0.05, 2);
                          }

                          telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                          telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                          telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                      }
                      telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {

         //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.9;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void forward(double power, int distance) {
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fldrive.setTargetPosition(distance);
        frdrive.setTargetPosition(distance);
        bldrive.setTargetPosition(distance);
        brdrive.setTargetPosition(distance);

        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fldrive.setPower(power);
        frdrive.setPower(power);
        brdrive.setPower(power);
        bldrive.setPower(power);

        while (fldrive.isBusy() && frdrive.isBusy() && bldrive.isBusy() && brdrive.isBusy()) {
            //until point reached
        }

        power = 0.0;
        fldrive.setPower(power);
        frdrive.setPower(power);
        brdrive.setPower(power);
        bldrive.setPower(power);
    }

    public void backward(double power, int distance) {
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fldrive.setTargetPosition(-distance);
        frdrive.setTargetPosition(-distance);
        bldrive.setTargetPosition(-distance);
        brdrive.setTargetPosition(-distance);

        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fldrive.setPower(-power);
        frdrive.setPower(-power);
        brdrive.setPower(-power);
        bldrive.setPower(-power);

        while (fldrive.isBusy() && frdrive.isBusy() && bldrive.isBusy() && brdrive.isBusy()) {
            //until point reached
        }

        power = 0.0;
        fldrive.setPower(-power);
        frdrive.setPower(-power);
        brdrive.setPower(-power);
        bldrive.setPower(-power);
    }

    public void left(double power, int distance) {
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fldrive.setTargetPosition(-distance);
        frdrive.setTargetPosition(distance);
        bldrive.setTargetPosition(distance);
        brdrive.setTargetPosition(-distance);

        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fldrive.setPower(-power);
        frdrive.setPower(power);
        brdrive.setPower(power);
        bldrive.setPower(-power);

        while (fldrive.isBusy() && frdrive.isBusy() && bldrive.isBusy() && brdrive.isBusy()) {
            //until point reached
        }

        power = 0.0;
        fldrive.setPower(power);
        frdrive.setPower(power);
        brdrive.setPower(power);
        bldrive.setPower(power);
    }

    public void right(double power, int distance) {
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fldrive.setTargetPosition(distance);
        frdrive.setTargetPosition(-distance);
        bldrive.setTargetPosition(-distance);
        brdrive.setTargetPosition(distance);

        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fldrive.setPower(power);
        frdrive.setPower(-power);
        brdrive.setPower(-power);
        bldrive.setPower(power);

        while (fldrive.isBusy() && frdrive.isBusy() && bldrive.isBusy() && brdrive.isBusy()) {
            //until point reached
        }

        power = 0.0;
        fldrive.setPower(-power);
        frdrive.setPower(-power);
        brdrive.setPower(-power);
        bldrive.setPower(-power);
    }
    public void strafeleft(double power, int distance) {
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fldrive.setTargetPosition(-distance);
        frdrive.setTargetPosition(distance);
        bldrive.setTargetPosition(-distance);
        brdrive.setTargetPosition(distance);

        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fldrive.setPower(-power);
        frdrive.setPower(power);
        brdrive.setPower(-power);
        bldrive.setPower(power);

        while (fldrive.isBusy() && frdrive.isBusy() && bldrive.isBusy() && brdrive.isBusy()) {
            //until point reached
        }

        power = 0.0;
        fldrive.setPower(power);
        frdrive.setPower(power);
        brdrive.setPower(power);
        bldrive.setPower(power);
    }
    public void straferight(double power, int distance) {
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fldrive.setTargetPosition(distance);
        frdrive.setTargetPosition(-distance);
        bldrive.setTargetPosition(distance);
        brdrive.setTargetPosition(-distance);

        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fldrive.setPower(power);
        frdrive.setPower(-power);
        brdrive.setPower(power);
        bldrive.setPower(-power);

        while (fldrive.isBusy() && frdrive.isBusy() && bldrive.isBusy() && brdrive.isBusy()) {
            //until point reached
        }

        power = 0.0;
        fldrive.setPower(power);
        frdrive.setPower(power);
        brdrive.setPower(power);
        bldrive.setPower(power);
    }

    //servo functions
    public void head(double position){
        head_servo.setPosition(position);
    }
    public void hand(double position){
        hand_servo.setPosition(position);
    }
    public void arm(double position) {
        arm_servo.setPosition(position);
    }
    public void hair_close(){
        hair1_servo.setPosition(0);
        hair2_servo.setPosition(0);
    }
    public void hair_open(){
        hair1_servo.setPosition(0.5);
        hair2_servo.setPosition(0.5);
    }
    public void fly_wheels_in(double power, int distance) {
        fly_Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly_Wheel.setTargetPosition(distance);
        fly_Wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fly_Wheel.setPower(power);

        while (fly_Wheel.isBusy()) {
            //until rotations complete
        }

        power = 0.0;
        fly_Wheel.setPower(power);
    }

    //linear slide functions
    public void slide_up(double power, int distance) {
        back_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_Slide.setTargetPosition(distance);
        left_Slide.setTargetPosition(distance);

        back_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_Slide.setPower(power);
        left_Slide.setPower(power);

        while (back_Slide.isBusy() && left_Slide.isBusy()) {
            //until point reached
        }

        power = 0.0;
        back_Slide.setPower(power);
        left_Slide.setPower(power);
    }
    public void slide_down(double power, int distance) {
        back_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_Slide.setTargetPosition(distance);
        left_Slide.setTargetPosition(distance);

        back_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        back_Slide.setPower(-power);
        left_Slide.setPower(-power);

        while (back_Slide.isBusy() && left_Slide.isBusy()) {
            //until point reached
        }

        power = 0.0;
        back_Slide.setPower(power);
        left_Slide.setPower(power);
    }
    public void slide_out(double power, int distance){
        top_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        top_Slide.setTargetPosition(distance);
        top_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        top_Slide.setPower(power);

        while (top_Slide.isBusy()){
            //until point reached
        }
        power = 0.0;
        top_Slide.setPower(power);
    }
    public void slide_in(double power, int distance){
        top_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        top_Slide.setTargetPosition(distance);
        top_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        top_Slide.setPower(-power);

        while (top_Slide.isBusy()){
            //until point reached
        }
        power = 0.0;
        top_Slide.setPower(power);
    }
}
