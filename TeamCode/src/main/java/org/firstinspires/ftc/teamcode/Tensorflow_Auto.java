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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;


@Autonomous(name = "Tensorflow_Auto", group = "Concept")

public class Tensorflow_Auto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fldrive, frdrive, bldrive, brdrive, fly_Wheel_R, fly_Wheel_L, right_Slide, left_Slide; // initialize all motors
    Servo lift_left, lift_right, rotate_left, rotate_right, rotate_center; // initialize all servos

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

        fldrive  = hardwareMap.get(DcMotor.class, "fldrive");
        frdrive  = hardwareMap.get(DcMotor.class, "frdrive");
        brdrive = hardwareMap.get(DcMotor.class, "brdrive");
        bldrive = hardwareMap.get(DcMotor.class, "bldrive");

        //flywheels and slide initializations
        fly_Wheel_R = hardwareMap.get(DcMotor.class, "fly_Wheel_R");
        fly_Wheel_L = hardwareMap.get(DcMotor.class, "fly_Wheel_L");
        right_Slide = hardwareMap.get(DcMotor.class, "right_Slide");
        left_Slide = hardwareMap.get(DcMotor.class, "left_Slide");

        //servo initialization
        lift_left = hardwareMap.get(Servo.class, "left-lifter");
        lift_right = hardwareMap.get(Servo.class, "right-lifter");
        rotate_center = hardwareMap.get(Servo.class, "center-rotator");
        rotate_left = hardwareMap.get(Servo.class, "left-rotator");
        rotate_right = hardwareMap.get(Servo.class, "right-rotator");

        //drive train motor directions
        fldrive.setDirection(DcMotor.Direction.FORWARD);
        frdrive.setDirection(DcMotor.Direction.REVERSE);
        brdrive.setDirection(DcMotor.Direction.REVERSE);
        bldrive.setDirection(DcMotor.Direction.FORWARD);

        //other DcMotor directions
        fly_Wheel_R.setDirection(DcMotor.Direction.FORWARD);
        fly_Wheel_L.setDirection(DcMotor.Direction.REVERSE);
        right_Slide.setDirection(DcMotor.Direction.FORWARD);
        left_Slide.setDirection(DcMotor.Direction.FORWARD);

        //set up encoders
        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fly_Wheel_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly_Wheel_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD.");
        }

        //activates Tensorflow and displays camera view on init
        if (tfod != null) {
            tfod.activate();
        }


        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                straferight(0.05, 1000);
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("% Object Detected", updatedRecognitions.size());
                      int i = 0;
                      float distance_ratio;
                      int img_height;
                      float obj_height;

                      for (Recognition recognition : updatedRecognitions) {
                          img_height = recognition.getImageHeight();
                          obj_height = recognition.getHeight();
                          distance_ratio = img_height/obj_height;

                          if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)){
                              //do nothing, keep moving
                          }
                          if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
                              //when SkyStone detected
                              //distance for functions are currently all estimates, need to be changed
                              while (distance_ratio != .2) {
                                  forward(0.4, 10);
                              }
                              forward(0.1, 2);
                              //suck in block
                              fly_wheels_in(0.5, 100);
                              backward(0.1, 2);
                              right(0.05, 2);
                              forward(0.1, 100);
                              //drop skystone
                              fly_wheels_out (0.5, 100);
                              //park on line
                              backward(0.1, 4);
                              tfod.deactivate();
                              break;
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

        fldrive.setPower(0);
        frdrive.setPower(0);
        brdrive.setPower(0);
        bldrive.setPower(0);
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
        
        fldrive.setPower(0);
        frdrive.setPower(0);
        brdrive.setPower(0);
        bldrive.setPower(0);
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

        fldrive.setPower(0);
        frdrive.setPower(0);
        brdrive.setPower(0);
        bldrive.setPower(0);
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

        fldrive.setPower(0);
        frdrive.setPower(0);
        brdrive.setPower(0);
        bldrive.setPower(0);
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

        fldrive.setPower(0);
        frdrive.setPower(0);
        brdrive.setPower(0);
        bldrive.setPower(0);
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

        fldrive.setPower(0);
        frdrive.setPower(0);
        brdrive.setPower(0);
        bldrive.setPower(0);
    }

    // servo functions
    public void center_rotater (double position) {
        rotate_center.setPosition(position);
    }

    public void double_rotater (double position){
        rotate_left.setPosition(position);
        rotate_right.setPosition(position);
    }

    public void extend_arm (double position){
        lift_left.setPosition(position);
        lift_right.setPosition(position);
    }

    public void unextend_arm (double position){
        lift_left.setPosition(-position);
        lift_right.setPosition(-position);
    }

    public void raise_arm (double position){
        rotate_left.setPosition(position);
        rotate_right.setPosition(position);
    }

    public void lower_arm (double position){
        rotate_left.setPosition(-position);
        rotate_right.setPosition(-position);
    }

    // flywheel functions
    public void fly_wheels_in(double power, int distance) {
        fly_Wheel_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly_Wheel_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fly_Wheel_R.setTargetPosition(distance);
        fly_Wheel_L.setTargetPosition(distance);

        fly_Wheel_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fly_Wheel_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fly_Wheel_R.setPower(power);
        fly_Wheel_L.setPower(power);

        while (fly_Wheel_R.isBusy() && fly_Wheel_L.isBusy()) {
            //until rotations complete
        }

        fly_Wheel_R.setPower(0);
        fly_Wheel_L.setPower(0);
    }

    public void fly_wheels_out(double power, int distance) {
        fly_Wheel_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly_Wheel_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fly_Wheel_R.setTargetPosition(distance);
        fly_Wheel_L.setTargetPosition(distance);

        fly_Wheel_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fly_Wheel_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fly_Wheel_R.setPower(-power);
        fly_Wheel_L.setPower(-power);

        while (fly_Wheel_R.isBusy() && fly_Wheel_L.isBusy()) {
            //until rotations complete
        }

        fly_Wheel_R.setPower(0);
        fly_Wheel_L.setPower(0);
    }

    // linear slide functions
    public void slide_up(double power, int distance) {
        right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_Slide.setTargetPosition(distance);
        left_Slide.setTargetPosition(distance);

        right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right_Slide.setPower(power);
        left_Slide.setPower(power);

        while (right_Slide.isBusy() && left_Slide.isBusy()) {
            //until point reached
        }

        power = 0.0;
        right_Slide.setPower(power);
        left_Slide.setPower(power);
    }

    public void slide_down(double power, int distance) {
        right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_Slide.setTargetPosition(distance);
        left_Slide.setTargetPosition(distance);

        right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right_Slide.setPower(-power);
        left_Slide.setPower(-power);

        while (right_Slide.isBusy() && left_Slide.isBusy()) {
            //until point reached
        }

        power = 0.0;
        right_Slide.setPower(power);
        left_Slide.setPower(power);
    }

}
