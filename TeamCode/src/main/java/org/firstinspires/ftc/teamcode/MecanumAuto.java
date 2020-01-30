package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//this is the Drivetrain Autonomous Code for Team Taro.
//this uses Mecanum wheels. They are a bit different from normal wheels: not only can they turn and move forward/backward, but they can also strafe left and strafe right.

@Autonomous(name="MecanumAuto", group="Linear OpMode")

public class MecanumAuto extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fldrive, frdrive, bldrive, brdrive, flywheel, leftslide, rightslide; // arm;
    private Servo leftextender, rightextender, leftrotater, rightrotater, centerrotater, grabber1, grabber2;


    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Intialized");
        telemetry.update();

        fldrive  = hardwareMap.get(DcMotor.class, "fl_drive");
        frdrive  = hardwareMap.get(DcMotor.class, "fr_drive");
        brdrive = hardwareMap.get(DcMotor.class, "br_drive");
        bldrive = hardwareMap.get(DcMotor.class, "bl_drive");
        flywheel = hardwareMap.get(DcMotor.class, "fly_wheel");

        leftslide = hardwareMap.get(DcMotor.class, "left_slide");
        rightslide = hardwareMap.get(DcMotor.class, "right_slide");
        //arm = hardwareMap.get(DcMotor.class, "arm");

        /*
        leftextender = hardwareMap.get(Servo.class, "left_extender");
        rightextender = hardwareMap.get(Servo.class, "right_extender");
        leftrotater = hardwareMap.get(Servo.class, "left_rotater");
        rightrotater = hardwareMap.get(Servo.class, "right_rotater");
        centerrotater = hardwareMap.get(Servo.class, "center_rotater");
        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");
        */

        fldrive.setDirection(DcMotor.Direction.FORWARD);
        frdrive.setDirection(DcMotor.Direction.REVERSE);
        brdrive.setDirection(DcMotor.Direction.REVERSE);
        bldrive.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        leftslide.setDirection(DcMotor.Direction.FORWARD);
        rightslide.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        //write code here using functions
        forward(0.1, 2000);
        straferight(1, 3000);
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
    public void rotate_center (double position) {
        centerrotater.setPosition(position);
    }

    public void double_rotater (double position){
        leftrotater.setPosition(position);
        rightrotater.setPosition(-position);
    }

    public void slide_top (double position){
        leftextender.setPosition(position);
        rightextender.setPosition(position);
    }

    public void fly_wheels_in(double power, int distance) {
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setTargetPosition(distance);
        flywheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flywheel.setPower(power);

        while (flywheel.isBusy()) {
            //until rotations complete
        }

        power = 0.0;
        flywheel.setPower(power);
    }

    public void fly_wheels_out(double power, int distance) {
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setTargetPosition(distance);
        flywheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flywheel.setPower(-1 * power);

        while (flywheel.isBusy()) {
            //until rotations complete
        }

        power = 0.0;
        flywheel.setPower(power);
    }

    //linear slide functions
    public void slide_up(double power, int distance) {
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightslide.setTargetPosition(distance);
        leftslide.setTargetPosition(distance);

        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightslide.setPower(power);
        leftslide.setPower(power);

        while (rightslide.isBusy() && leftslide.isBusy()) {
            //until point reached
        }

        power = 0.0;
        rightslide.setPower(power);
        leftslide.setPower(power);
    }

    public void slide_down(double power, int distance) {
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightslide.setTargetPosition(distance);
        leftslide.setTargetPosition(distance);

        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightslide.setPower(-power);
        leftslide.setPower(-power);

        while (rightslide.isBusy() && leftslide.isBusy()) {
            //until point reached
        }

        power = 0.0;
        rightslide.setPower(power);
        leftslide.setPower(power);
    }
}

