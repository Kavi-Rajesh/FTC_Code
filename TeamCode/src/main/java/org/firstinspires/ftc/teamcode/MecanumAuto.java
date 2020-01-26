package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//this is the Drivetrain Autonomous Code for Team Taro.
//this uses Mecanum wheels. They are a bit different from normal wheels: not only can they turn and move forward/backward, but they can also strafe left and strafe right.

@Autonomous(name="MecanumAuto", group="Linear OpMode")

public class MecanumAuto extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    //define motors
    DcMotor fldrive; //motor 0
    DcMotor frdrive; //motor 1
    DcMotor brdrive; //motor 2
    DcMotor bldrive; //motor 3
    //DcMotor flywheel; //motor for flywheels
    

    //main program
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Intialized");
        telemetry.update();

        //to 'get' means to configure the robot according to the motor names
        fldrive  = hardwareMap.get(DcMotor.class, "fl_drive");
        frdrive  = hardwareMap.get(DcMotor.class, "fr_drive");
        brdrive = hardwareMap.get(DcMotor.class, "br_drive");
        bldrive = hardwareMap.get(DcMotor.class, "bl_drive");
        //flywheel = hardwareMap.get(DcMotor.class, "fly_wheel")


        //set the direction of each of the motors
        fldrive.setDirection(DcMotor.Direction.FORWARD);
        frdrive.setDirection(DcMotor.Direction.REVERSE);
        brdrive.setDirection(DcMotor.Direction.REVERSE);
        bldrive.setDirection(DcMotor.Direction.FORWARD);
        //flywheel.setDirection(DcMotor.Direction.REVERSE)

        //wait for the START button to be pressed
        waitForStart();
        runtime.reset();




    }

    //functions we will call into the main program

    /*
    //using fly wheels function
    public void flywheels(double flypower, int flytime)
    {
        //set power
        flyWheel.setPower(flypower);

        //rotate for amount of time
        sleep(flytime);

        //stop fly wheels
        flypower = 0.0;

        flyWheel.setPower(flypower);
    }
    */

    //driving and strafing functions below

    //drive forward using encoders
    public void driveforward(double drivepower, int distance)
    {
        //reset encoders
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target distance
        fldrive.setTargetPosition(distance);
        frdrive.setTargetPosition(distance);
        brdrive.setTargetPosition(distance);
        bldrive.setTargetPosition(distance);

        //Set to run to position
        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        forward(drivepower);

        //stop driving and set modes back to normal
        stopdriving();
        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //used in driving forward using encoders
    public void forward(double drivepower)
    {
        fldrive.setPower(-drivepower);
        frdrive.setPower(-drivepower);
        brdrive.setPower(-drivepower);
        bldrive.setPower(-drivepower);
    }

    //drive backward using encoders
    public void drivebackward(double drivepower, int distance)
    {
        //reset encoders
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target distance
        fldrive.setTargetPosition(distance);
        frdrive.setTargetPosition(distance);
        brdrive.setTargetPosition(distance);
        bldrive.setTargetPosition(distance);

        //Set to run to position
        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        backward(drivepower);

        //stop driving and set modes back to normal
        stopdriving();
        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //used in driving backward using encoders
    public void backward(double drivepower)
    {
        fldrive.setPower(-drivepower);
        frdrive.setPower(-drivepower);
        brdrive.setPower(-drivepower);
        bldrive.setPower(-drivepower);
    }

    //strafe left using encoders
    public void drivestrafeleft(double drivepower, int distance)
    {
        //reset encoders
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target distance
        fldrive.setTargetPosition(distance);
        frdrive.setTargetPosition(distance);
        brdrive.setTargetPosition(distance);
        bldrive.setTargetPosition(distance);

        //Set to run to position
        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        strafeleft(drivepower);

        //stop driving and set modes back to normal
        stopdriving();
        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //used in strafing left using encoders
    public void strafeleft(double drivepower)
    {
        fldrive.setPower(-drivepower);
        frdrive.setPower(drivepower);
        brdrive.setPower(-drivepower);
        bldrive.setPower(drivepower);
    }

    //strafe right using encoders
    public void drivestraferight(double drivepower, int distance)
    {
        //reset encoders
        fldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target distance
        fldrive.setTargetPosition(distance);
        frdrive.setTargetPosition(distance);
        brdrive.setTargetPosition(distance);
        bldrive.setTargetPosition(distance);

        //Set to run to position
        fldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        straferight(drivepower);

        //stop driving and set modes back to normal
        stopdriving();
        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //used in strafing right using encoders
    public void straferight(double drivepower)
    {
        fldrive.setPower(drivepower);
        frdrive.setPower(-drivepower);
        brdrive.setPower(drivepower);
        bldrive.setPower(-drivepower);
    }

    //used in driving and strafing using encoders
    public void stopdriving()
    {
        fldrive.setPower(0);
        frdrive.setPower(0);
        brdrive.setPower(0);
        bldrive.setPower(0);
    }

    //rotate left function
    public void rotateleft(double rotatepower, long rotatetime)
    {
        //set power
        fldrive.setPower(-rotatepower);
        frdrive.setPower(rotatepower);
        brdrive.setPower(rotatepower);
        bldrive.setPower(-rotatepower);

        //rotate for amount of time
        sleep(rotatetime);

        //stop rotating
        rotatepower = 0.0;

        fldrive.setPower(rotatepower);
        frdrive.setPower(rotatepower);
        brdrive.setPower(rotatepower);
        bldrive.setPower(rotatepower);
    }

    //rotate right function
    public void rotateright(double rotatepower, long rotatetime)
    {
        //set power
        fldrive.setPower(rotatepower);
        frdrive.setPower(-rotatepower);
        brdrive.setPower(-rotatepower);
        bldrive.setPower(rotatepower);

        //rotate for amount of time
        sleep(rotatetime);

        //stop rotating
        rotatepower = 0.0;

        fldrive.setPower(rotatepower);
        frdrive.setPower(rotatepower);
        brdrive.setPower(rotatepower);
        bldrive.setPower(rotatepower);
    }
}

