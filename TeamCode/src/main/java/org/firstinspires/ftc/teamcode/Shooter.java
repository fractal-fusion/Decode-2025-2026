package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter{
    public Gamepad currentGamepad = new Gamepad(); //gamepads for rising edge detector
    public Gamepad previousGamepad = new Gamepad();
    public DcMotor shooterLeft;
    public DcMotor shooterRight;
    public Servo shooterRampRight;
    public Servo shooterRampLeft;
    public Servo shooterPitchRight;
    public Servo shooterPitchLeft;
    private OpMode opMode;
    public static double PITCH_INTAKE_POSITION = 0.075;
    public static double RAMP_SCORE_POSITION = 0.25;
    public double testShootPower = 0;
    public double testRampPosition = 0;
    public double testPitchPosition = 0;
    boolean on = false; //boolean for on or off intake

    //test servo variables
    public static String testServo = "rampright";
    public Servo TESTSERVO;
    private double testPosition = 0.5;
    public Shooter(OpMode linearOpmode) {
        this.opMode = linearOpmode;
        shooterLeft = opMode.hardwareMap.get(DcMotor.class, "shooterleft");
        shooterRight = opMode.hardwareMap.get(DcMotor.class, "shooterright");

        shooterRampRight = opMode.hardwareMap.get(Servo.class, "rampright" );
        shooterRampLeft = opMode.hardwareMap.get(Servo.class, "rampleft" );
        shooterPitchRight = opMode.hardwareMap.get(Servo.class, "pitchright" );
        shooterPitchLeft = opMode.hardwareMap.get(Servo.class, "pitchleft" );

        TESTSERVO = opMode.hardwareMap.get(Servo.class, testServo);

        shooterRight.setDirection(DcMotor.Direction.REVERSE); //reverse shooter right motor so positive is out

        shooterPitchRight.setDirection(Servo.Direction.REVERSE);
        shooterRampRight.setDirection(Servo.Direction.REVERSE); //reverse servos used for rotation so positive is rotate up
    }

    public void updateGamepad(Gamepad gamepad) {
        previousGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad);
    }

    public void controlRampPosition(Gamepad gamepad){

        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            testRampPosition += 0.05;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            testRampPosition -= 0.05;
        }

        testRampPosition = Math.max(0, Math.min(testRampPosition, 1));

        shooterRampRight.setPosition(testRampPosition);
        shooterRampLeft.setPosition(testRampPosition);

        opMode.telemetry.addData("servoposRight", shooterRampRight.getPosition());
        opMode.telemetry.addData("servoposLeft", shooterRampLeft.getPosition());
        opMode.telemetry.update();
    }

    public void controlPitchPosition(Gamepad gamepad){

        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            testPitchPosition += 0.05;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            testPitchPosition -= 0.05;
        }

        testPitchPosition = Math.max(0, Math.min(testPitchPosition, 1));

        shooterPitchRight.setPosition(testPitchPosition);
        shooterPitchLeft.setPosition(testPitchPosition);

        opMode.telemetry.addData("servoposRight", shooterPitchRight.getPosition());
        opMode.telemetry.addData("servoposLeft", shooterPitchLeft.getPosition());
        opMode.telemetry.update();
    }
    public void setTestRampPosition(double position){
        shooterPitchRight.setPosition(position);
        shooterPitchLeft.setPosition(position);
    }

    public void setTestPitchPosition(double position){
        shooterPitchRight.setPosition(position);
        shooterPitchLeft.setPosition(position);
    }
    public void shoot(){
        //lower pitch and set ramp height, start shooter motors
        //start flicker servo
    }
    //TODO: transfer tickspersecond to rpm

    public void testShoot(Gamepad gamepad){

        if(currentGamepad.dpad_up && !previousGamepad.dpad_up){
            testShootPower += .1;
        } else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            testShootPower -= .1;
        }

        if (currentGamepad.a && !previousGamepad.a){
            on = !on;
        }

        testShootPower = Math.max(0, Math.min(testShootPower, 1));

        if(on){
            setTestPitchPosition(0);
            shooterLeft.setPower(testShootPower);
            shooterRight.setPower(testShootPower);
        }
        else {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }
    }

    public void controlTestServo(Gamepad gamepad){ //method to test individual servos

        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            testPosition += 0.05;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            testPosition -= 0.05;
        }

        testPosition = Math.max(0, Math.min(testPosition, 1));

        TESTSERVO.setPosition(testPosition);

        opMode.telemetry.addData("servopos:", TESTSERVO.getPosition());
        opMode.telemetry.update();
    }


// back motors outside, front motors at second to outside, shooters at second to inside, and rest at inside.
}
