package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter{
    public Gamepad currentGamepad = new Gamepad(); //gamepads for rising ed
    public Gamepad previousGamepad = new Gamepad();
    public DcMotor shooterLeft;
    public DcMotor shooterRight;
    public Servo shooterRampRight;
    public Servo shooterRampLeft;
    public Servo shooterPitchRight;
    public Servo shooterPitchLeft;
    public double shootPower = 0;
    public double rampPosition = 0;
    public double pitchPosition = 0;
    private OpMode opMode;
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

        shooterLeft.setDirection(DcMotor.Direction.REVERSE); //reverse shooter left motor

        shooterPitchRight.setDirection(Servo.Direction.REVERSE);
        shooterRampRight.setDirection(Servo.Direction.REVERSE); //reverse servos used for pitch rotation
    }

    public void updateGamepad(Gamepad gamepad) {
        previousGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad);
    }
    public void testShoot(Gamepad gamepad){

        updateGamepad(gamepad);

        boolean on = false;

        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            shootPower += .1;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            shootPower -= .1;
        }

        if (currentGamepad.a && !previousGamepad.a){
            on = !on;
        }
        shootPower = Math.max(0, Math.min(shootPower, 1));
        if(on){
            shooterLeft.setPower(shootPower);
            shooterRight.setPower(shootPower);
        }
    }

    public void controlRampPosition(Gamepad gamepad){
        updateGamepad(gamepad);

        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            rampPosition += 0.05;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            rampPosition -= 0.05;
        }

        rampPosition = Math.max(0, Math.min(rampPosition, 1));

        shooterRampRight.setPosition(rampPosition);
        shooterRampLeft.setPosition(rampPosition);

        opMode.telemetry.addData("servoposRight", shooterRampRight.getPosition());
        opMode.telemetry.addData("servoposLeft", shooterRampLeft.getPosition());
        opMode.telemetry.update();
    }

    public void controlPitchPosition(Gamepad gamepad){
        updateGamepad(gamepad);

        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            pitchPosition += 0.05;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            pitchPosition -= 0.05;
        }

        pitchPosition = Math.max(0, Math.min(pitchPosition, 1));

        shooterPitchRight.setPosition(pitchPosition);
        shooterPitchLeft.setPosition(pitchPosition);

        opMode.telemetry.addData("servoposRight", shooterPitchRight.getPosition());
        opMode.telemetry.addData("servoposLeft", shooterPitchLeft.getPosition());
        opMode.telemetry.update();
    }

    public void shoot(){
        //lower pitch and set ramp height, start shooter motors
        //start flicker servo
    }

    public void controlTestServo(Gamepad gamepad){ //method to test individual servos
        updateGamepad(gamepad);

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
