package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


public class Shooter{
    public Gamepad currentGamepad = new Gamepad(); //gamepads for rising ed
    public Gamepad previousGamepad = new Gamepad();
    public DcMotor shooterLeft;
    public DcMotor shooterRight;
    public Servo shooterRampRight;
    public Servo shooterRampLeft;
    public Servo shooterRaiserRight;
    public Servo shooterRaiserLeft;
    public double shootPower = 0;
    public double rampPosition = 0;
    private final double AngleToServo = 1/180;
    private final double raisedRampPosition = 0.8;
    private OpMode opMode;
    public Shooter(OpMode linearOpmode) {
        this.opMode = linearOpmode;
        shooterLeft = opMode.hardwareMap.get(DcMotor.class, "shooterleft");
        shooterRight = opMode.hardwareMap.get(DcMotor.class, "shooterright");

        shooterRampRight = opMode.hardwareMap.get(Servo.class, "rampright" );
        shooterRampLeft = opMode.hardwareMap.get(Servo.class, "rampleft" );
        shooterRaiserLeft = opMode.hardwareMap.get(Servo.class, "raiserleft" );
        shooterRaiserRight = opMode.hardwareMap.get(Servo.class, "raiserright" );

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
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

    public void controlRampAngle(Gamepad gamepad){
        updateGamepad(gamepad);

        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            rampPosition += 0.05;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            rampPosition -= 0.05;
        }

        rampPosition = Math.max(0, Math.min(shootPower, 1));

        shooterRampRight.setPosition(rampPosition);
        shooterRampLeft.setPosition(rampPosition);
    }
    public void raiseRamp(){
        shooterRaiserRight.setPosition(raisedRampPosition);
        shooterRaiserLeft.setPosition(raisedRampPosition);
    }

}
