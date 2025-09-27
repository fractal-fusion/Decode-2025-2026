package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Shooter{
    public Gamepad currentGamepad = new Gamepad(); //gamepads for rising ed
    public Gamepad previousGamepad = new Gamepad();
    public DcMotor shooterLeft;
    public DcMotor shooterRight;
    public double power = 0;

    private OpMode opMode;
    public Shooter(OpMode linearOpmode) {
        this.opMode = linearOpmode;
        shooterLeft = opMode.hardwareMap.get(DcMotor.class, "shooterleft");
        shooterRight = opMode.hardwareMap.get(DcMotor.class, "shooterright");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void updateGamepad(Gamepad gamepad) {
        previousGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad);
    }
    public void shoot(Gamepad gamepad){

        updateGamepad(gamepad);

        boolean on = false;

        if(currentGamepad.right_bumper && !previousGamepad.right_bumper){
            power += .1;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            power -= .1;
        }

        if (currentGamepad.a && !previousGamepad.a){
            on = !on;
        }
        power = Math.max(0, Math.min(power, 1));
        if(on){
            shooterLeft.setPower(power);
            shooterRight.setPower(power);
        }
    }

}
