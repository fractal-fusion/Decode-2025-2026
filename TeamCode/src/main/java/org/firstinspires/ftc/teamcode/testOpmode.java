package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="testShooterOpmode", group="Robot")
public class testOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        Shooter shooter = new Shooter(this);
        Intake intake = new Intake(this);
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
//            shooter.testShoot(gamepad1);
//            telemetry.addData("power", shooter.shootPower);
//            telemetry.update();
//            if(gamepad1.dpad_up) {
//                shooter.servoTest(shooter.shooterRampRight, .5);
//            }

            if(gamepad1.x) {
                intake.intake.setPower(1);
            }
            else if (gamepad1.b) {
                intake.intake.setPower(-1);
            }
            if (gamepad1.dpad_right){
                intake.flickerPos(.6);

            }
            else if(gamepad1.dpad_left){
                intake.flickerPos(0);
            }
        }
    }
}