package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="testOpmode", group="Robot")
public class testOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(this);
        Intake intake = new Intake(this, Intake.FLICKER_OPEN_POSITION);
        Drivetrain drivetrain = new Drivetrain(this);
        IndicatorLight indicatorLight = new IndicatorLight(this);
//        ColorDetector colorDetector = new ColorDetector(this);
        shooter.setPitchPosition(0);
//        intake.setFlickerPosition(0);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            shooter.updateGamepad(gamepad1);

//            drivetrain.drive(gamepad1);
//            drivetrain.testWheel();

            shooter.testShoot(gamepad1);

            indicatorLight.setRGB();

            telemetry.addData("shooter power:", shooter.on);

            if (gamepad1.a) {
                intake.intakeFront.setPower(1);
                intake.intakeBack.setPower(1);
            } else if (gamepad1.b) {
                intake.intakeFront.setPower(-1);
                intake.intakeBack.setPower(-1);
            } else {
                intake.intakeFront.setPower(0);
                intake.intakeBack.setPower(0);
            }
            if (gamepad1.dpad_right) {
//                drivetrain.setTiltPosition(0.5);
            } else if (gamepad1.dpad_left) {
//                drivetrain.setTiltPosition(0);
            }

//            shooter.testControlServo(gamepad1);
//            shooter.testControlRampPosition(gamepad1);
//            shooter.testControlPitchPosition(gamepad1);
//            colorDetector.telemetryColors();

            telemetry.update();
        }
    }
}