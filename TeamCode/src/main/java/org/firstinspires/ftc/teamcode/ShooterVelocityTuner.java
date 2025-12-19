
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="shooterVelocityTuner", group="Robot")
@Config
public class ShooterVelocityTuner extends LinearOpMode {
    DcMotorEx shooterRight;
    DcMotorEx shooterLeft;
    double currentVelocity;
    public static double PID_OFFSET = 9.002; //right motor is always slower
    public static double P = 1.375;
    public static double I = 0.1375;
    public static double D = 0;
    public static double F = 13.750004;
    public static double TARGET_RPM = Shooter.CLOSE_TARGET_RPM;


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterright");
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders on initialization
        shooterRight.setVelocityPIDFCoefficients(P, I, D, F);

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterleft");
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders on initialization
        shooterLeft.setVelocityPIDFCoefficients(P, I, D, F);

        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            shooterRight.setVelocity(TARGET_RPM * Shooter.RPM_TO_TICKS_PER_SECOND);
            shooterLeft.setVelocity(TARGET_RPM * Shooter.RPM_TO_TICKS_PER_SECOND);

            currentVelocity = shooterRight.getVelocity();

            telemetry.addData("current velocity", currentVelocity * Shooter.TICKS_PER_SECOND_TO_RPM);
            telemetry.addData("target velocity", TARGET_RPM);
            telemetry.update();
        }
    }
}
