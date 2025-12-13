
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="shooterVelocityTuner", group="Robot")
@Config
public class ShooterVelocityTuner extends LinearOpMode {

    public static String hardwareMapName = "shooterright";
    DcMotorEx motor;
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

        motor = hardwareMap.get(DcMotorEx.class, hardwareMapName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders on initialization
        motor.setVelocityPIDFCoefficients(P, I, D, F);


        waitForStart();


        while (opModeIsActive()) {
            motor.setVelocity(TARGET_RPM * Shooter.RPM_TO_TICKS_PER_SECOND);

            currentVelocity = motor.getVelocity();

            telemetry.addData("current velocity", currentVelocity * Shooter.TICKS_PER_SECOND_TO_RPM);
            telemetry.addData("target velocity", TARGET_RPM);
            telemetry.update();
        }
    }
}
