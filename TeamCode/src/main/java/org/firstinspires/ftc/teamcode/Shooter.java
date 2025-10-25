package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    //static variables for shooter
    public static double RPM_TO_TICKS_PER_SECOND = 28.0 / 60.0; //divide rpm by 60 to get rotations per second, then multiply by 28 since that's the ticks per revolution

    public static double CYCLING_RPM = 800;
    public static double PITCH_INTAKE_POSITION = 0.05;
    public static double PITCH_CYCLE_POSITION = 0.2;
    public static double RAMP_CYCLE_POSITION = 0.2;
    public static double FAR_RAMP_SCORE_POSITION = 0.2; //temporarily set to the close position
    public static double FAR_TARGET_RPM = 4150; //temporarily set to the close position
    public static double FAR_TARGET_RPM_TICKS_PER_SECOND = FAR_TARGET_RPM * RPM_TO_TICKS_PER_SECOND;

    public static double CLOSE_RAMP_SCORE_POSITION = 0.2;
    public static double CLOSE_TARGET_RPM = 4500;
    public static double CLOSE_TARGET_RPM_TICKS_PER_SECOND = CLOSE_TARGET_RPM * RPM_TO_TICKS_PER_SECOND;

    public double currentRampScorePosition;
    public double currentTargetRPMTicksPerSecond;

    public static double TARGET_RPM_TOLERANCE_TICKS_PER_SECOND = 50;
                                                                            //which multiplied by 28 ticks per revolution returns ticks per second
    public double testShootPower = 0;
    public double testRampPosition = 0;
    public double testPitchPosition = 0;
    public boolean on = false; //boolean for on or off shooter
    public boolean cycling = false; //boolean for cycling or not
    private ElapsedTime timer;
    public double currentPitchTime;
    public static double PITCH_DEBOUNCE_SECONDS = 0.2;

    //test servo variables
    public static String testServo = "rampright";
    public Servo TESTSERVO;
    private double testPosition = 0.5;
    public Shooter(OpMode linearOpmode) {
        this.opMode = linearOpmode;
        timer = new ElapsedTime();

        shooterLeft = opMode.hardwareMap.get(DcMotor.class, "shooterleft");
        shooterRight = opMode.hardwareMap.get(DcMotor.class, "shooterright");

        shooterRampRight = opMode.hardwareMap.get(Servo.class, "rampright" );
        shooterRampLeft = opMode.hardwareMap.get(Servo.class, "rampleft" );
        shooterPitchRight = opMode.hardwareMap.get(Servo.class, "pitchright" );
        shooterPitchLeft = opMode.hardwareMap.get(Servo.class, "pitchleft" );

        TESTSERVO = opMode.hardwareMap.get(Servo.class, testServo);

        shooterRight.setDirection(DcMotor.Direction.REVERSE); //reverse shooter right motor so positive is out

//        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //slow down the motor faster

        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders on initialization

        shooterPitchRight.setDirection(Servo.Direction.REVERSE);
        shooterRampRight.setDirection(Servo.Direction.REVERSE); //reverse servos used for rotation so positive is rotate up

        shooterRampRight.setPosition(0); //zero ramp servoes on initialization
        shooterRampLeft.setPosition(0);

        setCurrentTargetRPMTicksPerSecond(CLOSE_TARGET_RPM); //default the current target rpm ticks per second to close target rpm
    }

    public void turnOnShooter(){
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ((DcMotorEx) shooterRight).setVelocity(currentTargetRPMTicksPerSecond);
        ((DcMotorEx) shooterLeft).setVelocity(currentTargetRPMTicksPerSecond);
    }

    public void turnOnShooter(double RPM){
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ((DcMotorEx) shooterRight).setVelocity(RPM * RPM_TO_TICKS_PER_SECOND);
        ((DcMotorEx) shooterLeft).setVelocity(RPM * RPM_TO_TICKS_PER_SECOND);
    }

    public void turnOffShooter(){
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ((DcMotorEx) shooterRight).setVelocity(0);
        ((DcMotorEx) shooterLeft).setVelocity(0);
    }

    public void setRampPosition(double position){
        shooterRampRight.setPosition(position);
        shooterRampLeft.setPosition(position);
    }

    public void setPitchPosition(double position){
        shooterPitchRight.setPosition(position);
        shooterPitchLeft.setPosition(position);
    }

    public boolean shooterAtTargetVelocity() {
        return ((DcMotorEx) shooterLeft).getVelocity() >= currentTargetRPMTicksPerSecond - TARGET_RPM_TOLERANCE_TICKS_PER_SECOND
                && ((DcMotorEx) shooterRight).getVelocity() >= currentTargetRPMTicksPerSecond - TARGET_RPM_TOLERANCE_TICKS_PER_SECOND;
    }

    public double shooterLeftGetVelocity() {
        return (((DcMotorEx) shooterLeft).getVelocity());
    }

    public double shooterRightGetVelocity() {
        return (((DcMotorEx) shooterRight).getVelocity());
    }

    public void toggleShooterClose(){
        if (currentGamepad.x && !previousGamepad.x){

            setCurrentRampScorePosition(CLOSE_RAMP_SCORE_POSITION);
            setCurrentTargetRPMTicksPerSecond(CLOSE_TARGET_RPM);

            on = !on;
        }

        if(on){
            setRampPosition(currentRampScorePosition);
            turnOnShooter();
        }
        else {
            setRampPosition(0);
            turnOffShooter();
        }
    }
    public void toggleShooterFar(){

        if (currentGamepad.y && !previousGamepad.y){

            setCurrentRampScorePosition(CLOSE_RAMP_SCORE_POSITION);
            setCurrentTargetRPMTicksPerSecond(CLOSE_TARGET_RPM);

            on = !on;
        }

        if(on){
            setRampPosition(currentRampScorePosition);
            turnOnShooter();
        }
        else {
            setRampPosition(0);
            turnOffShooter();
        }
    }

    public void setCurrentRampScorePosition(double position) {
        currentRampScorePosition = position;
    }

    public void setCurrentTargetRPMTicksPerSecond(double RPM) {
        currentTargetRPMTicksPerSecond = RPM * RPM_TO_TICKS_PER_SECOND;
    }

    public void updateGamepad(Gamepad gamepad) { //debounce method
        previousGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad);
    }

    public void resetPitchTimer(){
        timer.reset();
    }
    public void updatePitchDebounceTimer(){
        currentPitchTime = timer.time();
    }
    public boolean checkPitchDebounceTimer(){
        return currentPitchTime > PITCH_DEBOUNCE_SECONDS;
    }

    //test methods
    public void testControlRampPosition(Gamepad gamepad){

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

    public void testControlPitchPosition(Gamepad gamepad){

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

    public void testShoot(Gamepad gamepad){

//        if(currentGamepad.dpad_up && !previousGamepad.dpad_up){
//            testShootPower += .1;
//        } else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
//            testShootPower -= .1;
//        }
//
        if (currentGamepad.a && !previousGamepad.a){
            on = !on;
        }
//
//        testShootPower = Math.max(0, Math.min(testShootPower, 1));

        if(on){
            turnOnShooter();
        }
        else {
            turnOffShooter();
        }
    }

    public void testControlServo(Gamepad gamepad){ //method to test individual servos

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
