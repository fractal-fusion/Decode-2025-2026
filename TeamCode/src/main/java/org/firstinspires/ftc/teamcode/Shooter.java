package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    public Servo shooterGate;
    private OpMode opMode;

    //static variables for shooter
    public static double RPM_TO_TICKS_PER_SECOND = 28.0 / 60.0; //divide rpm by 60 to get rotations per second, then multiply by 28 since that's the ticks per revolution
    public static double TICKS_PER_SECOND_TO_RPM = 60.0 / 28.0; //inverse of above
    public static double CYCLING_RPM = 0;
    public static double PITCH_SCORE_POSITION = 0.0; //flatten pitch so balls can pass for scoring
    public static double PITCH_INTAKE_POSITION = 0.054;
    public static double PITCH_CYCLE_POSITION = 0.18;
    public static double GATE_OPEN_POSITION = 0;
    public static double GATE_CLOSED_POSITION = 0.14;
    public static double RAMP_CYCLE_POSITION = 0.2;
    public static double FAR_RAMP_SCORE_POSITION = 0.23;
    public static double FAR_TARGET_RPM = 4900;
    public static double FAR_AUTO_TARGET_RPM = 4800; //untested
    public static double FAR_DEBOUNCE = 3.0; //untested

    public static double FAR_TARGET_RPM_TICKS_PER_SECOND = FAR_TARGET_RPM * RPM_TO_TICKS_PER_SECOND;

    public static double CLOSE_RAMP_SCORE_POSITION = 0.25;
    public static double CLOSE_TARGET_RPM = 3850;
    public static double CLOSE_AUTO_TARGET_RPM = 3850;
    public static double CLOSE_DEBOUNCE = 0.4;

    public static double CLOSE_TARGET_RPM_TICKS_PER_SECOND = CLOSE_TARGET_RPM * RPM_TO_TICKS_PER_SECOND;

    public double currentRampScorePosition;
    public double currentTargetRPMTicksPerSecond;
    public static double TARGET_RPM_TOLERANCE_RPM_CLOSE = 100;
    public static double TARGET_RPM_TOLERANCE_RPM_FAR = 30;
    public double targetRPMToleranceRPM = TARGET_RPM_TOLERANCE_RPM_CLOSE; //initially set to the tolerance for close
    public double testShootPower = 0;
    public double testRampPosition = 0;
    public double testPitchPosition = 0;
    public boolean on = false; //boolean for on or off shooter
    public boolean cycling = false; //boolean for cycling or not
    public boolean passedThreshold = false; //boolean for once the shooter reaches velocity
    public static double LOWER_THRESHOLD_RPM = 3800;
    public int ballsShot = 0;
    private boolean wasAboveThreshold = false; //boolean for keeping track of balls shot
    private ElapsedTime shooterClosedTimer;
    public double currentShooterClosedTime;
    public double currentShooterClosedSeconds = CLOSE_DEBOUNCE; //set to close by default
    private ElapsedTime shooterOpenTimer;
    public double currentShooterOpenTime;
    public static double SHOOTER_OPEN_PITCH_SECONDS = 0.18;
    public static double SHOOTER_OPEN_GATE_SECONDS = 0.3;

    private ElapsedTime shooterTimeoutTimer; //timer for lowering the pitch when enough time has passed, overriding threshold
    public double currentShooterTimeoutTime;
    public static double SHOOTER_TIMEOUT_SECONDS = 1.2;
    private Timer atVelocityTimer = new Timer();
    public double atVelocityTime;
//    private boolean timerDebounce = false; //debounce to prevent timer from resetting when it has already reset
    public static double PID_OFFSET = 9.002; //right motor is always slower
    public static double P = 1.575;
    public static double I = 0.1375;
    public static double D = 0;
    public static double F = 14.750004;

    //test servo variables
    public static String testServo = "gate";
    public Servo TESTSERVO;
    private double testPosition = 0;
    public Shooter(OpMode linearOpmode) {
        this.opMode = linearOpmode;

        shooterClosedTimer = new ElapsedTime();
        shooterOpenTimer = new ElapsedTime();
        shooterTimeoutTimer = new ElapsedTime();
//        atVelocityTimer = new ElapsedTime();

        shooterLeft = opMode.hardwareMap.get(DcMotor.class, "shooterleft");
        shooterRight = opMode.hardwareMap.get(DcMotor.class, "shooterright");

        shooterRampRight = opMode.hardwareMap.get(Servo.class, "rampright" );
        shooterRampLeft = opMode.hardwareMap.get(Servo.class, "rampleft" );
        shooterPitchRight = opMode.hardwareMap.get(Servo.class, "pitchright" );
        shooterPitchLeft = opMode.hardwareMap.get(Servo.class, "pitchleft" );

        shooterGate = opMode.hardwareMap.get(Servo.class, "gate");

        TESTSERVO = opMode.hardwareMap.get(Servo.class, testServo);

        shooterRight.setDirection(DcMotor.Direction.REVERSE); //reverse shooter right motor so positive is out

//        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //slow down the motor faster

        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders on initialization

        shooterPitchRight.setDirection(Servo.Direction.REVERSE);
        shooterRampRight.setDirection(Servo.Direction.REVERSE); //reverse servos used for rotation so positive is rotate up

        shooterGate.setDirection(Servo.Direction.REVERSE); //reverse gate servo so open position is zero and positive closes inwards

        shooterRampRight.setPosition(0); //zero ramp servoes on initialization
        shooterRampLeft.setPosition(0);

        ((DcMotorEx) shooterLeft).setVelocityPIDFCoefficients(P, I, D, F); //TODO: change these pidfs to be near max velocity
        ((DcMotorEx) shooterRight).setVelocityPIDFCoefficients(P, I, D, F);

        setCurrentTargetRPMTicksPerSecond(CLOSE_TARGET_RPM); //default the current target rpm ticks per second to close target rpm
        setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_CLOSE);
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

    public void setGatePosition(double position){
        shooterGate.setPosition(position);
    }

    public boolean shooterAtTargetVelocity() {
        return ((DcMotorEx) shooterLeft).getVelocity() >= currentTargetRPMTicksPerSecond - (targetRPMToleranceRPM * RPM_TO_TICKS_PER_SECOND)
                && ((DcMotorEx) shooterRight).getVelocity() >= currentTargetRPMTicksPerSecond - (targetRPMToleranceRPM * RPM_TO_TICKS_PER_SECOND);
    }

    public boolean shooterAtLowerThresholdVelocity() {
        return ((DcMotorEx) shooterLeft).getVelocity() <= (LOWER_THRESHOLD_RPM * RPM_TO_TICKS_PER_SECOND)
                && ((DcMotorEx) shooterRight).getVelocity() <= (LOWER_THRESHOLD_RPM * RPM_TO_TICKS_PER_SECOND);
    }

    public void setTargetRPMToleranceRPM(double RPM) {
        targetRPMToleranceRPM = RPM;
    }

    public double shooterLeftGetVelocity() {
        return (((DcMotorEx) shooterLeft).getVelocity());
    }

    public double shooterRightGetVelocity() {
        return (((DcMotorEx) shooterRight).getVelocity());
    }

    public double calculateShooterVelocityRPM(double ta){
        return Range.clip(94.97007*Math.pow(ta, 4) - 913.82549*Math.pow(ta, 3) + 3324.27197*Math.pow(ta, 2) - 5508.49822*ta + 7442.54335, 3850, 5175);
    }

    public void toggleShooterClose(){
        if (currentGamepad.x && !previousGamepad.x){
            resetShooterClosedTimer();
            resetShooterOpenTimer();
            atVelocityTimer.resetTimer();

            setCurrentTargetRPMTicksPerSecond(CLOSE_TARGET_RPM);
//            setCurrentShooterClosedSeconds(CLOSE_DEBOUNCE); TODO: ^ make this toggleble for far auto

            on = !on;
        }

        if(on){
            setRampPosition(CLOSE_RAMP_SCORE_POSITION);
            turnOnShooter();
        }
        else {
            setRampPosition(0);
            turnOffShooter();
        }
    }
    public void toggleShooterFar(){

        if (currentGamepad.y && !previousGamepad.y){
            resetShooterClosedTimer();
            resetShooterOpenTimer();
            atVelocityTimer.resetTimer();

            setCurrentTargetRPMTicksPerSecond(FAR_TARGET_RPM);
//            setCurrentShooterClosedSeconds(FAR_DEBOUNCE);


            on = !on;
        }

        if(on){
            setRampPosition(FAR_RAMP_SCORE_POSITION);
            turnOnShooter();
        }
        else {
            setRampPosition(0);
            turnOffShooter();
        }
    }

    public void controlShooterPitch(){
//        opMode.telemetry.addLine("controlling pitch");
        if ((passedThreshold && !cycling && shooterClosedTimerOver())){
            setPitchPosition(Shooter.PITCH_SCORE_POSITION);
        }
        else if (!passedThreshold && !cycling && shooterOpenPitchTimerOver()){
            setPitchPosition(Shooter.PITCH_INTAKE_POSITION); //automatically raise the pitch when not ready to shoot
        }
    }

    public void controlShooterGate(){
        //TODO: can change cycling boolean to separate between sorting mode for the gate and normal shooting mode
        if ((passedThreshold && !cycling && shooterClosedTimerOver())){
            atVelocityTime = atVelocityTimer.getElapsedTimeSeconds();
            setGatePosition(Shooter.GATE_OPEN_POSITION);
        }
        else if (!passedThreshold && !cycling && shooterOpenGateTimerOver()){
            setPitchPosition(PITCH_SCORE_POSITION);
//            setGatePosition(Shooter.GATE_CLOSED_POSITION); //automatically raise the pitch when not ready to shoot
        }
    }

    public void update(){
        //update the debounce timers
        updateShooterOpenTimer();
        updateShooterClosedTimer();
        updateShooterTimeoutTimer();

        //check if shooter is past threshold
        updateShooterThreshold();
        updateShotBalls();
    }

    public void setCurrentTargetRPMTicksPerSecond(double RPM) {
        currentTargetRPMTicksPerSecond = RPM * RPM_TO_TICKS_PER_SECOND;
    }
    public void setCurrentShooterClosedSeconds(double seconds){
        currentShooterClosedSeconds = seconds;
    }
    
    public void updateGamepad(Gamepad gamepad) { //debounce method
        previousGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad);
    }

    public void resetShooterOpenTimer(){
        shooterOpenTimer.reset();
    }
    public void updateShooterOpenTimer(){
        currentShooterOpenTime = shooterOpenTimer.time();
    }
    public boolean shooterOpenPitchTimerOver(){
        return currentShooterOpenTime > SHOOTER_OPEN_PITCH_SECONDS;
    }
    public boolean shooterOpenGateTimerOver(){
        return currentShooterOpenTime > SHOOTER_OPEN_GATE_SECONDS;
    }
    public void resetShooterClosedTimer(){
        shooterClosedTimer.reset();
    }
    public void updateShooterClosedTimer(){
        currentShooterClosedTime = shooterClosedTimer.time();
    }
    public boolean shooterClosedTimerOver(){
        return currentShooterClosedTime > currentShooterClosedSeconds;
    }

    public void resetShooterTimeoutTimer(){
        shooterTimeoutTimer.reset();
    }
    public void updateShooterTimeoutTimer(){
        currentShooterTimeoutTime = shooterClosedTimer.time();
    }
    public boolean shooterTimeoutTimerOver(){
        return currentShooterTimeoutTime > SHOOTER_TIMEOUT_SECONDS;
    }

    public void updateShooterThreshold(){
//        opMode.telemetry.addData("passed threshold", passedThreshold);
        if (shooterAtTargetVelocity()) {
            passedThreshold = true;
        } else if (shooterAtLowerThresholdVelocity()) {
            passedThreshold = false;
        }
    }

    public void updateShotBalls() {
//        opMode.telemetry.addLine("counting shot balls");
        if (on && passedThreshold) {
            wasAboveThreshold = true;
            resetShooterOpenTimer();
        }
        if (on && wasAboveThreshold && shooterAtLowerThresholdVelocity()) {
            wasAboveThreshold = false;

            resetShooterClosedTimer();
            ballsShot += 1;
        }
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
