package overcharged.opmode;


import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;



@Config
@TeleOp(name="teleop1", group="Teleop")
public class teleop1 extends OpMode {
    RobotMecanum robot;
    boolean down = false;
    long startTime;
    long intakeDoorDelay;
    long depoTiltDelay;
    long shootDelay;
    boolean intakeDDelay = false;
    double slowPower = 1;
    private final static float SLOW_POWER_MULT = 0.65f;
    boolean isSlow = true;
    SlowMode slowMode = SlowMode.SLOW;
    IntakeMode intakeMode = IntakeMode.OFF;
    FlatButtonState flatbuttonState = FlatButtonState.NO2;
    TransferButtonState transferButtonState = TransferButtonState.NO2;
    DepoMode depoMode = DepoMode.OPEN2;
    boolean iSmallTilt = false;
    boolean dTilt = false;
    boolean dOpen = true;
    boolean iOpen = false;
    boolean slideGoBottom = false;
    boolean firstLoop = true;
    boolean rPixelOpen = false;
    boolean lPixelOpen = false;
    boolean dTiltIn = false;
    boolean shooting = false;
    boolean isLocked = true;

    boolean hSlideGoBottom = false;

    boolean hSlideisOut = false;


    public enum SlowMode {
        OVERRIDE,
        SLOW;
    }

    public enum DepoMode {
        ClOSED,
        OPEN1,
        OPEN2;
    }

    public enum IntakeMode {
        IN,
        OUT,
        OFF;
    }

    public enum FlatButtonState {
        PRESSED,
        PRESSING,
        NO,
        NO2;
    }

    public enum TransferButtonState {
        PRESSED,
        PRESSING,
        NO,
        NO2;
    }

    @Override
    public void init() {
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            startTime = System.currentTimeMillis();
            robot.setBulkReadManual();
            robot.vSlides.vSlides.setTargetPositionPIDFCoefficients(21,0,0,0);
        } catch (Exception e){
            RobotLog.ee(TAG_T, "Teleop init failed: " + e.getMessage());
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
        }
    }
    @Override
    public void loop() {
        if(firstLoop) {
            robot.intakeSmallTilt.setOut();
            robot.intakeBigTilt.setOut();
            robot.intakeDoor.setClosed();
            iOpen = false;
            firstLoop = false;
        }
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator)*slowPower;
        double backLeftPower = ((y - x + rx) / denominator)*slowPower;
        double frontRightPower = ((y - x - rx) / denominator)*slowPower;
        double backRightPower = ((y + x - rx) / denominator)*slowPower;

        robot.driveLeftFront.setPower(frontLeftPower);
        robot.driveLeftBack.setPower(backLeftPower);
        robot.driveRightFront.setPower(frontRightPower);
        robot.driveRightBack.setPower(backRightPower);

        /**
         * Turn on slow mode for drive train
         */
        if (gamepad1.a && Button.BTN_SLOW_MODE.canPress(timestamp)) {
            isSlow = true;
        } else {
            isSlow = false;
        }

        if(slowMode == SlowMode.SLOW) {
            if (isSlow) {
                slowPower = 0.4f;
                y *= SLOW_POWER_MULT;
                x *= SLOW_POWER_MULT;
                rx *= SLOW_POWER_MULT;
            } else {
                slowPower = 1;
            }
        }

        //Intake in
        if(gamepad1.right_trigger > 0.9 && Button.INTAKE.canPress(timestamp)) {//gamepad1.right_bumper && Button.INTAKE.canPress(timestamp)){
            if (intakeMode == IntakeMode.OFF || intakeMode == IntakeMode.OUT) {
                robot.intakeSmallTilt.setOut();
                robot.intakeBigTilt.setOut();
                robot.intakeDoor.setClosed();
                iOpen = false;
                robot.intake.in();
                intakeMode = IntakeMode.IN;
            }
            else{
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }

        //Intake out
        if(gamepad1.left_trigger > 0.9 && Button.INTAKEOUT.canPress(timestamp)){//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if(intakeMode == IntakeMode.IN || intakeMode == IntakeMode.OFF) {
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }

        /*//Intake tilt
        if(gamepad2.right_bumper && Button.TRANSFER.canPress(timestamp)){
            if(!iSmallTilt){
                robot.intakeSmallTilt.setTransfer();
                robot.intakeBigTilt.setTransfer();
                robot.intakeDoor.setOpen();
                robot.depoDoor.setOpen();
                iSmallTilt = true;
            } else {
                robot.intakeSmallTilt.setOut();
                robot.intakeBigTilt.setOut();
                robot.intakeDoor.setClosed();
                robot.depoDoor.setClosed();
                iSmallTilt = false;
            }
        }*/

        //Hold for transfer position
        if(gamepad2.right_bumper){
            if(transferButtonState == TransferButtonState.NO2) {
                transferButtonState = TransferButtonState.PRESSED;
            }
            else {
                transferButtonState = TransferButtonState.PRESSING;
            }
        } else {
            if(transferButtonState == TransferButtonState.PRESSING)
                transferButtonState = TransferButtonState.NO;
            else
                transferButtonState = TransferButtonState.NO2;
        }

        if(transferButtonState == TransferButtonState.PRESSED){
            //robot.intakeSmallTilt.setTransfer();
            robot.intakeBigTilt.setTransfer();
            intakeDDelay = true;
            intakeDoorDelay = System.currentTimeMillis();
            //robot.intakeDoor.setOpen();
            robot.depoDoor.setOpen2();
            depoMode = DepoMode.OPEN2;
        } else if(transferButtonState == TransferButtonState.NO){
            robot.intakeSmallTilt.setOut();
            robot.intakeBigTilt.setOut();
            robot.intakeDoor.setClosed();
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            intakeDDelay = false;
            iOpen = false;
        }

        if(intakeDDelay && System.currentTimeMillis()-intakeDoorDelay > 100){
            robot.intakeSmallTilt.setTransfer();
        }

        if(intakeDDelay && System.currentTimeMillis()-intakeDoorDelay > 300){
            robot.intakeDoor.setOpen();
            iOpen = true;
        }

        //Hold for flat position
        if(gamepad2.right_trigger > 0.9){
            if(flatbuttonState == FlatButtonState.NO2) {
                flatbuttonState = FlatButtonState.PRESSED;
            }
            else {
                flatbuttonState = FlatButtonState.PRESSING;
            }
        } else {
            if(flatbuttonState == FlatButtonState.PRESSING)
                flatbuttonState = FlatButtonState.NO;
            else
                flatbuttonState = FlatButtonState.NO2;
        }

        if(flatbuttonState == FlatButtonState.PRESSED){
            robot.intakeSmallTilt.setFlat();
            robot.intakeBigTilt.setFlat();
            robot.intakeDoor.setClosed();
            iOpen = false;
        } else if(flatbuttonState == FlatButtonState.NO){
            robot.intakeSmallTilt.setOut();
            robot.intakeBigTilt.setOut();
            robot.intakeDoor.setClosed();
            robot.depoDoor.setClosed();
            iOpen = false;
            depoMode = DepoMode.ClOSED;
        }

        //IntakeDoor
        if(gamepad2.b && Button.INTAKEDOOR.canPress(timestamp)){
            if(!iOpen){
                robot.intakeDoor.setOpen();
                iOpen = true;
            }
            else{
                robot.intakeDoor.setClosed();
                iOpen = false;
            }
        }

        //DepoTilt
        if(gamepad2.left_bumper && Button.DEPOTILT.canPress(timestamp)){
            if(!dTilt){
                robot.depoTilt.setOut();
                robot.depoDoor.setClosed();
                dTilt = true;
                depoMode = DepoMode.ClOSED;
            }
            else{
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlides.getCurrentPosition())+250, 1);
                depoTiltDelay = System.currentTimeMillis();
                dTiltIn = true;
            }
        }
        if(dTiltIn && System.currentTimeMillis() - depoTiltDelay > 120){
            robot.depoTilt.setIn();
            dOpen = false;
            dTilt = false;
            dTiltIn = false;
        }

        //DepoDoor all the way open
        if(gamepad1.left_bumper && Button.DEPODOOR.canPress(timestamp)){
            if(depoMode == DepoMode.ClOSED || depoMode == DepoMode.OPEN1){
                robot.depoDoor.setOpen2();
                depoMode = DepoMode.OPEN2;
            }
            else{
                robot.depoDoor.setClosed();
                depoMode = DepoMode.ClOSED;
            }
        }

        if(gamepad1.right_bumper && Button.DEPODOOR.canPress(timestamp)){
            if(depoMode == DepoMode.ClOSED){
                robot.depoDoor.setOpen1();
                depoMode = DepoMode.OPEN1;
            } else if(depoMode == DepoMode.OPEN1){
                robot.depoDoor.setOpen2();
                depoMode = DepoMode.OPEN2;
            } else if(depoMode == DepoMode.OPEN2){
                robot.depoDoor.setClosed();
                depoMode = DepoMode.ClOSED;
            }
        }
//      UNCOMMENT THIS LATER
        if(gamepad2.y && Button.SLIGHT_UP.canPress(timestamp)){
            if(robot.vSlides.vSlides.getCurrentPosition() < 1950){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlides.getCurrentPosition())+160, 1);
            }
        }

        if(gamepad2.a && Button.SLIGHT_DOWN.canPress(timestamp)){
            if(robot.vSlides.vSlides.getCurrentPosition() > 180){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlides.getCurrentPosition())-180, 1);
            }
        }

        if (gamepad2.y && Button.BTN_SLIDE_OUT.canPress(timestamp) && !hSlideisOut) {
            hSlideisOut = true;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.hslides.moveEncoderTo(robot.hslides.OUT, 1);
        }

        if (gamepad2.a && Button.BTN_SLIDE_OUT.canPress(timestamp) && hSlideisOut) {
            hSlideisOut = false;
//            robot.intake.off();
//            intakeMode = IntakeMode.OFF;
//            robot.depoDoor.setClosed();
//            depoMode = DepoMode.ClOSED;
//            robot.intake.in();
              robot.hslides.in();
            hSlideGoBottom = true;
        }


//        if (hSlideGoBottom) {
//            if (robot.hslides.slideIn()) {
//                robot.hslides.forceStop();
//                robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                hSlideGoBottom = false;
//                robot.intake.in();
//                intakeMode = IntakeMode.OFF;
//                RobotLog.ii(TAG_SL, "Force stopped");
//            }
//            else {
//                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.hslides.in();
//            }
//        }

//        if (!robot.hslides.slideIn()) {
//            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.hslides.in();
//        } else {
//            robot.hslides.forceStop();
//            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////              robot.intake.in();
////              intakeMode = IntakeMode.OFF;
//        }

//        if (!robot.hslides.slideIn() && hSlideGoBottom) {// && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
//            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.hslides.in();
//            RobotLog.ii(TAG_SL, "Going down");
//        } else if (robot.hslides.slideIn() && hSlideGoBottom) {
//            robot.hslides.forceStop();
//            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            hSlideGoBottom = false;
//            robot.intake.in();
//            intakeMode = IntakeMode.IN;
//            RobotLog.ii(TAG_SL, "Force stopped");
//        }

//        if (!robot.hslides.slideIn() && hSlideGoBottom) {
//            hSlideGoBottom = false;
//            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.hslides.forceStop();
//            robot.intake.in();
//            intakeMode = IntakeMode.IN;
//        }

        // vSlides down
        if((gamepad2.left_trigger > 0.9 || gamepad1.dpad_down) && Button.BTN_SLIDE_DOWN.canPress(timestamp)){
            robot.intake.in();
            slideGoBottom = true;
        }
        //Slide height 1
        if(gamepad2.dpad_left && Button.BTN_LEVEL1.canPress(timestamp)){
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
        }
        //Slide height 2
        if(gamepad2.dpad_down && Button.BTN_LEVEL2.canPress(timestamp)){
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.vSlides.moveEncoderTo(robot.vSlides.level2, 1);
        }
        //Slide height 3
        if(gamepad2.dpad_right && Button.BTN_LEVEL3.canPress(timestamp)){
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.vSlides.moveEncoderTo(robot.vSlides.level3, 1);
        }
        //Slide height 4
        if(gamepad2.dpad_up && Button.BTN_LEVEL4.canPress(timestamp)){
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.vSlides.moveEncoderTo(robot.vSlides.level4, 1);
        }
        if(slideGoBottom)  {
            robot.depoTilt.setIn();
            robot.depoDoor.setOpen2();
            depoMode = DepoMode.OPEN2;
            dTilt = false;
            slideBottom();
        }

        if(gamepad1.y && Button.NOPOWER.canPress(timestamp)){
            robot.vSlides.setPower(0);
            robot.vSlides.forceStop();
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideGoBottom = false;
        }

        if(gamepad1.x && Button.LEFTPIXEL.canPress(timestamp)){
            if(lPixelOpen){
                lPixelOpen = false;
                robot.leftPixel.setIn();
            } else if(!lPixelOpen){
                lPixelOpen = true;
                robot.leftPixel.setDump();
            }
        }

        if(gamepad1.b && Button.RIGHTPIXEL.canPress(timestamp)){
            if(rPixelOpen){
                rPixelOpen = false;
                robot.rightPixel.setIn();
            } else if (!rPixelOpen){
                rPixelOpen = true;
                robot.rightPixel.setDump();
            }
        }

        if(gamepad2.x && Button.DRONESHOOTER.canPress(timestamp)){
            if(!shooting){
                robot.droneShooter.setShoot();
                shooting = true;
            } else{
                robot.droneShooter.setInit();
                shooting = false;
            }
        }

        if(gamepad1.dpad_up && Button.BTN_LATCH_READY.canPress(timestamp)){
            if(isLocked) {
                robot.leftHang.setPosition(95f);
                robot.rightHang.setHang();
                isLocked = false;
            } else {
                robot.leftHang.setPosition(210f);
                robot.rightHang.setIn();
                isLocked = true;
            }
        }

        telemetry.addData("vSlides encoder: ", robot.vSlides.vSlides.getCurrentPosition());
        telemetry.addData("limit switch: ", robot.vSlides.switchSlideDown.isTouch());
        telemetry.addData("vSlide power: ", robot.vSlides.vSlides.getPower());
        telemetry.addData("test:", robot.hslides.hslides.getPower());
        telemetry.addData("test2: ", robot.hslides.hslides.getCurrentPosition());
        telemetry.addData("reach bottom: ", !robot.vSlides.slideReachedBottom());
        telemetry.addData("driveLF", robot.driveLeftFront.getCurrentPosition());
        telemetry.addData("driveLB", robot.driveLeftBack.getCurrentPosition());
        telemetry.addData("driveRF", robot.driveRightFront.getCurrentPosition());
        telemetry.addData("intake", robot.intake.intake.getCurrentPosition());
        telemetry.addData("driveRB", robot.driveRightBack.getCurrentPosition());
        telemetry.addData("hslideOut", robot.hslides.slideIn());
        telemetry.addData("hslidePower", robot.hslides.getPower());
        telemetry.update();
    }
    public void slideBottom() {
        if (!robot.vSlides.slideReachedBottom()) {// && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.down();
            RobotLog.ii(TAG_SL, "Going down");
        } else {
            robot.vSlides.forceStop();
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideGoBottom = false;
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
    }
}