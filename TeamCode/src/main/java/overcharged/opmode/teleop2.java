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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.components.hslides;


@Config
@TeleOp(name="teleop2", group="Teleop")
public class teleop2 extends OpMode {
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
    DepoMode depoMode = DepoMode.OPEN;
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
    long depoTiltInDelay;
    boolean depoTiltOutDelay = false;
    boolean intakeTransfer = false;
    boolean intakeOutDelay = false;
    long intakeOutTime;
    boolean stayIn = true;
    boolean isOut = false;
    boolean fClawClosed = false;
    boolean bClawClosed = false;
    boolean wristL = false;
    boolean wristR = false;
    boolean dGoFlat = false;
    boolean dGoVert = false;
    WristMode wristMode = WristMode.VERT_IN;
    boolean hSlideGoIn = false;
    long hSlideInDelay;
    int vSlideLevel = 1;
    long delayForClaw;
    boolean firstTime = false;

    boolean hSlideGoBottom = false;

    boolean hSlideisOut = false;

    public enum WristMode{
        FLAT,
        FLAT_OPP,
        VERT_IN,
        VERT_OPP,
        R_DIAG,
        L_DIAG;
    }

    public enum SlowMode {
        OVERRIDE,
        SLOW;
    }

    public enum DepoMode {
        ClOSED,
        F_OPEN,
        B_OPEN,
        OPEN;
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
            //robot.vSlides.vSlidesB.setTargetPositionPIDFCoefficients(21,0,0,0);
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
            //robot.hslides.moveEncoderTo(0,1);
            iOpen = false;
            firstLoop = false;
            robot.hang.setRightIn();
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

        if(stayIn && !robot.hslides.switchSlideDown.isTouch()){
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.in();
        } /*else {
            robot.hslides.hslides.resetPosition();
            robot.hslides.forceStop();
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }*/

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

        if(gamepad2.right_bumper && Button.TRANSFER.canPress(timestamp)){//bumper && Button.INTAKEOUT.canPress(timestamp)){
            stayIn = true;
            if(!intakeTransfer) {
                robot.intakeBigTilt.setTransfer();
                intakeDDelay = true;
                intakeDoorDelay = System.currentTimeMillis();
                //robot.intakeDoor.setOpen();
                //robot.depoDoor.setOpen2();
                robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                fClawClosed = false;
                bClawClosed = false;
                intakeTransfer = true;
            }
            else {
                robot.intakeSmallTilt.setOut();
                intakeOutDelay = true;
                intakeOutTime = System.currentTimeMillis();
                robot.intakeBigTilt.setOut();
                robot.intakeDoor.setClosed();
                //robot.depoDoor.setClosed();
                robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                fClawClosed = true;
                bClawClosed = true;
                intakeDDelay = false;
                iOpen = false;
                intakeTransfer = false;
            }
        }

        if(intakeDDelay && System.currentTimeMillis()-intakeDoorDelay > 100){
            robot.intakeSmallTilt.setTransfer();
            //intakeDDelay = false;
        }

        if(intakeDDelay && System.currentTimeMillis()-intakeDoorDelay > 650){
            robot.intakeDoor.setOpen();
            iOpen = true;
            intakeDDelay = false;
        }

        /*if(intakeOutDelay && System.currentTimeMillis()-intakeOutTime > 100){
            robot.intakeBigTilt.setOut();
            intakeOutDelay = false;
        }*/

        /*//Hold for transfer position
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
        }*/



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
            //robot.depoDoor.setClosed();
            robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
            robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
            iOpen = false;
            fClawClosed = true;
            bClawClosed = true;
            intakeTransfer = false;
        }

        //WristRIGHT
        if(gamepad2.x && Button.INTAKEDOOR.canPress(timestamp)){
            if(wristMode == WristMode.VERT_IN){
                robot.depo.setWristPos(robot.depo.WRIST_L_DIAG);
                wristMode = WristMode.L_DIAG;
            } else if(wristMode == WristMode.L_DIAG){
                robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                wristMode = WristMode.FLAT;
            } else if(wristMode == WristMode.FLAT_OPP){
                robot.depo.setWristPos(robot.depo.WRIST_R_DIAG);
                wristMode = WristMode.R_DIAG;
            } else if(wristMode == WristMode.R_DIAG){
                robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                wristMode = WristMode.VERT_IN;
            }

            /*if(wristMode == WristMode.FLAT){
                robot.depo.setWristPos(robot.depo.WRIST_L_DIAG);
                wristMode = WristMode.L_DIAG;
            } else if(wristMode == WristMode.L_DIAG){
                robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                wristMode = WristMode.VERT_IN;
            } else if(wristMode == WristMode.VERT_OPP){
                robot.depo.setWristPos(robot.depo.WRIST_R_DIAG);
                wristMode = WristMode.R_DIAG;
            } else if(wristMode == WristMode.R_DIAG){
                robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                wristMode = WristMode.FLAT;
            }*/

            /*if(!wristR){
                robot.depo.setWristPos(robot.depo.WRIST_R_DIAG);
                wristR = true;
                wristL = false;
            } else {
                robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                wristR = false;
                wristL = false;
            }*/
        }

        //WristLEFT
        if(gamepad2.b && Button.BTN_LATCH_READY.canPress(timestamp)){
            if(wristMode == WristMode.VERT_IN){
                robot.depo.setWristPos(robot.depo.WRIST_R_DIAG);
                wristMode = WristMode.R_DIAG;
            } else if(wristMode == WristMode.R_DIAG){
                robot.depo.setWristPos(robot.depo.WRIST_OPP_FLAT);
                wristMode = WristMode.FLAT_OPP;
            } else if(wristMode == WristMode.FLAT){
                robot.depo.setWristPos(robot.depo.WRIST_L_DIAG);
                wristMode = WristMode.L_DIAG;
            } else if(wristMode == WristMode.L_DIAG){
                robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                wristMode = WristMode.VERT_IN;
            }

            /*if(wristMode == WristMode.FLAT){
                robot.depo.setWristPos(robot.depo.WRIST_R_DIAG);
                wristMode = WristMode.R_DIAG;
            } else if(wristMode == WristMode.R_DIAG){
                robot.depo.setWristPos(robot.depo.WRIST_OPP_VERT);
                wristMode = WristMode.VERT_OPP;
            } else if(wristMode == WristMode.VERT_IN){
                robot.depo.setWristPos(robot.depo.WRIST_L_DIAG);
                wristMode = WristMode.L_DIAG;
            } else if(wristMode == WristMode.L_DIAG){
                robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                wristMode = WristMode.FLAT;
            }*/
            /*if(!wristL){
                robot.depo.setWristPos(robot.depo.WRIST_L_DIAG);
                wristR = false;
                wristL = true;
            } else {
                robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                wristR = false;
                wristL = false;
            }*/
        }

        //DepoTilt
        if(gamepad2.left_bumper && Button.DEPOTILT.canPress(timestamp)){
            if(!dTilt){
                robot.depo.setArmPos(robot.depo.ARM_OUT);
                dGoFlat = true;
                //robot.depoDoor.setClosed();
                robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                dTilt = true;
                fClawClosed = true;
                bClawClosed = true;
            }
            else{
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlidesB.getCurrentPosition())+80, 1);
                depoTiltDelay = System.currentTimeMillis();
                dTiltIn = true;
            }
        }

        if(dGoFlat && robot.depo.getArmVolt() > 3.3-((robot.depo.ARM_OUT+45)*(3.3/255))){
            robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
            wristMode = WristMode.VERT_IN;
            dGoFlat = false;
        }

        if(dTiltIn && System.currentTimeMillis() - depoTiltDelay > 120){
            //robot.depoDoor.setClosed();
            robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
            robot.depo.setBackClawPos(robot.depo.BACK_DUMP);

            fClawClosed = false;
            bClawClosed = false;
            //robot.depoTilt.setIn();
            robot.depo.setArmPos(robot.depo.ARM_IN);
            robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
            wristMode = WristMode.VERT_IN;
            dOpen = false;
            dTilt = false;
            dTiltIn = false;
        }

        //DepoDoor all the way open
        if(gamepad1.right_bumper && Button.DEPODOOR.canPress(timestamp)){
            if(bClawClosed){
                robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                bClawClosed = false;
            }
            else{
                robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                bClawClosed = true;
            }
        }

        if(gamepad1.x && Button.DEPODOOR.canPress(timestamp)){
            if(bClawClosed && fClawClosed){
                robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                bClawClosed = false;
                fClawClosed = false;
            }
            else{
                robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                bClawClosed = true;
                fClawClosed = true;
            }
        }

        if(gamepad1.left_bumper && Button.DEPODOOR.canPress(timestamp)){
            if(fClawClosed){
                robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                fClawClosed = false;
            }
            else{
                robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                fClawClosed = true;
            }
        }

//      UNCOMMENT THIS LATER
        if(gamepad2.y && Button.SLIGHT_UP.canPress(timestamp)){
            if(robot.vSlides.vSlidesB.getCurrentPosition() < 460){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlidesB.getCurrentPosition())+65, 1);
            }
        }

        if(gamepad2.a && Button.SLIGHT_DOWN.canPress(timestamp)){
            if(robot.vSlides.vSlidesB.getCurrentPosition() > 55){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlidesB.getCurrentPosition())-50, 1);
            }
        }



//        if (gamepad2.y && Button.BTN_SLIDE_OUT.canPress(timestamp) && !hSlideisOut) {
//            hSlideisOut = true;
//            robot.intake.off();
//            intakeMode = IntakeMode.OFF;
//            robot.depoDoor.setClosed();
//            depoMode = DepoMode.ClOSED;
//            robot.hslides.moveEncoderTo(robot.hslides.OUT, 1);
//        }

//        if (gamepad2.a && Button.BTN_SLIDE_OUT.canPress(timestamp) && hSlideisOut) {
//            hSlideisOut = false;
//            robot.intake.off();
//            intakeMode = IntakeMode.OFF;
//            robot.depoDoor.setClosed();
//            depoMode = DepoMode.ClOSED;
//            robot.intake.in();
//              robot.hslides.in();
//            hSlideGoBottom = true;
//        }


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

        /*if (!robot.hslides.slideIn()) {
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.in();
        } else {
            robot.hslides.forceStop();
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//              robot.intake.in();
//              intakeMode = IntakeMode.OFF;
        }*/

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
            if(robot.vSlides.vSlidesB.getCurrentPosition() < robot.vSlides.level4-20){
                robot.vSlides.moveEncoderTo(robot.vSlides.vSlidesB.getCurrentPosition()+120,1);
            }
            robot.intake.in();
            slideGoBottom = true;
            //robot.depoTilt.setIn();
            //robot.depoDoor.setOpen2();
            robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
            robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
            fClawClosed = false;
            bClawClosed = false;
            dTilt = false;
            depoTiltInDelay = System.currentTimeMillis();
        }
        //Slide height 1
        if(gamepad2.dpad_left && Button.BTN_LEVEL1.canPress(timestamp)){
            depoTiltOutDelay = true;
            delayForClaw = System.currentTimeMillis();
            firstTime = true;
            vSlideLevel = 1;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
            robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
            fClawClosed = true;
            bClawClosed = true;
            robot.intakeDoor.setClosed();
            iOpen = false;
           //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
        }
        //Slide height 2
        if(gamepad2.dpad_down && Button.BTN_LEVEL2.canPress(timestamp)){
            depoTiltOutDelay = true;
            delayForClaw = System.currentTimeMillis();
            firstTime = true;
            vSlideLevel = 2;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
            robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
            fClawClosed = true;
            bClawClosed = true;
            robot.intakeDoor.setClosed();
            iOpen = false;
            //robot.vSlides.moveEncoderTo(robot.vSlides.level2, 1);
        }
        //Slide height 3
        if(gamepad2.dpad_right && Button.BTN_LEVEL3.canPress(timestamp)){
            depoTiltOutDelay = true;
            delayForClaw = System.currentTimeMillis();
            firstTime = true;
            vSlideLevel = 3;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
            robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
            fClawClosed = true;
            bClawClosed = true;
            robot.intakeDoor.setClosed();
            iOpen = false;
            //robot.vSlides.moveEncoderTo(robot.vSlides.level3, 1);
        }
        //Slide height 4
        if(gamepad2.dpad_up && Button.BTN_LEVEL4.canPress(timestamp)){
            depoTiltOutDelay = true;
            delayForClaw = System.currentTimeMillis();
            firstTime = true;
            vSlideLevel = 4;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
            robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
            fClawClosed = true;
            bClawClosed = true;
            robot.intakeDoor.setClosed();
            iOpen = false;
            //robot.vSlides.moveEncoderTo(robot.vSlides.level4, 1);
        }

        if(firstTime && depoTiltOutDelay && System.currentTimeMillis() - delayForClaw > 250){
            if(vSlideLevel == 1){ robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);}
            else if(vSlideLevel == 2){ robot.vSlides.moveEncoderTo(robot.vSlides.level2, 1);}
            else if(vSlideLevel == 3){ robot.vSlides.moveEncoderTo(robot.vSlides.level3, 1);}
            else {robot.vSlides.moveEncoderTo(robot.vSlides.level4, 1);}
            firstTime = false;
        }
        if(depoTiltOutDelay && robot.vSlides.vSlidesB.getCurrentPosition() > robot.vSlides.level1 - 10 ){
            robot.depo.setArmPos(robot.depo.ARM_OUT);
            dGoFlat = true;
            //robot.depoTilt.setOut();
            dTilt = true;
            depoTiltOutDelay = false;
        }

        if(slideGoBottom){
            if(System.currentTimeMillis() - depoTiltInDelay > 225){
                robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                wristMode = WristMode.VERT_IN;
            }
        }

        if(slideGoBottom)  {
            if(System.currentTimeMillis() - depoTiltInDelay > 500){//(robot.vSlides.vSlidesB.getCurrentPosition() > robot.vSlides.level2+ 20 && System.currentTimeMillis() - depoTiltInDelay > 100)
                //|| (robot.vSlides.vSlidesB.getCurrentPosition() <= robot.vSlides.level2 + 20 && System.currentTimeMillis() - depoTiltInDelay > 200)) {
                //robot.depoDoor.setClosed();
                robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                fClawClosed = false;
                bClawClosed = false;
                robot.depo.setArmPos(robot.depo.ARM_IN);
                //robot.depoTilt.setIn();
                dTilt = false;
            }
        }

        if(slideGoBottom)  {
            if(System.currentTimeMillis() - depoTiltInDelay > 1000){//(robot.vSlides.vSlidesB.getCurrentPosition() > robot.vSlides.level2+ 20 && System.currentTimeMillis() - depoTiltInDelay > 100)
                //|| (robot.vSlides.vSlidesB.getCurrentPosition() <= robot.vSlides.level2 + 20 && System.currentTimeMillis() - depoTiltInDelay > 200)) {
                slideBottom();
            }
        }

        if(gamepad1.y && Button.NOPOWER.canPress(timestamp)){
            slideGoBottom = false;
            robot.vSlides.stopMotors();
            robot.vSlides.forcestop();
            robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.vSlides.vSlidesF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        /*if(gamepad1.x && Button.LEFTPIXEL.canPress(timestamp)){
            if(lPixelOpen){
                lPixelOpen = false;
                robot.pixel.setLeftIn();
            } else if(!lPixelOpen){
                lPixelOpen = true;
                robot.pixel.setLeftDump();
            }
        }*/

        if(gamepad1.b && Button.RIGHTPIXEL.canPress(timestamp)){
            if(!iOpen){
                robot.intakeDoor.setOpen();
                iOpen = true;
            }
            else{
                robot.intakeDoor.setClosed();
                iOpen = false;
            }
        }

        if(gamepad1.a && Button.DRONESHOOTER.canPress(timestamp)){
            if(!shooting){
                robot.droneShooter.setShoot();
                shooting = true;
            } else{
                robot.droneShooter.setInit();
                shooting = false;
            }
        }

        /*if(gamepad2.left_stick_y > 0.1 && !robot.vSlides.slideReachedBottom()){
            robot.vSlides.vSlidesF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlidesB.setPower(-gamepad2.left_stick_y/2);
            robot.vSlides.vSlidesF.setPower(-gamepad2.left_stick_y/2);
        }

        if(gamepad2.left_stick_y < -0.1 && robot.vSlides.vSlidesB.getCurrentPosition() < 480){
            robot.vSlides.vSlidesF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlidesB.setPower(-gamepad2.left_stick_y/2);
            robot.vSlides.vSlidesF.setPower(-gamepad2.left_stick_y/2);
        }*/

        if(gamepad2.right_stick_button && Button.BTN_HORIZONTAL.canPress(timestamp)){
            if(isOut){
                //hSlideGoIn = true;
                //hSlideInDelay = System.currentTimeMillis();
                stayIn = true;
                isOut = false;
            } else{
                stayIn = false;
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.moveEncoderTo(1200, 1);
                isOut = true;
            }
        }

        /*if(hSlideGoIn && System.currentTimeMillis()-hSlideInDelay > 350){
            robot.intakeBigTilt.setTransfer();
            intakeDDelay = true;
            intakeDoorDelay = System.currentTimeMillis();
            //robot.intakeDoor.setOpen();
            //robot.depoDoor.setOpen2();
            robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
            robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
            fClawClosed = false;
            bClawClosed = false;
            intakeTransfer = true;
            hSlideGoIn = false;
        }*/

        if(gamepad2.left_stick_button && Button.BTN_HANG.canPress(timestamp)){
            if(isLocked) {
                robot.hang.setLeftHang();
                robot.hang.setRightHang();
                isLocked = false;
            } else {
                robot.hang.setLeftIn();
                robot.hang.setRightIn();
                isLocked = true;
            }
        }

        telemetry.addData("sensorR: ", robot.sensorR.getDistance(DistanceUnit.CM));
        telemetry.addData("sensorR: ", robot.sensorL.getDistance(DistanceUnit.CM));
        telemetry.addData("sensorF: ", robot.sensorF.getDistance(DistanceUnit.CM));
        telemetry.addData("armVolt: ", robot.depo.getArmVolt());
        telemetry.addData("vSlidesF encoder: ", robot.vSlides.vSlidesF.getCurrentPosition());
        telemetry.addData("vSlidesB encoder: ", robot.vSlides.vSlidesB.getCurrentPosition());
        telemetry.addData("vSlideB power: ", robot.vSlides.vSlidesB.getPower());
        telemetry.addData("vSlideF power: ", robot.vSlides.vSlidesF.getPower());
        telemetry.addData("v limit switch: ", robot.vSlides.switchSlideDown.isTouch());
        telemetry.addData("h limit switch: ", robot.hslides.switchSlideDown.isTouch());
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
        telemetry.addData("hSlidePos", robot.hslides.hslides.getCurrentPosition());
        telemetry.update();
    }
    public void slideBottom() {
        if (!robot.vSlides.slideReachedBottom()) {
            robot.vSlides.vSlidesF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.down();
            RobotLog.ii(TAG_SL, "Going down");
        } else {
            robot.vSlides.forcestop();
            robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideGoBottom = false;
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
    }
}