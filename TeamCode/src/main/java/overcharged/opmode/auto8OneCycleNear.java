package overcharged.opmode;


import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.RobotMecanum;
import overcharged.components.hslides;
import overcharged.components.propLocation;
import overcharged.config.RobotConstants;
import overcharged.drive.DriveConstants;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.drive.SampleMecanumDrive;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.HSVPipeline;
import overcharged.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="auto8OneCycleNear")
public class auto8OneCycleNear extends LinearOpMode {

    private RobotMecanum robot;
    SampleMecanumDrive drive;
    SelectLinear sl = new SelectLinear(this);
    int detectionWaitTime = 650;
    long startTime;
    long currentTime;
    HSVPipeline detector;
    OpenCvWebcam webcam;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;
    boolean Blue = true;
    double waitTime = 0;
    boolean DropHeight = true;
    propLocation location = propLocation.Middle;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    float xPurpleDump, yPurpleDump, xYellowDump, yYellowDump, xPark, yPark;
    TrajectorySequence test, dumpMPurplePixel, dumpLPurplePixel, dumpRPurplePixel, extraForPurple,
            dumpMYellowPixel, dumpLYellowPixel, dumpRYellowPixel, park, goToIntake, goToDump1, extraPush2;
    Pose2d start = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
            robot = new RobotMecanum(this, true, true);
            drive = new SampleMecanumDrive(hardwareMap);
            WaitLinear lp = new WaitLinear(this);
            initCamera(); // initializes camera and sets up pipeline for team shipping element detection
            robot.hang.setRightIn();
            robot.intakeDoor.setOpen();
            robot.depo.setBothClawsClose();

            Blue = sl.selectAlliance();
            if(Blue){robot.pixel.setLeftIn();}
            else{robot.pixel.setRightIn();}
            DropHeight = sl.selectDropHeight(); //true = low, false = high drop
            waitTime = sl.adjustDelay();

            float yYellowDump = Blue? -38f:38;//91f;
            float xIntake = Blue ? -53:-53;
            float yMidIntake = 18f;
            float yIntake = Blue? 45f : -45f;

            //MIDDLE
            float xMPurpleDump = Blue? -29: -29;
            float yMPurpleDump = Blue? -3: 3f;
            float xMYellowDump = Blue? -25: -25;

            //LEFT
            float xLPurpleDump = Blue? -26: -26;
            float yLPurpleDump = Blue? -10.5f: -3;//9.5f;
            float xLYellowDump = Blue? -18f: -33f;//-18;

            //RIGHT
            float xRPurpleDump = Blue? -26: -26;
            float yRPurpleDump = Blue? 4f: 10.5f;//-4;
            float xRYellowDump = Blue? -33.5f: -18f;//-36;

            //initialize trajectories
            dumpMPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xMPurpleDump, yMPurpleDump, Math.toRadians(Blue? 90 : -90)))
                    .build();
            dumpLPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xLPurpleDump, yLPurpleDump, Math.toRadians(Blue? 90 : -90)))
                    .build();
            dumpRPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xRPurpleDump, yRPurpleDump, Math.toRadians(Blue? 90 : -90)))
                    .build();
            extraForPurple = drive.trajectorySequenceBuilder(Blue? dumpRPurplePixel.end() : dumpLPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(Blue? xRPurpleDump : xLPurpleDump, Blue? yRPurpleDump+8.5 : yLPurpleDump-9))
                    .build();
            dumpMYellowPixel = drive.trajectorySequenceBuilder(dumpMPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xMYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        robot.depo.setDepoOutVert();
                    })
                    .build();
            dumpLYellowPixel = drive.trajectorySequenceBuilder(dumpLPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xLYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        robot.depo.setDepoOutVert();
                    })
                    .build();
            dumpRYellowPixel = drive.trajectorySequenceBuilder(dumpRPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xRYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        robot.depo.setDepoOutVert();
                    })
                    .build();
            goToIntake = drive.trajectorySequenceBuilder(Blue? (new Pose2d(dumpRYellowPixel.end().getX(), dumpRYellowPixel.end().getY()+5, Math.toRadians(92.5))) : new Pose2d(dumpLYellowPixel.end().getX(), dumpLYellowPixel.end().getY()-3, Math.toRadians(-92.5)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .splineToConstantHeading(new Vector2d(xIntake, Blue? -yMidIntake:yMidIntake), Math.toRadians(Blue? 92.5 : -92.5))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+5):(yMidIntake+5)), () -> {
                        robot.depo.setDepoIn();
                        //robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+1):(yMidIntake+1)), () -> {
                        slidesDown(lp);
                        robot.hslides.moveEncoderTo(1850, 1);
                    })
                    .lineTo(new Vector2d(xIntake, yIntake))
                    .build();
            goToDump1 = drive.trajectorySequenceBuilder(goToIntake.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(xIntake, Blue? -yMidIntake:yMidIntake))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-3:-(yIntake-3)), () -> {
                        robot.intakeDoor.setClosed();
                        robot.intakeBigTilt.setTransfer();
                        robot.intakeSmallTilt.setTransfer();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-6:-(yIntake-6)), () -> {
                        robot.intake.out();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-7:-(yIntake-7)), () -> {
                        robot.intake.off();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-10:-(yIntake-10)), () -> {
                        robot.intake.slowIn();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-13:-(yIntake-13)), () -> {
                        robot.intake.slowIn();
                        robot.intakeDoor.setOpen();
                        // robot.depoDoor.setOpen2();
                        robot.depo.setBothClawsOpen();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-44:-(yIntake-44)), () -> {
                        robot.intakeDoor.setClosed();
                        // robot.depoDoor.setClosed();
                        robot.depo.setBothClawsClose();
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -19:19), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.level1+100, 1);
                    })
                    .splineToConstantHeading(new Vector2d(xMYellowDump, Blue? (yYellowDump+13) : yYellowDump-13), Math.toRadians(Blue? 90:-90))
                    .build();
            extraPush2 = drive.trajectorySequenceBuilder(goToDump1.end())
                    .lineToLinearHeading(new Pose2d(xMYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? (yYellowDump+11) : yYellowDump-11), () -> {
                        robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                    })*/
                    .build();
            park = drive.trajectorySequenceBuilder(new Pose2d(dumpMYellowPixel.end().getX(), Blue ? dumpMYellowPixel.end().getY()+7 : dumpMYellowPixel.end().getY()-7, Math.toRadians(Blue? 90 : -90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToConstantHeading(new Vector2d(2, Blue? yYellowDump+2:yYellowDump-2))
                    .addSpatialMarker(new Vector2d(-10, Blue? yYellowDump:yYellowDump), () -> {
                        robot.depo.setDepoIn();
                    })
                    .addSpatialMarker(new Vector2d(-3, Blue? yYellowDump+2:yYellowDump-2), () -> {
                        slidesDown(lp);
                    })
                    .build();

            this.detector = new HSVPipeline();
            //this.detector.useDefaults();
            webcam.setPipeline(detector);
            //detector.isLeft(Top);

            try {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            } catch (Exception e) {
                try {
                    this.detector = new HSVPipeline();
                    //this.detector.useDefaults();
                    webcam.setPipeline(detector);
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                } catch (Exception f) {
                    telemetry.addLine("Error");
                    telemetry.update();
                    location = propLocation.Middle;
                }
            }

            telemetry.update();

            long time1 = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();
            while (currentTime - time1 < detectionWaitTime) {
                location = detector.getLocation(!Blue, false);
                currentTime = System.currentTimeMillis();
            }

            //detector.reset();
            telemetry.addData("Prop Location", location);
            telemetry.addData("Blue?", Blue);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }

            waitForStart();

            /*test = drive.actionBuilder(drive.pose)
                    .lineToY(10)
                    .build();*/


            if (opModeIsActive()) {
                robot.clearBulkCache();
                telemetry.addLine("running");
                telemetry.update();
                RobotLog.ii(RobotConstants.TAG_R, "location" + location);

                startTime = System.currentTimeMillis();
                time1 = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();
                while (currentTime - time1 < detectionWaitTime) {
                    location = detector.getLocation(!Blue, false);
                    currentTime = System.currentTimeMillis();
                }

                robot.hang.setLeftIn();
                robot.hang.setRightIn();

                robot.vSlides.reset(robot.vSlides.vSlidesB);
                robot.vSlides.reset(robot.vSlides.vSlidesF);

                //detector.reset();
                telemetry.addData("Blue alliance", Blue);
                telemetry.addData("Prop location", location);
                telemetry.update();

                /*robot.intakeDoor.setOpen();
                lp.waitMillis(500);

                telemetry.addLine("before stopSteaming");
                telemetry.update();
                lp.waitMillis(3000);*/

                webcam.stopStreaming();
                webcam.closeCameraDevice();

                telemetry.addLine("before AutoBody");
                telemetry.update();

                AutoBody(lp, Blue);
            } else {
                telemetry.addLine("Not active");
                telemetry.update();
            }

        } catch (Exception e) {
            RobotLog.e("Overcharged: Autonomous Failed: ", e.getMessage());
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void AutoBody(WaitLinear lp, boolean Blue) throws InterruptedException {
        RobotLog.ii(TAG_SL, "started");
        telemetry.addLine("running auto body");
        telemetry.update();
        RobotLog.ii(RobotConstants.TAG_R, "running auto body");

        lp.waitMillis(waitTime);

        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMPurplePixel);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLPurplePixel);}
        else { drive.followTrajectorySequence(dumpRPurplePixel); }

        if((location == propLocation.Right && Blue) || (location == propLocation.Left && !Blue))
            drive.followTrajectorySequence(extraForPurple);

        //robot.leftPixel.setDump();
        //lp.waitMillis(500);
        RobotLog.ii(RobotConstants.TAG_R, "left pixel isBlue" + Blue + "dump" + robot.pixel.LEFT_DUMP);

        if(Blue) {
            float lPixelPos = robot.pixel.pixel.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (lPixelPos >= robot.pixel.LEFT_OUT && System.currentTimeMillis() - dropperTime < 1000) {
                RobotLog.ii(RobotConstants.TAG_R, "left pixel pos" + lPixelPos + "dump" + robot.pixel.LEFT_DUMP);
                RobotLog.ii(RobotConstants.TAG_R, "moving left pixel");
                lPixelPos -= 3;
                robot.pixel.setPos(lPixelPos);
                lp.waitMillis(5);
            }
        }
        else {
            float rPixelPos = robot.pixel.pixel.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (rPixelPos <= robot.pixel.RIGHT_OUT && System.currentTimeMillis() - dropperTime < 500) {//hSlidesOut >= hSlides.MIN+10) {
                rPixelPos += 3;
                robot.pixel.setPos(rPixelPos);
                lp.waitMillis(5);
            }
        }

        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        //lp.waitMillis(400);

        //lp.waitMillis(1000);
        //robot.depo.setArmPos(robot.depo.ARM_OUT);
        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMYellowPixel);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLYellowPixel);}
        else { drive.followTrajectorySequence(dumpRYellowPixel); }
        //robot.depoDoor.setOpen2();
        robot.depo.setBothClawsOpen();
        lp.waitMillis(150);

        //cycle starts
        robot.intakeDoor.setClosed();
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH);
        robot.intake.slowIn();
        drive.followTrajectorySequence(goToIntake);
        lp.waitMillis(75);
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH);
        lp.waitMillis(200);

        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH+8);
        robot.hslides.moveEncoderTo(hslides.START,-1);
        /*lp.waitMillis(150);
        robot.intakeDoor.setClosed();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();*/
        drive.followTrajectorySequence(goToDump1);
        robot.depo.setDepoOutVert();
        drive.followTrajectorySequence(extraPush2);
        robot.depo.setBothClawsOpen();
        lp.waitMillis(150);
        //cycle endssd

        //cycle starts
        robot.intakeDoor.setClosed();
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.THIRD);
        robot.intake.slowIn();
        //robot.depo.setArmPos(robot.depo.ARM_IN);
        drive.followTrajectorySequence(goToIntake);
        lp.waitMillis(75);
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.SECOND);
        lp.waitMillis(200);

        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH+8);
        robot.hslides.moveEncoderTo(hslides.START,-1);
        /*lp.waitMillis(150);
        robot.intakeDoor.setClosed();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();*/
        drive.followTrajectorySequence(goToDump1);
        robot.depo.setDepoOutVert();
        drive.followTrajectorySequence(extraPush2);
        robot.depo.setBothClawsOpen();
        lp.waitMillis(150);
        //cycle ends

        robot.vSlides.moveEncoderTo(robot.vSlides.level3-200, 1);
        drive.followTrajectorySequence(park);
        /*lp.waitMillis(250);
        robot.depo.setDepoIn();
        lp.waitMillis(250);
        slidesDown(lp);*/

        robot.hang.setRightIn();
        //lowerSlidesThread(lp);
        lp.waitMillis(30000-System.currentTimeMillis()+startTime);
    }

    public void slidesDown(WaitLinear lp){
        robot.vSlides.vSlidesF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(!robot.vSlides.slideReachedBottom()){
            robot.vSlides.down();
        }
        robot.vSlides.forcestop();
        robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void lowerSlidesThread(WaitLinear lp) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(0.6f, false, lp, this, robot);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }
}