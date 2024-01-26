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
@Autonomous(name="auto4OneCycleFar")
public class auto4OneCycleFar extends LinearOpMode {

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
    propLocation location = propLocation.Middle;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    float xPurpleDump, yPurpleDump, xYellowDump, yYellowDump, xPark, yPark, xIntake;
    TrajectorySequence test, dumpPurplePixel, extraForPurple, dumpYellowPixel1, dumpYellowPixel2,
            dumpYellowPixel3, park, goToIntake, cycleIntake1, cycleIntake2, additionalCycleDump, additionalCycleDump2, cycleIntake3;
    Pose2d start = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
            robot = new RobotMecanum(this, true, true);
            drive = new SampleMecanumDrive(hardwareMap);
            WaitLinear lp = new WaitLinear(this);
            initCamera(); // initializes camera and sets up pipeline for team shipping element detection

            //initialize trajectories
            /*dumpPurplePixel = drive.trajectorySequenceBuilder(start)
                    //.lineTo(new Vector2d(50, 0))
                    .lineToLinearHeading(new Pose2d(xPurpleDump, yPurpleDump, Math.toRadians(Blue? 90 : -90)))
                    .build();
            extraForPurple = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                    .lineTo(new Vector2d(xPurpleDump, Blue? yPurpleDump-15 : yPurpleDump+18))
                    .build();
            goToIntake = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                    .lineToLinearHeading(new Pose2d(xIntake, Blue? 18: -17, Math.toRadians(Blue? 90:-90)))
                    .build();
            dumpYellowPixel1 = drive.trajectorySequenceBuilder(goToIntake.end())
                    .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-23, Blue ? 18: -18, Math.toRadians(Blue? 90:-90)))
                    .build();
            dumpYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                    .lineTo(new Vector2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? 10:-10), () -> {
                        robot.intakeSmallTilt.setTransfer();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? 7:-7), () -> {
                        robot.intakeDoor.setOpen();
                        robot.depoDoor.setOpen2();
                        robot.intake.in();
                    })
                    .build();
            dumpYellowPixel3 = drive.trajectorySequenceBuilder(dumpYellowPixel2.end())
                    .lineToLinearHeading(new Pose2d(xYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -35:35), () -> {
                        robot.depoTilt.setOut();
                    })
                    .build();
            cycleIntake1 = drive.trajectorySequenceBuilder(dumpYellowPixel3.end())
                    .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48, Math.toRadians(Blue? 90 : -90)))
                    .build();
            cycleIntake2 = drive.trajectorySequenceBuilder(cycleIntake1.end())
                    .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-19, Blue ? -15: 15, Math.toRadians(Blue? 90 : -95)))
                    .build();
            additionalCycleDump = drive.trajectorySequenceBuilder(cycleIntake2.end())
                    .lineTo(new Vector2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48))
                    .build();*/

            Blue = sl.selectAlliance();

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
                location = detector.getLocation(!Blue, true);
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
                    location = detector.getLocation(!Blue, true);
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
                yYellowDump = Blue? -90: 92;
                xIntake = -26;
                if(location==propLocation.Middle){
                    xPurpleDump = Blue? -29: -29;
                    yPurpleDump = Blue? -2: 1f;
                    xYellowDump = Blue? -26: -24;
                    //yYellowDump = Blue? -36: 36;
                }
                else if(location==propLocation.Left){
                    xPurpleDump = Blue? -26: -26;
                    yPurpleDump = Blue? -2f: -5;//9.5f;
                    xYellowDump = Blue? -18: -26.5f;//-18;
                    //yYellowDump = Blue? -36: 36;
                }
                else{ //right
                    xPurpleDump = Blue? -26: -26;
                    yPurpleDump = Blue? 6.5f: 0f;//-4;
                    xYellowDump = Blue? -26f: -14f;//-36;
                    //yYellowDump = Blue? -36: 36;
                }

                xPark = -40;
                yPark = yYellowDump+5;

                //initialize trajectories
                dumpPurplePixel = drive.trajectorySequenceBuilder(start)
                        //.lineTo(new Vector2d(50, 0))
                        .lineToLinearHeading(new Pose2d(xPurpleDump, yPurpleDump, Math.toRadians(Blue? 90 : -90)))
                        .build();
                extraForPurple = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                        .lineTo(new Vector2d(xPurpleDump, Blue? yPurpleDump-15 : yPurpleDump+18))
                        .build();
                goToIntake = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                        .lineToLinearHeading(new Pose2d(xIntake, Blue? 18: -17, Math.toRadians(Blue? 90:-90)))
                        .build();
                dumpYellowPixel1 = drive.trajectorySequenceBuilder(goToIntake.end())
                        .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-23, Blue ? 17: -17, Math.toRadians(Blue? 90:-90)))
                        .build();
                dumpYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(70, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineTo(new Vector2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? 10:-10), () -> {
                            robot.intakeSmallTilt.setTransfer();
                        })
                        .addSpatialMarker(new Vector2d(xIntake, Blue? 7:-7), () -> {
                            robot.intakeDoor.setOpen();
                            robot.depoDoor.setOpen2();
                            robot.intake.in();
                        })
                        .build();
                dumpYellowPixel3 = drive.trajectorySequenceBuilder(dumpYellowPixel2.end())
                        //.splineTo(new Vector2d(xYellowDump, yYellowDump), Math.toRadians(Blue? 90 : -90))
                        .lineToLinearHeading(new Pose2d(xYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -35:35), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();
                cycleIntake1 = drive.trajectorySequenceBuilder(dumpYellowPixel3.end())
                        .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48, Math.toRadians(Blue? 90 : -90)))
                        .build();
                cycleIntake2 = drive.trajectorySequenceBuilder(cycleIntake1.end())
                        .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-19, Blue ? -10: 10, Math.toRadians(Blue? 90 : -90)))
                        .build();
                additionalCycleDump = drive.trajectorySequenceBuilder(cycleIntake2.end())
                        .lineTo(new Vector2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48))
                        .build();
                additionalCycleDump2 = drive.trajectorySequenceBuilder(dumpYellowPixel2.end())
                        //.splineTo(new Vector2d(xYellowDump, yYellowDump), Math.toRadians(Blue? 90 : -90))
                        .lineToLinearHeading(new Pose2d(Blue? -26: -23, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -35:35), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();

                cycleIntake3 = drive.trajectorySequenceBuilder(cycleIntake2.end())
                        .lineToLinearHeading(new Pose2d(xIntake, Blue? 18: -18, Math.toRadians(Blue? 90:-90)))
                        .build();

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

        robot.depoDoor.setClosed();
        robot.intakeDoor.setClosed();

        drive.followTrajectorySequence(dumpPurplePixel);

        if((location == propLocation.Right && !Blue) || (location == propLocation.Left && Blue))
            drive.followTrajectorySequence(extraForPurple);

        //robot.leftPixel.setDump();
        //lp.waitMillis(500);
        RobotLog.ii(RobotConstants.TAG_R, "left pixel isBlue" + Blue + "dump" + robot.leftPixel.DUMP);

        if(Blue) {
            float lPixelPos = robot.leftPixel.pixelDropper.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            RobotLog.ii(RobotConstants.TAG_R, "left pixel pos" + lPixelPos + "dump" + robot.leftPixel.DUMP);
            while (lPixelPos > robot.leftPixel.DUMP && System.currentTimeMillis() - dropperTime < 1000) {
                RobotLog.ii(RobotConstants.TAG_R, "moving left pixel");
                lPixelPos -= 3;
                robot.leftPixel.setPosition(lPixelPos);
                lp.waitMillis(7);
            }
        }
        else {
            float rPixelPos = robot.rightPixel.pixelDropper.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (rPixelPos <= robot.rightPixel.DUMP && System.currentTimeMillis() - dropperTime < 500) {//hSlidesOut >= hSlides.MIN+10) {
                rPixelPos += 3;
                robot.rightPixel.setPosition(rPixelPos);
                lp.waitMillis(7);
            }
        }

        robot.intake.in();
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH);
        //robot.intakeSmallTilt.setPosition(robot.intakeSmallTilt.FIFTHP);
        drive.followTrajectorySequence(goToIntake);
        lp.waitMillis(150);

        robot.intake.off();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();
        drive.followTrajectorySequence(dumpYellowPixel1);
        robot.intakeSmallTilt.setOut();
        drive.followTrajectorySequence(dumpYellowPixel2);
        robot.depoDoor.setClosed();
        robot.intake.off();
        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        lp.waitMillis(250);
        robot.depoTilt.setOut();
        drive.followTrajectorySequence(dumpYellowPixel3);

        //robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        //lp.waitMillis(600);

        //robot.depoTilt.setOut();
        //drive.followTrajectorySequence(dumpYellowPixel2);

        robot.depoDoor.setOpen2();
        lp.waitMillis(250);

        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+150, 1);
        lp.waitMillis(300);

        //drive.followTrajectorySequence(park);
        robot.depoTilt.setIn();
        //lp.waitMillis(500);
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH);
        robot.intakeDoor.setClosed();
        lowerSlidesThread(lp);

        drive.followTrajectorySequence(cycleIntake1);
        robot.hslides.moveEncoderTo(1900, 1);
        robot.intake.in();

        drive.followTrajectorySequence(cycleIntake2);

        lp.waitMillis(100);
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH);
        lp.waitMillis(250);

        robot.hslides.moveEncoderTo(robot.hslides.START, -1);
        //robot.intake.off();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();
        drive.followTrajectorySequence(additionalCycleDump);
        //transfer code
        robot.intake.in();
        robot.intakeDoor.setOpen();
        robot.depoDoor.setOpen2();
        lp.waitMillis(400);
        robot.depoDoor.setClosed();
        robot.intake.off();
        robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
        lp.waitMillis(250);
        robot.depoTilt.setOut();
        drive.followTrajectorySequence(additionalCycleDump2);

        robot.depoDoor.setOpen2();
        lp.waitMillis(500);

        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+180, 1);
        lp.waitMillis(200);
        robot.depoTilt.setIn();

        lowerSlidesThread(lp);

        lp.waitMillis(30000-System.currentTimeMillis()+startTime);
    }

    public void lowerSlidesThread(WaitLinear lp) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(0.6f, false, lp, this, robot);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }
}