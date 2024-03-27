package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.MecanumDrive;
import overcharged.components.RobotMecanum;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;

@Autonomous(name="parkAuto")
public class parkAuto extends LinearOpMode {

    private RobotMecanum robot;
    private MecanumDrive drive;
    SelectLinear sl = new SelectLinear(this);
    long currentTime;
    double drivePower = -0.4f;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new RobotMecanum(this, true, false);
            drive = robot.getDrive();
            WaitLinear lp = new WaitLinear(this);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }

            waitForStart();
            //drive.resetAngle();
            if (opModeIsActive()) {
                robot.vSlides.vSlides.setTargetPositionPIDFCoefficients(23,0,0,0);
                robot.intakeDoor.setClosed();
                robot.intakeSmallTilt.setOut();
                //function here
                park(lp);
            }

        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    public void park(WaitLinear lp) throws InterruptedException {
        robot.drive.setPower(0.4f);
        robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
        lp.waitMillis(1200);
        robot.drive.setPower(0f);
        robot.depoTilt.setOut();
        lp.waitMillis(2000);
        robot.depoDoor.setOpen2();
        lp.waitMillis(500);
        robot.depoTilt.setIn();
        lp.waitMillis(1000);
        while (!robot.vSlides.slideReachedBottom()) {// && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.down();
            RobotLog.ii(TAG_SL, "Going down");
        }
        robot.vSlides.forcestop();
        robot.vSlides.vSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*robot.drive.setPower(-0.4f);
        lp.waitMillis(1200);
        robot.drive.setPower(0f);
        robot.intake.out();
        lp.waitMillis(1000);*/
    }
}
