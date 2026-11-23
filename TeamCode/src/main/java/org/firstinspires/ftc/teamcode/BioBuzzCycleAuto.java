package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Launch - collect - relaunch autonomous for the goBILDA StarterBot (BIOBUZZ, 2026-2027).
 *
 * The robot starts against its own alliance wall with the intake touching the wall, exactly like
 * StarterBotAuto, then:
 *   1. launches the 4 preloaded Pollen,
 *   2. backs off the wall just far enough to turn, and turns toward its Garden,
 *   3. drives into the Garden corner with the intake running to collect the 4 Pollen there,
 *   4. retraces its path back to the wall (spinning the launcher up on the way),
 *   5. launches what it collected,
 *   6. backs away from the wall to finish.
 *
 * This class is abstract: it holds the whole routine but does not show up on the Driver Station.
 * BioBuzzCycleAutoRed and BioBuzzCycleAutoBlue are the two OpModes you select.
 *
 * RED AND BLUE RUN THE SAME PATH. The BIOBUZZ field is not a mirror image, it is the same layout
 * rotated half a turn: the Red Garden is on tile A1 and the Blue Garden is on tile F6 (Event
 * Field Setup Guide, sections 6 and 8.4). Seen from its own alliance wall, each robot has its
 * Garden in the same direction, so both alliances turn the same way.
 *
 * HOW TO TUNE: everything you should need to change is in the "PATH" and "TUNING" blocks below.
 */
public abstract class BioBuzzCycleAuto extends OpMode {

    /*
     * ---------------------------------- PATH ----------------------------------
     * Distances are in inches, because the field is built from 24 inch tiles. Positive drives
     * forward (intake first), negative drives backward. Positive turns are counter-clockwise
     * (left) when looking down at the robot.
     *
     * START POSITION these numbers assume (measure and adjust if yours differs):
     *   Red:  intake against the Red alliance wall, robot centered on tile A3.
     *   Blue: intake against the Blue alliance wall, robot centered on tile F4.
     * From there the Garden corner is 2 tiles to the robot's left along the wall. With an
     * 18 inch robot that is about 46 inches of driving after the turn, split below into a fast
     * approach and a slow final push into the Pollen.
     */
    static final double WALL_CLEARANCE_IN = -6;     // back off the wall so the corners clear it in the turn
    static final double TURN_TO_GARDEN_DEG = 90;    // turn to face the Garden corner
    static final double GARDEN_APPROACH_IN = 34;    // fast part of the drive toward the Garden
    static final double GARDEN_COLLECT_IN = 12;     // slow final push, intake running
    static final double LEAVE_IN = -12;             // final move away from the wall

    static final double PRELOAD_LAUNCH_SECONDS = 5; // time allowed to launch the preloads
    static final double CYCLE_LAUNCH_SECONDS = 5;   // time allowed to launch the collected Pollen

    /*
     * --------------------------------- TUNING ---------------------------------
     */
    static final int LAUNCHER_TARGET_VELOCITY = 1250;   // encoder ticks per second
    static final int LAUNCHER_MIN_VELOCITY = 1200;      // only feed when spinning at least this fast

    /*
     * Direction the windmill turns to feed the launcher. StarterBotAuto feeds with +1 and
     * BioBuzzStarterbotTeleop feeds with -1 (both with the servo set to REVERSE), so one of them
     * is backwards. Flip this sign if the windmill pushes elements away from the launcher.
     */
    static final double WINDMILL_FEED_POWER = 1;

    static final double INTAKE_COLLECT_POWER = 1;       // intake power while picking up
    static final double INTAKE_LAUNCH_ASSIST_POWER = 0.5; // nudges stuck elements while launching

    static final double DRIVE_SPEED = 0.5;
    static final double COLLECT_SPEED = 0.3;            // slower, so the intake has time to grab
    static final double ROTATE_SPEED = 0.2;

    static final double WHEEL_DIAMETER_MM = 96;
    static final double ENCODER_TICKS_PER_REV = 537.7;
    static final double TICKS_PER_MM = ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    static final double TRACK_WIDTH_MM = 402;

    static final double MOVE_TOLERANCE_MM = 10;         // how close counts as "arrived"
    static final double MOVE_SETTLE_SECONDS = 0.25;     // stay inside tolerance this long
    static final double MOVE_TIMEOUT_SECONDS = 4;       // give up on a blocked move and carry on

    /*
     * One step of the routine. A step is either a launch (timed) or a move (drive or turn).
     * A move can also run the intake, or spin the launcher up early so it is at speed on arrival.
     */
    private static class Step {
        final String name;
        final boolean isLaunch;
        final double seconds;       // launch only
        final double leftMm;        // move only
        final double rightMm;       // move only
        final double speed;         // move only
        boolean intakeOn = false;
        boolean launcherOn = false;

        private Step(String name, boolean isLaunch, double seconds,
                     double leftMm, double rightMm, double speed) {
            this.name = name;
            this.isLaunch = isLaunch;
            this.seconds = seconds;
            this.leftMm = leftMm;
            this.rightMm = rightMm;
            this.speed = speed;
        }

        static Step launch(String name, double seconds) {
            Step step = new Step(name, true, seconds, 0, 0, 0);
            step.launcherOn = true;
            return step;
        }

        static Step drive(String name, double inches, double speed) {
            double mm = DistanceUnit.INCH.toMm(inches);
            return new Step(name, false, 0, mm, mm, speed);
        }

        /*
         * The robot has rotated one radian when each wheel has travelled half the track width
         * in opposite directions. A positive angle turns counter-clockwise (left wheel backward).
         */
        static Step rotate(String name, double degrees, double speed) {
            double mm = Math.toRadians(degrees) * (TRACK_WIDTH_MM / 2);
            return new Step(name, false, 0, -mm, mm, speed);
        }

        Step withIntake() {
            intakeOn = true;
            return this;
        }

        Step withLauncherSpinUp() {
            launcherOn = true;
            return this;
        }
    }

    private Step[] steps;
    private int stepIndex;
    private boolean stepStarted;
    private int leftTarget;
    private int rightTarget;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * The whole routine, in order. To change what the auto does, edit this list.
     */
    private Step[] buildSteps() {
        double toGarden = GARDEN_APPROACH_IN + GARDEN_COLLECT_IN;
        return new Step[] {
                Step.launch("Launch preloads", PRELOAD_LAUNCH_SECONDS),
                Step.drive("Back off wall", WALL_CLEARANCE_IN, DRIVE_SPEED),
                Step.rotate("Turn to Garden", TURN_TO_GARDEN_DEG, ROTATE_SPEED),
                Step.drive("Approach Garden", GARDEN_APPROACH_IN, DRIVE_SPEED).withIntake(),
                Step.drive("Collect Pollen", GARDEN_COLLECT_IN, COLLECT_SPEED).withIntake(),
                Step.drive("Back out of Garden", -toGarden, DRIVE_SPEED).withIntake(),
                Step.rotate("Turn to wall", -TURN_TO_GARDEN_DEG, ROTATE_SPEED).withLauncherSpinUp(),
                Step.drive("Return to wall", -WALL_CLEARANCE_IN, DRIVE_SPEED).withLauncherSpinUp(),
                Step.launch("Launch collected", CYCLE_LAUNCH_SECONDS),
                Step.drive("Leave", LEAVE_IN, DRIVE_SPEED),
        };
    }

    @Override
    public void init() {
        steps = buildSteps();
        stepIndex = 0;
        stepStarted = false;

        // These names must match the robot configuration on the Driver Station.
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        autoTimer.reset();
    }

    @Override
    public void loop() {
        /*
         * We work out what every mechanism should be doing first, then write to each motor and
         * servo exactly once at the bottom of the loop. Setting the same motor twice in one loop
         * can make it behave erratically.
         */
        boolean launcherOn = false;
        boolean feed = false;
        double intakePower = 0;

        if (stepIndex < steps.length) {
            Step step = steps[stepIndex];

            if (!stepStarted) {
                beginStep(step);
                stepStarted = true;
            }

            launcherOn = step.launcherOn;
            if (step.intakeOn) {
                intakePower = INTAKE_COLLECT_POWER;
            }

            boolean finished;
            if (step.isLaunch) {
                // Only feed once the wheel is up to speed, so every shot leaves at full power.
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    feed = true;
                    intakePower = INTAKE_LAUNCH_ASSIST_POWER;
                }
                finished = stepTimer.seconds() > step.seconds;
            } else {
                finished = moveFinished();
            }

            if (finished) {
                stepIndex++;
                stepStarted = false;
            }
        } else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            telemetry.addLine("Auto Complete!");
        }

        launcher.setVelocity(launcherOn ? LAUNCHER_TARGET_VELOCITY : 0);
        windmillServo.setPower(feed ? WINDMILL_FEED_POWER : 0);
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        telemetry.addData("Step", "%d/%d %s", Math.min(stepIndex + 1, steps.length), steps.length,
                stepIndex < steps.length ? steps[stepIndex].name : "done");
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Launcher velocity", "%.0f", launcher.getVelocity());
        telemetry.addData("Drive position", "left (%d), right (%d)",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Drive target", "left (%d), right (%d)", leftTarget, rightTarget);
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        launcher.setVelocity(0);
        intake.setPower(0);
        windmillServo.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    /*
     * Runs once at the start of each step. For a move, the targets are measured from where the
     * wheels are right now, so every step in the list is relative to the end of the one before.
     */
    private void beginStep(Step step) {
        stepTimer.reset();
        settleTimer.reset();

        if (step.isLaunch) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            return;
        }

        leftTarget = leftDrive.getCurrentPosition() + (int) Math.round(step.leftMm * TICKS_PER_MM);
        rightTarget = rightDrive.getCurrentPosition() + (int) Math.round(step.rightMm * TICKS_PER_MM);

        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(step.speed);
        rightDrive.setPower(step.speed);
    }

    /*
     * A move is finished when BOTH wheels have stayed within tolerance of their targets for
     * MOVE_SETTLE_SECONDS. If the robot is blocked and never arrives, we move on after
     * MOVE_TIMEOUT_SECONDS instead of sitting there for the rest of the autonomous period.
     */
    private boolean moveFinished() {
        double toleranceTicks = MOVE_TOLERANCE_MM * TICKS_PER_MM;
        boolean leftArrived = Math.abs(leftTarget - leftDrive.getCurrentPosition()) <= toleranceTicks;
        boolean rightArrived = Math.abs(rightTarget - rightDrive.getCurrentPosition()) <= toleranceTicks;

        if (!leftArrived || !rightArrived) {
            settleTimer.reset();
        }

        return settleTimer.seconds() > MOVE_SETTLE_SECONDS
                || stepTimer.seconds() > MOVE_TIMEOUT_SECONDS;
    }
}
