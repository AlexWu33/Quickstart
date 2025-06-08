package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "The_Buckets", group = "Examples")
public class Samples extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(18, 130, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(18, 125, Math.toRadians(355));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(18, 130, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(19, 133, Math.toRadians(25));

    /**Intake from Submersible */
    private final Pose pickupSubPose = new Pose(65, 95, Math.toRadians(270));

    /** Used as the control point for the BeizerCurve for SubPose*/
    private final Pose pickupSubControlPose = new Pose(50, 120, Math.toRadians(270));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(63, 96, Math.toRadians(90));

    /** Used as the control point for the BeizerCurve for parkPose*/
    private final Pose parkControlPose = new Pose(63, 110, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park, score2;

    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, grabSubPose1, scoreSubPose1, grabSubPose2, scoreSubPose2;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        //Since we are not returning to a point we will use a path for this singular movement

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        grabSubPose1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupSubControlPose), new Point(pickupSubPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading())
                .build();

        scoreSubPose1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSubPose), new Point(pickupSubControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupSubPose.getHeading(), scorePose.getHeading())
                .build();

        grabSubPose2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupSubControlPose), new Point(pickupSubPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading())
                .build();

        scoreSubPose2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSubPose), new Point(pickupSubControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupSubPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1,false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup2,false);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 5 || !follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 5 || !follower.isBusy()) {
                    follower.followPath(grabPickup3,false);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 5 || !follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 5 || !follower.isBusy()) {
                    follower.followPath(grabSubPose1, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSubPose1, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(grabSubPose2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSubPose2, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(park,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("follower is busy", follower.isBusy());
        telemetry.addData("pathtimer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
