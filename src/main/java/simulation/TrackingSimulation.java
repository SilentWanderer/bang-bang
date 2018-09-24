package simulation;

import control.DrivePlanner;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.trajectory.Trajectory;
import lib.trajectory.timing.CentripetalAccelerationConstraint;
import lib.trajectory.timing.TimedState;
import lib.trajectory.timing.TimingConstraint;
import lib.util.ReflectingCSVWriter;
import odometry.Kinematics;
import odometry.RobotState;
import odometry.RobotStateEstimator;
import profiles.LockdownProfile;
import profiles.RobotProfile;

import java.util.Arrays;
import java.util.List;

public class TrackingSimulation {

    private final double kDt = 1.0 / 100.0;

    private RobotProfile mRobotProfile = new LockdownProfile();
    private DrivePlanner mDrivePlanner = new DrivePlanner(mRobotProfile, DrivePlanner.PlannerMode.FEEDBACK);
    private Kinematics mKinematicModel = new Kinematics(mRobotProfile);
    private RobotState mRobotState = new RobotState(mKinematicModel);
    private RobotStateEstimator mRobotStateEstimator = new RobotStateEstimator(mRobotState, mKinematicModel);

    private List<Pose2d> mToScale = Arrays.asList(new Pose2d[] {
            new Pose2d(0.0, 50.0, Rotation2d.fromDegrees(0.0)),
            new Pose2d(140.0, 50.0, Rotation2d.fromDegrees(0.0)),
            new Pose2d(290.0, 80.0, Rotation2d.fromDegrees(0.0)),
    });

    private List<Pose2d> mToSwitch = Arrays.asList(new Pose2d[] {
            mToScale.get(mToScale.size() - 1),
            new Pose2d(210.0, 90.0, new Rotation2d(-1, 0, false)),
    });

    // in / s
    private final double kMaxVel = 120.0; // 10 ft/s -> 120 in / s
    private final double kMaxAccel = 60.0;
    private final double kMaxCentripetalAccel = 50;
    private final double kMaxVoltage = 12.0;

    private List<TimingConstraint<Pose2dWithCurvature>> mTrajectoryConstraints = Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel));


    public void simulate() {

        ReflectingCSVWriter<Pose2d> csvPoseWriter = new ReflectingCSVWriter<>("tracking.csv", Pose2d.class);
        ReflectingCSVWriter<DrivePlanner> csvDrivePlanner = new ReflectingCSVWriter<>("trajectory.csv", DrivePlanner.class);
        DriveSimulation driveSimulation = new DriveSimulation(csvPoseWriter, csvDrivePlanner, mRobotStateEstimator, mDrivePlanner);

        driveSimulation.driveTrajectory(generateTrajectory(mToScale), kDt, true);
        driveSimulation.driveTrajectory(generateTrajectory(mToSwitch), kDt, false);

    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(List<Pose2d> pTrajectory) {
        return mDrivePlanner.generateTrajectory(false, pTrajectory, mTrajectoryConstraints, kMaxVel, kMaxAccel, kMaxVoltage);
    }

}
