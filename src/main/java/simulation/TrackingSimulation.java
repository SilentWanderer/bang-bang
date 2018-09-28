package simulation;

import control.DrivePlanner;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.trajectory.TimedView;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.TrajectorySamplePoint;
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
import java.util.Collections;
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
            new Pose2d(290.0, 80.0, Rotation2d.fromDegrees(-180.0)),
            new Pose2d(210, 90.0, Rotation2d.fromDegrees(-180.0)),
    });

    private List<Pose2d> mSwitchToScale = Arrays.asList(new Pose2d[] {
            new Pose2d(210.0, 90.0, Rotation2d.fromDegrees(0.0)),
            new Pose2d(290.0, 80.0, Rotation2d.fromDegrees(0.0))
    });

    // in / s
    private final double kMaxLinearVel = 120.0; // 10 ft/s -> 120 in / s
    private final double kMaxLinearAccel = 120.0;
    private final double kMaxCentripetalAccel = Double.POSITIVE_INFINITY;
    private final double kMaxVoltage = 12.0;

    private List<TimingConstraint<Pose2dWithCurvature>> mTrajectoryConstraints = Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel));


    public void simulate() {

        ReflectingCSVWriter<Pose2d> csvPoseWriter = new ReflectingCSVWriter<>("tracking.csv", Pose2d.class);
        ReflectingCSVWriter<DrivePlanner> csvDrivePlanner = new ReflectingCSVWriter<>("trajectory.csv", DrivePlanner.class);
        DriveSimulation driveSimulation = new DriveSimulation(csvPoseWriter, csvDrivePlanner, mRobotStateEstimator, mDrivePlanner);

        double timeDriven =
        driveSimulation.driveTrajectory(generateTrajectory(mToScale), kDt, true) +
        driveSimulation.driveTrajectory(generateTrajectory(0.0, 180.0, 120.0, 120.0), kDt) +
        driveSimulation.driveTrajectory(generateTrajectory(mToSwitch), kDt, false) +
        driveSimulation.driveTrajectory(generateTrajectory(180.0, 1.0, 120.0, 120.0), kDt) +
        driveSimulation.driveTrajectory(generateTrajectory(mSwitchToScale), kDt, false);

        System.out.println(timeDriven);

    }

    public void testRotation() {

        ReflectingCSVWriter<Rotation2d> csvRotationWriter = new ReflectingCSVWriter<>("rotation.csv", Rotation2d.class);
        Trajectory<TimedState<Rotation2d>> trajectory = mDrivePlanner.generateTurnInPlaceTrajectory(false, Rotation2d.fromRadians(0.0), Rotation2d.fromRadians(180.0),
                                                                                                    Collections.emptyList(),120.0, 120.0, 12.0);
        TrajectoryIterator<TimedState<Rotation2d>> iterator = new TrajectoryIterator<>(new TimedView<>(trajectory));
        for(double time = 0.0; time < 2.0; time += 0.01) {
            TrajectorySamplePoint<TimedState<Rotation2d>> sample = iterator.advance(0.01);
            System.out.println(sample.state());
            csvRotationWriter.add(sample.state().state());
            csvRotationWriter.flush();
        }

    }


    /**
     * Generate trajectory with default max velocity and acceleration.
     * @param pTrajectory
     * @return
     */
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(List<Pose2d> pTrajectory) {
        return generateTrajectory(pTrajectory, kMaxLinearVel, kMaxLinearAccel);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(List<Pose2d> pTrajectory, double pMaxVel, double pMaxAccel) {
        return mDrivePlanner.generateTrajectory(false, pTrajectory, mTrajectoryConstraints, pMaxVel, pMaxAccel, kMaxVoltage);
    }

    public Trajectory<TimedState<Rotation2d>> generateTrajectory(double initialHeadingDegrees, double finalHeadingDegrees, double pMaxVel, double pMaxAccel) {
        return mDrivePlanner.generateTurnInPlaceTrajectory(false, Rotation2d.fromDegrees(initialHeadingDegrees), Rotation2d.fromDegrees(finalHeadingDegrees),
                                                            mTrajectoryConstraints, pMaxVel, pMaxAccel, kMaxVoltage);
    }

}
