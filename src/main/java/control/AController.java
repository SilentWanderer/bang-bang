package control;

import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.physics.ChassisState;
import lib.physics.DifferentialDrive;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.TimedState;

public abstract class AController {

    protected DifferentialDrive mDriveModel;

    protected TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    protected TimedState<Pose2dWithCurvature> mSetpoint;
    protected Pose2d mError;


    public AController(DifferentialDrive pDriveModel) {
        mDriveModel = pDriveModel;
    }

    public abstract DriveOutput update(DifferentialDrive.DriveDynamics pDynamics, ChassisState pPrevVelocity, Pose2d pCurrentState, double pDt);

}
