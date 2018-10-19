package ui;

import lib.geometry.Pose2d;

public interface ISimulationListener {

    void update(double pTimeStamp, Pose2d pCurrentPose);

}
