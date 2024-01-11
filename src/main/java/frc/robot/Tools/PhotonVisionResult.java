package frc.robot.Tools;

import edu.wpi.first.math.geometry.Pose3d;

public class PhotonVisionResult {
	private boolean _targetFound = false;
	private Pose3d _pose;
	private double _imageCaptureTime;

	public PhotonVisionResult(boolean targetFound, Pose3d pose, double imageCaptureTime) {
		_targetFound = targetFound;
		_pose = pose;
		_imageCaptureTime = imageCaptureTime;
	}

	public boolean targetFound() {
		return _targetFound;
	}

	public Pose3d pose3d() {
		return _pose;
	}

	public double imageCaptureTime() {
		return _imageCaptureTime;
	}
}
