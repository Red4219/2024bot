package frc.robot.Tools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.SimVisionSystem;
//import org.photonvision.SimVisionTarget;
import org.photonvision.simulation.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PhotonVisionConstants;
import edu.wpi.first.cscore.VideoMode;

//import edu.wpi.first.cscore.VideoMode.PixelFormat;

public class PhotonVision {
	private PhotonCamera _camera = new PhotonCamera(PhotonVisionConstants.CameraName);
	//private PhotonCamera _camera = null;
	
	// simulation variables
	//private SimVisionSystem _simVisionSystem;
	private VisionSystemSim _visionSystemSim;
	private Pose3d _sim_farTargetPose;
	private double _sim_targetWidth;
	private double _sim_targetHeight;
	private AprilTagFieldLayout _aprilTagFieldLayout;
	private PhotonPoseEstimator _photonPoseEstimator;

	Pose3d camPose = new Pose3d();
	private Pose2d _lastPhotonPoseEstimatorPose = new Pose2d();
	
	public PhotonVision() {

		try {
			_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

			// Set if we are blue or red

			if(DriverStation.getAlliance().isPresent()) {
			
				if(DriverStation.getAlliance().get() == Alliance.Blue) {
					_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
				} else {
					_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
				}
			} else {
				_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
			}
		} catch (IOException e) {
			System.out.println("PhotonVision::PhotonVision() - error:" + e.toString());
			return;
		}

		// Change this for testing
		if(Constants.getMode() == Mode.SIM && !PhotonVisionConstants.PhysicalCamera) {
			System.out.println("running setupSimulation()");
			setupSimulation(new Pose3d());
		} 

		if(_aprilTagFieldLayout != null) {
			if(_camera != null) {
				if(_camera.isConnected()) {
					_photonPoseEstimator = new PhotonPoseEstimator(
						_aprilTagFieldLayout, 
						Constants.PhotonVisionConstants.poseStrategy,
						_camera, 
						Constants.PhotonVisionConstants.cameraToRobot
						);
				} else {
					System.out.println("-------> the camera is not connected");
				}
			} else {
				System.out.println("PhotonVision::PhotonVision() - _camera is null");
			}
		}
	}

	public boolean isConnected() {
		if(_camera != null) {
			return _camera.isConnected();
		} else {
			return false;
		}
	}

	public PhotonVisionResult getPose(Pose2d prevEstimatedRobotPose) {

		try {
			
			// Change this for testing
			if(Constants.getMode() == Mode.SIM && !PhotonVisionConstants.PhysicalCamera) {
				// Update PhotonVision based on our new robot position.
				//_simVisionSystem.processFrame(prevEstimatedRobotPose);
				_visionSystemSim.update(prevEstimatedRobotPose);
			}

			Optional<EstimatedRobotPose> o = getPhotonPose(prevEstimatedRobotPose);

			if(o.isPresent()) {
				EstimatedRobotPose estimatedRobotPose = o.get();

				List<Pose3d> allTagPoses = new ArrayList<>();

				for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
					//if (target.getFiducialId() != -1) {
						allTagPoses.add(
							_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
						);
					//}
				}

				Logger.recordOutput(
					"AprilTagVision/TagPoses",
					allTagPoses.toArray(new Pose3d[allTagPoses.size()])
				);

				return new PhotonVisionResult(true, estimatedRobotPose.estimatedPose, estimatedRobotPose.timestampSeconds);
				
			} else {
				// Since we do not have any tags that we can see, blank out the list
				List<Pose3d> allTagPoses = new ArrayList<>();
				Logger.recordOutput(
					"AprilTagVision/TagPoses",
					allTagPoses.toArray(new Pose3d[allTagPoses.size()])
				);
			}

			// Return this if we do not have a value
			return new PhotonVisionResult(false, camPose, 0);
		
			/*PhotonPipelineResult result = _camera.getLatestResult();
			if (result.hasTargets()) {

				List<Pose3d> allTagPoses = new ArrayList<>();

				double imageCaptureTime = result.getTimestampSeconds();
				Transform3d camToTargetTrans = result.getBestTarget().getBestCameraToTarget();
				// Pose3d pose = _sim_farTargetPose.transformBy(camToTargetTrans.inverse());

				// result.getBestTarget().getFiducialId()
				PhotonTrackedTarget t = result.getBestTarget();
				// Pose3d farTargetPose =
				// allTagPoses.get(result.getBestTarget().getFiducialId());

				for (PhotonTrackedTarget target : result.targets) {
					if (target.getFiducialId() != -1) {
						allTagPoses.add(
							_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
						);
					}
				}

				Logger.getInstance().recordOutput(
					"AprilTagVision/TagPoses",
					allTagPoses.toArray(new Pose3d[allTagPoses.size()])
				);

				Pose3d farTargetPose = _aprilTagFieldLayout.getTagPose(t.getFiducialId()).get();

				//camPose = farTargetPose.transformBy(camToTargetTrans.inverse()).transformBy(Constants.PhotonVisionConstants.cameraToRobot);
				camPose = farTargetPose.transformBy(camToTargetTrans.inverse());

				

				//Transform3d CAMERA_TO_ROBOT = new Transform3d();
				//var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);

				//return new PhotonVisionResult(true, camPose, imageCaptureTime);

				EstimatedRobotPose estimateRobotPose = getPhotonPose(prevEstimatedRobotPose).get();

				return new PhotonVisionResult(true, estimateRobotPose.estimatedPose, estimateRobotPose.timestampSeconds);
			} else {
				// Since we do not have any tags that we can see, blank out the list
				List<Pose3d> allTagPoses = new ArrayList<>();
				Logger.getInstance().recordOutput(
					"AprilTagVision/TagPoses",
					allTagPoses.toArray(new Pose3d[allTagPoses.size()])
				);
			}

			return new PhotonVisionResult(false, camPose, 0);*/
		} catch (Exception e) {
			System.out.println("PhotonVision::getPose() - " + e.toString());
			e.printStackTrace();
			return new PhotonVisionResult(false, new Pose3d(), 0);
		}
	}

	public void setReferencePose(Pose2d referencePose) {
		
		if(_photonPoseEstimator != null) {
			_photonPoseEstimator.setReferencePose(referencePose);
		}

		//tempReferencePose = referencePose;
	}

	public void setReferencePose(Pose3d referencePose) {
		if(_photonPoseEstimator != null) {
			_photonPoseEstimator.setReferencePose(referencePose);
		}
	}

	public Optional<EstimatedRobotPose> getPhotonPose(Pose2d prevEstimatedRobotPose) {

		if(_photonPoseEstimator != null) {

			// Check if we are in simulation and the previousEstimatedRobotPose is not null
			// and we are not connected to the camera
			// Change this for testing
			if(
				Constants.getMode() == Mode.SIM 
				&& prevEstimatedRobotPose != null
				&& !PhotonVisionConstants.PhysicalCamera) {
				// Update PhotonVision based on our new robot position.
				//_simVisionSystem.processFrame(prevEstimatedRobotPose);
				_visionSystemSim.update(prevEstimatedRobotPose);
			}

			if(prevEstimatedRobotPose != null) {
				_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
			} else {
				System.out.println("PhotonVision::getPhotonPose() - prevEstimatedRobotPose is null");
			}

			Optional<EstimatedRobotPose> estimatedRobotPose = _photonPoseEstimator.update();

			List<Pose3d> allTagPoses = new ArrayList<>();

			if (estimatedRobotPose.isPresent()) {
				_lastPhotonPoseEstimatorPose = estimatedRobotPose.get().estimatedPose.toPose2d();

				try {

					for (PhotonTrackedTarget target : estimatedRobotPose.get().targetsUsed) {
						if (target.getFiducialId() >= 1 && target.getFiducialId() <= 8) {
							allTagPoses.add(_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());
						}
					}

					Logger.recordOutput(
							"AprilTagVision/TargetsUsed",
							allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

					Logger.recordOutput(
							"PhotonVisionEstimator/Robot",
							estimatedRobotPose.get().estimatedPose.toPose2d());
				} catch (Exception e) {
					System.out.println(e.toString());
				}
			} else {
				Logger.recordOutput(
							"AprilTagVision/TargetsUsed",
							allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

				Logger.recordOutput(
							"PhotonVisionEstimator/Robot",
							_lastPhotonPoseEstimatorPose);
				return Optional.empty();
			}

			return estimatedRobotPose;			
		}

		return Optional.empty();
	}

	// setup the tags and set the origin to how to show the tags
	public void setupAprilTagFieldLayoutSim() {
		//_simVisionSystem.clearVisionTargets();
		_visionSystemSim.clearVisionTargets();

		if(DriverStation.getAlliance().isPresent()) {
			if(DriverStation.getAlliance().get() == Alliance.Blue) {
				_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
			} else {
				_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
			}
		} else {
			_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
		}

		if(_aprilTagFieldLayout != null) {
			_visionSystemSim.addAprilTags(_aprilTagFieldLayout);

			/*_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(1).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				1)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(2).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				2)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(3).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				3)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(4).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				4)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(5).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				5)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(6).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				6)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(7).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				7)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(8).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				8)
			);*/
		}
	}

	private void setupSimulation(Pose3d aprilTagFieldLayoutOrigin) {

		// Simulated Vision System.
    	// Configure these to match your PhotonVision Camera,
    	// pipeline, and LED setup.

		/*_simVisionSystem =
            new SimVisionSystem(
                PhotonVisionConstants.CameraName,
                PhotonVisionConstants.sim_camDiagFOV,
                
				Constants.PhotonVisionConstants.cameraToRobot,
                PhotonVisionConstants.sim_maxLEDRange,
                PhotonVisionConstants.sim_camResolutionWidth,
                PhotonVisionConstants.sim_camResolutionHeight,
                PhotonVisionConstants.sim_minTargetArea
			);*/

		_visionSystemSim = new VisionSystemSim("main");
		/*SimCameraProperties cameraProp = new SimCameraProperties();
		// A 640 x 480 camera with a 100 degree diagonal FOV.
		cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
		// Approximate detection noise with average and standard deviation error in pixels.
		cameraProp.setCalibError(0.25, 0.08);
		// Set the camera image capture framerate (Note: this is limited by robot loop rate).
		cameraProp.setFPS(20);
		// The average and standard deviation in milliseconds of image data latency.
		cameraProp.setAvgLatencyMs(35);
		cameraProp.setLatencyStdDevMs(5);*/
		
		//PhotonCameraSim cameraSim = new PhotonCameraSim(_camera, cameraProp);
		//_visionSystemSim.addCamera(cameraSim, Constants.PhotonVisionConstants.cameraToRobot);
		_visionSystemSim.addCamera(new PhotonCameraSim(_camera), Constants.PhotonVisionConstants.cameraToRobot);
			
		
		// See
    	// https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    	// page 208
    	_sim_targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    	// See
    	// https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    	// page 197
    	_sim_targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    	// See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    	// pages 4 and 5
    	//double tgtXPos = Units.feetToMeters(54);
		double tgtXPos = 5;
    	//double tgtYPos =
            //Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
		double tgtYPos = 5;
    	_sim_farTargetPose =
            new Pose3d(
                    new Translation3d(tgtXPos, tgtYPos, 0.5),
                    new Rotation3d(0.0, 0.0, 0.0));
		
		/*_simVisionSystem.addSimVisionTarget(
			new SimVisionTarget(_sim_farTargetPose, _sim_targetWidth, _sim_targetHeight, -1));*/
		
		
		setupAprilTagFieldLayoutSim();
	}
}
