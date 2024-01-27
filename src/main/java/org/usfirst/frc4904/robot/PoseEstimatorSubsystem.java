package org.usfirst.frc4904.robot;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import java.util.Optional;
import com.kauailabs.navx.frc.AHRS;
import java.util.Properties;

// inspired by team 5712's implementation: https://github.com/Hemlock5712/2023-Robot/blob/Joe-Test/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
// docs are here for anyone else working on this in the near future (anna): https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html
public class PoseEstimatorSubsystem {

    private final PhotonCamera placeholderCam = new PhotonCamera("placeholder"); //TODO: replace with actual camera name and add as many cameras as we need
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo); //TODO: get the resource for this
    private final Transform3d robotToPlaceholderCam = new Transform3d(new Translation3d(-1, -1, -1), new Rotation3d(-1,-1,-1)); //TODO: stores the camera position relative to the robot
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, placeholderCam, robotToPlaceholderCam);
    
    private final Rotation2d gyroAngle;
    private final SwerveDrivePoseEstimator poseEstimator;


    public PoseEstimatorSubsystem(SwerveDriveKinematics kinematics, AHRS navx, SwerveModulePosition[] positions){
        this.gyroAngle = navx.getRotation2d();
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, this.gyroAngle, positions, null, null, null); //TODO: fill in remaining values (none of them should be null)

        //TODO: get odometry data suppliers and make the standard swerve poseestimator
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        // poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }

    public void applyVision(){
        Optional <EstimatedRobotPose> optionalPose = getEstimatedRobotPose();
        if (optionalPose.isEmpty()){
            return;
        }
        EstimatedRobotPose pose = optionalPose.get();
        poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds); //is this legal? will the error maybe just go away please
    }

    
    public Optional<EstimatedRobotPose> getEstimatedRobotPose(){
        //this can contain any extra parameters and logic if we want to alter our estimator strategy
        return photonPoseEstimator.update();
    }

}
