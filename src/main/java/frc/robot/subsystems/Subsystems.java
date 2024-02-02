// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class Subsystems {
    public SwerveSubsystem drivetrain = new SwerveSubsystem();
    public AprilTagSubsystem aprilTag = new AprilTagSubsystem();
    public IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    public AddressableLEDSubsystem addressableLEDSubsystem = new AddressableLEDSubsystem();
    //public IntakeSubsystem intake = new IntakeSubsystem();

    public void periodic(){
        var visionEst = aprilTag.getEstimateGlobalPose();
        visionEst.ifPresent((est) ->{
            var estPose = est.estimatedPose.toPose2d();
            var estStdDevs = aprilTag.getEstimationStdDevs(estPose);
            
            drivetrain.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
        });
    }
}
