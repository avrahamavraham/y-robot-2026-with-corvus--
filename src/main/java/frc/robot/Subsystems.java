// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LinearArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/** Add your docs here. */
public class Subsystems {
    //subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LinearArm linear_arm = new LinearArm();
    public final Shooter shooter = new Shooter();
    public final IntakeRoller intakeRoller = new IntakeRoller();
    public final Indexer indexer = new Indexer();
    public final Kicker kicker = new Kicker();
    public final Vision vision;

    public Subsystems(){
        if (Robot.isReal()) {
            vision = new Vision(drivetrain, 
            new VisionIOLimelight("limelight-gg", ()->drivetrain.getState().Pose.getRotation(), 0.2));
        } else {
            vision = new Vision(drivetrain,
            new VisionIOPhotonVisionSim("limelight-gg", 
                new Transform3d(-0.1808, -0.023073, 0.44788, new Rotation3d(0,0,Math.PI)), 
                ()->drivetrain.getState().Pose, 0.2)            
            );
        }
    }
}
