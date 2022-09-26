package frc.robot.automodes;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSetBallCount;
import frc.robot.subsystems.gpm.GPMFireAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.shooter.ShooterSpinUpAction;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TwoBallRightAuto extends SwerveDriveAutoMode {
    public TwoBallRightAuto(SwerveDriveRobotAutoController ctrl) throws Exception {
        super(ctrl, "two-ball-right") ;

        Swerve2022RobotSubsystem robot = getRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;
        SwerveBaseSubsystem db = getRobotSubsystem().getDB() ;

        TargetTrackerSubsystem tracker = robot.getTracker() ;
        TurretSubsystem turret = robot.getTurret() ;

        // Set state of the conveyor to reflect a single ball preloaded
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBallCount(gpm.getConveyor()), false);

        // Start the limelight
        startLimelightTracking() ;

        // Start spin up of the shooter
        addSubActionPair(gpm.getShooter(), new ShooterSpinUpAction(gpm.getShooter()), false) ;

        // Drive and collect the second ball
        drivePath("p1", false) ;
        
        // Start firing the two balls
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);
    }
}
