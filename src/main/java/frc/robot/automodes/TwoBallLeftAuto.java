package frc.robot.automodes;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSetBall;
import frc.robot.subsystems.gpm.GPMFireAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.shooter.ShooterSpinUpAction;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TwoBallLeftAuto extends SwerveDriveAutoMode{

    public TwoBallLeftAuto(SwerveDriveRobotAutoController ctrl) throws Exception {
        super(ctrl, "twoballleft") ;

        Swerve2022RobotSubsystem robot = getSwerveRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;
        SwerveBaseSubsystem db = getSwerveRobotSubsystem().getDB() ;

        TargetTrackerSubsystem tracker = robot.getTracker() ;
        TurretSubsystem turret = robot.getTurret() ;

        // Set state of the conveyor to reflect a single ball preloaded
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), true);

        // Start the limelight
        startLimelightTracking() ;

        // Start spin up of the shooter
        addSubActionPair(gpm.getShooter(), new ShooterSpinUpAction(gpm.getShooter()), false) ;

        // Drive and collect the second ball
        drivePath("p1", true, true) ;

        drivePath("p2", false, false) ;
        
        // Start firing the two balls
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);
    }
}