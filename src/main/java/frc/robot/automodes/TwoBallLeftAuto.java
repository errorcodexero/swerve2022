package frc.robot.automodes;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSetBall;
import frc.robot.subsystems.gpm.GPMFireAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.shooter.SetShooterAction;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TwoBallLeftAuto extends SwerveDriveAutoMode{

    private final double FirstFireAngle = 0.0 ;
    private final double ShooterWheelsSpinupSpeed = 4000.0 ;
    private final double FirstFireHood = 6.0 ;

    public TwoBallLeftAuto(SwerveDriveRobotAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        Swerve2022RobotSubsystem swerve = (Swerve2022RobotSubsystem)ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = swerve.getGPM() ;
        SwerveBaseSubsystem db = swerve.getDB() ;
        TargetTrackerSubsystem tracker = swerve.getTracker() ;
        TurretSubsystem turret = swerve.getTurret() ;
        ShooterSubsystem shooter = gpm.getShooter() ;
        
        // Set state of the conveyor to reflect a single ball preloaded
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);

        // Start the shooter wheels so they are ready to fire
        var shoot = new SetShooterAction(shooter, ShooterWheelsSpinupSpeed, FirstFireHood) ;

        // Drive and collect the second ball
        driveAndCollect("twoball_p1", 1.0, 0.0, FirstFireAngle, shoot, 1);

        // Start the limelight
        startLimelightTracking();

        // Start firing the two balls
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);
    }
}