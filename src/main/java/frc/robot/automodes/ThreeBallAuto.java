package frc.robot.automodes;


import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSetBall;
import frc.robot.subsystems.gpm.GPMFireAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.shooter.SetShooterAction;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.tankdrive.TankDriveSubsystem;

public class ThreeBallAuto extends SwerveDriveAutoMode {

    private final double FirstFireAngle = 30.0 ;
    private final double SecondFireAngle = 30.0 ;
    private final double FirstFireHood = 20.0 ;
    private final double ShooterWheelsSpinupSpeed = 6000.0 ;

    private final double Endangle1 = 0;
    private final double Endangle2 = 0;
    private final double Endangle3 = 0;


    public ThreeBallAuto(SwerveDriveRobotAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        Swerve2022RobotSubsystem swerve = (Swerve2022RobotSubsystem) ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = swerve.getGPM() ;
        TargetTrackerSubsystem tracker = swerve.getTracker() ;
        SwerveBaseSubsystem db = swerve.getDB() ;
        TurretSubsystem turret = swerve.getTurret() ;
        ShooterSubsystem shooter = gpm.getShooter();

        // Set state of the conveyor to reflect a single ball preloaded
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);

        // Start the shooter wheels so they are ready to fire
        var shoot = new SetShooterAction(shooter, ShooterWheelsSpinupSpeed, FirstFireHood) ;

        // drive and collect the second ball
        driveAndCollect("threeball_p1", 0.5, 0.0, FirstFireAngle, shoot, Endangle1) ;

        // Start the limelight
        startLimelightTracking() ;

        // Start firing the two balls
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);

        // Drive and collect the ball near the terminal
        driveAndCollect("threeball_p2", 0.0, 0.5, 0.0, shoot, Endangle2);

        // Start the shooter wheels so they are ready to fire
        addSubActionPair(shooter, new SetShooterAction(shooter, ShooterWheelsSpinupSpeed, FirstFireHood), false) ;

        // Drive back to the target and fire the ball
        driveAndFire("threeball_p3", SecondFireAngle, Endangle3) ;
    }
}