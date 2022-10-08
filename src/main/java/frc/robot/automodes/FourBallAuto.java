package frc.robot.automodes;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.tankdrive.TankDriveSubsystem;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSetBall;
import frc.robot.subsystems.gpm.GPMFireAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class FourBallAuto extends SwerveDriveAutoMode{
    private final double FirstShotAngle = 43.0 ;
    private final double Endangle1 = 0;
    private final double Endangle2 = 0;
    private final double Endangle3 = 0;

    public FourBallAuto(SwerveDriveRobotAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        Swerve2022RobotSubsystem swerve = (Swerve2022RobotSubsystem)ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = swerve.getGPM() ;
        TargetTrackerSubsystem tracker = swerve.getTracker() ;
        SwerveBaseSubsystem db = swerve.getDB() ;
        TurretSubsystem turret = swerve.getTurret() ;

        // Set state of the conveyor to reflect a single ball preloaded
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);

        // Start the limelight
        startLimelightTracking() ;

        // Start firing the two balls
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);

        // Drive and pick up two balls from the field
        driveAndCollect("fourball_p1", 0.0, 0.3, FirstShotAngle, null, Endangle1);

        // Fire the two we collected in this first path
        startLimelightTracking() ;
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);

        // Drive out and collect the balls at the terminal
        driveAndCollect("fourball_p2", 0.0, 0.3, 0.0, null, Endangle2);

        // Drive back and fire while driving back
        driveAndFire("fourball_p3", 0.0, Endangle3) ;
    }
}