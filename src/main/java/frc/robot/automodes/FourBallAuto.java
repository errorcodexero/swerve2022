package frc.robot.automodes;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSetBallCount;
import frc.robot.subsystems.gpm.GPMFireAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.shooter.ShooterSpinUpAction;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class FourBallAuto extends SwerveDriveAutoMode {
    public FourBallAuto(SwerveDriveRobotAutoController ctrl) throws Exception {
        super(ctrl, "four-ball") ;

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

        // Drive to the first ball we are going to pick up
        // This requires that the path be named four-ball-p1
        // The four-ball comes from the name of the automode
        // The p1 comes from name of the path (e.g. four-ball-p1)
        // This requires a value in the settings file of "automodes:four-ball:p1-end-angle" 
        //     to specify the end angle for the path
        // Since the second argument is true, we will collect while driving
        // This requires a value in the settings file of "automodes:four-ball:p1-collect-delay"
        //     which is the delay after the collect action is deployed before the robot starts
        //     driving.  This is needed if we get to a ball before our collector is down
        drivePath("p1", true) ;

        // Shoot the two balls we have
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);

        // Drive and get the third ball, and the fourth if the human player rolls
        // the ball in correctly
        drivePath("p2", true) ;

        // Drive back to the shooting location
        drivePath("p3", false) ;

        // Shoot the one or two balls we now have
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);
    }
}
