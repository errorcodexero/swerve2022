package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;

import frc.robot.subsystems.Swerve2022RobotSubsystem;
import frc.robot.subsystems.gpm.GPMStartCollectAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.turret.TurretFollowTargetAction;

public abstract class SwerveDriveAutoMode extends AutoMode {
    SwerveDriveAutoMode(AutoController ctrl, String name) {
        super(ctrl, name) ;
    }

    protected Swerve2022RobotSubsystem getSwerveRobotSubsystem() {
        return (Swerve2022RobotSubsystem) getAutoController().getRobot().getRobotSubsystem();
    }

    protected void startLimelightTracking() throws BadParameterTypeException, MissingParameterException, InvalidActionRequest {
        Swerve2022RobotSubsystem swerve = getSwerveRobotSubsystem() ;
        TurretFollowTargetAction act = new TurretFollowTargetAction(swerve.getTurret(), swerve.getTracker()) ;
        addSubActionPair(swerve.getTurret(), act, false);
    }

    // protected void driveAndFire(String path, double angle, double endangle) throws Exception {
        
    //     Swerve2022RobotSubsystem swerve = getSwerveRobotSubsystem() ;
    //     GPMSubsystem gpm = swerve.getGPM();
    //     TurretSubsystem turret = swerve.getTurret() ;
    //     ParallelAction parallel;

    //     parallel = new ParallelAction(getAutoController().getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);
    //     parallel.addSubActionPair(turret, new MotorEncoderGotoAction(turret, angle, true), false) ;
    //     parallel.addSubActionPair(swerve.getDB(), new SwerveHolonomicPathFollower(swerve.getDB(), path, endangle), true);
    //     addAction(parallel);
        
    //     startLimelightTracking();
    //     addSubActionPair(gpm, new GPMFireAction(gpm, swerve.getTracker(), swerve.getDB(), swerve.getTurret()), true) ;
    // }

    // protected void driveAndCollect(String path, double delay1, double delay2, double angle, SetShooterAction act, double endangle) throws Exception {
    //     GPMSubsystem gpm = getSwerveRobotSubsystem().getGPM();
    //     SwerveBaseSubsystem db = getSwerveRobotSubsystem().getDB();
    //     TurretSubsystem turret = getSwerveRobotSubsystem().getTurret() ;
    //     ParallelAction parallel;
    //     SequenceAction drive;

    //     parallel = new ParallelAction(getAutoController().getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);

    //     //
    //     // The drive series that delays, drives, then delays.  This series should controle the life time of the
    //     // parallel.
    //     //

    //     drive = new SequenceAction(getAutoController().getRobot().getMessageLogger());

    //     if (Math.abs(delay1) > 0.05) {
    //         drive.addAction(new DelayAction(getAutoController().getRobot(), delay1));
    //     }
    //     drive.addSubActionPair(db, new SwerveHolonomicPathFollower(db, path, endangle), true);
    //     if (Math.abs(delay2) > 0.05) {
    //         drive.addAction(new DelayAction(getAutoController().getRobot(), delay2));
    //     } 
    //     parallel.addAction(drive);

    //     //
    //     // The turret positioning so we are sure the turret is pointed at the lime
    //     // light when we are done.
    //     //
    //     parallel.addSubActionPair(turret, new MotorEncoderGotoAction(turret, angle, true), true);

    //     //
    //     // The collect sequence
    //     //
    //     GPMStartCollectAction collect = new GPMStartCollectAction(gpm) ;
    //     parallel.addSubActionPair(gpm, collect, false) ;

    //     //
    //     // Now add the parallel to the action to the automode that does the drive and collect with
    //     // turret alignment at the end of the drive
    //     //
    //     addAction(parallel);

    //     //
    //     // When the path and delay is done, stop collecting
    //     //
    //     GPMStopCollectAction stop = new GPMStopCollectAction(gpm) ;
    //     addSubActionPair(gpm, stop, false);
    // }

    protected SettingsValue getSetting(String name) throws MissingParameterException {
        ISettingsSupplier settings = getAutoController().getRobot().getSettingsSupplier() ;
        String settingname = "automodes:" + getName() + ":" + name ;
        return settings.get(settingname) ;
    }

    protected void drivePath(String name, boolean collect) throws Exception {
        SwerveBaseSubsystem db = getSwerveRobotSubsystem().getDB() ;
        if (collect) {
            GPMSubsystem gpm = getSwerveRobotSubsystem().getGPM();
            addSubActionPair(gpm, new GPMStartCollectAction(gpm), false);

            double delay = getSetting(name + ":collect-delay").getDouble() ;
            if (delay > 0.01) {
                addAction(new DelayAction(getAutoController().getRobot(), delay));
            }
        }
 
        double angle = getSetting(name + ":end-angle").getDouble() ;
        addSubActionPair(db, new SwerveHolonomicPathFollower(db, getName() + "_" + name, angle), true) ;
    }
}
