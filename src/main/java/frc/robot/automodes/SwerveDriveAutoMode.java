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

    protected Swerve2022RobotSubsystem getRobotSubsystem() {
        return (Swerve2022RobotSubsystem) getAutoController().getRobot().getRobotSubsystem();
    }

    protected void startLimelightTracking() throws BadParameterTypeException, MissingParameterException, InvalidActionRequest {
        Swerve2022RobotSubsystem robot = getRobotSubsystem() ;
        TurretFollowTargetAction act = new TurretFollowTargetAction(robot.getTurret(), robot.getTracker()) ;
        addSubActionPair(robot.getTurret(), act, false);
    }

    protected SettingsValue getSetting(String name) throws MissingParameterException {
        ISettingsSupplier settings = getAutoController().getRobot().getSettingsSupplier() ;
        String settingname = "automodes:" + getName() + ":" + name ;
        return settings.get(settingname) ;
    }

    protected void drivePath(String name, boolean collect) throws Exception {
        SwerveBaseSubsystem db = getRobotSubsystem().getDB() ;
        if (collect) {
            GPMSubsystem gpm = getRobotSubsystem().getGPM();
            addSubActionPair(gpm, new GPMStartCollectAction(gpm), false);

            double delay = getSetting("-collect-delay").getDouble() ;
            if (delay > 0.01) {
                addAction(new DelayAction(getAutoController().getRobot(), delay));
            }
        }
 
        double angle = getSetting(name + "-end-angle").getDouble() ;
        addSubActionPair(db, new SwerveHolonomicPathFollower(db, getName() + "-p1", angle), true) ;
    }
}
