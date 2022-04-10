package frc.robot.automodes;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.swervedrive.SwerveDriveSubsystem;
import org.xero1425.base.swervedrive.SwerveSetMotorPowerAction;

public class SwerveMotorTestAutoMode extends SwerveDriveAutoMode {
    public SwerveMotorTestAutoMode(AutoController ctrl) throws InvalidActionRequest {
        super(ctrl, "SwerveMotorTest") ;

        SwerveDriveSubsystem db = (SwerveDriveSubsystem)ctrl.getRobot().getRobotSubsystem().getDB() ;


        addSubActionPair(db, new SwerveSetMotorPowerAction(db, SwerveDriveSubsystem.FL, 0.25, 0.0, 2.0), true) ;
        addSubActionPair(db, new SwerveSetMotorPowerAction(db, SwerveDriveSubsystem.FL, 0.0, 0.25, 2.0), true) ;
        addSubActionPair(db, new SwerveSetMotorPowerAction(db, SwerveDriveSubsystem.FR, 0.25, 0.0, 2.0), true) ;
        addSubActionPair(db, new SwerveSetMotorPowerAction(db, SwerveDriveSubsystem.FR, 0.0, 0.25, 2.0), true) ;
        addSubActionPair(db, new SwerveSetMotorPowerAction(db, SwerveDriveSubsystem.BL, 0.25, 0.0, 2.0), true) ;
        addSubActionPair(db, new SwerveSetMotorPowerAction(db, SwerveDriveSubsystem.BL, 0.0, 0.25, 2.0), true) ;
        addSubActionPair(db, new SwerveSetMotorPowerAction(db, SwerveDriveSubsystem.BR, 0.25, 0.0, 2.0), true) ;
        addSubActionPair(db, new SwerveSetMotorPowerAction(db, SwerveDriveSubsystem.BR, 0.0, 0.25, 2.0), true) ;                
    }
}
