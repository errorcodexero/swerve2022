package frc.robot.automodes;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.xeroswerve.XeroSwerveDriveSubsystem;
import org.xero1425.base.xeroswerve.XeroSwerveSetMotorPowerAction;

public class SwerveMotorTestAutoMode extends SwerveDriveAutoMode {
    public SwerveMotorTestAutoMode(AutoController ctrl) throws InvalidActionRequest {
        super(ctrl, "SwerveMotorTest") ;

        XeroSwerveDriveSubsystem db = (XeroSwerveDriveSubsystem)ctrl.getRobot().getRobotSubsystem().getDB() ;


        addSubActionPair(db, new XeroSwerveSetMotorPowerAction(db, XeroSwerveDriveSubsystem.FL, 0.25, 0.0, 2.0), true) ;
        addSubActionPair(db, new XeroSwerveSetMotorPowerAction(db, XeroSwerveDriveSubsystem.FL, 0.0, 0.25, 2.0), true) ;
        addSubActionPair(db, new XeroSwerveSetMotorPowerAction(db, XeroSwerveDriveSubsystem.FR, 0.25, 0.0, 2.0), true) ;
        addSubActionPair(db, new XeroSwerveSetMotorPowerAction(db, XeroSwerveDriveSubsystem.FR, 0.0, 0.25, 2.0), true) ;
        addSubActionPair(db, new XeroSwerveSetMotorPowerAction(db, XeroSwerveDriveSubsystem.BL, 0.25, 0.0, 2.0), true) ;
        addSubActionPair(db, new XeroSwerveSetMotorPowerAction(db, XeroSwerveDriveSubsystem.BL, 0.0, 0.25, 2.0), true) ;
        addSubActionPair(db, new XeroSwerveSetMotorPowerAction(db, XeroSwerveDriveSubsystem.BR, 0.25, 0.0, 2.0), true) ;
        addSubActionPair(db, new XeroSwerveSetMotorPowerAction(db, XeroSwerveDriveSubsystem.BR, 0.0, 0.25, 2.0), true) ;                
    }
}
