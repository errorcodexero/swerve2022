package frc.robot;

import org.xero1425.base.RobotSubsystem;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.swervedrive.SwerveDriveSubsystem;

import frc.robot.oi.Swerve2021OISubsystem;

public class SwerveDriveRobotSubsystem extends RobotSubsystem {
    public SwerveDriveRobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "SwerveRobotSubsystem") ;

        SwerveDriveSubsystem db = new SwerveDriveSubsystem(this, "swervedrive", "hw:swervedrive") ;
        addChild(db) ;

        Swerve2021OISubsystem oi = new Swerve2021OISubsystem(this, db) ;
        addChild(oi) ;
    }
}
