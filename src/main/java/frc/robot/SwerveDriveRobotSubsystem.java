package frc.robot;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.RobotSubsystem;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.swerve.swervedrive.SDSSwerveDriveSubsystem;
import org.xero1425.base.swerve.xeroswerve.XeroSwerveDriveSubsystem;

import frc.robot.oi.Swerve2021OISubsystem;

public class SwerveDriveRobotSubsystem extends RobotSubsystem {
    public SwerveDriveRobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "SwerveRobotSubsystem") ;

        DriveBaseSubsystem db ;
        boolean usehwpid = true ;
        boolean usesds = true ;

        if (XeroRobot.isSimulation())
            usehwpid = false ;


        if (usesds) {
            db = new SDSSwerveDriveSubsystem(this, "swervedrive") ;
        }
        else {
            db = new XeroSwerveDriveSubsystem(this, "xeroswervedrive", usehwpid) ;
        }
        addChild(db) ;

        Swerve2021OISubsystem oi = new Swerve2021OISubsystem(this, db) ;
        addChild(oi) ;
    }
}
