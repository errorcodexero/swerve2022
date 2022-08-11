package frc.robot.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.intake2motor.Intake2MotorSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.swerve.xeroswerve.XeroSwerveDriveSubsystem;

import frc.robot.oi.Swerve2021OISubsystem;
import frc.robot.subsystems.gpm.GPMSubsystem;

public class SwerveDriveRobotSubsystem extends RobotSubsystem {
    private GPMSubsystem gpm_;

    public SwerveDriveRobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "SwerveRobotSubsystem") ;

        SwerveBaseSubsystem db = null ;
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

        gpm_ = new GPMSubsystem(this, "gpm") ;
        addChild(gpm_) ;
    }

    public GPMSubsystem getGPM() {
        return gpm_;
    }
}
