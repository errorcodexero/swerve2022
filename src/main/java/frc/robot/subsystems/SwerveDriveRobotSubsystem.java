package frc.robot.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.limelight.LimeLightSubsystem;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.intake2motor.Intake2MotorSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.swerve.xeroswerve.XeroSwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.oi.Swerve2021OISubsystem;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SwerveDriveRobotSubsystem extends RobotSubsystem {
    private GPMSubsystem gpm_;
    private SwerveBaseSubsystem db_;

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

        LimeLightSubsystem limelight_ = new LimeLightSubsystem(this, "limelight");
        addChild(limelight_);

        TurretSubsystem turret_ = new TurretSubsystem(this);
        addChild(turret_);

        TargetTrackerSubsystem targettracker_ = new TargetTrackerSubsystem(this, limelight_, turret_);
        addChild(targettracker_);

    }

    public GPMSubsystem getGPM() {
        return gpm_;
    }

}
