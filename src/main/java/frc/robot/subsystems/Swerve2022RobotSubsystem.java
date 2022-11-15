package frc.robot.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.oi.Swerve2022OISubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class Swerve2022RobotSubsystem extends RobotSubsystem {
    private GPMSubsystem gpm_;
    private SwerveBaseSubsystem db_;
    private TurretSubsystem turret_ ;
    private TargetTrackerSubsystem tracker_ ;
    private LimeLightSubsystem limelight_ ;
    private ClimberSubsystem climber_ ;

    public Swerve2022RobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "SwerveRobotSubsystem") ;

        db_ = new SDSSwerveDriveSubsystem(this, "swervedrive") ;
        addChild(db_) ;

        Swerve2022OISubsystem oi = new Swerve2022OISubsystem(this, db_) ;
        addChild(oi) ;

        gpm_ = new GPMSubsystem(this, "gpm") ;
        addChild(gpm_) ;

        limelight_ = new LimeLightSubsystem(this, "limelight");
        addChild(limelight_);

        turret_ = new TurretSubsystem(this);
        addChild(turret_);

        tracker_ = new TargetTrackerSubsystem(this, limelight_, turret_);
        addChild(tracker_);

        climber_ = new ClimberSubsystem(this) ;
        addChild(climber_);
    }

    public ClimberSubsystem getClimber() {
        return climber_ ;
    }

    public GPMSubsystem getGPM() {
        return gpm_;
    }

    public SwerveBaseSubsystem getDB() {
        return db_ ;
    }

    public TurretSubsystem getTurret() {
        return turret_; 
    }

    public TargetTrackerSubsystem getTracker() {
        return tracker_ ;
    }
}
