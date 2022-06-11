package org.xero1425.base.swervedrive;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.Subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveDriveSubsystem extends DriveBaseSubsystem {
    private final SwerveModule fl_ ;
    private final SwerveModule fr_ ;
    private final SwerveModule bl_ ;
    private final SwerveModule br_ ;

    private final SwerveDriveOdometry odometry_ ;
    private final SwerveDriveKinematics kinematics_ ;
    private ChassisSpeeds speeds_ ;

    private final double MAX_VOLTAGE ;
    public final double MAX_VELOCITY_METERS_PER_SECOND ;
    public final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND ;

    public SwerveDriveSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        ShuffleboardLayout lay ;
        int drive, steer, encoder ;
        double offset ;
        double w, l ;

        w = getSettingsValue("physical:width").getDouble() ;
        l = getSettingsValue("physical:length").getDouble() ;

        MAX_VOLTAGE = 12.0;
        MAX_VELOCITY_METERS_PER_SECOND = getSettingsValue("physical:maxspeed").getDouble() ;
        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(w / 2.0, l / 2.0);

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        lay = shuffleboardTab.getLayout("FLModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:fl:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:fl:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:fl:encoder:canid").getInteger() ;
        offset = getSettingsValue("hw:fl:encoder:offset").getDouble() ;
        fl_ = Mk4iSwerveModuleHelper.createFalcon500(lay, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        lay = shuffleboardTab.getLayout("FRModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:fr:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:fr:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:fr:encoder:canid").getInteger() ;
        offset = getSettingsValue("hw:fr:encoder:offset").getDouble() ;
        fr_ = Mk4iSwerveModuleHelper.createFalcon500(lay, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        lay = shuffleboardTab.getLayout("BLModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:bl:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:bl:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:bl:encoder:canid").getInteger() ;
        offset = getSettingsValue("hw:bl:encoder:offset").getDouble() ;
        bl_ = Mk4iSwerveModuleHelper.createFalcon500(lay, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        lay = shuffleboardTab.getLayout("BRModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:br:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:br:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:br:encoder:canid").getInteger() ;
        offset = getSettingsValue("hw:br:encoder:offset").getDouble() ;
        br_ = Mk4iSwerveModuleHelper.createFalcon500(lay, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        kinematics_ = new SwerveDriveKinematics(new Translation2d(w / 2.0, l / 2.0), new Translation2d(w / 2.0, -l / 2.0), 
                        new Translation2d(-w / 2.0, l / 2.0), new Translation2d(-w / 2.0, -l / 2.0)) ;

        odometry_ = new SwerveDriveOdometry(kinematics_, getRotation()) ;

        speeds_ = new ChassisSpeeds(0.0, 0.0, 0.0);

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry_.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry_.getPoseMeters().getY());
    }

    public void zeroGyro() {
        odometry_.resetPosition(
                new Pose2d(odometry_.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)), getRotation()) ;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(gyro().getYaw()) ;
    }

    public void drive(ChassisSpeeds speed) {
        speeds_ = speed ;
    }

    @Override
    public void run() {
        odometry_.update(getRotation(), 
            new SwerveModuleState(fl_.getDriveVelocity(), new Rotation2d(fl_.getSteerAngle())),
            new SwerveModuleState(fr_.getDriveVelocity(), new Rotation2d(fr_.getSteerAngle())),
            new SwerveModuleState(bl_.getDriveVelocity(), new Rotation2d(bl_.getSteerAngle())),
            new SwerveModuleState(br_.getDriveVelocity(), new Rotation2d(br_.getSteerAngle())));

        SwerveModuleState[] states = kinematics_.toSwerveModuleStates(speeds_);

        fl_.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        fr_.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        bl_.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        br_.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}
