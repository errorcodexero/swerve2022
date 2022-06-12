package org.xero1425.base.swerve.swervedrive;

import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.xero1425.base.Subsystem;
import org.xero1425.base.swerve.common.SwerveBaseSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

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

public class SDSSwerveDriveSubsystem extends SwerveBaseSubsystem {

    enum Mode {
        RawPower,
        RawSpeed,
        Chassis
    } ;

    private final SwerveModule fl_ ;
    private final SwerveModule fr_ ;
    private final SwerveModule bl_ ;
    private final SwerveModule br_ ;

    private PIDCtrl[] pid_ctrls_ ;

    private final SwerveDriveOdometry odometry_ ;
    private final SwerveDriveKinematics kinematics_ ;
    private ChassisSpeeds chassis_speed_ ;

    private Mode mode_ ;
    private double [] speeds_ ;
    private double [] powers_ ;
    private double [] angles_ ;

    public SDSSwerveDriveSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        ShuffleboardLayout lay ;
        int drive, steer, encoder ;
        double offset ;
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration() ;

        speeds_ = new double[4] ;
        powers_ = new double[4] ;
        angles_ = new double[4] ;

        pid_ctrls_ = new PIDCtrl[4] ;
        pid_ctrls_[FL] = createPIDCtrl("fl") ;
        pid_ctrls_[FR] = createPIDCtrl("fr") ;
        pid_ctrls_[BL] = createPIDCtrl("bl") ;
        pid_ctrls_[BR] = createPIDCtrl("br") ;

        if (isSettingDefined("electrical:drive-current-limit")) {
            config.setDriveCurrentLimit(getSettingsValue("electrical:drive-current-limit").getDouble()) ;
        }

        if (isSettingDefined("electrical:steer-current-limit")) {
            config.setSteerCurrentLimit(getSettingsValue("electrical:steer-current-limit").getDouble()) ;
        }

        if (isSettingDefined("electrical:nominal-voltage")) {
            config.setNominalVoltage(getSettingsValue("electrical:nominal-voltage").getDouble()) ;
        }

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        lay = shuffleboardTab.getLayout("FLModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:fl:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:fl:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:fl:encoder:canid").getInteger() ;
        offset = getSettingsValue("hw:fl:encoder:offset").getDouble() ;
        fl_ = Mk4iSwerveModuleHelper.createFalcon500(lay, config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        lay = shuffleboardTab.getLayout("FRModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:fr:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:fr:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:fr:encoder:canid").getInteger() ;
        offset = getSettingsValue("hw:fr:encoder:offset").getDouble() ;
        fr_ = Mk4iSwerveModuleHelper.createFalcon500(lay, config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        lay = shuffleboardTab.getLayout("BLModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:bl:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:bl:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:bl:encoder:canid").getInteger() ;
        offset = getSettingsValue("hw:bl:encoder:offset").getDouble() ;
        bl_ = Mk4iSwerveModuleHelper.createFalcon500(lay, config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        lay = shuffleboardTab.getLayout("BRModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:br:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:br:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:br:encoder:canid").getInteger() ;
        offset = getSettingsValue("hw:br:encoder:offset").getDouble() ;
        br_ = Mk4iSwerveModuleHelper.createFalcon500(lay, config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        kinematics_ = new SwerveDriveKinematics(new Translation2d(getWidth() / 2.0, getLength() / 2.0), new Translation2d(getWidth() / 2.0, -getLength() / 2.0), 
                        new Translation2d(-getWidth() / 2.0, getLength() / 2.0), new Translation2d(-getWidth() / 2.0, -getLength() / 2.0)) ;

        odometry_ = new SwerveDriveOdometry(kinematics_, getRotation()) ;

        mode_ = Mode.Chassis ;
        chassis_speed_ = new ChassisSpeeds(0.0, 0.0, 0.0);

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry_.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry_.getPoseMeters().getY());
    }

    private PIDCtrl createPIDCtrl(String name) throws MissingParameterException, BadParameterTypeException {
        String pidname = "subsystems:" + getName() + ":pids:" + name ;
        return new PIDCtrl(getRobot().getSettingsSupplier(), pidname, false) ;
    }

    public SwerveModuleState getModuleState(int which) {
        SwerveModuleState st = null ;

        switch(which) {
            case FL:
                st = new SwerveModuleState(fl_.getDriveVelocity(), Rotation2d.fromDegrees(fl_.getSteerAngle())) ;
                break ;

            case FR:
                st = new SwerveModuleState(fr_.getDriveVelocity(), Rotation2d.fromDegrees(fr_.getSteerAngle())) ;
                break ;
                
            case BL:
                st = new SwerveModuleState(bl_.getDriveVelocity(), Rotation2d.fromDegrees(bl_.getSteerAngle())) ;
                break ;
                
            case BR:
                st = new SwerveModuleState(br_.getDriveVelocity(), Rotation2d.fromDegrees(br_.getSteerAngle())) ;
                break ;                
        }

        return st ;
    }

    public SwerveModuleState getModuleTarget(int which) {
        SwerveModuleState st = null ;

        st = new SwerveModuleState(speeds_[which], Rotation2d.fromDegrees(angles_[which])) ;
        return st ;
    }

    public void zeroGyro() {
        odometry_.resetPosition(
                new Pose2d(odometry_.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)), getRotation()) ;
    }

    public void resetOdometry(Pose2d pose) {
        odometry_.resetPosition(pose, getHeading()) ;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(gyro().getYaw()) ;
    }

    public void drive(ChassisSpeeds speed) {
        chassis_speed_ = speed ;     
    }

    @Override
    public Pose2d getPose() {
        return odometry_.getPoseMeters() ;
    }

    @Override
    public void setRawTargets(boolean power, double [] angles, double [] speeds_powers)  {
        angles.clone() ;
        if (power) {
            mode_ = Mode.RawPower ;
            powers_ = speeds_powers.clone() ;
        }
        else {
            mode_ = Mode.RawPower ;
            speeds_ = speeds_powers.clone() ;
        }
    }

    @Override
    public void run() {
        //
        // Just in case, paranoid code.  Be sure the arrays we are intersted in are there.  SHould be set
        // in the constructor and in any setter that provides new values
        //
        if (angles_ == null || angles_.length != 4)
            angles_ = new double[4] ;

        if (speeds_ == null || speeds_.length != 4)
            speeds_ = new double[4] ;

        if (powers_ == null || powers_.length != 4)
            powers_ = new double[4] ;   

        odometry_.update(getRotation(), 
            new SwerveModuleState(fl_.getDriveVelocity(), new Rotation2d(fl_.getSteerAngle())),
            new SwerveModuleState(fr_.getDriveVelocity(), new Rotation2d(fr_.getSteerAngle())),
            new SwerveModuleState(bl_.getDriveVelocity(), new Rotation2d(bl_.getSteerAngle())),
            new SwerveModuleState(br_.getDriveVelocity(), new Rotation2d(br_.getSteerAngle())));

        if (mode_ == Mode.Chassis) {

            // Convert chassis speeds to module speeds and angles
            SwerveModuleState[] states = kinematics_.toSwerveModuleStates(chassis_speed_);
            
            angles_[FL] = states[FL].angle.getDegrees() ;
            speeds_[FL] = states[FL].speedMetersPerSecond ;
            angles_[FR] = states[FR].angle.getDegrees() ;
            speeds_[FR] = states[FR].speedMetersPerSecond ;
            angles_[BL] = states[BL].angle.getDegrees() ;
            speeds_[BL] = states[BL].speedMetersPerSecond ;
            angles_[BR] = states[BR].angle.getDegrees() ;
            speeds_[BR] = states[BR].speedMetersPerSecond ;                                    
        }

        if (mode_ == Mode.Chassis || mode_ == Mode.RawSpeed)
        {
            powers_[FL] = pid_ctrls_[FL].getOutput(speeds_[FL], getModuleState(FL).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[FR] = pid_ctrls_[FR].getOutput(speeds_[FR], getModuleState(FR).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[BL] = pid_ctrls_[BL].getOutput(speeds_[BL], getModuleState(BL).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[BR] = pid_ctrls_[BR].getOutput(speeds_[BR], getModuleState(BR).speedMetersPerSecond, getRobot().getDeltaTime()) ;
        }

        fl_.set(powers_[FL], Math.toRadians(angles_[FL])) ;
        fr_.set(powers_[FR], Math.toRadians(angles_[FR])) ;
        bl_.set(powers_[BL], Math.toRadians(angles_[BL])) ;
        br_.set(powers_[BR], Math.toRadians(angles_[BR])) ;
    }
}
