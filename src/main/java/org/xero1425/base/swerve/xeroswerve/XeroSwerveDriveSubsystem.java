package org.xero1425.base.swerve.xeroswerve;

import org.xero1425.base.LoopType;
import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.swerve.common.SwerveBaseSubsystem;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.Speedometer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/// \brief The swerve drive subsystem for driving a robot using a drive base where each wheel can be independently steered.
public class XeroSwerveDriveSubsystem extends SwerveBaseSubsystem {
    //
    // This object stores the names of the swerve modules
    //
    public class Names {
        public Names(String sname, String lname) {
            ShortName = sname ;
            LongName = lname ;
        }

        public String ShortName ;
        public String LongName ;
    } ;

    private double width_;                                                                      // The width of the robot, from the settings file
    private double length_;                                                                     // The length of the robot, from the settings file
    private XeroSwerveModule[] pairs_;                                                          // The serve modules, created by this class
    private Speedometer angular_;                                                               // The angular position, velocity, and acceleration of the robot


    static private final Names[] names_ = new Names[4] ;                                        // The names of each module (short name and long name)

    static private final String AngularSamplesName = "samples:angular";              // The settings file entry for the angular speedometer

    private SwerveDriveKinematics kinematics_ ;
    private SwerveDriveOdometry odometry_ ;

    /// \brief create the serve drive subsystem
    /// \param parent the parent subsystem
    /// \param name the name of the subsystem
    /// \param config the prefix for configuration entries in the settings file
    public XeroSwerveDriveSubsystem(Subsystem parent, String name, boolean hwpid) throws Exception {
        super(parent, name);

        String config = "subsystems:" + name  ;

        // Set the module names (short and long)
        names_[FL] = new Names("fl",  "FrontLeft") ;
        names_[FR] = new Names("fr", "FrontRight") ;
        names_[BL] = new Names("bl", "BackLeft") ;
        names_[BR] = new Names("br", "BackRight") ;

        //
        // Get the parameters from the settings file
        //
        ISettingsSupplier settings = getRobot().getSettingsSupplier();
        width_ = getSettingsValue("physical:width").getDouble() ;
        length_ = getSettingsValue("physical:length").getDouble();

        //
        // Create the swerve modules
        //
        pairs_ = new XeroSwerveModule[getModuleCount()];
        for (int i = 0; i < getModuleCount(); i++) {
            pairs_[i] = new XeroSwerveModule(getRobot(), this, names_[i].LongName, config, names_[i].ShortName, hwpid);
        }

        //
        // Create the angular speedometer to track angular position, velocity, and acceleration
        //
        int angularsamples = 2 ;
        if (settings.isDefined(AngularSamplesName) && settings.get(AngularSamplesName).isInteger()) {
            angularsamples = settings.get(AngularSamplesName).getInteger();
        }
        angular_ = new Speedometer("angles", angularsamples, true);

        //
        // Reset the GYRO to zero degrees
        //
        gyro().reset() ;

        Translation2d fl = new Translation2d(width_ / 2.0, length_ / 2.0) ;
        Translation2d fr = new Translation2d(width_ / 2.0, -length_ / 2.0) ;
        Translation2d bl = new Translation2d(-width_ / 2.0, length_ / 2.0) ;
        Translation2d br = new Translation2d(-width_ / 2.0, -length_ / 2.0) ;

        kinematics_ = new SwerveDriveKinematics(fl, fr, bl, br) ;
        odometry_ = new SwerveDriveOdometry(kinematics_, Rotation2d.fromDegrees(gyro().getAngle())) ;
    }

    public void zeroGyro() {
        odometry_.resetPosition(
                new Pose2d(odometry_.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)), getHeading()) ;
    }

    public void resetOdometry(Pose2d pose) {
        odometry_.resetPosition(pose, getHeading()) ;
    }

    public Pose2d getPose() {
        return odometry_.getPoseMeters() ;
    }

    public int getModuleCount() {
        return names_.length ;
    }

    public void setRawTargets(boolean power, double[] angles, double [] speed_powers) {
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics_.toSwerveModuleStates(speeds);

        pairs_[FL].setTargets(states[FL].angle.getDegrees(), states[FL].speedMetersPerSecond);
        pairs_[FR].setTargets(states[FR].angle.getDegrees(), states[FR].speedMetersPerSecond);
        pairs_[BL].setTargets(states[BL].angle.getDegrees(), states[BL].speedMetersPerSecond);
        pairs_[BR].setTargets(states[BR].angle.getDegrees(), states[FR].speedMetersPerSecond);
    }

    public SwerveModuleState getModuleState(int which) {
        return pairs_[which].getModuleState() ;
    }

    public SwerveModuleState getModuleTarget(int which) {
        return pairs_[which].getModuleTarget() ;
    }

    /// \brief This method is called when the robot enters one of its specifc modes.
    /// The modes are Autonomous, Teleop, Test, or Disabled. It is used to set the
    /// neutral mode specifically for the robot mode.
    public void init(LoopType ltype) {
        super.init(ltype);

        MotorController.NeutralMode nm = MotorController.NeutralMode.Coast ;
        switch(ltype) {
            case Disabled:
                nm = MotorController.NeutralMode.Coast ;
                break ;            
            case Teleop:
                nm = MotorController.NeutralMode.Coast ;
                break ;
            case Autonomous:
                nm = MotorController.NeutralMode.Coast ;
                break ;
            case Test:
                nm = MotorController.NeutralMode.Coast ;
                break ;            
        }

        for (int i = 0; i  < getModuleCount() ; i++) {
            try {
                pairs_[i].setNeutralMode(nm);
            } catch (Exception ex) {
            }
        }
    }

    public String getPairName(int which, boolean sname) {

        return sname ? names_[which].ShortName : names_[which].LongName ;
    }

    public void computeMyState() throws BadMotorRequestException {
        for (int i = 0; i < getModuleCount(); i++)
            pairs_[i].computeMyState(getRobot().getDeltaTime());

        if (gyro() != null) {
            double angle = gyro().getYaw();
            angular_.update(getRobot().getDeltaTime(), angle);
        }
       
        SwerveModuleState fl = pairs_[FL].getModuleState() ;
        SwerveModuleState fr = pairs_[FR].getModuleState() ;
        SwerveModuleState bl = pairs_[BL].getModuleState() ;
        SwerveModuleState br = pairs_[BR].getModuleState() ;
        odometry_.updateWithTime(getRobot().getTime(), getHeading(), fl, fr, bl, br) ;

        setRobotLocation(getPose());
    }

    @Override
    public void run() throws Exception {
        super.run();

        double dt = getRobot().getDeltaTime();
        for (int i = 0; i < getModuleCount(); i++)
            pairs_[i].run(dt);

    }

    public void stop() throws BadMotorRequestException, MotorRequestFailedException {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            pairs_[i].setPower(0.0, 0.0) ;
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics_.toSwerveModuleStates(speeds);

        for (int i = 0; i < getModuleCount(); i++) {
            pairs_[i].setTargets(states[i].angle.getDegrees(), states[i].speedMetersPerSecond) ;
        }
    }

    public void setPower(int which, double steer, double drive) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        pairs_[which].setPower(steer, drive) ;
    }

    
    public void setSteerMotorPower(int which, double steer) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        pairs_[which].setSteerMotorPower(steer) ;
    }

    
    public void setDriveMotorPower(int which, double drive) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        pairs_[which].setDriveMotorPower(drive) ;
    }

    public void setTargets(double[] angles, double[] speeds) {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            pairs_[i].setTargets(angles[i], speeds[i]) ;
        }
    }

    public void setAngleTarget(double angle) {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            pairs_[i].setAngle(angle);
        }
    }
}
