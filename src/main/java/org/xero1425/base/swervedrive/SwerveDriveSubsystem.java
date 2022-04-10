package org.xero1425.base.swervedrive;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.LoopType;
import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.Speedometer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/// \brief The swerve drive subsystem for driving a robot using a drive base where each wheel can be independently steered.
public class SwerveDriveSubsystem extends DriveBaseSubsystem {
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
    private SwerveModule[] pairs_;                                                              // The serve modules, created by this class
    private Speedometer angular_;                                                               // The angular position, velocity, and acceleration of the robot
    private double rotate_angle_ ;                                                              // The base angle for a rotate vector, calculated from the length and width
    private double maxspeed_ ;                                                                  // The maximum linear speed of the 

    static public final int FL = 0;                                                             // Index of the front left module
    static public final int FR = 1;                                                             // Index of the front right module
    static public final int BL = 2;                                                             // Index of the back left module
    static public final int BR = 3;                                                             // Index of the back right module

    static private Names[] names_ = new Names[4] ;                                              // The names of each module (short name and long name)

    static private final String AngularSamplesName = "samples:angular";              // The settings file entry for the angular speedometer

    private SwerveDriveKinematics kinematics_ ;
    private SwerveDriveOdometry odometry_ ;

    private double plot_start_time_ ;
    private boolean plotting_ ;
    private int plot_id_ ;
    private Double [] plot_data_ ;
    static final String[] plot_columns_ = {             
        "time", 
        "fl-spd-target", "fl-spd-actual", "fl-ang-target", "fl-ang-actual",
        "fr-spd-target", "fr-spd-actual", "fr-ang-target", "fr-ang-actual",
        "bl-spd-target", "bl-spd-actual", "bl-ang-target", "bl-ang-actual",
        "br-spd-target", "br-spd-actual", "br-ang-target", "br-ang-actual",                        
    } ;


    /// \brief create the serve drive subsystem
    /// \param parent the parent subsystem
    /// \param name the name of the subsystem
    /// \param config the prefix for configuration entries in the settings file
    public SwerveDriveSubsystem(Subsystem parent, String name, String config) throws Exception {
        super(parent, name);

        plotting_ = false ;
        plot_id_ = initPlot("swervepids") ;
        plot_data_ = new Double[plot_columns_.length] ;

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
        maxspeed_ = getSettingsValue("physical:maxspeed").getDouble();

        //
        // Create the swerve modules
        //
        pairs_ = new SwerveModule[getModuleCount()];
        for (int i = 0; i < getModuleCount(); i++) {
            pairs_[i] = new SwerveModule(getRobot(), this, names_[i].LongName, config + ":" + names_[i].ShortName);
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
        // Calculate the angle for the velocity vector to rotate the robot
        //
        rotate_angle_ = Math.toDegrees(Math.atan2(length_, width_)) ;

        //
        // Reset the GYRO to zero degrees
        //
        gyro().reset() ;

        Translation2d fl = new Translation2d(width_ / 2.0, length_ / 2.0) ;
        Translation2d fr = new Translation2d(width_ / 2.0, -length_ / 2.0) ;
        Translation2d bl = new Translation2d(-width_ / 2.0, length_ / 2.0) ;
        Translation2d br = new Translation2d(-width_ / 2.0, -length_ / 2.0) ;

        kinematics_ = new SwerveDriveKinematics(fl, fr, bl, br) ;

        resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0))) ;
    }

    public void startSwervePlot() {
        plotting_ = true ;
        startPlot(plot_id_, plot_columns_) ;
        plot_start_time_ = getRobot().getTime() ;
    }

    public void endSwervePlot() {
        plotting_ = false ;
        endPlot(plot_id_) ;;
    }

    public void resetOdometry(Pose2d pose) {
        Rotation2d gyroangle = Rotation2d.fromDegrees(getAngle()) ;
        odometry_ = new SwerveDriveOdometry(kinematics_, gyroangle, pose) ;
    }

    public Pose2d getPose() {
        return odometry_.getPoseMeters() ;
    }

    public double getMaxSpeed() {
        return maxspeed_ ;
    }

    /// \brief return the base angle for a vector to rotate the robot.
    /// \returns the base angle for a vector to rotate the robot
    public double getPHI() {
        return rotate_angle_ ;
    }

    public int getModuleCount() {
        return names_.length ;
    }

    public SwerveModule getModule(int index) {
        return pairs_[index] ;
    }

    public double getWidth() {
        return width_;
    }

    public double getLength() {
        return length_;
    }

    /// \brief returns the current angle in degrees of the robot
    /// \returns the current angle of the robot
    public double getAngle() {
        return angular_.getDistance() ;
    }

    public double getModuleAngle(int module) {
        return getModule(module).getAngle() ;
    }

    public double getAcceleration() {
        double total = 0.0 ;

        for(int i = 0 ; i < getModuleCount() ; i++) {
            total += getModule(i).getAcceleration() ;
        }

        return total / getModuleCount() ;
    }

    public double getVelocity() {
        double total = 0.0 ;

        for(int i = 0 ; i < getModuleCount() ; i++) {
            total += getModule(i).getSpeed() ;
        }

        return total / getModuleCount() ;
    }

    public double getDistance() {
        double total = 0.0 ;

        for(int i = 0 ; i < getModuleCount() ; i++) {
            total += getModule(i).getDistance() ;
        }

        return total / getModuleCount() ;
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
                getModule(i).setNeutralMode(nm);
            } catch (Exception ex) {
            }
        }
    }

    public String getPairName(int which, boolean sname) {

        return sname ? names_[which].ShortName : names_[which].LongName ;
    }

    public void computeMyState() throws BadMotorRequestException {
        for (int i = 0; i < getModuleCount(); i++)
            getModule(i).computeMyState(getRobot().getDeltaTime());

        if (gyro() != null) {
            double angle = gyro().getYaw();
            angular_.update(getRobot().getDeltaTime(), angle);
        }
       
        if (odometry_ != null) {
            SwerveModuleState fl = getModule(FL).getModuleState() ;
            SwerveModuleState fr = getModule(FR).getModuleState() ;
            SwerveModuleState bl = getModule(BL).getModuleState() ;
            SwerveModuleState br = getModule(BR).getModuleState() ;
            odometry_.updateWithTime(getRobot().getTime(), Rotation2d.fromDegrees(getAngle()), fl, fr, bl, br) ;
        }

        putDashboard("fl", DisplayType.Always, getModule(FL).status());
        putDashboard("fr", DisplayType.Always, getModule(FR).status());
        putDashboard("bl", DisplayType.Always, getModule(BL).status());
        putDashboard("br", DisplayType.Always, getModule(BR).status());

        putDashboard("flticks", DisplayType.Verbose, getModule(FL).getTicks()) ;
        putDashboard("frticks", DisplayType.Verbose, getModule(FR).getTicks()) ;
        putDashboard("blticks", DisplayType.Verbose, getModule(BL).getTicks()) ;
        putDashboard("brticks", DisplayType.Verbose, getModule(BR).getTicks()) ;

        putDashboard("dbangle", DisplayType.Always, getAngle()) ;

        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("angle", getAngle()) ;
        logger.add("fl", getModule(FL).status()) ;
        logger.add("fr", getModule(FR).status()) ;
        logger.add("bl", getModule(BL).status()) ;        
        logger.add("br", getModule(BR).status()) ;
        logger.endMessage();        

        if (plotting_) {
            int index = 0 ;
            plot_data_[index++] = getRobot().getTime() - plot_start_time_ ;

            for(int i = 0 ; i < getModuleCount() ; i++) {
                SwerveModule module = getModule(i) ;
                plot_data_[index++] = module.getSpeedTarget() ;
                plot_data_[index++] = module.getSpeed() ;
                plot_data_[index++] = module.getAngleTarget() ;
                plot_data_[index++] = module.getAngle() ;
            }

            addPlotData(plot_id_, plot_data_) ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run();

        double dt = getRobot().getDeltaTime();
        for (int i = 0; i < getModuleCount(); i++)
            getModule(i).run(dt);

    }

    public void stop() throws BadMotorRequestException, MotorRequestFailedException {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            getModule(i).setPower(0.0, 0.0) ;
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics_.toSwerveModuleStates(speeds);

        for (int i = 0; i < getModuleCount(); i++) {
            getModule(i).setTargets(states[i].angle.getDegrees(), states[i].speedMetersPerSecond) ;
        }
    }

    public void setPower(int which, double steer, double drive) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        getModule(which).setPower(steer, drive) ;
    }

    
    public void setSteerMotorPower(int which, double steer) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        getModule(which).setSteerMotorPower(steer) ;
    }

    
    public void setDriveMotorPower(int which, double drive) throws Exception {
        if (which < 0 || which > BR)
            throw new Exception("invalid swerve pair address") ;
       
        getModule(which).setDriveMotorPower(drive) ;
    }

    public void setTargets(double[] angles, double[] speeds) {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            getModule(i).setTargets(angles[i], speeds[i]) ;
        }
    }

    public void setAngleTarget(double angle) {
        for(int i = 0 ; i < getModuleCount() ; i++) {
            getModule(i).setAngle(angle);
        }
    }
}
