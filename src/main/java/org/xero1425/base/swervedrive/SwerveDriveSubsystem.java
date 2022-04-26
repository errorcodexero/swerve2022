package org.xero1425.base.swervedrive;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.LoopType;
import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.Speedometer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

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

    static private final Names[] names_ = new Names[4] ;                                        // The names of each module (short name and long name)

    static private final String AngularSamplesName = "samples:angular";              // The settings file entry for the angular speedometer

    private SwerveDriveKinematics kinematics_ ;
    private SwerveDriveOdometry odometry_ ;

    private boolean pathing_ ;
    private NetworkTableEntry robot_loc_ ;
    private NetworkTableEntry path_loc_ ;
    private double [] path_values_ ;

    private int plotid_ ;
    private double plotstart_ ;
    private Double[] plotdata_ ;
    private static final String [] columns_ = {
        "time",
        "fl-ang-t (deg)", "fl-ang-a (deg)","fl-drv-t (m/s)","fl-drv-a (m/s)",
        "fr-ang-t (deg)", "fr-ang-a (deg)","fr-drv-t (m/s)","fr-drv-a (m/s)",
        "bl-ang-t (deg)", "bl-ang-a (deg)","bl-drv-t (m/s)","bl-drv-a (m/s)",
        "br-ang-t (deg)", "br-ang-a (deg)","br-drv-t (m/s)","br-drv-a (m/s)",
    } ;

    /// \brief create the serve drive subsystem
    /// \param parent the parent subsystem
    /// \param name the name of the subsystem
    /// \param config the prefix for configuration entries in the settings file
    public SwerveDriveSubsystem(Subsystem parent, String name, boolean hwpid) throws Exception {
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
        maxspeed_ = getSettingsValue("physical:maxspeed").getDouble();

        //
        // Create the swerve modules
        //
        pairs_ = new SwerveModule[getModuleCount()];
        for (int i = 0; i < getModuleCount(); i++) {
            pairs_[i] = new SwerveModule(getRobot(), this, names_[i].LongName, config, names_[i].ShortName, hwpid);
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
        odometry_ = new SwerveDriveOdometry(kinematics_, Rotation2d.fromDegrees(gyro().getAngle())) ;

        plotdata_ = new Double[columns_.length] ;
        plotid_ = -1 ;
        pathing_ = false ;

        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        NetworkTable table = inst.getTable("XeroLocation") ;
        robot_loc_ = table.getEntry("Robot") ;
        path_loc_ = table.getEntry("Path") ;
        path_values_ = new double[3] ;
    }

    public void startPathing() {
        if (!DriverStation.isFMSAttached())
        {
            pathing_ = true ;
        }
    }

    public void setPathLocation(Pose2d loc) {
        if (pathing_) {
            path_values_[0] = loc.getX() ;
            path_values_[1] = loc.getY() ;
            path_values_[2] = loc.getRotation().getDegrees() ;
            path_loc_.setDoubleArray(path_values_) ;
        }
    }

    public void setRobotLocation(Pose2d loc) {
        if (pathing_) {
            path_values_[0] = loc.getX() ;
            path_values_[1] = loc.getY() ;
            path_values_[2] = loc.getRotation().getDegrees() ;
            robot_loc_.setDoubleArray(path_values_) ;
        }
    }

    public void endPathing() {
        pathing_ = false ;
    }

    public void startSwervePlot(String name) {
        if (plotid_ != -1)
            return ;

        plotid_ = initPlot(name) ;
        startPlot(plotid_, columns_);

        plotstart_ = getRobot().getTime();
    }

    public void endSwervePlot() {
        endPlot(plotid_);
        plotid_ = -1 ;
    }

    public void resetOdometry(Pose2d pose) {
        odometry_.resetPosition(pose, Rotation2d.fromDegrees(getAngle())) ;
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
       
        SwerveModuleState fl = getModule(FL).getModuleState() ;
        SwerveModuleState fr = getModule(FR).getModuleState() ;
        SwerveModuleState bl = getModule(BL).getModuleState() ;
        SwerveModuleState br = getModule(BR).getModuleState() ;
        odometry_.updateWithTime(getRobot().getTime(), Rotation2d.fromDegrees(getAngle()), fl, fr, bl, br) ;

        setRobotLocation(getPose());

        int index = 0 ;
        plotdata_[index++] = getRobot().getTime() - plotstart_ ;
        plotdata_[index++] = getModule(FL).getAngleTarget() ;
        plotdata_[index++] = getModule(FL).getAngle() ;
        plotdata_[index++] = getModule(FL).getSpeedTarget() ;
        plotdata_[index++] = getModule(FL).getSpeed() ;
        plotdata_[index++] = getModule(FR).getAngleTarget() ;
        plotdata_[index++] = getModule(FR).getAngle() ;
        plotdata_[index++] = getModule(FR).getSpeedTarget() ;
        plotdata_[index++] = getModule(FR).getSpeed() ;
        plotdata_[index++] = getModule(BL).getAngleTarget() ;
        plotdata_[index++] = getModule(BL).getAngle() ;
        plotdata_[index++] = getModule(BL).getSpeedTarget() ;
        plotdata_[index++] = getModule(BL).getSpeed() ;
        plotdata_[index++] = getModule(BR).getAngleTarget() ;
        plotdata_[index++] = getModule(BR).getAngle() ;
        plotdata_[index++] = getModule(BR).getSpeedTarget() ;
        plotdata_[index++] = getModule(BR).getSpeed() ;
        addPlotData(plotid_, plotdata_);
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
