package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.MinMaxData;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class SwerveBaseSubsystem extends DriveBaseSubsystem {
    private static final boolean UseEstimator = false ;

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
    
    private int index_ ;

    private SwerveDriveKinematics kinematics_ ;
    private SwerveDrivePoseEstimator estimator_ ;
    private SwerveDriveOdometry odometry_ ;

    private double [] angles_ ;
    private double [] powers_ ;

    private double width_ ;
    private double length_ ;
    private double rotate_angle_ ;
    private Pose2d last_pose_ ;

    private MinMaxData velocity_ ;
    private MinMaxData rotational_velocity_ ;
   
    static public final int FL = 0;                                                             // Index of the front left module
    static public final int FR = 1;                                                             // Index of the front right module
    static public final int BL = 2;                                                             // Index of the back left module
    static public final int BR = 3;                                                             // Index of the back right module

    public SwerveBaseSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        angles_ = new double[4] ;
        powers_ = new double[4] ;

        // Note: Change to 1 to get previous behavior
        velocity_ = new MinMaxData(10) ;
        rotational_velocity_ = new MinMaxData(10) ;

        for(int i = 0 ; i < 4 ; i++) {
            angles_[i] = 0.0 ;
            powers_[i] = 0.0 ;
        }

        plotdata_ = new Double[columns_.length] ;
        plotid_ = -1 ;
       
        width_ = getSettingsValue("physical:width").getDouble() ;
        length_ = getSettingsValue("physical:length").getDouble() ;

        //
        // Calculate the angle for the velocity vector to rotate the robot
        //
        rotate_angle_ = Math.toDegrees(Math.atan2(length_, width_)) ;

        kinematics_ = new SwerveDriveKinematics(new Translation2d(getWidth() / 2.0, getLength() / 2.0), new Translation2d(getWidth() / 2.0, -getLength() / 2.0), 
                        new Translation2d(-getWidth() / 2.0, getLength() / 2.0), new Translation2d(-getWidth() / 2.0, -getLength() / 2.0)) ;

        odometry_ = new SwerveDriveOdometry(kinematics_, Rotation2d.fromDegrees(gyro().getYaw())) ;
        estimator_ = new SwerveDrivePoseEstimator(Rotation2d.fromDegrees(gyro().getYaw()), new Pose2d(), kinematics_,
                VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.01), // Gyroscope rotation std-dev
                VecBuilder.fill(0.1, 0.1, 0.01)); // Vision (x, y, rotation) std-devs

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        shuffleboardTab.addNumber("Heading", () -> getHeading().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> getPose().getX());
        shuffleboardTab.addNumber("Pose Y", () -> getPose().getY());

        last_pose_ = new Pose2d() ;
    }

    // Control the swerve drive by settings a ChassisSppeds object
    public abstract void drive(ChassisSpeeds speeds) ;

    // Control the swerve drive by supplying raw targets.  If power is true then the values
    // supplied by the speeds_power array are power numbers to go directly to the drive motors.
    // If power is false, these are speed numbers to feed into the drive motor PID controller 
    // for the modules.
    public abstract void setRawTargets(boolean power, double [] angles, double [] speeds_powers) ;

    public abstract SwerveModuleState getModuleState(int which) ;

    public abstract SwerveModuleState getModuleTarget(int which) ;

    public void stop() throws BadMotorRequestException, MotorRequestFailedException {
        setRawTargets(false, powers_, angles_);
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        odometry_.update(Rotation2d.fromDegrees(gyro().getYaw()), getModuleState(FL), getModuleState(FR), getModuleState(BL), getModuleState(BR));
        estimator_.update(Rotation2d.fromDegrees(gyro().getYaw()), getModuleState(FL), getModuleState(FR), getModuleState(BL), getModuleState(BR));

        Pose2d p = getPose() ;
        double dist = p.getTranslation().getDistance(last_pose_.getTranslation()) ;
        double v = dist / getRobot().getDeltaTime() ;
        velocity_.addData(v);

        v = (p.getRotation().getDegrees() - last_pose_.getRotation().getDegrees()) / getRobot().getDeltaTime() ;
        rotational_velocity_.addData(v) ;

        last_pose_ = p ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    protected SwerveDriveKinematics getKinematics() {
        return kinematics_ ;
    }
   
    public Pose2d getPose() {
        if (UseEstimator) {
            return estimator_.getEstimatedPosition() ;
        }

        return odometry_.getPoseMeters() ;
    }

    public void setPose(Pose2d pose) {
        Rotation2d rot = Rotation2d.fromDegrees(gyro().getYaw()) ;
        odometry_.resetPosition(pose, rot) ;
        gyro().reset() ;

        estimator_.resetPosition(pose, Rotation2d.fromDegrees(gyro().getYaw())) ;
    }

    // This is a hack for this one event.  Need to rethink this after block party
    public double getVelocity() {
        return velocity_.getMax() ;
    }

    // This is a hack for this one event.  Need to rethink this after block party
    public double getRotationalVelocity() {
        return rotational_velocity_.getMax() ;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation() ;
    }

    public int getModuleCount() {
        return 4 ;
    }
    
    /// \brief return the base angle for a vector to rotate the robot.
    /// \returns the base angle for a vector to rotate the robot
    public double getPHI() {
        return rotate_angle_ ;
    }

    public double getWidth() {
        return width_ ;
    }

    public double getLength() {
        return length_ ;
    }
   
    public void startSwervePlot(String name) {
        if (plotid_ == -1) {
            plotid_ = initPlot(name) ;
        }

        startPlot(plotid_, columns_);
        plotstart_ = getRobot().getTime();
    }

    public void endSwervePlot() {
        endPlot(plotid_);
        plotid_ = -1 ;
    }    

    protected void newPlotData() {
        index_ = 0 ;
        
        putData(getRobot().getTime() - plotstart_) ;

        putData(getModuleTarget(FL).angle.getDegrees()) ;
        putData(getModuleState(FL).angle.getDegrees()) ;
        putData(getModuleTarget(FL).speedMetersPerSecond) ;
        putData(getModuleState(FL).speedMetersPerSecond) ;

        putData(getModuleTarget(FR).angle.getDegrees()) ;
        putData(getModuleState(FR).angle.getDegrees()) ;
        putData(getModuleTarget(FR).speedMetersPerSecond) ;
        putData(getModuleState(FR).speedMetersPerSecond) ;

        putData(getModuleTarget(BL).angle.getDegrees()) ;
        putData(getModuleState(BL).angle.getDegrees()) ;
        putData(getModuleTarget(BL).speedMetersPerSecond) ;
        putData(getModuleState(BL).speedMetersPerSecond) ;
        
        putData(getModuleTarget(BR).angle.getDegrees()) ;
        putData(getModuleState(BR).angle.getDegrees()) ;
        putData(getModuleTarget(BR).speedMetersPerSecond) ;
        putData(getModuleState(BR).speedMetersPerSecond) ;

        addPlotData(plotid_, plotdata_);
    }

    private void putData(double d) {
        plotdata_[index_++] = d ;
    }
}
