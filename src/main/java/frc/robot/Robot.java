// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import javax.lang.model.util.ElementScanner6;
import javax.swing.GroupLayout.Group;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.ejml.equation.Variable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code
 * necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private MotorControllerGroup RightSide;
  private MotorControllerGroup LeftSide;
  private DifferentialDrive ArcadeDrive;
  private Joystick LeftStick;
  private Joystick RightStick;
  private AHRS AHRS;
  private double RightStickX;
  private double RightStickY;
  private double RightStickTwist;
  private double RobotAng;
  private double P;
  private double I;
  private double D;
  Translation2d FrontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d FrontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d BackLeftLocation = new Translation2d(-0.381, -0.381);
  Translation2d BackRightLocation = new Translation2d(-0.381, 0.381);
  SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(FrontRightLocation, FrontLeftLocation, BackLeftLocation, BackRightLocation);
  SwerveDriveOdometry Odometry = new SwerveDriveOdometry(Kinematics, AHRS.getRotation2d());

  private class Wheel {
    private CANSparkMax Drive;
    private CANSparkMax Steer;
    private RelativeEncoder Encoder;
    private SparkMaxPIDController PIDController;
    private double WantedAng;
    private double NeededAng;

    private Wheel(CANSparkMax Drive, CANSparkMax Steer, RelativeEncoder Encoder, SparkMaxPIDController PIDController, double WantedAng, double NeededAng) {
      this.Drive = Drive;
      this.Steer = Steer;
      this.Encoder = Encoder;
      this.PIDController = PIDController;
      this.WantedAng = WantedAng;
      this.NeededAng = NeededAng;
    }
  }

  private Wheel FrontRight = new Wheel(null, null, null, null, D, D);
  private Wheel FrontLeft = new Wheel(null, null, null, null, D, D);
  private Wheel BackLeft = new Wheel(null, null, null, null, D, D);
  private Wheel BackRight = new Wheel(null, null, null, null, D, D);

  @Override
  public void robotInit() {
    LeftStick = new Joystick(1);
    RightStick = new Joystick(0);
    AHRS = new AHRS(I2C.Port.kMXP);
    FrontRight.Drive = new CANSparkMax(1, MotorType.kBrushless);
    FrontLeft.Drive = new CANSparkMax(2, MotorType.kBrushless);
    BackLeft.Drive = new CANSparkMax(3, MotorType.kBrushless);
    BackRight.Drive = new CANSparkMax(4, MotorType.kBrushless);
    FrontRight.Steer = new CANSparkMax(5, MotorType.kBrushed);
    FrontLeft.Steer = new CANSparkMax(6, MotorType.kBrushed);
    BackLeft.Steer = new CANSparkMax(7, MotorType.kBrushed);
    BackRight.Steer = new CANSparkMax(8, MotorType.kBrushed);
    RightSide = new MotorControllerGroup(FrontRight.Drive,BackRight.Drive);
    LeftSide = new MotorControllerGroup(FrontLeft.Drive,BackLeft.Drive);
    ArcadeDrive = new DifferentialDrive(LeftSide, RightSide);
    FrontRight.Encoder = FrontRight.Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, (28));
    FrontLeft.Encoder = FrontLeft.Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, (28));
    BackLeft.Encoder = BackLeft.Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, (28));
    BackRight.Encoder = BackRight.Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, (28));
    FrontRight.Encoder.setPosition(0);
    FrontLeft.Encoder.setPosition(0);
    BackLeft.Encoder.setPosition(0);
    BackRight.Encoder.setPosition(0);
    AHRS.calibrate();
    FrontRight.PIDController = FrontRight.Steer.getPIDController();
    FrontLeft.PIDController = FrontLeft.Steer.getPIDController();
    BackLeft.PIDController = BackLeft.Steer.getPIDController();
    BackRight.PIDController = BackRight.Steer.getPIDController();
    P = 1;
    I = 0;
    D = 0;
    FrontRight.PIDController.setOutputRange(-1, 1);
    FrontLeft.PIDController.setOutputRange(-1, 1);
    BackLeft.PIDController.setOutputRange(-1, 1);
    BackRight.PIDController.setOutputRange(-1, 1);
    FrontRight.PIDController.setP(P);
    FrontLeft.PIDController.setP(P);
    BackLeft.PIDController.setP(P);
    BackRight.PIDController.setP(P);
    FrontRight.PIDController.setI(I);
    FrontLeft.PIDController.setI(I);
    BackLeft.PIDController.setI(I);
    BackRight.PIDController.setI(I);
    FrontRight.PIDController.setD(D);
    FrontLeft.PIDController.setD(D);
    BackLeft.PIDController.setD(D);
    BackRight.PIDController.setD(D);

  }

  @Override
  public void teleopPeriodic() {
    //Find right joystick positions once, to prevent discrepancies
    RightStickX = RightStick.getX();
    RightStickY = RightStick.getY();
    RightStickTwist = RightStick.getRawAxis(3);

    // Find angle of the robot, to allow for strafing while rotating
    RobotAng = AHRS.getYaw();

    if (RobotAng < 0) {
      RobotAng = (Math.abs(RobotAng) + 180);
    }

    if (Math.abs(RightStickX) < 0.1) {
      RightStickX = 0;
    }
    if (Math.abs(RightStickY) < 0.1) {
      RightStickY = 0;
    }
    if (Math.abs(RightStickTwist) < 0.1) {
      RightStickTwist = 0;
    }

    //Set ChassisSpeeds for actual movement
    ChassisSpeeds speeds = new ChassisSpeeds((RightStickY * -1), RightStickX, RightStickTwist);

    //Convert to module states
    SwerveModuleState[] ModuleStates = Kinematics.toSwerveModuleStates(speeds);

    //Front left module state
    SwerveModuleState frontLeft = ModuleStates[0];

    //Front right module state
    SwerveModuleState frontRight = ModuleStates[1];

    //Back left module state
    SwerveModuleState backLeft = ModuleStates[2];

    //Back right module state
    SwerveModuleState backRight = ModuleStates[3];

    //Update Odometry for gyro implementation
    Odometry.update(AHRS.getRotation2d(), ModuleStates);

    if(RightStick.getRawButtonPressed(4)) {
      ArcadeDrive.arcadeDrive(RightStickY, RightStickTwist);
      FrontRight.PIDController.setReference(0, ControlType.kPosition);
      FrontLeft.PIDController.setReference(0, ControlType.kPosition);
      BackLeft.PIDController.setReference(0, ControlType.kPosition);
      BackRight.PIDController.setReference(0, ControlType.kPosition);
    }
    else {
      FrontRight.WantedAng = ((frontRight.angle.getDegrees() / 360.0) * (59.0 + (1.0/6.0)));
      FrontLeft.WantedAng = ((frontLeft.angle.getDegrees() / 360.0) * (59.0 + (1.0/6.0)));
      BackLeft.WantedAng = ((backLeft.angle.getDegrees() / 360.0) * (59.0 + (1.0/6.0)));
      BackRight.WantedAng = ((backRight.angle.getDegrees() / 360.0) * (59.0 + (1.0/6.0)));

      FrontRight.NeededAng = (((RobotAng / 360) * (59.0 + (1.0/6.0))) - FrontRight.WantedAng);
      FrontLeft.NeededAng = (((RobotAng / 360) * (59.0 + (1.0/6.0))) - FrontLeft.WantedAng);
      BackLeft.NeededAng = (((RobotAng / 360) * (59.0 + (1.0/6.0))) - BackLeft.WantedAng);
      BackRight.NeededAng = (((RobotAng / 360) * (59.0 + (1.0/6.0))) - BackRight.WantedAng);

      FrontRight.PIDController.setReference(FrontRight.WantedAng, ControlType.kPosition);
      FrontLeft.PIDController.setReference(FrontLeft.WantedAng, ControlType.kPosition);
      BackLeft.PIDController.setReference(BackLeft.WantedAng, ControlType.kPosition);
      BackRight.PIDController.setReference(BackRight.WantedAng, ControlType.kPosition);

      FrontRight.Drive.set(frontRight.speedMetersPerSecond / 3);
      FrontLeft.Drive.set(frontLeft.speedMetersPerSecond / 3);
      BackLeft.Drive.set(backLeft.speedMetersPerSecond / 3);
      BackRight.Drive.set(backRight.speedMetersPerSecond / 3);
    };
  };
};
