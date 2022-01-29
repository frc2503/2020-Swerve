// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import javax.lang.model.util.ElementScanner6;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.ejml.equation.Variable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code
 * necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick LeftStick;
  private Joystick RightStick;
  private AHRS AHRS;
  private double RightStickX;
  private double RightStickY;
  private double RightStickZ;
  private double RobotAng;
  private double StrafeAng;
  private double StrafeX;
  private double StrafeY;
  private double StrafeMag;
  private double P;
  private double I;
  private double D;
  private int StrafeAngQuadrantMod;
  private int StrafeAngCosFix;

  private class Wheel {
    private CANSparkMax Drive;
    private CANSparkMax Steer;
    private RelativeEncoder Encoder;
    private SparkMaxPIDController PIDController;
    private int RotateAng;
    private int WheelAngQuadrantMod;
    private double EncoderPos;
    private double WheelX;
    private double WheelY;
    private double WheelAng;
    private double WheelSpd;
    private double RotateXY;

    private Wheel(CANSparkMax Drive, CANSparkMax Steer, RelativeEncoder Encoder, SparkMaxPIDController PIDController,
        int RotateAng, int WheelAngQuadrantMod, double EncoderPos, double WheelX, double WheelY, double WheelAng,
        double WheelSpd, double RotateXY) {
      this.Drive = Drive;
      this.Steer = Steer;
      this.Encoder = Encoder;
      this.PIDController = PIDController;
      this.RotateAng = RotateAng;
      this.WheelAngQuadrantMod = WheelAngQuadrantMod;
      this.EncoderPos = EncoderPos;
      this.WheelX = WheelX;
      this.WheelY = WheelY;
      this.WheelAng = WheelAng;
      this.WheelSpd = WheelSpd;
      this.RotateXY = RotateXY;
    };

    // Used to find the angle to add to vector angles to fix the quadrant of wheel
    // vectors after trig later
    private int FindWheelAngQuadrantMod(double X, double Y) {
      if (X > 0) {
        if (Y > 0) { // Upper right quadrant
          return 270;
        } else { // Lower right quadrant
          return 180;
        }
      } else {
        if (Y > 0) { // Upper left quadrant
          return 0;
        } else { // Lower left quadrant
          return 90;
        }
      }
    };
  };

  private Wheel FrontRight = new Wheel(null, null, null, null, StrafeAngCosFix, StrafeAngCosFix, RightStickX,
      RightStickX, RightStickX, RightStickX, RightStickX, RightStickX);
  private Wheel FrontLeft = new Wheel(null, null, null, null, StrafeAngCosFix, StrafeAngCosFix, RightStickX,
      RightStickX, RightStickX, RightStickX, RightStickX, RightStickX);
  private Wheel BackLeft = new Wheel(null, null, null, null, StrafeAngCosFix, StrafeAngCosFix, RightStickX, RightStickX,
      RightStickX, RightStickX, RightStickX, RightStickX);
  private Wheel BackRight = new Wheel(null, null, null, null, StrafeAngCosFix, StrafeAngCosFix, RightStickX,
      RightStickX, RightStickX, RightStickX, RightStickX, RightStickX);

  @Override
  public void robotInit() {
    LeftStick = new Joystick(0);
    RightStick = new Joystick(1);
    AHRS = new AHRS(I2C.Port.kMXP);
    FrontRight.Drive = new CANSparkMax(1, MotorType.kBrushless);
    FrontLeft.Drive = new CANSparkMax(2, MotorType.kBrushless);
    BackLeft.Drive = new CANSparkMax(3, MotorType.kBrushless);
    BackRight.Drive = new CANSparkMax(4, MotorType.kBrushless);
    FrontRight.Steer = new CANSparkMax(5, MotorType.kBrushed);
    FrontLeft.Steer = new CANSparkMax(6, MotorType.kBrushed);
    BackLeft.Steer = new CANSparkMax(7, MotorType.kBrushed);
    BackRight.Steer = new CANSparkMax(8, MotorType.kBrushed);
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
  };

  @Override
  public void teleopPeriodic() {

    // Find right joystick positions once, to prevent discrepancies
    RightStickX = RightStick.getX();
    RightStickY = RightStick.getY();
    RightStickZ = RightStick.getRawAxis(3);

    // Find angle of the robot, to allow for strafing while rotating
    RobotAng = AHRS.getYaw();

    if (RobotAng < 0) {
      RobotAng = (Math.abs(RobotAng) + 180);
    }
    ;

    // Fixes the 270 degree discrepancy between how wheel angles are measured and
    // how the joystick angles are measured. This also puts the angle into the
    // correct quadrant after trig.
    // Also fixes the sign of outputs of cosine later
    if (RightStickX > 0) {
      if (RightStickY > 0) { // Upper right quadrant
        StrafeAngQuadrantMod = 270;
        StrafeAngCosFix = -1;
      } else { // Lower right quadrant
        StrafeAngQuadrantMod = 180;
        StrafeAngCosFix = 1;
      }
      ;
    } else {
      if (RightStickY > 0) { // Upper left quadrant
        StrafeAngQuadrantMod = 0;
        StrafeAngCosFix = 1;
      } else { // Lower left quadrant
        StrafeAngQuadrantMod = 90;
        StrafeAngCosFix = -1;
      }
      ;
    }
    ;

    // Find the angle the robot should end up strafing
    StrafeAng = (Math.toDegrees(Math.atan(Math.abs(RightStickY) / Math.abs(RightStickX))) + StrafeAngQuadrantMod);

    // Set angle wheels should be at for rotating CW and CCW
    if (RightStickZ > 0.05) { // CW
      FrontRight.RotateAng = 225;
      FrontLeft.RotateAng = 315;
      BackLeft.RotateAng = 45;
      BackRight.RotateAng = 135;
    } 
    else if (RightStickZ < -0.05) { // CCW
      FrontRight.RotateAng = 45;
      FrontLeft.RotateAng = 135;
      BackLeft.RotateAng = 225;
      BackRight.RotateAng = 315;
    };

    // Find the magnitude of the strafe vector
    StrafeMag = (Math.sqrt(Math.pow(RightStickX, 2) + Math.pow(RightStickY, 2)));

    // Find angle for strafing in relation to the robot, break the vector down into
    // X and Y components, and apply the cosine fix
    StrafeX = (((Math.cos(StrafeAng - RobotAng)) * StrafeMag) * StrafeAngCosFix);
    StrafeY = ((Math.sin(StrafeAng - RobotAng)) * StrafeMag);

    // Break rotational vector down into X and Y components for each wheel. The X
    // and Y components are always equal, so only one variable is used.
    FrontRight.RotateXY = ((Math.sin(FrontRight.RotateAng)) * RightStickZ);
    FrontLeft.RotateXY = ((Math.sin(FrontLeft.RotateAng)) * RightStickZ);
    BackLeft.RotateXY = ((Math.sin(BackLeft.RotateAng)) * RightStickZ);
    BackRight.RotateXY = ((Math.sin(BackRight.RotateAng)) * RightStickZ);

    // Find the X and Y components of each wheel's vector
    FrontRight.WheelX = (StrafeX + FrontRight.RotateXY);
    FrontLeft.WheelX = (StrafeX + FrontLeft.RotateXY);
    BackLeft.WheelX = (StrafeX + BackLeft.RotateXY);
    BackRight.WheelX = (StrafeX + BackRight.RotateXY);
    FrontRight.WheelY = (StrafeY + FrontRight.RotateXY);
    FrontLeft.WheelY = (StrafeY + FrontLeft.RotateXY);
    BackLeft.WheelY = (StrafeY + BackLeft.RotateXY);
    BackRight.WheelY = (StrafeY + BackRight.RotateXY);

    // Find the angle to add to vector angles to fix the quadrant of wheel vectors
    // after trig
    FrontRight.WheelAngQuadrantMod = FrontRight.FindWheelAngQuadrantMod(FrontRight.WheelX, FrontRight.WheelY);
    FrontLeft.WheelAngQuadrantMod = FrontLeft.FindWheelAngQuadrantMod(FrontLeft.WheelX, FrontLeft.WheelY);
    BackLeft.WheelAngQuadrantMod = BackLeft.FindWheelAngQuadrantMod(BackLeft.WheelX, BackLeft.WheelY);
    BackRight.WheelAngQuadrantMod = BackLeft.FindWheelAngQuadrantMod(BackRight.WheelX, BackRight.WheelY);

    // Find the angle of the wheel vectors, and fix angle quadrant after trig
    FrontRight.WheelAng = ((Math.toDegrees(Math.atan(Math.abs(FrontRight.WheelY) / Math.abs(FrontRight.WheelX))))
        + FrontRight.WheelAngQuadrantMod);
    FrontLeft.WheelAng = ((Math.toDegrees(Math.atan(Math.abs(FrontLeft.WheelY) / Math.abs(FrontLeft.WheelX))))
        + FrontLeft.WheelAngQuadrantMod);
    BackLeft.WheelAng = ((Math.toDegrees(Math.atan(Math.abs(BackLeft.WheelY) / Math.abs(BackLeft.WheelX))))
        + BackLeft.WheelAngQuadrantMod);
    BackRight.WheelAng = ((Math.toDegrees(Math.atan(Math.abs(BackRight.WheelY) / Math.abs(BackRight.WheelX))))
        + BackRight.WheelAngQuadrantMod);

    // Find the magnitude of the wheel vectors
    FrontRight.WheelSpd = (Math.sqrt(Math.pow(FrontRight.WheelX, 2) + Math.pow(FrontRight.WheelY, 2)));
    FrontLeft.WheelSpd = (Math.sqrt(Math.pow(FrontLeft.WheelX, 2) + Math.pow(FrontLeft.WheelY, 2)));
    BackLeft.WheelSpd = (Math.sqrt(Math.pow(BackLeft.WheelX, 2) + Math.pow(BackLeft.WheelY, 2)));
    BackRight.WheelSpd = (Math.sqrt(Math.pow(BackRight.WheelX, 2) + Math.pow(BackRight.WheelY, 2)));

    if (FrontRight.WheelSpd < 0.05) {
      if (FrontRight.WheelSpd > -0.05) {
        FrontRight.WheelSpd = 0;
      }
    }
    if (FrontLeft.WheelSpd < 0.05) {
      if (FrontLeft.WheelSpd > -0.05) {
        FrontLeft.WheelSpd = 0;
      }    
    }
    if (BackLeft.WheelSpd < 0.05) {
      if (BackLeft.WheelSpd > -0.05) {
        BackLeft.WheelSpd = 0;
      }   
    }
    if (BackRight.WheelSpd < 0.05) {
      if (BackRight.WheelSpd > -0.05) {
        BackRight.WheelSpd = 0;
      }
    }

    // Compare current wheel angle to desired wheel angle, and determine whether to
    // spin CW, CCW, or not at all to reach desired wheel angle
    FrontRight.PIDController.setReference((FrontRight.WheelAng / 360.0) * (59.0 + (1.0/6.0)), ControlType.kPosition);
    FrontLeft.PIDController.setReference((FrontLeft.WheelAng / 360.0) * (59.0 + (1.0/6.0)), ControlType.kPosition);
    BackLeft.PIDController.setReference((BackLeft.WheelAng / 360.0) * (59.0 + (1.0/6.0)), ControlType.kPosition);
    BackRight.PIDController.setReference((BackRight.WheelAng / 360.0) * (59.0 + (1.0/6.0)), ControlType.kPosition);

    System.out.println("FrontRight.WheelAng:" + FrontRight.WheelAng);
    System.out.println("FrontLeft.WheelAng:" + FrontLeft.WheelAng);
    System.out.println("BackLeft.WheelAng:" + BackLeft.WheelAng);
    System.out.println("BackRight.WheelAng:" + BackRight.WheelAng);
    System.out.println("FrontRight.EncoderPos:" + FrontRight.Encoder.getPosition());
    System.out.println("FrontLeft.EncoderPos:" + FrontLeft.Encoder.getPosition());
    System.out.println("BackLeft.EncoderPos:" + BackLeft.Encoder.getPosition());
    System.out.println("BackRight.EncoderPos:" + BackRight.Encoder.getPosition());
    //System.out.println(FrontRight.WheelSpd);

    // Set speed of each wheel to desired speed
    // FrontRight.Drive.set(FrontRight.WheelSpd);
    // FrontLeft.Drive.set(FrontLeft.WheelSpd);
    // BackLeft.Drive.set(BackLeft.WheelSpd);
    // BackRight.Drive.set(BackRight.WheelSpd);
  };
};