/**
 * file name: Swerve Drive Simulator
 * written by Baylee Carpenter
 * Uploaded: 11/8/2020
 *
 * this is code for simulating the Hooper Bot but with a swerve drive
 * code is heavily modified from PennCompHooperBot3-13-2020
 * 
 * tested features:
 * mecanum drive, manual intake, shooter, and elevator
 * 
 * untested features:
 * auto shooter, swervce drivetrain
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to
 * change the parameter class to the startRobot call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
