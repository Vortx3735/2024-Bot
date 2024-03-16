/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * Add your docs here.
 */
public class VorTXControllerXbox extends CommandXboxController {
	public Trigger aButton, bButton, xButton, yButton, view, menu, ls, rs, lb, rb, lt, rt,
					povUp, povUpRight, povRight, povDownRight, povDown, povDownLeft, povLeft, povUpLeft;

	public VorTXControllerXbox(int port) {
		super(port);
        		
		aButton = this.a();
		bButton = this.b();
		xButton = this.x();
		yButton = this.y();
		lb = this.leftBumper();
		rb = this.rightBumper();
		view = this.back();
		menu = this.start();
		ls = this.leftStick();
		rs = this.rightStick();
		lt = this.leftTrigger();
		rt = this.rightTrigger();

		
		povUp = this.povUp();
		povUpRight = this.povUpRight();
		povRight = this.povRight();
		povDownRight = this.povDownRight();
		povDown = this.povDown();
		povDownLeft = this.povDownLeft();
		povLeft = this.povLeft();
		povUpLeft = this.povUpLeft();
	}


	public double joystickFix(boolean isLeft, boolean isX) {
		double root2 = Math.sqrt(2);
		double x;
		double y;
		x = (isLeft) ? getHID().getLeftX() : getHID().getRightX();
		y = (isLeft) ? getHID().getLeftY() : getHID().getRightY();
		double magnitude = Math.sqrt(x*x + y*y);
		double[] joystick = {
			Math.signum(x) * Math.min(Math.abs(x*root2), magnitude),
			Math.signum(y) * Math.min(Math.abs(y*root2), magnitude)
		};
		return (isX) ? joystick[0] : joystick[1];
	}

	@Override
	public double getLeftX() {
		return joystickFix(true, true);
	}

	@Override
	public double getLeftY() {
		return joystickFix(true, false);
	}
	
	@Override
	public double getRightX() {
		return joystickFix(false, true);
	}

	@Override
	public double getRightY() {
		return joystickFix(false, false);
	}


}