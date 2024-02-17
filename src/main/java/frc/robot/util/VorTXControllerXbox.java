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

}