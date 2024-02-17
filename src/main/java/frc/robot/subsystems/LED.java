package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer ledBuffer;

    public LED(int port, int length) {
        m_led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        m_led.setLength(ledBuffer.getLength());
        m_led.setData(ledBuffer);

        m_led.start();
    }

    public void noteCheck() {
        if(RobotContainer.intake.getRing() == true) {
            setGreen();
        } else {
            setRed();
        }
    }

    public void setCustom(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) { 
            ledBuffer.setRGB(i, r, g, b);
         }
         
         m_led.setData(ledBuffer);
    }

    public void setGreen() {
        for (var i = 0; i < ledBuffer.getLength(); i++) { 
            ledBuffer.setRGB(i, 0, 255, 0);
         }
         
         m_led.setData(ledBuffer);
    }

    public void setRed() {
        for (var i = 0; i < ledBuffer.getLength(); i++) { 
            ledBuffer.setRGB(i, 255, 0, 0);
         }
         
         m_led.setData(ledBuffer);
    }

    public void setYellow() {
        for (var i = 0; i < ledBuffer.getLength(); i++) { 
            ledBuffer.setRGB(i, 255, 255, 0);
         }
         
         m_led.setData(ledBuffer);
    }

    public void setVorTXGreen() {
        for (var i = 0; i < ledBuffer.getLength(); i++) { 
            ledBuffer.setRGB(i, 197, 124, 65);
         }
         
         m_led.setData(ledBuffer);
    }

    public void setVorTXBlue() {
        for (var i = 0; i < ledBuffer.getLength(); i++) { 
            ledBuffer.setRGB(i, 0, 51, 76);
         }
         
         m_led.setData(ledBuffer);
    }

    private void rainbow() {
        int rainbowFirstPixelHue = 0;
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        rainbow();
        m_led.setData(ledBuffer);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
