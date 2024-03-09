package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase{
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;


    public LED(int port, int length) {
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(36);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        // if (DriverStation.isEnabled()) {
        //     setLEDs();
        // }
    }

    @Override
    public void simulationPeriodic() {

    }


    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    private void setLEDs() {
        setVorTXGreen();
        m_led.setData(m_ledBuffer);
    }

    private void setColor(Color color) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
            m_ledBuffer.setLED(i, color);
        }
    }

    private void setVorTXGreen() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
            m_ledBuffer.setRGB(i, 197, 124, 65);
        }
    }

    private void funny() {
        int r = (int) RobotContainer.con2.getLeftX();
        int g = (int) RobotContainer.con2.getLeftY();
        int b = (int) RobotContainer.con2.getRightX();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }
}