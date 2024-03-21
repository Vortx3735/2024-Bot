package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase{
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue, startOfStreak, endOfStreak;


    public LED(int port, int length) {
        m_rainbowFirstPixelHue = 0;
        startOfStreak = 0;
        endOfStreak = 0;
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(length);
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
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void noteCheck() {
        if(RobotContainer.intake.getRing()) {
            for(int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, Color.kYellow);
            }
        } else {
            for(int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, Color.kBlue);
            }
        }
        m_led.setData(m_ledBuffer);
    }

    public void vorTXStreak() {
        endOfStreak = startOfStreak + m_ledBuffer.getLength()/2;
        for(int i = startOfStreak; i < endOfStreak; i++) {
            m_ledBuffer.setLED(i % m_ledBuffer.getLength(), Color.kGreen);
            m_ledBuffer.setLED((i+(m_ledBuffer.getLength()/2)) % m_ledBuffer.getLength(), Color.kBlue);
        }
        startOfStreak++;
        startOfStreak %= m_ledBuffer.getLength();
        m_led.setData(m_ledBuffer);      
    }

    public void setColor(Color color) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) { 
            m_ledBuffer.setLED(i, color);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setVorTXGreen() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) { 
            m_ledBuffer.setRGB(i, 127, 194, 65);
        }
        m_led.setData(m_ledBuffer);
    }

    public void funny() {
        int r = (int) RobotContainer.con2.getLeftX()*25500;
        int g = (int) RobotContainer.con2.getLeftY()*25500;
        int b = (int) RobotContainer.con2.getRightX()*25500;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) { 
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

}