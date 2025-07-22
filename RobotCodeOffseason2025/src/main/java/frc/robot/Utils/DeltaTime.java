package frc.robot.Utils;

public class DeltaTime {
    
    private static double  m_currentTime = 0, m_lastUpdateTime = m_currentTime; 

    public DeltaTime() {
        m_currentTime = System.currentTimeMillis();
        m_lastUpdateTime = m_currentTime;
    }

    public static double getDeltaTime(){
        m_currentTime = System.currentTimeMillis();
        double deltaTime = (m_currentTime - m_lastUpdateTime) / 1000.0; // Convert milliseconds to seconds
        m_lastUpdateTime = m_currentTime;
        return deltaTime;
    }

    public static double getCurrentTime(){
        return m_currentTime;
    }

    public static double getLastUpdateTime(){
        return m_lastUpdateTime;
    }

    public static void reset() {
        m_currentTime = System.currentTimeMillis();
        m_lastUpdateTime = m_currentTime;
    }

}
