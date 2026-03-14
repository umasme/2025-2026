// UM ASME
// SOFTWARE AND ELECTRICAL TEAMS
// By Ahmed and Juan

#include <iostream>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "SparkMax.hpp"

class KeyboardReader {
public:
    KeyboardReader() {
        tcgetattr(STDIN_FILENO, &oldSettings);
        newSettings = oldSettings;
        newSettings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    ~KeyboardReader() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    }

    int getKey() {
        unsigned char ch;
        if (read(STDIN_FILENO, &ch, 1) == 1) return ch;
        return -1;
    }

private:
    struct termios oldSettings, newSettings;
};

int main() {
    try {
        SparkMax motor1("can0", 1);
        SparkMax motor2("can0", 2);
        SparkMax motor3("can0", 3);
        SparkMax motor4("can0", 4);
        KeyboardReader keyboard;

        // --- MOTOR 1 INIT ---
        motor1.SetIdleMode(IdleMode::kBrake);
        motor1.SetMotorType(MotorType::kBrushless);
        motor1.SetSensorType(SensorType::kHallSensor);
        motor1.SetRampRate(1.0);
        motor1.SetInverted(false);
        motor1.SetSmartCurrentStallLimit(20);
        motor1.SetSmartCurrentFreeLimit(20);
        motor1.SetPeriodicStatus0Period(20);
        motor1.SetPeriodicStatus1Period(20);
        
        // --- MOTOR 2 INIT ---
        motor2.SetIdleMode(IdleMode::kBrake);
        motor2.SetMotorType(MotorType::kBrushless);
        motor2.SetSensorType(SensorType::kHallSensor);
        motor2.SetRampRate(1.0);
        motor2.SetInverted(true);
        motor2.SetSmartCurrentStallLimit(20);
        motor2.SetSmartCurrentFreeLimit(20);
        motor2.SetPeriodicStatus0Period(20);
        motor2.SetPeriodicStatus1Period(20);
        
        // --- MOTOR 3 INIT ---
        motor3.SetIdleMode(IdleMode::kBrake);
        motor3.SetMotorType(MotorType::kBrushless);
        motor3.SetSensorType(SensorType::kHallSensor);
        motor3.SetRampRate(1.0);
        motor3.SetInverted(true);
        motor3.SetSmartCurrentStallLimit(20);
        motor3.SetSmartCurrentFreeLimit(20);
        motor3.SetPeriodicStatus0Period(20);
        motor3.SetPeriodicStatus1Period(20);
        
        // --- MOTOR 4 INIT ---
        motor4.SetIdleMode(IdleMode::kBrake);
        motor4.SetMotorType(MotorType::kBrushless);
        motor4.SetSensorType(SensorType::kHallSensor);
        motor4.SetRampRate(1.0);
        motor4.SetInverted(true);
        motor4.SetSmartCurrentStallLimit(20);
        motor4.SetSmartCurrentFreeLimit(20);
        motor4.SetPeriodicStatus0Period(20);
        motor4.SetPeriodicStatus1Period(20);

        std::cout << "--- Control ---" << std::endl;
        std::cout << "[W] Increase Speed  [S] Decrease Speed  [Q] Stop & Quit" << std::endl;
        
        float targetVelocity = 0.0f;
        const float velocityStep = 100.0f;
        const float maxVelocity = 2000.0f;

        bool running = true;
        while (running) {
            int key = keyboard.getKey();
            if (key != -1) {
                switch (tolower(key)) {
                    case 'w':
                        targetVelocity += velocityStep;
                        if (targetVelocity > maxVelocity) targetVelocity = maxVelocity;
                        std::cout << "Current Velocity: " << targetVelocity << "RPM" << std::endl;
                        break;
                    case 's':
                        targetVelocity -= velocityStep;
                        if (targetVelocity < -maxVelocity) targetVelocity = -maxVelocity;
                        std::cout << "Current Velocity: " << targetVelocity << "RPM" << std::endl;
                        break;
                    case 'q':
                        std::cout << "Quitting..." << std::endl;
                        running = false;
                        break;
                }
            }

            SparkBase::Heartbeat();
            
            // Set velocity of the motors.
            motor1.SetVelocity(targetVelocity);
            motor2.SetVelocity(targetVelocity);
            motor3.SetVelocity(targetVelocity);
            motor4.SetVelocity(targetVelocity);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        motor1.SetDutyCycle(0.0f);
        motor2.SetDutyCycle(0.0f);
        motor3.SetDutyCycle(0.0f);
        motor4.SetDutyCycle(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    } catch (const std::exception & e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
