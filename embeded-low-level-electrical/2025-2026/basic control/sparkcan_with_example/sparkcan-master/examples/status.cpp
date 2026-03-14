// UM ASME
// SOFTWARE AND ELECTRICAL TEAMS
// By Ahmed and Juan

#include <chrono>
#include <iomanip>
#include <iostream>
#include "SparkMax.hpp"

int main()
{
  try {

    SparkMax motor("can0", 1); 

    while (true)
    {
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "\r"
                << "Duty Cycle: " << motor.GetDutyCycle() << " | "
                << "Velocity: " << motor.GetVelocity() << " RPM | "
                << "Position: " << motor.GetPosition() << " ticks" << std::flush;
      
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
  return 0;
}
