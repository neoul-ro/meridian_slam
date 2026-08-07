// Zero the VN-100 gyro bias and persist it to flash, using VectorNav's
// official vnproglib (the same library the ROS driver talks through).
//
// Preconditions:
//   1. Stop every launch using the IMU port (vectornav driver included).
//   2. Power the IMU for 10+ minutes first (bias is temperature-dependent).
//   3. Keep the robot COMPLETELY still while this runs.
//
// Build (from ~/slam_ws2):
//   g++ -O2 tools/vn100_gyro_bias.cpp \
//     -I src/vectornav/vectornav/vnproglib-1.2.0.0/cpp/include \
//     -L install/vectornav/lib -lvncxx \
//     -Wl,-rpath,$HOME/slam_ws2/install/vectornav/lib \
//     -o tools/vn100_gyro_bias
//
// Run:  ./tools/vn100_gyro_bias [port]        (default /dev/ttyUSB0)

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "vn/sensors.h"

int main(int argc, char* argv[])
{
  const std::string port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
  const std::vector<uint32_t> bauds = {921600, 115200, 9600, 57600, 38400,
                                       19200, 128000, 230400, 460800};

  vn::sensors::VnSensor vs;
  uint32_t connected_baud = 0;
  for (uint32_t baud : bauds) {
    std::cout << "trying " << port << " @ " << baud << " ..." << std::endl;
    try {
      vs.connect(port, baud);
      if (vs.verifySensorConnectivity()) {
        connected_baud = baud;
        break;
      }
      vs.disconnect();
    } catch (const std::exception& e) {
      std::cerr << "  " << e.what() << std::endl;
      if (vs.isConnected()) vs.disconnect();
    }
  }
  if (connected_baud == 0) {
    std::cerr << "FAIL: no VN-100 found on " << port
              << " (stop all launches using the port, check the cable)"
              << std::endl;
    return 1;
  }

  std::cout << "connected @ " << connected_baud
            << ", model: " << vs.readModelNumber() << std::endl;

  std::cout << "\nConfirm ALL of these before continuing:\n"
               "  - every ROS launch using the IMU is stopped\n"
               "  - the IMU has been powered for 10+ minutes (warm-up)\n"
               "  - the robot is completely still and will stay still\n"
               "proceed? [y/N] " << std::flush;
  std::string answer;
  std::getline(std::cin, answer);
  if (answer != "y" && answer != "Y") {
    std::cout << "aborted" << std::endl;
    vs.disconnect();
    return 1;
  }

  try {
    std::cout << "setting gyro bias (keep the robot still) ..." << std::endl;
    vs.setGyroBias();
    std::cout << "  OK" << std::endl;
    std::cout << "writing settings to flash ..." << std::endl;
    vs.writeSettings();
    std::cout << "  OK" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << std::endl;
    vs.disconnect();
    return 1;
  }
  vs.disconnect();

  std::cout << "\ndone. Restart the IMU launch and verify while still:\n"
               "  ros2 topic echo /vectornav/imu --field angular_velocity\n"
               "(all three axes should sit near 0)" << std::endl;
  return 0;
}
