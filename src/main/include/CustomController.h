#include <frc/XboxController.h>
#include <cmath>

using namespace frc;

/**
 * Handle input from Xbox 360 or Xbox One controllers connected to the Driver
 * Station.
 *
 * This class handles Xbox input that comes from the Driver Station. Each time a
 * value is requested the most recent value is returned. There is a single class
 * instance for each controller and the mapping of ports to hardware buttons
 * depends on the code in the Driver Station.
 * 
 * This class is a modified version of XboxController.
 */
class CustomController : public XboxController {
  public:

    /**
     * Construct an instance of our custom Xbox controller.
     *
     * The controller index is the USB port on the Driver Station.
     *
     * @param port The port on the Driver Station that the controller is plugged
     *             into (0-5).
     */
    CustomController(int port) : XboxController(port) {
       square_scale = false;
       deadzone = 0.1;
    };
    
    /**
     * Sets square scaling to on or off.
     * 
     * @param input The boolean to set the square scaling to.
     */
    void SetSquareScale(bool input) {
      square_scale = input;
    }

    /**
     * Returns whether the controller axis value is in the deadzone.
     * 
     * @param value The value to check.
     * @return A boolean of whether the controller axis is in the deadzone.
     */
    bool InDeadzone(double value) {
      return value < deadzone && value > -deadzone;
    }

    /**
     * Get the value of the axis. A deadzone is implemented.
     *
     * @param axis The axis to read, starting at 0.
     * @return The value of the axis between -1 and 1.
     */
    double GetRawAxis(int axis) {
        double value = GenericHID::GetRawAxis(axis);
        if(InDeadzone(value)) { //If the value is in the deadzone, return 0.
            return 0;
        }
        if(square_scale) {
            if(value > 0) {
              return pow((value - deadzone) / (1 - deadzone), 2); //If the value is above 0, use a positive square curve.
            }
            else {
              return -pow((value + deadzone) / (1 - deadzone), 2); //If the value is below 0, use a negative square curve.
            }
        }
        else {
            return value;
        }
    }

    /**
     * Get the value of the x axis on the left stick. A deadzone is implemented.
     *
     * @return The value of the axis between -1 and 1.
     */
    double GetLeftX() {
      return GetRawAxis(0);
    }

    /**
     * Get the value of the y axis on the left stick. A deadzone is implemented.
     *
     * @return The value of the axis between -1 and 1.
     */
    double GetLeftY() {
      return GetRawAxis(1);
    }

    /**
     * Get the value of the x axis on the right stick. A deadzone is implemented.
     *
     * @return The value of the axis between -1 and 1.
     */
    double GetRightX() {
      return GetRawAxis(4);
    }

    /**
     * Get the value of the y axis on the right stick. A deadzone is implemented.
     *
     * @return The value of the axis between -1 and 1.
     */
    double GetRightY() {
      return GetRawAxis(5);
    }

  private:
    bool square_scale;
    float deadzone;
};