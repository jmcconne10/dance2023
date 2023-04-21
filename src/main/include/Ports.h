namespace Ports {
    //USB
    const int driver = 0;
    const int r_operator = 1;

    //CAN
    const int PH = 1;
    const int left_front = 2;
    const int left_back = 3;
    const int right_front = 4;
    const int right_back = 5;
    const int arm = 6;
    const int extension = 7;

    //DIO
    const int arm_limit_high = 0;
    const int arm_limit_low = 1;
    const int extension_limit_front = 2;
    const int extension_limit_back = 3;

    //Pneumatics
    const int box = 7;
    const int low_grabber = 6;
    const int high_grabber = 5;
    
    //I2C
    const frc::I2C::Port color_sensor = frc::I2C::Port::kMXP;
}