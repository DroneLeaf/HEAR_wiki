# Overview

## Battery status
From: https://mavlink.io/en/messages/common.html#BATTERY_STATUS

Check status (threshold configurable from QGC):
https://mavlink.io/en/messages/common.html#MAV_BATTERY_CHARGE_STATE

leafFC BatteryStatusManager listens to the MAV_BATTERY_CHARGE_STATE

leafFC inquiries current low battery COM_LOW_BAT_ACT safety behavior and stores it in BatteryStatusManager

Code snippet for char reservation:
```
#include <iostream>
#include <string>

int main() {
    // Example std::string
    std::string str = "Hello, World!";
    
    // Allocate a char array with size (str.size() + 1) for the null terminator
    char* char_array = new char[16 + 1];
    
    // Copy the contents of the std::string into the char array
    std::strcpy(char_array, str.c_str());

    // Output the C-style string
    std::cout << "C-style string: " << char_array << std::endl;

    // Don't forget to free the dynamically allocated memory
    delete[] char_array;

    return 0;
}

```

