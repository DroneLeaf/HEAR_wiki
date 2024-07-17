# Overview

This document summarizes the steps to generate `.so` libraries directly from Simulink without the need for intermediate compilation steps with cmake/make

# Test platforms

Tested so far on:


| OS           | MATLAB ver | Model(s)                            |
| :------------- | ------------ | ------------------------------------- |
| Ubuntu 20.04 | 2023a      | PD_controller_for_SOIPTD_parametric |

# Simulink configurations

You need *Embedded Coder* add-on installed first.

Make sure you follow these steps:

![](assets/1.png)

![](assets/2.png)

![](assets/3.png)

![](assets/4.png)

![](assets/5.png)

![](assets/6.png)

![](assets/7.png)

![](assets/8.png)

![](assets/9.png)

![](assets/10.png)

![](assets/11.png)

![](assets/12.png)

**Make sure simulation time is set to `inf`**

![](assets/13.png)

# Checking the code generation report

Click 'Open Report' under 'C Code' tap in Simulink.

![](assets/14.png)

1. Check the 'Hardware Device Type' if it is what you want.
2. Check the path for exporting the `.so` library.

![](assets/15.png)

Above are the three functions we will call from our Python/C++ code.

![](assets/16.png)

Above are all the variables accessible globally the `.so` library. What is under the 'Code Identifier' cloumn is what we can access from our Python/C++ code. Data type 'real_T' corresponds to `double` as per the 'Data Type Replacement' tab in the 'Model Settings'.
