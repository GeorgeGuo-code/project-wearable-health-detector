# project-wearable-health-detector
The code part of our final project of Digital System.
## overall structure
  top <br>
    ├── SM_Main <br>
    │   ├── Timer <br>
    │   ├── Input_Debouncer <br>
    │   └── Timeout_Detector <br>
    ├── SM_DataFlow <br>
    │   ├── Data_Collection <br>
    │   │   ├── HR_Driver <br>
    │   │   ├── Temp_Driver <br>
    │   │   └── Accel_Driver <br>
    │   ├── Data_Process <br>
    │   │   ├── Bandpass_Filter <br>
    │   │   ├── Moving_Avg_Filter <br>
    │   │   ├── Peak_Detector <br>
    │   │   ├── HR_Calculator <br>
    │   │   ├── Activity_Calculator <br>
    │   │   └── Sport_Intensity_Evaluator <br>
    │   ├── Data_Analysis <br>
    │   │   └── Health_Status_Evaluator <br>
    │   └── Session_Manager <br>
    ├── Display <br>
    │   ├── OLED <br>
    │   │   ├── I2C_OLED_Driver <br>
    │   │   ├── Font_Library <br>
    │   │   └── Display_Buffer <br>
    │   ├── LED_Controller <br>
    │   └── Buzzer <br>
    │       └── PWM_Controller <br>
    └── Storage <br>
        ├── Flash_Driver <br>
        ├── BRAM_Buffer <br>
        └── Storage_Manager <br>

- the structure may change slightly.
- "top" can be omitted.        

## how to commit a module
1. Clone the repository to your PC.
2. Create module folders. (e.g. Display/OLED/I2C_OLED_Driver)
3. Create /src, /sim folders.
4. Put your sources, testbenches and bit file to the module's folder. <br>

## an ideal folder structure
  Display <br>
    └── OLED <br>
        └── I2C_OLED_Driver <br>
            ├── <bit_file> <br>
            ├── src <br>
                  └── …… （your source files) <br>
            └── sim <br>
                  └── ……  (your testbenches) <br>

## workflow
- We have two main branches: main & develop.
- Only stable version that has been verified can be pushed to main.
- Push to develop during your development. After verification, merge it to main.
- This is for the sake of safety.
