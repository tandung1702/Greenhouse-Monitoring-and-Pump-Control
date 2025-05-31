#include <LPC17xx.h>
#include "cmsis_os.h"
#include "Board_LED.h"
#include "Board_GLCD.h"
#include <stdio.h>
#include <string.h>

#define JOYSTICK_UP_PIN     (1 << 23)  // P1.23
#define JOYSTICK_DOWN_PIN   (1 << 25)  // P1.25
#define JOYSTICK_CENTER_PIN (1 << 20)  // P1.20
#define JOYSTICK_LEFT_PIN   (1 << 24)  // P1.24
#define JOYSTICK_RIGHT_PIN  (1 << 26)  // P1.26

extern GLCD_FONT GLCD_Font_16x24;

#define White 0xFFFFFF
#define Black 0x000000
#define Blue 0x0000FF
#define Red 0xFF0000
#define Green 0x00FF00

// Function Prototypes
void ADC_Init(void);
void GPIO_Init(void);
void UART0_Init(void);
void UART0_SendString(const char *str);
void Sensor_Thread(const void *arg);
void UART_Thread(const void *arg);
void HeaterMonitor_Thread(const void *arg);
void HeaterControl_Thread(const void *arg);
void SprinklerMonitor_Thread(const void *arg);
void SprinklerControl_Thread(const void *arg);
void LightMonitor_Thread(const void *arg);
void LightControl_Thread(const void *arg);
void UART_ReceiveThread(const void *arg);
void Menu_Thread(const void *arg);
void Menu_Display(int prev_selected_menu);
void actuator_control(void);
void show_sensors_on_glcd(void);
void toggle_gpio(int actuator);
int Read_ADC(int channel);
void adjustHeaterThreshold(void);
void adjustSprinklerThreshold(void);
void adjustLightThreshold(void);

// Global Variables
osMutexId adc_mutex;
osMutexId glcd_mutex;
osSemaphoreId heater_sem;
osSemaphoreId sprinkler_sem;
osSemaphoreId light_sem;

volatile int temp_adc, moist_adc, light_adc;
volatile int threadHoldtemp_adc = 1600;   // Heater threshold
volatile int threadHoldmoist_adc = 5000;  // Sprinkler threshold
volatile int threadHoldlight_adc = 4091;  // Light threshold

volatile int Heater_ON_Duration = 500;   // Heater ON Duration
volatile int sprinkler_ON_Duration = 600;  // Sprinkler ON Duration
volatile int light_ON_Duration = 700;  // Light ON Duration

int selected_menu = 0;

uint32_t lastJoystickState = 0;
uint32_t lastActionTime = 0;
#define DEBOUNCE_TIME 100

void ADC_Init(void) {
    LPC_PINCON->PINSEL1 |= (1 << 14) | (1 << 16) | (1 << 18); // Configure AD0.0–0.2 (P0.23–25)
    LPC_SC->PCONP |= (1 << 12); // Enable ADC power
    LPC_ADC->ADCR = (7 << 0) | (4 << 8) | (1 << 21); // Enable 3 channels, set clock division, power ON
}

void GPIO_Init(void) {
    LPC_GPIO1->FIODIR |= (1 << 29) | (1 << 31) | (1 << 28); // Set P1.29 (heater), P1.31 (sprinkler), P1.28 (status LED) as outputs
    LPC_GPIO2->FIODIR |= (1 << 2); // Set P2.2 (light) as output
}

void GPIO_Joystick_Init(void) {
    LPC_PINCON->PINSEL3 &= ~((3 << 14) | (3 << 18) | (3 << 8) | (3 << 16) | (3 << 20)); // Set P1.23, P1.24, P1.25, P1.26, P1.20 to GPIO
    LPC_PINCON->PINMODE3 &= ~((3 << 14) | (3 << 18) | (3 << 8) | (3 << 16) | (3 << 20)); // Enable pull-up resistors
    LPC_GPIO1->FIODIR &= ~(JOYSTICK_UP_PIN | JOYSTICK_DOWN_PIN | JOYSTICK_CENTER_PIN | JOYSTICK_LEFT_PIN | JOYSTICK_RIGHT_PIN); // Set as input
}

uint32_t readJoystick(void) {
    uint32_t state = 0;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_UP_PIN))     state |= 0x01;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_DOWN_PIN))   state |= 0x02;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_CENTER_PIN)) state |= 0x04;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_LEFT_PIN))   state |= 0x08;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_RIGHT_PIN))  state |= 0x10;
    return state;
}

void UART0_Init(void) {
    LPC_SC->PCONP |= (1 << 3); // Power up UART0
    LPC_PINCON->PINSEL0 |= (1 << 4) | (1 << 6); // Configure P0.2 as TXD0 and P0.3 as RXD0
    LPC_UART0->LCR = 0x83; // 8 bits, 1 stop bit, enable DLAB
    LPC_UART0->DLM = 0; LPC_UART0->DLL = 97; // Set baud rate to 9600
    LPC_UART0->LCR = 0x03; // 8 bits, 1 stop bit, no parity
}

void UART0_SendString(const char *str) {
    while (*str) {
        while (!(LPC_UART0->LSR & (1 << 5))); // Wait for TX ready
        LPC_UART0->THR = *str++; // Send character
    }
}

int Read_ADC(int channel) {
    LPC_ADC->ADCR &= ~(0x7 << 0); // Clear channel selection
    LPC_ADC->ADCR |= (1 << channel); // Select the specified channel
    LPC_ADC->ADCR |= (1 << 24); // Start conversion
    while (!(LPC_ADC->ADGDR & (1U << 31))); // Wait for conversion to complete
    int result = (LPC_ADC->ADGDR >> 4) & 0xFFF; // Extract 12-bit ADC result
    return result;
}

void Sensor_Thread(const void *arg) {
    while (1) {
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex for safe access
        temp_adc = Read_ADC(0); // Read temperature sensor
        moist_adc = Read_ADC(1); // Read moisture sensor
        light_adc = Read_ADC(2); // Read light sensor
        osMutexRelease(adc_mutex); // Release mutex
        osDelay(1000); // Delay for 1 second
    }
}

void UART_Thread(const void *arg) {
    char buffer[64]; // Buffer to hold the formatted string
    while (1) {
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex
        sprintf(buffer, "TEMP:%d|MOIST:%d|LIGHT:%d\n", temp_adc, moist_adc, light_adc); // Format data
        osMutexRelease(adc_mutex); // Release mutex
        UART0_SendString(buffer); // Send string over UART
        osDelay(3000); // Delay for 3 seconds
    }
}

void HeaterMonitor_Thread(const void *arg) {
    while (1) {
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex
        if (temp_adc >= threadHoldtemp_adc) {
            osSemaphoreRelease(heater_sem); // Signal to activate heater
        }
        osMutexRelease(adc_mutex); // Release mutex
        osDelay(1000); // Check every 1 second
    }
}

void HeaterControl_Thread(const void *arg) {
    while (1) {
        osSemaphoreWait(heater_sem, osWaitForever); // Wait for semaphore
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex
        if (temp_adc >= threadHoldtemp_adc) {
            LPC_GPIO1->FIOSET = (1 << 29); // Turn on heater (P1.29)
            osDelay(Heater_ON_Duration); // Keep on for 5 seconds
            LPC_GPIO1->FIOCLR = (1 << 29); // Turn off heater
        }
        osMutexRelease(adc_mutex); // Release mutex
    }
}


void SprinklerMonitor_Thread(const void *arg) {
    while (1) {
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex
        if (moist_adc <= threadHoldmoist_adc) { // Lower moisture means drier
            osSemaphoreRelease(sprinkler_sem); // Signal to activate sprinkler
        }
        osMutexRelease(adc_mutex); // Release mutex
        osDelay(1000); // Check every 1 second
    }
}

void SprinklerControl_Thread(const void *arg) {
    while (1) {
        osSemaphoreWait(sprinkler_sem, osWaitForever); // Wait for semaphore
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex
        if (moist_adc <= threadHoldmoist_adc) {
            LPC_GPIO1->FIOSET = (1 << 31); // Turn on sprinkler (P1.31)
            osDelay(sprinkler_ON_Duration); // Keep on for 5 seconds
            LPC_GPIO1->FIOCLR = (1 << 31); // Turn off sprinkler
        }
        osMutexRelease(adc_mutex); // Release mutex
    }
}

void LightMonitor_Thread(const void *arg) {
    while (1) {
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex
        if (light_adc <= threadHoldlight_adc) { // Lower light means darker
            osSemaphoreRelease(light_sem); // Signal to activate light
        }
        osMutexRelease(adc_mutex); // Release mutex
        osDelay(1000); // Check every 1 second
    }
}

void LightControl_Thread(const void *arg) {
    while (1) {
        osSemaphoreWait(light_sem, osWaitForever); // Wait for semaphore
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex
        if (light_adc <= threadHoldlight_adc) {
            LPC_GPIO2->FIOSET = (1 << 2); // Turn on light (P2.2)
            osDelay(light_ON_Duration); // Keep on for 5 seconds
            LPC_GPIO2->FIOCLR = (1 << 2); // Turn off light
        }
        osMutexRelease(adc_mutex); // Release mutex
    }
}

void UART_ReceiveThread(const void *arg) {
    char buffer[64];
    int idx = 0; // Index for the buffer
    while (1) {
        if (LPC_UART0->LSR & 0x01) { // Check if data is available
            char c = LPC_UART0->RBR; // Read received byte
            if (c == '\n' || idx >= 63) { // End of command or buffer full
                buffer[idx] = '\0'; idx = 0; // Null-terminate the string and reset index
                if (strncmp(buffer, "CMD:", 4) == 0) { // Check for command prefix
                    if (strstr(buffer, "HEATER:ON")) LPC_GPIO1->FIOSET = (1 << 29); // Turn on heater
                    if (strstr(buffer, "HEATER:OFF")) LPC_GPIO1->FIOCLR = (1 << 29); // Turn off heater
                    if (strstr(buffer, "LIGHT:ON")) LPC_GPIO2->FIOSET = (1 << 2); // Turn on light
                    if (strstr(buffer, "LIGHT:OFF")) LPC_GPIO2->FIOCLR = (1 << 2); // Turn off light
                    if (strstr(buffer, "SPRINKLER:ON")) LPC_GPIO1->FIOSET = (1 << 31); // Turn on sprinkler
                    if (strstr(buffer, "SPRINKLER:OFF")) LPC_GPIO1->FIOCLR = (1 << 31); // Turn off sprinkler
                }
            } else {
                buffer[idx++] = c; // Store received character
            }
        }
        osDelay(10); // Short delay to prevent busy waiting
    }
}

void Menu_Display(int prev_selected_menu) {
    static int first_call = 1; // Flag to check if it's the first call
    const char *menu_items[] = {
        "Show Sensors Data",
        "Manual Control",
        "Adjust Heater Thresh",
        "Adjust Sprinkler Thresh",
        "Adjust Light Thresh",
        "Exit Menu"
    };

    osMutexWait(glcd_mutex, osWaitForever); // Acquire GLCD mutex

    if (first_call || prev_selected_menu == -1) {
        // Initial full display or forced redraw
        GLCD_SetBackgroundColor(White);
        GLCD_ClearScreen();
        GLCD_SetForegroundColor(Blue);
        GLCD_DrawString(0, 0, "Greenhouse Menu"); // Print title
        for (int i = 0; i < 6; i++) {
            char displayText[35];
            sprintf(displayText, i == selected_menu ? "> %s" : "%s", menu_items[i]);
            GLCD_SetBackgroundColor(i == selected_menu ? 0xC0C0C0 : White); // Gray for selected, white for others
            GLCD_DrawString(0, (i + 2) * 24, displayText);
        }
        first_call = 0; // Mark as initialized
    } else if (prev_selected_menu != selected_menu) {
        // Update only the changed menu items
        char displayText[35];
        // Clear and redraw previous selected item to remove highlight
        GLCD_SetBackgroundColor(White);
        GLCD_SetForegroundColor(Blue);
        GLCD_DrawString(0, (prev_selected_menu + 2) * 24, "                    "); // Clear line
        sprintf(displayText, "%s", menu_items[prev_selected_menu]); // Remove '>'
        GLCD_DrawString(0, (prev_selected_menu + 2) * 24, displayText);
        // Clear and draw new selected item with highlight
        GLCD_SetBackgroundColor(0xC0C0C0); // Gray for selected
        GLCD_DrawString(0, (selected_menu + 2) * 24, "                    "); // Clear line
        sprintf(displayText, "> %s", menu_items[selected_menu]); // Add '>'
        GLCD_DrawString(0, (selected_menu + 2) * 24, displayText);
    }

    GLCD_SetBackgroundColor(White); // Reset background to white
    osMutexRelease(glcd_mutex); // Release GLCD mutex
}

void actuator_control(void) {
    static int first_call = 1; // Flag for initial draw
    int selected_actuator = 0; // 0: Heater, 1: Sprinkler, 2: Light
    int prev_selected_actuator = -1; // Track previous selection
    uint32_t current_joystick_state = 0;
    uint32_t prev_joystick_state = 0;
    uint32_t last_action_time = 0;

    // Initial full display
    if (first_call) {
        osMutexWait(glcd_mutex, osWaitForever);
        GLCD_SetBackgroundColor(White);
        GLCD_ClearScreen();
        GLCD_SetForegroundColor(Blue);
        GLCD_DrawString(0, 0 * 24, "Actuators Control");
        // Draw all actuators initially
        const char *actuator_names[] = {"Heater", "Sprinkler", "Light"};
        for (int i = 0; i < 3; i++) {
            char displayText[35];
            GLCD_SetBackgroundColor(i == selected_actuator ? 0xC0C0C0 : White);
            GLCD_DrawString(0, (i + 2) * 24, "                    "); // Clear line
            sprintf(displayText, i == selected_actuator ? "> %s: %s" : "%s: %s",
                    actuator_names[i],
                    i == 0 ? (LPC_GPIO1->FIOPIN & (1 << 29) ? "ON" : "OFF") :
                    i == 1 ? (LPC_GPIO1->FIOPIN & (1 << 31) ? "ON" : "OFF") :
                             (LPC_GPIO2->FIOPIN & (1 << 2) ? "ON" : "OFF"));
            GLCD_SetForegroundColor(Blue);
            GLCD_DrawString(0, (i + 2) * 24, displayText);
        }
        GLCD_SetForegroundColor(Red);
        GLCD_DrawString(0, 6 * 24, "Up/Dn:Select L/R:ON/OFF");
        GLCD_DrawString(0, 7 * 24, "Center:Exit");
        osMutexRelease(glcd_mutex);
        first_call = 0;
    }

    while (1) {
        current_joystick_state = readJoystick();
        uint32_t currentTime = osKernelSysTick();

        // Only process input if debounce time has passed
        if (currentTime - last_action_time >= DEBOUNCE_TIME) {
            int update_display = 0;
            int toggled = 0;

            if ((current_joystick_state & 0x01) && !(prev_joystick_state & 0x01)) { // Up pressed
                prev_selected_actuator = selected_actuator;
                selected_actuator = (selected_actuator > 0) ? selected_actuator - 1 : 0;
                update_display = 1;
                last_action_time = currentTime;
            }
            if ((current_joystick_state & 0x02) && !(prev_joystick_state & 0x02)) { // Down pressed
                prev_selected_actuator = selected_actuator;
                selected_actuator = (selected_actuator < 2) ? selected_actuator + 1 : 2;
                update_display = 1;
                last_action_time = currentTime;
            }
            if ((current_joystick_state & (0x08 | 0x10)) && !(prev_joystick_state & (0x08 | 0x10))) { // Left or Right pressed
                toggle_gpio(selected_actuator); // Toggle selected actuator
                update_display = 1;
                toggled = 1;
                last_action_time = currentTime;
            }
            if ((current_joystick_state & 0x04) && !(prev_joystick_state & 0x04)) { // Center pressed
                first_call = 1; // Reset for next entry
                last_action_time = currentTime;
                break;
            }

            // Update display only if needed
            if (update_display) {
                osMutexWait(glcd_mutex, osWaitForever);
                const char *actuator_names[] = {"Heater", "Sprinkler", "Light"};
                char displayText[35];

                if (toggled) {
                    // Update only the toggled actuator
                    GLCD_SetBackgroundColor(0xC0C0C0); // Selected
                    GLCD_DrawString(0, (selected_actuator + 2) * 24, "                    ");
                    sprintf(displayText, "> %s: %s", actuator_names[selected_actuator],
                            selected_actuator == 0 ? (LPC_GPIO1->FIOPIN & (1 << 29) ? "ON" : "OFF") :
                            selected_actuator == 1 ? (LPC_GPIO1->FIOPIN & (1 << 31) ? "ON" : "OFF") :
                                                     (LPC_GPIO2->FIOPIN & (1 << 2) ? "ON" : "OFF"));
                    GLCD_SetForegroundColor(Blue);
                    GLCD_DrawString(0, (selected_actuator + 2) * 24, displayText);
                } else if (prev_selected_actuator != selected_actuator) {
                    // Update previous and current actuators
                    if (prev_selected_actuator != -1) {
                        GLCD_SetBackgroundColor(White);
                        GLCD_DrawString(0, (prev_selected_actuator + 2) * 24, "                    ");
                        sprintf(displayText, "%s: %s", actuator_names[prev_selected_actuator],
                                prev_selected_actuator == 0 ? (LPC_GPIO1->FIOPIN & (1 << 29) ? "ON" : "OFF") :
                                prev_selected_actuator == 1 ? (LPC_GPIO1->FIOPIN & (1 << 31) ? "ON" : "OFF") :
                                                             (LPC_GPIO2->FIOPIN & (1 << 2) ? "ON" : "OFF"));
                        GLCD_SetForegroundColor(Blue);
                        GLCD_DrawString(0, (prev_selected_actuator + 2) * 24, displayText);
                    }
                    GLCD_SetBackgroundColor(0xC0C0C0);
                    GLCD_DrawString(0, (selected_actuator + 2) * 24, "                    ");
                    sprintf(displayText, "> %s: %s", actuator_names[selected_actuator],
                            selected_actuator == 0 ? (LPC_GPIO1->FIOPIN & (1 << 29) ? "ON" : "OFF") :
                            selected_actuator == 1 ? (LPC_GPIO1->FIOPIN & (1 << 31) ? "ON" : "OFF") :
                                                     (LPC_GPIO2->FIOPIN & (1 << 2) ? "ON" : "OFF"));
                    GLCD_SetForegroundColor(Blue);
                    GLCD_DrawString(0, (selected_actuator + 2) * 24, displayText);
                }
                osMutexRelease(glcd_mutex);
            }
        }
        prev_joystick_state = current_joystick_state;
        osDelay(20); // Maintain responsiveness
    }

    // Return to the menu
    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    Menu_Display(-1); // Force full redraw on return
}

void show_sensors_on_glcd(void) {
    char line1[32];
    char line2[32];
    char line3[32];
    char header[32];
    char back[32];
    
    uint32_t current_joystick_state;
    uint32_t prev_joystick_state = 0;
    uint32_t last_action_time = 0;
    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);    

    while (1) {
        osMutexWait(glcd_mutex, osWaitForever);
        GLCD_SetBackgroundColor(White);
        GLCD_SetForegroundColor(Black);
        GLCD_DrawString(0, 3 * 24, "                    ");
        
        osMutexWait(adc_mutex, osWaitForever); // Acquire mutex
        sprintf(header, "Display sensor Data");
        sprintf(back, "Press center to return");
        sprintf(line1, "Temp: %d", temp_adc);
        sprintf(line2, "Moist %d", moist_adc);
        sprintf(line3, "Ligh %d", light_adc);
        osMutexRelease(adc_mutex); // Release mutex
        GLCD_SetForegroundColor(Blue);
        GLCD_DrawString(0, 24, header);
        GLCD_SetForegroundColor(Black);
        GLCD_DrawString(0, 4 * 24, line1);
        GLCD_DrawString(0, (4 * 24) + 24, line2);
        GLCD_DrawString(0, (4 * 24) + 48, line3);
        GLCD_SetForegroundColor(Red);
        GLCD_DrawString(0, (4 * 24) + 48 + 24, back);
        osMutexRelease(glcd_mutex);

        current_joystick_state = readJoystick();
        uint32_t currentTime = osKernelSysTick();

        if (currentTime - last_action_time >= DEBOUNCE_TIME) {
            if ((current_joystick_state & 0x04) && !(prev_joystick_state & 0x04)) {
                last_action_time = currentTime;
                break;
            }
        }
        prev_joystick_state = current_joystick_state;
        osDelay(20);
    }
    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    Menu_Display(-1); // Force full redraw on return
}

void toggle_gpio(int actuator) {
    switch (actuator) {
        case 0: // Heater
            if (LPC_GPIO1->FIOPIN & (1 << 29)) {
                LPC_GPIO1->FIOCLR = (1 << 29); // Turn off heater
            } else {
                LPC_GPIO1->FIOSET = (1 << 29); // Turn on heater
            }
            break;
        case 1: // Sprinkler
            if (LPC_GPIO1->FIOPIN & (1 << 31)) {
                LPC_GPIO1->FIOCLR = (1 << 31); // Turn off sprinkler
            } else {
                LPC_GPIO1->FIOSET = (1 << 31); // Turn on sprinkler
            }
            break;
        case 2: // Light
            if (LPC_GPIO2->FIOPIN & (1 << 2)) {
                LPC_GPIO2->FIOCLR = (1 << 2); // Turn off light
            } else {
                LPC_GPIO2->FIOSET = (1 << 2); // Turn on light
            }
            break;
    }
}

void adjustHeaterThreshold(void) {
    char thresholdString[20];
    uint32_t current_joystick_state;
    uint32_t prev_joystick_state = 0;
    uint32_t last_action_time = 0;

    osMutexWait(glcd_mutex, osWaitForever); // Use glcd_mutex for GLCD consistency
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    GLCD_SetForegroundColor(Blue);
    GLCD_DrawString(0, 0 * 24, "Set Heater Threshold");
    GLCD_DrawString(0, 2 * 24, "Use joystick Up/Down");
    GLCD_DrawString(0, 3 * 24, "to adjust value");
    GLCD_SetForegroundColor(Red);
    GLCD_DrawString(0, 5 * 24, "Center to confirm");
    GLCD_SetForegroundColor(Black);
    sprintf(thresholdString, "Threshold: %d", threadHoldtemp_adc);
    GLCD_DrawString(0, 4 * 24, thresholdString);
    osMutexRelease(glcd_mutex);

    while (1) {
        current_joystick_state = readJoystick();
        uint32_t currentTime = osKernelSysTick();

        if (currentTime - last_action_time >= DEBOUNCE_TIME) {
            if ((current_joystick_state & 0x01) && !(prev_joystick_state & 0x01)) { // Up pressed
                threadHoldtemp_adc += 10; // Increment by 10 for noticeable change
                if (threadHoldtemp_adc > 4095) threadHoldtemp_adc = 4095; // Max ADC value
                last_action_time = currentTime;
                osMutexWait(glcd_mutex, osWaitForever);
                GLCD_SetBackgroundColor(White);
                GLCD_SetForegroundColor(Black);
                GLCD_DrawString(0, 4 * 24, "                    "); // Clear previous value
                sprintf(thresholdString, "Threshold: %d", threadHoldtemp_adc);
                GLCD_DrawString(0, 4 * 24, thresholdString);
                osMutexRelease(glcd_mutex);
            }
            if ((current_joystick_state & 0x02) && !(prev_joystick_state & 0x02)) { // Down pressed
                threadHoldtemp_adc -= 10; // Decrement by 10
                if (threadHoldtemp_adc < 0) threadHoldtemp_adc = 0; // Min ADC value
                last_action_time = currentTime;
                osMutexWait(glcd_mutex, osWaitForever);
                GLCD_SetBackgroundColor(White);
                GLCD_SetForegroundColor(Black);
                GLCD_DrawString(0, 4 * 24, "                    "); // Clear previous value
                sprintf(thresholdString, "Threshold: %d", threadHoldtemp_adc);
                GLCD_DrawString(0, 4 * 24, thresholdString);
                osMutexRelease(glcd_mutex);
            }
            if ((current_joystick_state & 0x04) && !(prev_joystick_state & 0x04)) { // Center pressed
                last_action_time = currentTime;
                break;
            }
        }
        prev_joystick_state = current_joystick_state;
        osDelay(20);
    }

    // Return to the menu
    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    Menu_Display(-1); // Force full redraw on return
}

void adjustSprinklerThreshold(void) {
    char thresholdString[20];
    uint32_t current_joystick_state;
    uint32_t prev_joystick_state = 0;
    uint32_t last_action_time = 0;

    osMutexWait(glcd_mutex, osWaitForever); // Use glcd_mutex for GLCD consistency
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    GLCD_SetForegroundColor(Blue);
    GLCD_DrawString(0, 0 * 24, "Set Sprinkler Threshold");
    GLCD_DrawString(0, 2 * 24, "Use joystick Up/Down");
    GLCD_DrawString(0, 3 * 24, "to adjust value");
    GLCD_SetForegroundColor(Red);
    GLCD_DrawString(0, 5 * 24, "Center to confirm");
    GLCD_SetForegroundColor(Black);
    sprintf(thresholdString, "Threshold: %d", threadHoldmoist_adc);
    GLCD_DrawString(0, 4 * 24, thresholdString);
    osMutexRelease(glcd_mutex);

    while (1) {
        current_joystick_state = readJoystick();
        uint32_t currentTime = osKernelSysTick();

        if (currentTime - last_action_time >= DEBOUNCE_TIME) {
            if ((current_joystick_state & 0x01) && !(prev_joystick_state & 0x01)) { // Up pressed
                threadHoldmoist_adc += 10; // Increment by 10 for noticeable change
                if (threadHoldmoist_adc > 4095) threadHoldmoist_adc = 4095; // Max ADC value
                last_action_time = currentTime;
                osMutexWait(glcd_mutex, osWaitForever);
                GLCD_SetBackgroundColor(White);
                GLCD_SetForegroundColor(Black);
                GLCD_DrawString(0, 4 * 24, "                    "); // Clear previous value
                sprintf(thresholdString, "Threshold: %d", threadHoldmoist_adc);
                GLCD_DrawString(0, 4 * 24, thresholdString);
                osMutexRelease(glcd_mutex);
            }
            if ((current_joystick_state & 0x02) && !(prev_joystick_state & 0x02)) { // Down pressed
                threadHoldmoist_adc -= 10; // Decrement by 10
                if (threadHoldmoist_adc < 0) threadHoldmoist_adc = 0; // Min ADC value
                last_action_time = currentTime;
                osMutexWait(glcd_mutex, osWaitForever);
                GLCD_SetBackgroundColor(White);
                GLCD_SetForegroundColor(Black);
                GLCD_DrawString(0, 4 * 24, "                    "); // Clear previous value
                sprintf(thresholdString, "Threshold: %d", threadHoldmoist_adc);
                GLCD_DrawString(0, 4 * 24, thresholdString);
                osMutexRelease(glcd_mutex);
            }
            if ((current_joystick_state & 0x04) && !(prev_joystick_state & 0x04)) { // Center pressed
                last_action_time = currentTime;
                break;
            }
        }
        prev_joystick_state = current_joystick_state;
        osDelay(20);
    }

    // Return to the menu
    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    Menu_Display(-1); // Force full redraw on return
}

void adjustLightThreshold(void) {
    char thresholdString[20];
    uint32_t current_joystick_state;
    uint32_t prev_joystick_state = 0;
    uint32_t last_action_time = 0;

    osMutexWait(glcd_mutex, osWaitForever); // Use glcd_mutex for GLCD consistency
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    GLCD_SetForegroundColor(Blue);
    GLCD_DrawString(0, 0 * 24, "Set Light Threshold");
    GLCD_DrawString(0, 2 * 24, "Use joystick Up/Down");
    GLCD_DrawString(0, 3 * 24, "to adjust value");
    GLCD_SetForegroundColor(Red);
    GLCD_DrawString(0, 5 * 24, "Center to confirm");
    GLCD_SetForegroundColor(Black);
    sprintf(thresholdString, "Threshold: %d", threadHoldlight_adc);
    GLCD_DrawString(0, 4 * 24, thresholdString);
    osMutexRelease(glcd_mutex);

    while (1) {
        current_joystick_state = readJoystick();
        uint32_t currentTime = osKernelSysTick();

        if (currentTime - last_action_time >= DEBOUNCE_TIME) {
            if ((current_joystick_state & 0x01) && !(prev_joystick_state & 0x01)) { // Up pressed
                threadHoldlight_adc += 10; // Increment by 10 for noticeable change
                if (threadHoldlight_adc > 4095) threadHoldlight_adc = 4095; // Max ADC value
                last_action_time = currentTime;
                osMutexWait(glcd_mutex, osWaitForever);
                GLCD_SetBackgroundColor(White);
                GLCD_SetForegroundColor(Black);
                GLCD_DrawString(0, 4 * 24, "                    "); // Clear previous value
                sprintf(thresholdString, "Threshold: %d", threadHoldlight_adc);
                GLCD_DrawString(0, 4 * 24, thresholdString);
                osMutexRelease(glcd_mutex);
            }
            if ((current_joystick_state & 0x02) && !(prev_joystick_state & 0x02)) { // Down pressed
                threadHoldlight_adc -= 10; // Decrement by 10
                if (threadHoldlight_adc < 0) threadHoldlight_adc = 0; // Min ADC value
                last_action_time = currentTime;
                osMutexWait(glcd_mutex, osWaitForever);
                GLCD_SetBackgroundColor(White);
                GLCD_SetForegroundColor(Black);
                GLCD_DrawString(0, 4 * 24, "                    "); // Clear previous value
                sprintf(thresholdString, "Threshold: %d", threadHoldlight_adc);
                GLCD_DrawString(0, 4 * 24, thresholdString);
                osMutexRelease(glcd_mutex);
            }
            if ((current_joystick_state & 0x04) && !(prev_joystick_state & 0x04)) { // Center pressed
                last_action_time = currentTime;
                break;
            }
        }
        prev_joystick_state = current_joystick_state;
        osDelay(20);
    }

    // Return to the menu
    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(White);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    Menu_Display(-1); // Force full redraw on return
}

void Menu_Thread(const void *arg) {
    GPIO_Joystick_Init(); // Initialize joystick
    GLCD_Initialize();    // Initialize GLCD
    GLCD_SetFont(&GLCD_Font_16x24); // Set font
    Menu_Display(-1);     // Initial full menu display

    int last_menu = selected_menu;
    while (1) {
        int dir = readJoystick();
        int updated = 0;

        if (dir == 0) {
            // No button pressed
            osDelay(100);
            continue;
        }

        if (dir & 0x01 && selected_menu > 0) { // Up
            selected_menu--;
            updated = 1;
        } else if (dir & 0x02 && selected_menu < 5) { // Down (max 5 for 6 items)
            selected_menu++;
            updated = 1;
        } else if (dir & 0x04) { // Center pressed
            switch (selected_menu) {
                case 0: // Show Sensors
                    show_sensors_on_glcd();
                    break; // Full redraw handled in show_sensors_on_glcd
                case 1: // Actuator Control
                    actuator_control();
                    break; // Full redraw handled in actuator_control
                case 2: // Adjust Heater Threshold
                    adjustHeaterThreshold();
                    break; // Full redraw handled in adjustHeaterThreshold
                case 3: // Adjust Sprinkler Threshold
                    adjustSprinklerThreshold();
                    break; // Full redraw handled in adjustSprinklerThreshold
                case 4: // Adjust Light Threshold
                    adjustLightThreshold();
                    break; // Full redraw handled in adjustLightThreshold
                case 5: // Exit Menu
                    break;
                default: 
                    break;								
            }
        }

        if (updated || selected_menu != last_menu) {
            Menu_Display(last_menu);
            last_menu = selected_menu;
        }

        osDelay(250);
    }
}

osThreadDef(Sensor_Thread, osPriorityNormal, 1, 0);
osThreadDef(UART_Thread, osPriorityNormal, 1, 0);
osThreadDef(HeaterMonitor_Thread, osPriorityNormal, 1, 0);
osThreadDef(HeaterControl_Thread, osPriorityNormal, 1, 0);
osThreadDef(SprinklerMonitor_Thread, osPriorityNormal, 1, 0);
osThreadDef(SprinklerControl_Thread, osPriorityNormal, 1, 0);
osThreadDef(LightMonitor_Thread, osPriorityNormal, 1, 0);
osThreadDef(LightControl_Thread, osPriorityNormal, 1, 0);
osThreadDef(UART_ReceiveThread, osPriorityNormal, 1, 0);
osThreadDef(Menu_Thread, osPriorityNormal, 1, 0);

osMutexDef(adc_mutex);
osMutexDef(glcd_mutex); // Define glcd_mutex
osSemaphoreDef(heater_sem);
osSemaphoreDef(sprinkler_sem);
osSemaphoreDef(light_sem);

int main(void) {
    SystemCoreClockUpdate(); // Update the system clock frequency
    ADC_Init(); // Initialize ADC
    GPIO_Init(); // Initialize GPIO
    UART0_Init(); // Initialize UART
    
    osKernelInitialize(); // Initialize the RTX kernel
   
    adc_mutex = osMutexCreate(osMutex(adc_mutex));
    glcd_mutex = osMutexCreate(osMutex(glcd_mutex)); // Create glcd_mutex
    heater_sem = osSemaphoreCreate(osSemaphore(heater_sem), 1);
    sprinkler_sem = osSemaphoreCreate(osSemaphore(sprinkler_sem), 1);
    light_sem = osSemaphoreCreate(osSemaphore(light_sem), 1);
    
    // Create threads for each function
    osThreadCreate(osThread(Sensor_Thread), NULL);
    osThreadCreate(osThread(UART_Thread), NULL);
    osThreadCreate(osThread(HeaterMonitor_Thread), NULL);
    osThreadCreate(osThread(HeaterControl_Thread), NULL);
    osThreadCreate(osThread(SprinklerMonitor_Thread), NULL);
    osThreadCreate(osThread(SprinklerControl_Thread), NULL);
    osThreadCreate(osThread(LightMonitor_Thread), NULL);
    osThreadCreate(osThread(LightControl_Thread), NULL);
    osThreadCreate(osThread(UART_ReceiveThread), NULL);
    osThreadCreate(osThread(Menu_Thread), NULL);
    
    osKernelStart(); // Start the RTOS kernel
    
    while (1); // Infinite loop to keep the main function alive
}