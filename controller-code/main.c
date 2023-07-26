#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define BTN_LED_REG_DDR  DDRA   // Register for data direction of buttons/LEDs
#define BTN_LED_REG_PORT PORTA  // Register for setting pull-ups/output state of buttons/LEDs
#define BTN_REG_PIN      PINA   // Register for reading state of buttons
#define BTN_REG_INT      PCMSK0 // Register for button pin-change interrupts
#define BTN_REG_FLG      GIFR   // Register for button pin-change interrupt flags
#define BTN_INT_MSK      PCIE0  // Mask bit for button pin-change interrupts
#define BTN_INT_FLG      PCIF0  // Flag bit for button pin-change interrupts

#define BTN_SIZE_PIN PORTA5
#define BTN_SIZE_INT PCINT5
#define BTN_ARM_PIN PORTA6
#define BTN_ARM_INT PCINT6
#define BTN_PURGE_PIN PORTA7
#define BTN_PURGE_INT PCINT7
#define BTN_MASK ((1 << BTN_SIZE_PIN) | (1 << BTN_ARM_PIN) | (1 << BTN_PURGE_PIN))

#define TIMER_DEBOUNCE_REG_CNT TCNT0
#define TIMER_DEBOUNCE_REG_CMP OCR0A
#define TIMER_DEBOUNCE_REG_INT TIMSK0
#define TIMER_DEBOUNCE_REG_FLG TIFR0
#define TIMER_DEBOUNCE_INT_MSK OCIE0A
#define TIMER_DEBOUNCE_INT_FLG OCF0A
#define DEBOUNCE_TIME          40 // ~40ms debounce

#define LED_SIZE_SM_PIN PORTA0
#define LED_SIZE_MD_PIN PORTA1
#define LED_SIZE_LG_PIN PORTA2
#define LED_SIZE_XL_PIN PORTA3
#define LED_SIZE_PINS
#define LED_ARMED_PIN PORTA4

volatile uint8_t size = 0;

#pragma region Subroutines //////////////////////////////////////////////////
uint8_t debounce_button() {
    volatile uint8_t original_btn = BTN_REG_PIN & BTN_MASK;

    // Run timer to debounce
    TIMER_DEBOUNCE_REG_CMP = DEBOUNCE_TIME;                   // Set timer to go off at debounce time
    TIMER_DEBOUNCE_REG_CNT = 0;                               // Reset timer to 0
    TIMER_DEBOUNCE_REG_FLG |= (1 << TIMER_DEBOUNCE_INT_FLG);  // Clear existing output compare interrupt
    TIMER_DEBOUNCE_REG_INT |= (1 << TIMER_DEBOUNCE_INT_MSK);  // Enable output compare interrupt

    // Go to sleep until the timer clears
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();

    // --- SLEEP ---

    return (BTN_REG_PIN & BTN_MASK) == original_btn
        ? ~BTN_REG_PIN & BTN_MASK
        : 0;
}

void set_size_leds() {
    // Determine value for LED pins
    uint8_t led_port_value;
    switch (size) {
        case 3:
            led_port_value = (1 << LED_SIZE_SM_PIN)
                             | (1 << LED_SIZE_MD_PIN)
                             | (1 << LED_SIZE_LG_PIN)
                             | (1 << LED_SIZE_XL_PIN);
            break;
        case 2:
            led_port_value = (1 << LED_SIZE_SM_PIN)
                             | (1 << LED_SIZE_MD_PIN)
                             | (1 << LED_SIZE_LG_PIN);
            break;
        case 1:
            led_port_value = (1 << LED_SIZE_SM_PIN)
                             | (1 << LED_SIZE_MD_PIN);
            break;
        case 0:
        default:
            led_port_value = (1 << LED_SIZE_SM_PIN);
            break;
    }

    // Set size LED pins, leave other pins alone
    BTN_LED_REG_PORT = (
        BTN_LED_REG_PORT &
        ~(
            (1 << LED_SIZE_SM_PIN) |
            (1 << LED_SIZE_MD_PIN) |
            (1 << LED_SIZE_LG_PIN) |
            (1 << LED_SIZE_XL_PIN)
        )
    ) | led_port_value;
}
#pragma endregion

#pragma region ISRs /////////////////////////////////////////////////////////
ISR(PCINT0_vect) { // Button pin-chance ISR
    // Disable pin-change interrupts on buttons to prevent re-firing
    BTN_REG_INT = 0;
}

ISR(TIM0_COMPA_vect) { // Timer output comparison A ISR
    // Disable debounce timer to prevent re-firing
    TIMER_DEBOUNCE_REG_INT &= ~(1 << TIMER_DEBOUNCE_INT_FLG);
}
#pragma endregion ///////////////////////////////////////////////////////////

#pragma region MAIN /////////////////////////////////////////////////////////
#define STATE_SETUP 0
#define STATE_ARMED 1
#define STATE_DISPENSING 2
#define STATE_PURGING 3

void setup() {
    // Intialize GPIO
    BTN_LED_REG_DDR = (1 << LED_SIZE_SM_PIN) // Output small size LED
                    | (1 << LED_SIZE_MD_PIN) // Output medium size LED
                    | (1 << LED_SIZE_LG_PIN) // Output large size LED
                    | (1 << LED_SIZE_XL_PIN) // Output xtra size LED
                    | (1 << LED_ARMED_PIN)   // Output armed pin
                    | (0 << BTN_SIZE_PIN)    // Input size button
                    | (0 << BTN_ARM_PIN)     // Input arm button
                    | (0 << BTN_PURGE_PIN);  // Input purge button
    BTN_LED_REG_PORT |= (1 << BTN_SIZE_PIN)  // Pull up on buttons
                      | (1 << BTN_ARM_PIN)
                      | (1 << BTN_PURGE_PIN);
    GIMSK |= (1 << BTN_INT_FLG); // Enable pin-change interrupts on port A

    // Setup sleep mode and reduce power consumption
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    // Setup timer0 with 1024 prescaler in normal mode for use in debouncing
    TCCR0B |= (1 << CS00)  // Clock/1024 (bit 0)
            | (1 << CS02); // Clock/1024 (bit 2)

}

uint8_t state_setup() {
    while(1) {
        // Turn on LEDs for load size
        set_size_leds();

        // Enable pin-change interrupts on button pins and go to sleep
        BTN_REG_FLG |= (1 << BTN_INT_FLG);   // Clear existing pin-change interrupts
        BTN_REG_INT |= (1 << BTN_SIZE_INT)
                     | (1 << BTN_ARM_INT)
                     | (1 << BTN_PURGE_INT);
        sei();
        sleep_mode();

        // --- SLEEP ---

        // Awoken from sleep by pin change
        uint8_t button = debounce_button();
        switch (button) {
            case (1 << BTN_SIZE_PIN):
                size = (size + 1) % 4;
                break;
            case (1 << BTN_ARM_PIN):
                // @TODO
                break;
            case (1 << BTN_PURGE_PIN):
                // @TODO
                break;
            default:
                break;
        }
    }
}

uint8_t state_armed() {
    return STATE_SETUP;
}

uint8_t state_dispensing() {
    return STATE_SETUP;
}

uint8_t state_purging() {
    return STATE_SETUP;
}

int main() {
    setup();

    uint8_t state = 0;

    while(1) {
        switch (state) {
            case STATE_SETUP:
                state = state_setup();
                break;
            case STATE_ARMED:
                state = state_armed();
                break;
            case STATE_DISPENSING:
                state = state_dispensing();
                break;
            case STATE_PURGING:
                state = state_purging();
                break;
        }
    }
}

#pragma endregion ///////////////////////////////////////////////////////////
