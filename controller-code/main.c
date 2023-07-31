#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define BTN_LED_REG_DDR   DDRA   // Register for data direction of buttons/LEDs
#define BTN_LED_REG_PORT  PORTA  // Register for setting pull-ups/output state of buttons/LEDs
#define MTR_SNR_REG_DDR   DDRB   // Register for data direction of motor/sensor
#define MTR_SNR_REG_PORT  PORTB  // Register for setting pull-ups/output state of motor/sensor
#define PC_REG            GPIOR0 // Register for differentiating which port gave a pin-change register

#define BTN_REG_PIN       PINA   // Register for reading state of buttons
#define BTN_REG_INT       PCMSK0 // Register for button pin-change interrupts
#define BTN_GIMSK_MSK     PCIE0  // GIMSK bit for button pin-change interrupts
#define BTN_GIFR_FLG      PCIF0  // GIFR bit for button pin-change interrupts
#define BTN_SIZE_PIN      PORTA4
#define BTN_SIZE_INT_MSK  PCINT4
#define BTN_ARM_PIN       PORTA5
#define BTN_ARM_INT_MSK   PCINT5
#define BTN_PURGE_PIN     PORTA7
#define BTN_PURGE_INT_MSK PCINT7
#define BTN_MASK ((1 << BTN_SIZE_PIN) | (1 << BTN_ARM_PIN) | (1 << BTN_PURGE_PIN))

#define LED_SIZE_SM_PIN PORTA0
#define LED_SIZE_MD_PIN PORTA1
#define LED_SIZE_LG_PIN PORTA2
#define LED_SIZE_XL_PIN PORTA3
#define LED_ARMED_PIN PORTA6

#define MTR_PIN       PORTB0 // Pin/port for motor control

#define SNR_REG_PIN   PINB   // Register for reading state of sensor
#define SNR_REG_INT   PCMSK1 // Register for pin-change interrupt masks for sensor
#define SNR_GIMKS_MSK PCIE1  // GIMSK bit for sensor pin-change interrupts
#define SNR_GIFR_FLG  PCIF1  // GIFR bit for button pin-change interrupts
#define SNR_PIN       PORTB1 // Pin/port for sensor
#define SNR_INT_MSK   PCINT9 // Pin-change interrupt mask for sensor
#define SNR_MASK      (1 << SNR_PIN)

#define DEBOUNCE_TIME 40     // ~40ms @ clk/1024
#define PURGE_DELAY   7812.5 // ~0.5s @ clk/64

volatile uint8_t size = 0;

#pragma region Subroutines //////////////////////////////////////////////////
uint8_t debounce(volatile uint8_t* pin_register, uint8_t mask) {
    uint8_t original_pins = *pin_register & mask;

    // Run timer to debounce (clock/1024, normal mode)
    TCCR0B |= (1 << CS00)    // Clock/1024 (bit 0)
            | (1 << CS02);   // Clock/1024 (bit 2)
    OCR0A   = DEBOUNCE_TIME; // Set timer to go off at debounce time
    TCNT0   = 0;             // Reset timer to 0
    TIFR0  |= (1 << OCF0A);  // Clear any existing output compare interrupt
    TIMSK0 |= (1 << OCIE0A); // Enable output compare interrupt

    // Go to sleep until timer clears
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();

    // --- SLEEP ---

    // Turn off timer
    TCCR0A = 0;
    TCCR0B = 0;

    uint8_t current_pins = *pin_register & mask;
    return current_pins == original_pins ? current_pins : 0;
}

uint8_t debounce_button() {
    // Pins go low when set, so flip the debounced pin value
    uint8_t debounced = debounce(&BTN_REG_PIN, BTN_MASK);
    return debounced ^ BTN_MASK;
}

uint8_t debounce_sensor() {
    // Sensor goes high when set, so debounced value is already appropriately masked
    return debounce(&SNR_REG_PIN, SNR_MASK);
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
    const uint8_t all_leds = (1 << LED_SIZE_SM_PIN)
                           | (1 << LED_SIZE_MD_PIN)
                           | (1 << LED_SIZE_LG_PIN)
                           | (1 << LED_SIZE_XL_PIN);
    BTN_LED_REG_PORT = (BTN_LED_REG_PORT & ~all_leds) | led_port_value;
}
#pragma endregion

#pragma region ISRs /////////////////////////////////////////////////////////
ISR(PCINT0_vect) { // Button pin-chance ISR
    // Disable pin-change interrupts to prevent re-firing
    BTN_REG_INT = 0;
    SNR_REG_INT = 0;

    PC_REG = 0;
}

ISR(PCINT1_vect) { // Sensor pin-change ISR
    // Disable pin-change interrupts to prevent re-firing
    SNR_REG_INT = 0;
    BTN_REG_INT = 0;

    PC_REG = 1;
}

ISR(TIM0_COMPA_vect) { // Timer 0 output comparison A ISR
    // Disable timer 0 interrupt to prevent re-firing
    TIMSK0 = 0;
}

ISR(TIM1_COMPA_vect) { // Timer 1 output comparison A ISR
    // Disable timer 1 interrupt to prevent re-firing
    TIMSK1 = 0;
}
#pragma endregion ///////////////////////////////////////////////////////////

#pragma region MAIN /////////////////////////////////////////////////////////
#define STATE_SETUP 1
#define STATE_ARMED 2
#define STATE_DISPENSING 3
#define STATE_PURGING 4

void setup() {
    // Intialize GPIO
    BTN_LED_REG_DDR   = (1 << LED_SIZE_SM_PIN) // Output small size LED
                      | (1 << LED_SIZE_MD_PIN) // Output medium size LED
                      | (1 << LED_SIZE_LG_PIN) // Output large size LED
                      | (1 << LED_SIZE_XL_PIN) // Output xtra size LED
                      | (1 << LED_ARMED_PIN)   // Output armed pin
                      | (0 << BTN_SIZE_PIN)    // Input on buttons
                      | (0 << BTN_ARM_PIN)
                      | (0 << BTN_PURGE_PIN);
    BTN_LED_REG_PORT |= (1 << BTN_SIZE_PIN)    // Pull up on buttons
                      | (1 << BTN_ARM_PIN)
                      | (1 << BTN_PURGE_PIN);

    MTR_SNR_REG_DDR   = (1 << MTR_PIN)  // Output on motor control
                      | (0 << SNR_PIN); // Input on sensor pin
    MTR_SNR_REG_PORT |= (1 << SNR_PIN); // Pull up on sensor

    GIMSK |= (1 << BTN_GIMSK_MSK)  // Enable pin-change interrupts on button/led port
           | (1 << SNR_GIMKS_MSK); // Enable pin-change interrupts on sensor port

    // Setup sleep mode and reduce power consumption
    // @TODO: Turn off other stuff
    // @TODO: Use power down in setup state
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
}

uint8_t state_setup() {
    const uint8_t btn_ints = (1 << BTN_SIZE_INT_MSK) | (1 << BTN_ARM_INT_MSK) | (1 << BTN_PURGE_INT_MSK);

    uint8_t return_state = 0;
    while(return_state == 0) {
        // Turn on LEDs for load size
        set_size_leds();

        // Enable pin-change interrupts on button pins and go to sleep
        GIFR        |= (1 << BTN_GIFR_FLG);   // Clear existing pin-change interrupts
        BTN_REG_INT |= btn_ints;

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
                return_state = STATE_ARMED;
                break;
            case (1 << BTN_PURGE_PIN):
                return_state = STATE_PURGING;
                break;
            default:
                break;
        }
    }

    // Turn off pin-change interrupts
    BTN_REG_INT &= ~btn_ints;

    return return_state;
}

uint8_t state_armed() {
    // @TODO: Factor into subroutine
    // Turn on blinking blinking armed LED
    TCCR1A = (1 << WGM10)   // Fast PWM 10-bit (bit 0)
           | (1 << WGM11)   // Fast PWM 10-bit (bit 1)
           | (1 << COM1A1); // Clear OC1A(PA6) on compare match, set at bottom
    TCCR1B = (1 << CS12)    // IO clock/1024 (bit 2)
           | (1 << CS10)    // IO clock/1024 (bit 0)
           | (1 << WGM12);  // Fast PWM 10-bit (bit 2)
    TCNT1 = 450; // Initially set the counter close to output compare to shorten delay before
                 // LED goes on
    OCR1A = 500; // Set output compare to ~500ms to toggle LED at ~50% duty cycle

    const uint8_t btn_ints = (1 << BTN_ARM_INT_MSK);
    const uint8_t snr_ints = (1 << SNR_INT_MSK);

    uint8_t result_state = 0;
    while(result_state == 0) {
        // Enable pin-change interrupts on arm button and sensor pin, then go to sleep
        GIFR        |= (1 << BTN_GIFR_FLG) | (1 << SNR_GIFR_FLG);
        BTN_REG_INT |= btn_ints;
        SNR_REG_INT |= snr_ints;

        sei();
        sleep_mode();

        // --- SLEEP ---

        // Awoken from sleep by pin change
        if (PC_REG == 0) {
            // Button pressed
            if (debounce_button() == (1 << BTN_ARM_PIN)) {
                // Turn off armed LED
                BTN_LED_REG_PORT &= ~(1 << LED_ARMED_PIN);
                result_state = STATE_SETUP;
                break;
            }
        } else {
            // Sensor went off
            if (debounce_sensor() == (1 << SNR_PIN)) {
                // Turn on armed LED to indicate dispensing is occurring/has occurred
                BTN_LED_REG_PORT |= (1 << LED_ARMED_PIN);
                result_state = STATE_DISPENSING;
                break;
            }
            BTN_LED_REG_PORT ^= (1 << LED_SIZE_XL_PIN);
        }
    }

    // Turn off timer and interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    BTN_REG_INT &= ~btn_ints;
    SNR_REG_INT &= ~snr_ints;

    return result_state;
}

uint8_t state_dispensing() {
    // Start dispensing!
    MTR_SNR_REG_PORT |= (1 << MTR_PIN);

    // @TODO TEST CODE
    // Setup timer to run for 10s then interrupt
    TCCR1A = 0;              // Normal mode
    TCCR1B = (1 << CS12);    // IO clock/256
    OCR1A  = 39062;          // Set output compare to ~10s
    TCNT1  = 0;              // Reset timer to 0
    TIFR1  |= (1 << OCF1A);  // Clear existing output compare interrupt
    TIMSK1 |= (1 << OCIE1A); // Enable interrupt on timer

    sei();
    sleep_mode();

    // --- SLEEP ---

    // Awoken from sleep by timer. Stop timer, stop dispensing
    MTR_SNR_REG_PORT &= ~(1 << MTR_PIN);
    TCCR1A = 0;
    TCCR1B = 0;

    return STATE_SETUP;
}

uint8_t state_purging() {
    // Turn on 16-bit timer to run for 1s before beginning to dispense
    TCCR1A  = 0;
    TCCR1B  = (1 << CS00)    // IO clock/64 (bit 0)
            | (1 << CS01);   // IO clock/64 (bit 1)
    OCR1A   = 7812;          // Set output compare to ~.5s
    TCNT1   = 0;             // Reset timer to 0
    TIFR1  |= (1 << OCF1A);  // Clear existing output compare interrupt
    TIMSK1 |= (1 << OCIE1A); // Enable interrupt on timer

    // Turn on pin-change interrupt to see if user lets go of button
    GIFR        |= (1 << BTN_GIFR_FLG);
    BTN_REG_INT |= (1 << BTN_PURGE_INT_MSK);

    // Go to sleep
    sei();
    sleep_mode();

    // --- SLEEP ---

    // Awoken by pin-change or timer
    if (BTN_REG_PIN & (1 << BTN_PURGE_PIN)) {
        // Button has been released
    } else {
        // Timer elapsed - begin purging
        MTR_SNR_REG_PORT |= (1 << MTR_PIN);
        BTN_LED_REG_PORT |= (1 << LED_ARMED_PIN);

        // Go to sleep until button is released
        sei();
        sleep_mode();

        // --- SLEEP ---

        // Awoken by pin-change
        MTR_SNR_REG_PORT &= ~(1 << MTR_PIN);
        BTN_LED_REG_PORT &= ~(1 << LED_ARMED_PIN);
    }

    // Turn off timer
    TCCR0A = 0;
    TCCR0B = 0;

    return STATE_SETUP;
}

int main() {
    setup();

    uint8_t state = STATE_SETUP;

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
