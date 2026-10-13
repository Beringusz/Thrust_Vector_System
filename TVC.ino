#include <math.h>

//volatile for reading the variable as in memory
volatile unsigned long timer0_millis = 0;


//offset values for X and Y axis(for the sensor to sense upright correctly)
double offsetX = 0.75; 
double offsetY = -1.70;

//center is where the thrust vector is upright, min and max are the limits of rotation(due to bad printing)
const int centerX = 150;
const int minX = 100;
const int maxX = 180;

const int centerY = 72;
const int minY = 25;
const int maxY = 120;

//constants for PID control
double kp = 0.9;
double ki = 0.001;
double kd = 0.005;

//
double integral_X = 0, previous_X = 0;
double integral_Y = 0, previous_Y = 0;

//vars to measure dt for derivative part of control
unsigned long last_time = 0;
double dt;
double setpoint = 0.0; 

//interrupt on compare match for manual milis() function using TIMER0
ISR(TIMER0_COMPA_vect) 
{
    timer0_millis++;
}

//compare match for pin3, then reset pin to LOW
ISR(TIMER1_COMPA_vect) 
{
    PORTD &= ~(1 << PD3);
}

//compare match for pin5, then reset pin to LOW
ISR(TIMER1_COMPB_vect) 
{
    PORTD &= ~(1 << PD5);
}

//starts PWM, sets the pins 3 and 5 to HIGH
ISR(TIMER1_OVF_vect) 
{
    PORTD |= (1 << PD3) | (1 << PD5);
}

//function to mimic milis()
unsigned long get_millis() 
{
    unsigned long m;  //holding value of timer
    cli(); //clear interrupts, meaning that the stuff between cli() and sei() is uninterruptable
    m = timer0_millis;
    sei();
    return m;
}

void init_timers() 
{
    TCCR0A = (1 << WGM01); // set TIMER0 on CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00); //prescaler = 64 => T_clk = 4 microseconds
    OCR0A = 249; // compare value => TIMER0 counts 4 * (249 + 1) = 1000 microseconds = 1ms
    TIMSK0 = (1 << OCIE0A); // enable interrupt on TIMER0

    TCCR1A = (1 << WGM11);  //set TIMER1 to fast PWM mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // p = 8 => T_clk = 0.5 microseconds
    ICR1 = 39999; // 40000 * 0.5 microseconds = 20 ms refresh rate for servo
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1); //enable interrupt for TIMER1
}

void set_servo_X(int angle) 
{
    if (angle < minX) angle = minX;  //constrain angles to the measured value
    if (angle > maxX) angle = maxX;
    OCR1A = 1088 + ((long)angle * 3712) / 180; //angle conversion to value for PWM pulse length, achieved by setting the interrupt comp value
                                               //note: for some reason, the motor uses a wider pulse range of 544 microseconds for 0 to 2400 microseconds for 180
                                               //3712 is the difference between clk cycles(T_clk = 0.5 microseconds) of T_clk_180 and T_clk_0
}

void set_servo_Y(int angle) 
{
    if (angle < minY) angle = minY;
    if (angle > maxY) angle = maxY;
    OCR1B = 1088 + ((long)angle * 3712) / 180;
}

void twi_init() 
{
    TWSR = 0x00;  //initialize to 0 to remove previous effects
    TWBR = 72; // set clock speed of I2C
    TWCR = (1 << TWEN); //enable Two Wire interface
}

void twi_start() 
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);  //enable TWI, set start condition to 1(TWISTA), clear interrupt flag for TW(TWINT)
    while (!(TWCR & (1 << TWINT))); //wait for pulse to finish
}

void twi_stop() 
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); //set stop register to 1, reset TWINT by oring with 1(its weird)
}

void twi_write(uint8_t data) 
{
    TWDR = data;  //put data into data register
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));  //wait for it to finish
}

uint8_t twi_read_ack() 
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); //if ack then set TWEA(two wire enable acknowledge) to 1
    while (!(TWCR & (1 << TWINT)));//wait for it to finish
    return TWDR; 
}

uint8_t twi_read_nack() 
{
    TWCR = (1 << TWINT) | (1 << TWEN); // here TWEA is set to 0 initially and we dont change that(due to nack)
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}
//functions for implementing PID control
double pid_X(double error) 
{
    double proportional = error; //proportional(P) part, error is the deviation from the setpoint(0, which is the init pos of the sensor lying flat)
    integral_X += error * dt; //integral part, summation of dt wide bits of the error function
    double derivative = (error - previous_X) / dt; //derivative part, calculated as the difference between prev value and error over the dt time
    previous_X = error; //replace prev value with current one
    return (kp * proportional) + (ki * integral_X) + (kd * derivative); //return everything scaled with the factors
}
//same here, but for Y axis control
double pid_Y(double error) 
{
    double proportional = error; 
    integral_Y += error * dt;
    double derivative = (error - previous_Y) / dt;
    previous_Y = error;
    return (kp * proportional) + (ki * integral_Y) + (kd * derivative);
}

int main(void) 
{
    DDRD |= (1 << PD3) | (1 << PD5); //set pin 3 and pin 5 as output
    //init for timers(TIMER0, TIMER1)
    init_timers();
    sei(); //enable interrupts after timer init so they wont break
    //set servos to centering position
    set_servo_X(centerX);
    set_servo_Y(centerY);
    //init the I2C protocol
    twi_init();
    //0x68 is the MPU6050 address
    twi_start();
    twi_write(0x68 << 1); //left shift by one to leave place for read/write bit
    twi_write(0x6B);//memory address for power management
    twi_write(0x00);//wake up sensor
    twi_stop();


    unsigned long start_wait = get_millis(); //start wait initialized
    while (get_millis() - start_wait < 3000); //delay(3000)

    //set these to 0 just in case for not leaking wrong data
    integral_X = 0; previous_X = 0;
    integral_Y = 0; previous_Y = 0;

    last_time = get_millis(); //init last_time for dt calc

    while (1) 
    {
        unsigned long now = get_millis();
        if (now - last_time >= 15) // wait for 15ms eg delay(15) => f_motor_ctrl = 66 Hz
        {
            dt = (now - last_time) / 1000.0; //calculate timestep dt
            last_time = now;

            twi_start();
            twi_write(0x68 << 1);//start talking to MPU6050
            twi_write(0x3B);//memory address for accelerometer data
            twi_stop();

            twi_start();
            twi_write((0x68 << 1) | 1);
            
            //read data and save it into vars
            int16_t AcX = (twi_read_ack() << 8) | twi_read_ack();
            int16_t AcY = (twi_read_ack() << 8) | twi_read_ack();
            int16_t AcZ = (twi_read_ack() << 8) | twi_read_nack();//nack terminates the sensor comm
            
            twi_stop();//stop TWI

            //angle calculation using acceleration compared to ratio of the forces, then convert it to deg
            double actual_angle_X = (atan2(AcY, AcZ) * 180.0 / M_PI) - offsetX;
            double actual_angle_Y = (atan2(-AcX, AcZ) * 180.0 / M_PI) - offsetY;

            //calculate the error between upright and actual angles
            double error_X = setpoint - actual_angle_X;
            double error_Y = setpoint - actual_angle_Y;

            //feed it to PID controller
            double output_X = pid_X(error_X);
            double output_Y = pid_Y(error_Y);

            //calc target, cast everything into int
            int targetX = centerX + (int)output_X;
            int targetY = centerY - (int)output_Y;
            //move servo to target
            set_servo_X(targetX);
            set_servo_Y(targetY);
        }
    }
    return 0;
}