// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

void Rover::init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate();
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

void Rover::init_sonar(void)
{
    sonar.init();
}

void Rover::init_wind_vane(void)
{
    uint8_t pin;
    // 0:APM2 A0, 1:APM2 A1, ...
    for (pin = 0; pin < 4; pin++){
	    wind_vane_sensors[pin] = hal.analogin->channel(pin);
    }
}

// should be called at 5hz
void Rover::read_wind_vane(void)
{
    // Показания, считанные с датчиков
    float sensor_reading[4];
    // Храним максимальное показание каждого датчика и показания других датчиков в этот момент 
    // Основываемся на том, что максимальное показание соответствует положению, когда магнитное поле перпендикулярно плоскости датчика,
	// т.е. магнит напротив датчика.	
    static float sensor_max_pos[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
	
	uint8_t pin;
	uint8_t j;
	float tmp = 0;
	float dev;
	float wnd;
	uint8_t max_i;
	uint8_t right_i;
	uint8_t left_i;	
	uint8_t pair;
	
	// Считываем усредненные показания с датчиков	
	for (pin = 0; pin < 4; pin++){
		sensor_reading[pin] = wind_vane_sensors[pin]->read_average();
		// Определяем датчик с максимальным показанием
		if (sensor_reading[pin] > tmp){
			max_i = pin;
			tmp = sensor_reading[pin];			
		}
    }
	// Запоминаем максимальное показание каждого датчика и показания других датчиков в этот момент 
	// "калибровка на лету"
    for (pin = 0; pin < 4; pin++){        
        if (sensor_reading[pin] > sensor_max_pos[pin][pin]){
		    for( j = 0; j < 4; j++)
                sensor_max_pos[pin][j] = sensor_reading[j];	
        }
    }
	
    // Датчик слева 
    if (max_i == 0)
		left_i = 3;
    else
        left_i = max_i - 1; 
	
 	// Датчик справа
	if (max_i == 3)
		right_i = 0;
	else
        right_i = max_i + 1;
	
    //   
    if ((sensor_reading[right_i]- sensor_max_pos[max_i][right_i]) > (sensor_reading[left_i]-sensor_max_pos[max_i][left_i]))
        pair = right_i;
    else
        pair = left_i;
	
    // апроксимация пропорциональная 
	tmp = sensor_max_pos[max_i][max_i] - sensor_reading[max_i];
	dev = tmp + sensor_max_pos[pair][pair] - sensor_reading[pair];
	
	// прошли колибровку ?
    if (dev > 0){ 
		dev = 90 * (tmp / dev);
		wnd = mechanical_relative_bearing[max_i];
		if (pair == right_i){
		    wnd += dev;
			if ( wnd >= 360.0f )
			    wnd -= 360.0f;
		}
	    else{
		    wnd -= dev;
		    if ( wnd < 0.0f )
			    wnd += 360.0f;
	    }	
		apparent_wind = wnd;
	}	
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
void Rover::read_battery(void)
{
    battery.read();
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Rover::read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->voltage_average() * 50;
    receiver_rssi = constrain_int16(ret, 0, 255);
}

// read the sonars
void Rover::read_sonars(void)
{
    sonar.update();

    if (sonar.status() == RangeFinder::RangeFinder_NotConnected) {
        // this makes it possible to disable sonar at runtime
        return;
    }

    if (sonar.has_data(1)) {
        // we have two sonars
        obstacle.sonar1_distance_cm = sonar.distance_cm(0);
        obstacle.sonar2_distance_cm = sonar.distance_cm(1);
        if (obstacle.sonar1_distance_cm <= (uint16_t)g.sonar_trigger_cm &&
            obstacle.sonar1_distance_cm <= (uint16_t)obstacle.sonar2_distance_cm)  {
            // we have an object on the left
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar1 obstacle %u cm"),
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        } else if (obstacle.sonar2_distance_cm <= (uint16_t)g.sonar_trigger_cm) {
            // we have an object on the right
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar2 obstacle %u cm"),
                                  (unsigned)obstacle.sonar2_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = -g.sonar_turn_angle;
        }
    } else {
        // we have a single sonar
        obstacle.sonar1_distance_cm = sonar.distance_cm(0);
        obstacle.sonar2_distance_cm = 0;
        if (obstacle.sonar1_distance_cm <= (uint16_t)g.sonar_trigger_cm)  {
            // obstacle detected in front 
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar obstacle %u cm"),
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        }
    }

    Log_Write_Sonar();

    // no object detected - reset after the turn time
    if (obstacle.detected_count >= g.sonar_debounce &&
        hal.scheduler->millis() > obstacle.detected_time_ms + g.sonar_turn_time*1000) { 
        gcs_send_text_fmt(PSTR("Obstacle passed"));
        obstacle.detected_count = 0;
        obstacle.turn_angle = 0;
    }
}
