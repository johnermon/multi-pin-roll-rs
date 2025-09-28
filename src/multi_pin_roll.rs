use std::{ptr::copy_nonoverlapping, slice::from_raw_parts, str::pattern::StrSearcher};

use arduino_hal::*;
use esp_idf_sys::esp_now_recv_info_t;
use ina219::SyncIna219;
use embedded_hal::digital::OutputPin;
use esp_idf_hal::prelude::*;
use esp_idf_svc::{http::server::EspHttpServer, wifi::EspWifi};


//consts representinf the password and ssid of the connection 
const SSID:&str = "";
const PASSWORD:&str = "";


//consts for defining the pins on arduino that have different roles
const SWITCH_PIN:i16 = 0;
const SDA_PIN:i16 = 20;
const SCL_PIN:i16 = 21;

//global variables

//initializes an array signifying used pins
pub static mut USED_PINS:[&'static mut dyn OutputPin;_];

static mut MODE_FLAG:i16 = 0;

static mut LAST_COMMAND_TIME:u32 = 0;

static mut TEST_TIMER:i16 = 0;

static mut L_TEST_PERIOD:i16 = 150;

static mut INA_219:SycIna219;

static mut SCHEDULE:Vec<PulseEvent>;

static mut VOLTAGE:f32 = 0;

static mut CURRENT:f32 = 0;

static mut POWER:f32 = 0;

static mut SENSOR_AVAILABLE:bool = false;

pub static mut SERVER:EspHttpServer;

pub static mut SERIAL:_;

struct Solenoid{
    pin:u8,
    pulse_stop_time:u32 = 0,
    is_active:bool = false
}

struct PulseEvent{
    pin:u8,
    start_time:u32,
    end_time:u32,
    active:bool
}

struct PlayCommand{
    command:&str
}

fn set_all_pins(states:i16){
    for i in 0..NUM_PINS{
        unsafe{
            if (states >> 1) & 1{
                USED_PINS[i].set_high();
            }else{
                USED_PINS[i].set_low();
            }      
        }
    }
}

fn on_recieve(data:*const u8, len:i16){
    let mut cmd = PlayCommand{
        command:std::str::from_utf8(unsafe{from_raw_parts(data, len)})
    };
    if cmd.command == "play"{
        ufmt::uwriteln!(unsafe{&mut SERIAL}, "Recieved PLAY command via ESP-NOW");
        start_playback();
    }
}

fn start_playback(){}
