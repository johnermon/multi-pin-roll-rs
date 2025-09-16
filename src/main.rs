pub mod multi_pin_roll;

use arduino_hal::*;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};

use crate::multi_pin_roll::{USED_PINS, SERVER, SERIAL};
fn main(){
    
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    unsafe{
        //initializes used pins with this array of used pins
        USED_PINS = [
            pins.d1.into_output(),
            pins.d2.into_output(),
            pins.d3.into_output(),
            pins.d4.into_output(),
            pins.d5.into_output(),
            pins.d6.into_output(),
            pins.d7.into_output(),
            pins.d8.into_output(),
            pins.d9.into_output(),
            pins.d10.into_output(),
            pins.d20.into_output(),
            pins.d21.into_output(),
        ];

        SERVER = EspHttpServer::new(&Configuration{
            http_port:80,
            ..Default::default()
        });

        SERIAL = default_serial!(dp,pins, 9600);

    }

}
