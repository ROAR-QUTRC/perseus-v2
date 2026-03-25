#!/bin/bash

ros2 service call /read_ilmenite perseus_interfaces/srv/TakeAs7343Reading "{
  no_led_reading: {
    f1_405nm: 120, f2_425nm: 130, fz_450nm: 140, f3_475nm: 150,
    f4_515nm: 160, f5_550nm: 170, fy_555nm: 180, fxl_600nm: 190,
    f6_640nm: 200, f7_690nm: 210, f8_745nm: 220, nir_855nm: 230,
    vis_clear: 240, fd_flicker: 10,
    redo_reading: false, success: true
  },
  white_led_reading: {
    f1_405nm: 220, f2_425nm: 230, fz_450nm: 240, f3_475nm: 250,
    f4_515nm: 260, f5_550nm: 270, fy_555nm: 280, fxl_600nm: 290,
    f6_640nm: 300, f7_690nm: 310, f8_745nm: 320, nir_855nm: 330,
    vis_clear: 340, fd_flicker: 12,
    redo_reading: false, success: true
  },
  uv_led_reading: {
    f1_405nm: 320, f2_425nm: 330, fz_450nm: 340, f3_475nm: 350,
    f4_515nm: 360, f5_550nm: 370, fy_555nm: 380, fxl_600nm: 390,
    f6_640nm: 400, f7_690nm: 410, f8_745nm: 420, nir_855nm: 430,
    vis_clear: 440, fd_flicker: 15,
    redo_reading: false, success: true
  },
  success: true,
  message: 'Dummy reading OK'
}"
