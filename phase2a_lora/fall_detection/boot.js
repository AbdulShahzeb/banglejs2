(() => {
  const HR_SERVICE = 0x180D;
  const HR_UUID = 0x2A37;

  const MOTION_SERVICE = 0xFFA0;
  const MOTION_UUID = 0xFFA3;

  const GPS_SERVICE = 0x1819;
  const LAT_UUID = 0x2AAE;
  const LON_UUID = 0x2AAF;
  const TIME_UUID = 0x2A2B;

  const BARO_SERVICE_UUID = 0x181A;
  const PRESSURE_UUID = 0x2A6D;

  var impact = 0, no_motion = 0, alert = 0; pressure = 0;
  var motionTime = Date.now();
  const GPS_PRECISION = 6;
  const WAITRESPONSE = 10000;
  const NO_MOTION_TRIG = 7500;

  function setupAdvertising() {
    /*
     * This function prepares BLE advertisement.
     */

    NRF.setAdvertising(
      {
        0x180D: undefined,
        0x1819: undefined,
        0x181A: undefined,
        0xFFA0: undefined,
      },
      {
        // Set up advertisement settings
        connectable: true,
        discoverable: true,
        scannable: true,
        whenConnected: true,
      }
    );

    NRF.setServices({
      0x180D: { // heart_rate
        0x2A37: {
          notify: true,
          readable: true,
          value: [0x06, 0],
        },
        0x2A38: { // Sensor Location: Wrist
          value: 0x02,
        }
      },
      0x1819: { // gps
        0x2AAE: {
          notify: true,
          readable: true,
          value: [0,0,0,0], // lat
        },
        0x2AAF: {
          notify: true,
          readable: true,
          value: [0,0,0,0], // lon
        },
        0x2A2B: {
          notify: true,
          readable: true,
          value: [0,0,0,0,0,0,0,0,0,0,0,0,0], // time
        }
      },
      0x181A: { // barometer
        0x2A6D: {
          notify: true,
          readable: true,
          value: [0,0,0,0], // pressure
        }
      },
      0xFFA0: {
        0xFFA3: {
          notify: true,
          readable: true,
          value: [0,0,0], // impact, no_motion, alert
        }
      }
    });
  }


  function updateHRM(hrm) {
    /*
     * Send updated heart rate measurement via BLE
     */
    if (hrm === undefined || hrm.confidence < 50) return;
    try {
      NRF.updateServices({
        0x180D: {
          0x2A37: {
            value: [0x06, hrm.bpm],
            notify: true
          },
          0x2A38: {
            value: 0x02,
          }
        }
      });
    } catch (error) {
      if (error.message.includes("BLE restart")) {
        /*
         * BLE has to restart after service setup.  
         */
        NRF.disconnect();
      } else {
        console.log("[fall_detection]: Unexpected error occured while updating HRM over BLE! Error: " + error.message);
      }
    }
  }


  function updateGPS(fix) {
    /*
     * Send updated gps measurement via BLE
     */
    if (fix === undefined || fix.fix === 0) return;

    var latBuffer = new ArrayBuffer(4);
    var latView = new Uint32Array(latBuffer);
    latView[0] = Math.floor(fix.lat * Math.pow(10,GPS_PRECISION));

    var lonBuffer = new ArrayBuffer(4);
    var lonView = new Uint32Array(lonBuffer);
    lonView[0] = Math.floor(fix.lon * Math.pow(10, GPS_PRECISION));

    var timeBuffer = new ArrayBuffer(8);
    var timeView = new DataView(timeBuffer);
    var currentTime = Date.now().toString();
    var dateArray = [];

    for (var i = 0; i < 13; i++) {
        dateArray.push(parseInt(currentTime.charAt(i)));
    }

    try {
      NRF.updateServices({
        0x1819: {
          0x2AAE: {
            value: latBuffer,
            notify: true,
          },
          0x2AAF: {
            value: lonBuffer,
            notify: true,
          },
          0x2A2B: {
            value: dateArray,
            notify: true,
          },
        }
      });

    } catch (error) {
      if (error.message.includes("BLE restart")) {
        /*
         * BLE has to restart after service setup.  
         */
        NRF.disconnect();
      } else {
        console.log("[fall_detection]: Unexpected error occured while updating GPS over BLE! Error: " + error.message);
      }
    }
  }

  function updatePressure(e) {
    /*
     * Send updated pressure measurement via BLE
     */

    if (e === undefined) return;

    var pressureBuf = new ArrayBuffer(4);
    var pressureView = new Uint32Array(pressureBuf);
    pressureView[0] = Math.round(e.pressure);

    try {
      NRF.updateServices({
        0x181A: {
          0x2A6D: {
            value: pressureBuf,
            notify: true
          }
        }
      });
    } catch (error) {
      if (error.message.includes("BLE restart")) {
        NRF.disconnect();
      } else {
        console.log("[fall_detection]: Unexpected error occured while updating Pressure over BLE! Error: " + error.message);
      }
    }
  }

  function checkFall() {
    var a = Bangle.getAccel();

    // High accel in z direction could be a sign of fall
    var z = Math.round(a.z*100);

    // Absolute acceleration in all directions
    var d2 = [
      Math.round(a.x*100) * Math.round(a.x*100),
      Math.round(a.y*100) * Math.round(a.y*100),
      Math.round(a.z*100) * Math.round(a.z*100)
      ];

    var movement = Math.sqrt(d2[0] + d2[1] + d2[2]) - 100;

    // If high abs accel AND high z-axis accel, it could be a fall
    if (impact == 0 && movement > 175 && Math.abs(z) > 175) {
        impact = 1;
        respondToFall();
    }

    if (movement > 10) {
      motionTime = Date.now();
      no_motion = 0;
    } else if (Date.now() - motionTime > NO_MOTION_TRIG) {
      no_motion = 1;
    }
  }

  function updateMotion() {

    if (impact && no_motion) {
      alert = 1;
    } else {
      alert = 0;
    }

    try {
      NRF.updateServices({
        0xFFA0: {
          0xFFA3: {
            value: [impact, no_motion, alert],
            notify: true
          }
        }
      });
    } catch (error) {
      if (error.message.includes("BLE restart")) {
        NRF.disconnect();
      } else {
        console.log("[fall_detection]: Unexpected error occured while updating Motion over BLE! Error: " + error.message);
      }
    }
  }


  // restart algorithm
  function falseAlarm() {
    g.clear();
    Bangle.buzz();
    impact = 0;
  }

  // Function to respond to fall with BUTTON press
  function respondToFall() {

    Bangle.buzz(2000);

    // If button pressed, continue fall detection algorithm
    // else start transmitting biometrics
    Bangle.setLocked(false);
    E.showMessage("Have you fallen?\n Press BTN for NO!");
    var startTime = Date.now();
    var endTime = startTime + WAITRESPONSE;

    while (Date.now() < endTime) {
      if (BTN.read()) {
        falseAlarm();
        return;
      }
    }

    Bangle.buzz();
    E.showMessage("Impact Alert!");
  }


  setupAdvertising();
  Bangle.setHRMPower(1);
  Bangle.setGPSPower(1);
  Bangle.setBarometerPower(1);
  Bangle.on("HRM", function (hrm) { updateHRM(hrm); });
  setInterval(checkFall, 100);
  setInterval(updateMotion, 1000);
  Bangle.on("pressure", function(e) { updatePressure(e); });
  Bangle.on("GPS", function(fix) { updateGPS(fix); });
})();