//Parameters
#ifndef PARAMETERS_h
#define PARAMETERS_h

//Parameters for handling buttons pushes behaviours
#define LONGPRESSTIMER            2000    //Time in ms for considering button long pressed
#define TIMEOUT_MENU              10000   //Time in ms without button activity before leaving the menu (changes cancelled)

#define REPEAT_START_DELAY        500     // ms before auto-repeat
#define REPEAT_INITIAL_INTERVAL   200     // speed at start
#define REPEAT_FAST_INTERVAL      50      // max speed
#define REPEAT_ACCEL_TIME         3000    // time for reaching max speed

#define UNIVERSE 1                      // Default sACN universe (first one to listen for)
#define UNIVERSE_COUNT 1                // Total number of sACN universes to listen for, starting at UNIVERSE
#define ARTNET_DEFAULT_UNIVERSE 0       // Default Art-Net port-address (15 bit, 0..32767)

#define NET_TIMEOUT_SACN_MS       2500  // No sACN packet for this long => source considered lost
#define NET_TIMEOUT_ARTNET_MS     5000  // No Art-Net packet for this long => source considered lost

// Wi-Fi access point (enabled from the OLED menu), reachable on http://1.2.3.4
#define AP_SSID                   "Magnum850"
#define AP_PASSWORD               "fogmachine"   // WPA2: 8 characters minimum. CHANGE IT.
#define AP_CHANNEL                6
#define AP_MAX_CLIENTS            4

// Web "hold to fog" dead-man switch
#define WEB_FOG_TIMEOUT_MS        700     // no heartbeat from the page for this long => fog stops
#define WEB_FOG_MAX_MS            10000   // one press can never fog longer than this

#endif
