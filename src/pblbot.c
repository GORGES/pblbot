#include <pebble.h>

//  constants

#define  kEthan  0
#define  kMartijn  0
const int kAccelSamplesPerUpdate = 1;
const int kAccelMax = 640;
const int kAccelDeadZone = 50;
const int kAccelDeadZoneDirection = 10;
const int kAccelCross = 22;
const int kAccelRadius = 5;
const int kAccelDisk = 15;
const int kAccelRange = 70;
const int kRadiusMax = 60;
const int kRadiusMin = 50;
const int kRadiusExtra = -3;
const int kTimerInterval = 50;
const int kCornerRadius = 5;
const int kWheelWidth = 12;
const int kWheelMin = 0;
const int kWheelStop = 90;
const int kWheelMax = 180;
const int kTurnDamper = 3;

//  macro functions

#define  ABS2(x,y)  (((x)>=(y))?(x)-(y):(y)-(x))

//  variables

static Window *window;
static Layer* layer_robot;
static InverterLayer* inverter_screen;
static AccelData accel_data;
static int n_left, n_right;
static BTDevice g_device;
static bool b_connected, b_paused;
static BLECharacteristic left_characteristic, right_characteristic, command_characteristic;

//  functions

TextLayer* text_create(char* str_text, GRect rect, Layer* root_layer) {
  TextLayer* text_layer = text_layer_create(rect);
  strcpy(str_text, "");
  text_layer_set_background_color(text_layer, GColorClear);
  text_layer_set_font(text_layer, fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD));
  text_layer_set_text_alignment(text_layer, GTextAlignmentCenter);
  text_layer_set_text_color(text_layer, GColorWhite);
  text_layer_set_text(text_layer, str_text);
  layer_add_child(root_layer, text_layer_get_layer(text_layer));
  return text_layer;
}

static void update_robot(Layer* layer, GContext* context) {
  ///  center point
  GPoint center = GPoint(144 / 2, 168 / 2);
  //  draw in white
  graphics_context_set_stroke_color(context, GColorWhite);
  //  loop through circle
  for (int i = 0;  i < 360;  i += 15) {
    //  calculate angle
    int32_t angle = (TRIG_MAX_ANGLE * i) / 360;
    //  sin and cosine
    int32_t cos = cos_lookup(angle),
            sin = sin_lookup(angle);
    //  draw spoke
    GPoint p0 = GPoint(center.x + (kRadiusMin + ((i % 45) ? 0 : kRadiusExtra)) * sin / TRIG_MAX_RATIO,
                       center.y - (kRadiusMin + ((i % 45) ? 0 : kRadiusExtra)) * cos / TRIG_MAX_RATIO);
    GPoint p1 = GPoint(center.x + kRadiusMax * sin / TRIG_MAX_RATIO,
                       center.y - kRadiusMax * cos / TRIG_MAX_RATIO);
    graphics_draw_line(context, p0, p1);
    if (!(i % 45)) {
      graphics_draw_line(context, GPoint(p0.x + 1, p0.y), GPoint(p1.x + 1, p1.y));
      graphics_draw_line(context, GPoint(p0.x, p0.y + 1), GPoint(p1.x, p1.y));
      graphics_draw_line(context, GPoint(p0.x + 1, p0.y + 1), GPoint(p1.x + 1, p1.y + 1));
    }
  }
  //  start with center cross
  graphics_draw_line(context, GPoint(center.x - kAccelCross, center.y), GPoint(center.x - kAccelRadius, center.y));
  graphics_draw_line(context, GPoint(center.x + kAccelRadius, center.y), GPoint(center.x + kAccelCross, center.y));
  graphics_draw_line(context, GPoint(center.x, center.y - kAccelCross), GPoint(center.x, center.y - kAccelRadius));
  graphics_draw_line(context, GPoint(center.x, center.y + kAccelRadius), GPoint(center.x, center.y + kAccelCross));
  //  draw circle based on accelerometer data
  graphics_context_set_fill_color(context, GColorWhite);
  graphics_fill_circle(context, GPoint(center.x + kAccelRange * accel_data.x / kAccelMax, center.y - kAccelRange * accel_data.y / kAccelMax), kAccelDisk);
  //  draw left and right wheels
  if (n_left > kWheelStop)
    graphics_fill_rect(context, GRect(72, 0, (72 * (n_left - kWheelStop)) / kWheelStop, kWheelWidth), kCornerRadius, GCornersRight);
  else if (n_left < kWheelStop)
    graphics_fill_rect(context, GRect(72 - (72 * (kWheelStop - n_left)) / kWheelStop, 0, (72 * (kWheelStop - n_left)) / kWheelStop, kWheelWidth), kCornerRadius, GCornersLeft);
  if (n_right > kWheelStop)
    graphics_fill_rect(context, GRect(72, 168 - kWheelWidth, (72 * (n_right - kWheelStop)) / kWheelStop, kWheelWidth), kCornerRadius, GCornersRight);
  else if (n_right < kWheelStop)
    graphics_fill_rect(context, GRect(72 - (72 * (kWheelStop - n_right)) / kWheelStop, 168 - kWheelWidth, (72 * (kWheelStop - n_right)) / kWheelStop, kWheelWidth), kCornerRadius, GCornersLeft);
}

static void accel_callback(AccelData *data, uint32_t num_samples) {
  if ((num_samples == 1) && data && !data->did_vibrate) {
    //  check for redraw
    if ((ABS2(accel_data.x, data->x) > kAccelMax / (2 * kAccelCross)) ||
        (ABS2(accel_data.y, data->y) > kAccelMax / (2 * kAccelCross)))
      layer_mark_dirty(layer_robot);
    //  remember updated data
    accel_data = *data;
    //  speed [kWheelMin, kWheelMax]
    if (accel_data.x < -kAccelDeadZone)
      n_left = n_right = kWheelStop + (kWheelStop * (accel_data.x + kAccelDeadZone)) / kAccelMax;
    else if (accel_data.x > kAccelDeadZone)
      n_left = n_right = kWheelStop + (kWheelStop * (accel_data.x - kAccelDeadZone)) / kAccelMax;
    else
      n_left = n_right = kWheelStop;
    //  direction [-kWheelStop, +kWheelStop]
    int n_direction;
    if (accel_data.y < -kAccelDeadZoneDirection)
      n_direction = (kWheelStop * (accel_data.y + kAccelDeadZoneDirection)) / (kTurnDamper * kAccelMax);
    else if (accel_data.y > kAccelDeadZoneDirection)
      n_direction = (kWheelStop * (accel_data.y - kAccelDeadZoneDirection)) / (kTurnDamper * kAccelMax);
    else
      n_direction = 0;
#if  kEthan
    n_direction = -n_direction;
#endif
    n_left -= n_direction;
    n_right += n_direction;
    //  limit
    if (n_left < kWheelMin)
      n_left = kWheelMin;
    else if (n_left > kWheelMax)
      n_left = kWheelMax;
    if (n_right < kWheelMin)
      n_right = kWheelMin;
    else if (n_right > kWheelMax)
      n_right = kWheelMax;
    //  paused?
    if (b_paused)
      n_left = n_right = kWheelStop;
    //  send to robot
    uint8_t byte;
    if (left_characteristic != BLE_CHARACTERISTIC_INVALID) {
#if  kEthan
      byte = kWheelMax - n_left;
#else
      byte = n_left;
#endif
      ble_client_write_without_response(left_characteristic, &byte, sizeof(byte));
    }
    if (right_characteristic != BLE_CHARACTERISTIC_INVALID) {
#if  kEthan
      byte = n_right;
#else
      byte = kWheelMax - n_right;
#endif
      ble_client_write_without_response(right_characteristic, &byte, sizeof(byte));
    }
  }
}

void button_up(ClickRecognizerRef recognizer, void *context) {
  if (command_characteristic != BLE_CHARACTERISTIC_INVALID) {
    //APP_LOG(APP_LOG_LEVEL_DEBUG, "button_up");
#if  kMartijn
    uint8_t byte = 0;
#else
    uint8_t byte = 1;
#endif
    ble_client_write_without_response(command_characteristic, &byte, sizeof(byte));
  }
}

void button_select(ClickRecognizerRef recognizer, void *context) {
  b_paused = !b_paused;
  //APP_LOG(APP_LOG_LEVEL_DEBUG, "button_select  b_paused=%d", b_paused);
  layer_set_hidden(inverter_layer_get_layer(inverter_screen), !b_paused);
}

void button_down(ClickRecognizerRef recognizer, void *context) {
  if (command_characteristic != BLE_CHARACTERISTIC_INVALID) {
    //APP_LOG(APP_LOG_LEVEL_DEBUG, "button_down");
#if  kMartijn
    uint8_t byte = 90;
#else
    uint8_t byte = 2;
#endif
    ble_client_write_without_response(command_characteristic, &byte, sizeof(byte));
  }
}

void config_provider(void* context) {
  window_single_click_subscribe(BUTTON_ID_UP, button_up);
  window_single_click_subscribe(BUTTON_ID_SELECT, button_select);
  window_single_click_subscribe(BUTTON_ID_DOWN, button_down);
}

static void service_change_handler(BTDevice device, const BLEService services[], uint8_t num_services, BTErrno status) {
  APP_LOG(APP_LOG_LEVEL_DEBUG, "service_change_handler");
  //  invalidate any old references:
  left_characteristic = right_characteristic = command_characteristic = BLE_CHARACTERISTIC_INVALID;
  //  iterate through the found services:
  for (uint8_t i = 0; i < num_services; ++i) {
    Uuid service_uuid = ble_service_get_uuid(services[i]);
    //  compare with the Bean "Scratch Service" UUID:
    const Uuid bean_scratch_service_uuid =
    UuidMake(0xa4, 0x95, 0xff, 0x20, 0xc5, 0xb1, 0x4b, 0x44,
             0xb5, 0x12, 0x13, 0x70, 0xf0, 0x2d, 0x74, 0xde);
    if (!uuid_equal(&service_uuid, &bean_scratch_service_uuid))
      //  not the Bean's "Scratch Service"
      continue;
    char uuid_buffer[UUID_STRING_BUFFER_LENGTH];
    uuid_to_string(&service_uuid, uuid_buffer);
    const BTDeviceAddress address = bt_device_get_address(device);
    APP_LOG(APP_LOG_LEVEL_INFO,
            "Discovered Bean Scratch service %s (0x%08x) on " BT_DEVICE_ADDRESS_FMT,
            uuid_buffer, services[i], BT_DEVICE_ADDRESS_XPLODE(address));
    //  iterate over the characteristics within the "Scratch Service":
    BLECharacteristic characteristics[8];
    uint8_t num_characteristics = ble_service_get_characteristics(services[i], characteristics, 8);
    if (num_characteristics > 8)
      num_characteristics = 8;
    for (uint8_t c = 0; c < num_characteristics; ++c) {
      Uuid characteristic_uuid = ble_characteristic_get_uuid(characteristics[c]);
      // The characteristic UUIDs we're looking for:
      Uuid bean_scratch_char1_uuid = UuidMake(0xa4, 0x95, 0xff, 0x21, 0xc5, 0xb1, 0x4b, 0x44, 0xb5, 0x12, 0x13, 0x70, 0xf0, 0x2d, 0x74, 0xde);
      Uuid bean_scratch_char2_uuid = UuidMake(0xa4, 0x95, 0xff, 0x22, 0xc5, 0xb1, 0x4b, 0x44, 0xb5, 0x12, 0x13, 0x70, 0xf0, 0x2d, 0x74, 0xde);
      Uuid bean_scratch_char3_uuid = UuidMake(0xa4, 0x95, 0xff, 0x23, 0xc5, 0xb1, 0x4b, 0x44, 0xb5, 0x12, 0x13, 0x70, 0xf0, 0x2d, 0x74, 0xde);

      uint8_t scratch_num = 0; // Just for logging purposes
      if (uuid_equal(&characteristic_uuid, &bean_scratch_char1_uuid))
        left_characteristic = characteristics[c];
      else if (uuid_equal(&characteristic_uuid, &bean_scratch_char2_uuid))
        right_characteristic = characteristics[c];
      else if (uuid_equal(&characteristic_uuid, &bean_scratch_char3_uuid))
        command_characteristic = characteristics[c];
      else
        continue;
      uuid_to_string(&characteristic_uuid, uuid_buffer);
      APP_LOG(APP_LOG_LEVEL_INFO, "-- Found Scratch%u: %s (0x%08x)", scratch_num, uuid_buffer, characteristics[c]);
    }
  }
}

static void connection_handler(BTDevice device, BTErrno connection_status) {
  const BTDeviceAddress address = bt_device_get_address(device);
  const bool connected = (connection_status == BTErrnoConnected);
  APP_LOG(APP_LOG_LEVEL_DEBUG, "%s " BT_DEVICE_ADDRESS_FMT " (status=%d)",
          connected ? "Connected.\nDiscovering services..." : "Disconnected",
          BT_DEVICE_ADDRESS_XPLODE(address), connection_status);
  if (connected)
    ble_client_discover_services_and_characteristics(device);
}

static void ble_scan_handler(BTDevice device, int8_t rssi, const BLEAdData *ad_data) {
  if (!b_connected) {
    char temp_local_name[32] = {0};
    ble_ad_copy_local_name(ad_data, temp_local_name,
                           sizeof(temp_local_name));
#if  kMartijn
    if (strcmp(temp_local_name, "JAWS") != 0)
      return;
#endif
    //APP_LOG(APP_LOG_LEVEL_DEBUG, "found PblBot");
    const BTDeviceAddress address = bt_device_get_address(device);
    //APP_LOG(APP_LOG_LEVEL_INFO, "Got Advertisement from: " BT_DEVICE_ADDRESS_FMT " (%s)", BT_DEVICE_ADDRESS_XPLODE(address), temp_local_name);
    //APP_LOG(APP_LOG_LEVEL_DEBUG, "%x %x %x %x %x %x", address.octets[0], address.octets[1], address.octets[2], address.octets[3], address.octets[4], address.octets[5]);
    //  robots 1 or 2
#if  kEthan
    if ((address.octets[0] != 0x9E) || (address.octets[1] != 0xD7) || (address.octets[2] != 0xC9) || (address.octets[3] != 0x72) || (address.octets[4] != 0x39) || (address.octets[5] != 0xD0))
#else
#if  kMartijn
    if (false)
#else
    if ((address.octets[0] != 0x16) || (address.octets[1] != 0x24) || (address.octets[2] != 0xE5) || (address.octets[3] != 0x72) || (address.octets[4] != 0x39) || (address.octets[5] != 0xD0))
#endif
#endif
      return;
    APP_LOG(APP_LOG_LEVEL_DEBUG, "found our robot");
    //  connect to this device
    // Try to copy the first Service UUID, we'll display this in the list:
    //const uint8_t num_services = ble_ad_copy_service_uuids(ad_data, &result->first_service_uuid, 1);
    //if (num_services) {
    //  result->has_services = true;
    g_device = device;
    b_connected = true;
    //  handlers
    ble_central_set_connection_handler(connection_handler);
    ble_client_set_service_change_handler(service_change_handler);
    //  connect
    BTErrno e = ble_central_connect(g_device, /*auto_reconnect*/ true, /*is_pairing_required*/ false);
    if (e)
      APP_LOG(APP_LOG_LEVEL_ERROR, "Error connectiong: %d", e);
    //  stop scanning
    ble_scan_stop();
  }
}

static void window_load(Window *window) {
  //  get root layer
  Layer* layer_root = window_get_root_layer(window);
  //  create layer
  layer_robot = layer_create(GRect(0, 0, 144, 168));
  layer_set_update_proc(layer_robot, update_robot);
  layer_add_child(layer_root, layer_robot);
  //  inverter
  inverter_screen = inverter_layer_create(GRect(0, 0, 144, 168));
  layer_add_child(layer_root, inverter_layer_get_layer(inverter_screen));
  layer_set_hidden(inverter_layer_get_layer(inverter_screen), true);
  //  start accel
  accel_service_set_sampling_rate(ACCEL_SAMPLING_25HZ);
  accel_data_service_subscribe(kAccelSamplesPerUpdate, accel_callback);
  //  start BLE scanning
  ble_scan_start(ble_scan_handler);
}

static void window_unload(Window* window) {
  //  tick unsubscribe
  tick_timer_service_unsubscribe();
  //  accell unsubscribe
  accel_data_service_unsubscribe();
  //  destroy
  layer_destroy(layer_robot);
  inverter_layer_destroy(inverter_screen);
}

int main(void) {
  //  reset variables
  memset(&accel_data, '\0', sizeof(accel_data));
  n_left = n_right = kWheelStop;
  b_connected = b_paused = false;
  left_characteristic = right_characteristic = command_characteristic = BLE_CHARACTERISTIC_INVALID;
  //  create window
  window = window_create();
  window_set_window_handlers(window, (WindowHandlers) {
    .load = window_load,
    .unload = window_unload,
  });
  window_set_background_color(window, GColorBlack);
  window_set_click_config_provider(window, config_provider);
  window_set_fullscreen(window, true);
  window_stack_push(window, true);
  //  main event loop
  app_event_loop();
  //  destroy window
  window_destroy(window);
}
