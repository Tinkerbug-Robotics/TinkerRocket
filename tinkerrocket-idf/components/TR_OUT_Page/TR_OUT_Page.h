#pragma once

#include <compat.h>
#include <WiFi.h>
#include <WebServer.h>

// RocketState enum comes from your flight code types
#include "RocketComputerTypes.h"

class TR_OUT_Page
{
public:
  // Telemetry snapshot for JSON + UI
  struct Telemetry {
    float       soc       = NAN;             // %
    float       current   = NAN;             // mA
    float       voltage   = NAN;             // V
    double      latitude  = NAN;             // deg
    double      longitude = NAN;             // deg
    float       gdop      = NAN;
    int         num_sats  = 0;
    RocketState state     = INITIALIZATION;  // default
    bool        camera_recording = false;  
    bool        logging_active = false;
    String      active_file;
    float       rx_kbs = NAN;
    float       wr_kbs = NAN;
    uint32_t    frames_rx = 0;
    uint32_t    frames_drop = 0;
    float       max_alt_m = NAN;
    float       max_speed_mps = NAN;
  };

  struct FileItem
  {
    char name[32];
    uint32_t size_bytes;
  };

  // ap_ssid/ap_pass: softAP credentials
  explicit TR_OUT_Page(const char* ap_ssid,
                       const char* ap_pass);

  // Call once during setup.
  // Returns true on success (AP + server started).
  bool begin();

  // Call frequently from loop()
  void loop();

  // Update telemetry used by /batteryData and the main page
  void updateTelemetry(const Telemetry& t);

  // Enum-based version (no string)
  void updateTelemetry(float soc,
                       float current,
                       float voltage,
                       double latitude,
                       double longitude,
                       float gdop,
                       int num_sats,
                       RocketState state);
    
  // Called when user toggles camera recording.
  // recording == true  -> start recording
  // recording == false -> stop recording
  typedef void (*CameraControlCallback)(bool recording);
    
  // Set camera control callback (optional; page still 
  // renders without it)
  void setCameraControlCallback(CameraControlCallback cb);
  
  typedef void (*StateToggleCallback)(RocketState newState);

  void setStateToggleCallback(StateToggleCallback cb);

  typedef size_t (*ListFilesCallback)(FileItem* out_items, size_t max_items);
  typedef bool (*DeleteFileCallback)(const char* filename);
  typedef bool (*DownloadFileCallback)(const char* filename, WebServer& server);
  void setFileCallbacks(ListFilesCallback list_cb,
                        DeleteFileCallback delete_cb,
                        DownloadFileCallback download_cb);

  // Utility functions (public for use by other components)
  static const char* rocketStateToString(RocketState s);
  static const char* rocketStateToCss(RocketState s);

private:
  // Single instance pointer for static handlers
  static TR_OUT_Page* instance_;

  const char* ap_ssid_;
  const char* ap_pass_;
  WebServer   server_;
  Telemetry   telem_;
    
  CameraControlCallback camera_cb_ = nullptr;
  StateToggleCallback   state_toggle_cb_ = nullptr;
  ListFilesCallback     list_files_cb_ = nullptr;
  DeleteFileCallback    delete_file_cb_ = nullptr;
  DownloadFileCallback  download_file_cb_ = nullptr;
  
  // --- internal helpers ---
  void setupRoutes();
  String generateWebPage() const;

  // Route handlers (static trampolines)
  static void handleRootThunk();
  static void handleDataThunk();
  static void handleListFilesThunk();
  static void handleDownloadThunk();
  static void handleDeleteThunk();
  static void handleCameraToggleThunk();
  static void handleStateToggleThunk();

  // Actual implementations
  void handleRoot();
  void handleData();
  void handleListFiles();
  void handleDownload();
  void handleDelete();
  void handleCameraToggle();
  void handleStateToggle();

  // utils
  static String mimeTypeFor(const String& name);
};
