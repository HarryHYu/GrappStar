// Host stub of the ESP32 WebServer, backed by a real POSIX TCP HTTP/1.1 server so the sketch's API can be reached with
// curl, Python or a browser. Same model as the ESP32 one: handleClient() serves at most one request per call, on the
// caller's thread, then closes the connection. Supports the chunked streaming the sketch uses for /flightlog
// (setContentLength(CONTENT_LENGTH_UNKNOWN), send(code, type, ""), sendContent(...), sendContent("")).
//
// The port is not the one in the sketch (80): it comes from simSetHttpPort() (the simulator's --port option).
// Paths starting with /sim/ go to the simulator's control hook BEFORE the sketch's handlers.
#pragma once
#include <functional>
#include <string>
#include <vector>
#include <utility>
#include "Arduino.h"

#define CONTENT_LENGTH_UNKNOWN ((size_t)-1)
#define CONTENT_LENGTH_NOT_SET ((size_t)-2)

class WebServer;
typedef std::function<bool(WebServer& server, const String& uri)> SimHttpHook;   // true = handled
void simSetHttpPort(int port);
void simSetHttpHook(SimHttpHook hook);

class WebServer {
 public:
  typedef std::function<void()> THandlerFunction;
  explicit WebServer(int port = 80) : sketchPort_(port) {}
  ~WebServer();

  void on(const String& uri, THandlerFunction fn) { routes_.push_back({uri.str(), fn}); }
  void onNotFound(THandlerFunction fn) { notFound_ = fn; }
  void begin();
  void handleClient();

  String uri() const { return String(uri_); }
  int args() const { return (int)args_.size(); }
  String argName(int i) const { return i >= 0 && i < args() ? String(args_[i].first) : String(); }
  String arg(int i) const { return i >= 0 && i < args() ? String(args_[i].second) : String(); }
  String arg(const String& name) const;
  bool hasArg(const String& name) const;

  void send(int code, const char* contentType, const String& content = String());
  void send_P(int code, PGM_P contentType, PGM_P content) { send(code, contentType, String(content)); }
  void setContentLength(size_t len) { contentLength_ = len; }
  void sendContent(const String& s) { sendContent(s.c_str(), s.length()); }
  void sendContent(const char* data, size_t len);

 private:
  struct Route { std::string uri; THandlerFunction fn; };
  bool readRequest(int fd);
  void writeAll(const char* p, size_t n);
  void finishResponse();

  int sketchPort_, listenFd_ = -1, client_ = -1;
  std::vector<Route> routes_;
  THandlerFunction notFound_;
  std::string uri_, method_;
  bool http10_ = false;
  std::vector<std::pair<std::string, std::string>> args_;
  size_t contentLength_ = CONTENT_LENGTH_NOT_SET;
  bool sent_ = false, chunked_ = false, streaming_ = false;
};
