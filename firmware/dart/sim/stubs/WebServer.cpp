// Host implementation of the WebServer stub: a small blocking HTTP/1.1 server on a POSIX socket.
// One request per handleClient() call, served on the calling thread (the sketch's loop), then the connection closes.
#include "WebServer.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
int httpPort = 8080;
SimHttpHook simHook;

const char* reasonPhrase(int code) {
  switch (code) {
    case 200: return "OK"; case 400: return "Bad Request"; case 404: return "Not Found"; case 409: return "Conflict";
    case 500: return "Internal Server Error"; case 503: return "Service Unavailable"; default: return "Unknown";
  }
}
int hexVal(char c) { return c >= '0' && c <= '9' ? c - '0' : (c | 32) >= 'a' && (c | 32) <= 'f' ? (c | 32) - 'a' + 10 : -1; }
std::string urlDecode(const std::string& s) {        // like the ESP32 core: %XX and '+' = space
  std::string o;
  for (size_t i = 0; i < s.size(); i++) {
    if (s[i] == '+') o += ' ';
    else if (s[i] == '%' && i + 2 < s.size() && hexVal(s[i + 1]) >= 0 && hexVal(s[i + 2]) >= 0) {
      o += (char)(hexVal(s[i + 1]) * 16 + hexVal(s[i + 2])); i += 2;
    } else o += s[i];
  }
  return o;
}
}  // namespace

void simSetHttpPort(int port) { httpPort = port; }
void simSetHttpHook(SimHttpHook hook) { simHook = hook; }

WebServer::~WebServer() { if (listenFd_ >= 0) close(listenFd_); }

void WebServer::begin() {
  (void)sketchPort_;
  listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1; setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a = {}; a.sin_family = AF_INET; a.sin_port = htons((uint16_t)httpPort); a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(listenFd_, (sockaddr*)&a, sizeof a) != 0 || listen(listenFd_, 16) != 0) {
    fprintf(stderr, "SIM: cannot listen on port %d: %s\n", httpPort, strerror(errno)); exit(2);
  }
  fcntl(listenFd_, F_SETFL, O_NONBLOCK);
  fprintf(stderr, "SIM: HTTP server on http://127.0.0.1:%d (the dart's port 80)\n", httpPort);
}

// reads the request head (the sketch only takes GET query arguments, so a body is ignored); false = give up
bool WebServer::readRequest(int fd) {
  std::string head;
  char buf[1024];
  while (head.find("\r\n\r\n") == std::string::npos && head.find("\n\n") == std::string::npos) {
    pollfd p = {fd, POLLIN, 0};
    if (poll(&p, 1, 2000) <= 0) return false;         // 2 s for the whole head (the ESP32 waits up to 5 s)
    ssize_t n = recv(fd, buf, sizeof buf, 0);
    if (n <= 0) return false;
    head.append(buf, (size_t)n);
    if (head.size() > 16384) return false;
  }
  size_t eol = head.find('\n');
  std::string line = head.substr(0, eol);
  if (!line.empty() && line.back() == '\r') line.pop_back();
  size_t s1 = line.find(' '), s2 = line.rfind(' ');
  if (s1 == std::string::npos || s2 == s1) return false;
  method_ = line.substr(0, s1);
  std::string target = line.substr(s1 + 1, s2 - s1 - 1);
  http10_ = line.compare(s2 + 1, std::string::npos, "HTTP/1.0") == 0;
  size_t q = target.find('?');
  uri_ = urlDecode(target.substr(0, q));
  args_.clear();
  if (q != std::string::npos) {
    std::string qs = target.substr(q + 1);
    size_t pos = 0;
    while (pos <= qs.size()) {
      size_t amp = qs.find('&', pos); if (amp == std::string::npos) amp = qs.size();
      std::string kv = qs.substr(pos, amp - pos);
      if (!kv.empty()) {
        size_t eq = kv.find('=');
        args_.push_back({urlDecode(kv.substr(0, eq)), eq == std::string::npos ? "" : urlDecode(kv.substr(eq + 1))});
      }
      pos = amp + 1;
    }
  }
  return true;
}

void WebServer::handleClient() {
  if (listenFd_ < 0) return;
  int fd = accept(listenFd_, nullptr, nullptr);
  if (fd < 0) return;
  int one = 1; setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
  client_ = fd; sent_ = chunked_ = streaming_ = false; contentLength_ = CONTENT_LENGTH_NOT_SET;
  if (readRequest(fd)) {
    String u(uri_);
    bool done = false;
    if (simHook && u.startsWith("/sim/")) done = simHook(*this, u);   // simulator control, before the sketch
    if (!done) {
      for (const Route& r : routes_) if (r.uri == uri_) { r.fn(); done = true; break; }
    }
    if (!done) { if (notFound_) notFound_(); else send(404, "text/plain", "Not found\n"); }
    if (!sent_) send(500, "text/plain", "handler sent no response\n");
    finishResponse();
  }
  shutdown(fd, SHUT_WR);
  close(fd);
  client_ = -1;
}

String WebServer::arg(const String& name) const {
  for (const auto& a : args_) if (a.first == name.str()) return String(a.second);
  return String();
}
bool WebServer::hasArg(const String& name) const {
  for (const auto& a : args_) if (a.first == name.str()) return true;
  return false;
}

void WebServer::writeAll(const char* p, size_t n) {
  while (n && client_ >= 0) {
    ssize_t w = ::send(client_, p, n, MSG_NOSIGNAL);
    if (w <= 0) { if (w < 0 && errno == EINTR) continue; client_ = -1; return; }   // client gone: drop the rest
    p += w; n -= (size_t)w;
  }
}

void WebServer::send(int code, const char* contentType, const String& content) {
  if (sent_) return;
  sent_ = true;
  char h[512];
  int n = snprintf(h, sizeof h, "HTTP/1.%d %d %s\r\nContent-Type: %s\r\n", http10_ ? 0 : 1, code, reasonPhrase(code), contentType);
  if (contentLength_ == CONTENT_LENGTH_UNKNOWN) {
    streaming_ = true;
    chunked_ = !http10_;                               // HTTP/1.0 clients get the raw stream ended by the close
    if (chunked_) n += snprintf(h + n, sizeof h - n, "Transfer-Encoding: chunked\r\n");
  } else {
    size_t len = contentLength_ == CONTENT_LENGTH_NOT_SET ? content.length() : contentLength_;
    n += snprintf(h + n, sizeof h - n, "Content-Length: %zu\r\n", len);
  }
  n += snprintf(h + n, sizeof h - n, "Connection: close\r\n\r\n");
  writeAll(h, (size_t)n);
  if (streaming_) { if (content.length()) sendContent(content); }
  else writeAll(content.c_str(), content.length());
}

// streaming: one chunk per call; a zero-length chunk ends the response (as in the ESP32 core)
void WebServer::sendContent(const char* data, size_t len) {
  if (!chunked_) { writeAll(data, len); return; }
  char h[16]; int n = snprintf(h, sizeof h, "%zx\r\n", len);
  writeAll(h, (size_t)n);
  writeAll(data, len);
  writeAll("\r\n", 2);
  if (len == 0) chunked_ = false;
}

void WebServer::finishResponse() {
  if (chunked_) sendContent("", 0);                   // the handler did not end its stream
  streaming_ = false;
}
