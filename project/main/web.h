
/*
 *   Header [web.h]
 */

#ifndef WEB_HEADER_GUARD
#define WEB_HEADER_GUARD

void web_init(void);                     // connects to WiFi and serves a WebSocket at /ws
void web_broadcast(const char *text);    // sends text to every connected WebSocket client

#endif /* WEB_HEADER_GUARD */
