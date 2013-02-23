#include <OneWire.h>
#include <DallasTemperature.h>
#include <EtherCard.h>
#include <EEPROMEx.h>

// EEPROM address alocations
#define EE_COSM_INTERVAL  0 // one byte
#define EE_COSM_FEED      1 // max len 19
#define EE_COSM_KEY       20 // max len 130

// Ethernet addresses
#define ETHER_MAC {0x74,0x69,0x69,0x2D,0x30,0x33}
#define ETHER_IP  {192,168,1,200}
#define ETHER_GW  {192,168,1,254}

// tcp/ip send and receive buffer
byte Ethernet::buffer[1100];   
// used as cursor while filling the buffer
static BufferFiller bfill; 

// Data wire is plugged into port 18
#define ONE_WIRE_BUS 18
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Convert temp every 7 sec.
#define TEMP_CONVERT_FREQ 7000
// Timer for temp conversion
static unsigned long timerTempConv;

// Timer for send data to cosm
static unsigned long timerSendToCosm;
// Interval for sending data (zero = never send)
static byte intervalSendToCosm;
// Store the post data id
static byte wwwIdSendToCosm;
 

// Calculate free RAM
static int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// Infinite loop that blink led on pin
void errorLoop(int pin, int dly) {
  pinMode(pin, OUTPUT);
  while (true) {
    digitalWrite(pin, HIGH);
    delay(dly);
    digitalWrite(pin, LOW);
    delay(dly);
  }
}

// Check if need to wait for temp conversion to finish (750ms after conversion start) 
boolean watingForTemp()
{
  return ((long)(millis() - (timerTempConv - TEMP_CONVERT_FREQ + 750)) < 0);
}

// Request temp conversion every X sec.
void requestTempConv()
{
  // If time not elapsed, return
  if ((long)(millis() - timerTempConv) < 0) return;
  // Set next timeout
  timerTempConv += TEMP_CONVERT_FREQ;
  // Send convert
  sensors.requestTemperatures();
}

// Post data to COSM every X min.
void postDataToCosm()
{
  // Dont post if not sending data or waiting for temp conversion
  if (intervalSendToCosm == 0 || watingForTemp()) return;
  // If time not elapsed, return
  if ((long)(millis() - timerSendToCosm) < 0) return;
  // Set next timeout (interval is in minutes)
  timerSendToCosm += intervalSendToCosm * 60000;
  // Post data to COSM
  wwwIdSendToCosm = ether.clientTcpReq(&postDateToCosm_result_cb, &postDateToCosm_datafill_cb, ether.hisport);
}

// post date to COSM callback
static word postDateToCosm_datafill_cb(byte fd) {
  BufferFiller bfill = ether.tcpOffset();
  if (fd == wwwIdSendToCosm) {
    // Write headers
    bfill.emit_p(PSTR(
      "PUT /v2/feeds/$E.csv HTTP/1.1\r\n"
      "Host: api.cosm.com\r\n"
      "X-ApiKey: $E\r\n"
      "Content-Type: text/csv\r\n"
      "Content-Length: 000\r\n"
      "Connection: close\r\n\r\n"), (byte*)EE_COSM_FEED, (byte*)EE_COSM_KEY);
    
    word start = bfill.position();
    
    // Make sure reading are ready
    while (watingForTemp());
    
    // Loop over all devices
    DeviceAddress addr;
    oneWire.reset_search();
    while (oneWire.search(addr)) {
      float tempC = sensors.getTempC(addr);
      bfill.emit_p(PSTR("$H$H$H$H$H$H$H$H,$T\r\n")
        , addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7], tempC);
    }
    
    word length = bfill.position() - start;
    writeContentLengthValue(bfill.buffer(), length);
  }
  
  return bfill.position();
}

// post date to COSM result callback
static byte postDateToCosm_result_cb(byte fd, byte statuscode, word datapos, word len_of_data) {
  if (fd == wwwIdSendToCosm && statuscode == 0 && len_of_data > 12) {
    byte f = strncmp("200",(char *)&(ether.buffer[datapos+9]),3) != 0;
    //(*client_browser_cb)(f, ((word)TCP_SRC_PORT_H_P+(gPB[TCP_HEADER_LEN_P]>>4)*4),len_of_data);
  }
  return 0;
}

// setup/reset the EEPROM
void setupEEPROM(boolean rst) {
  // ATmega32 EEPROM is 1KB
  EEPROM.setMemPool(0, 1024);

  intervalSendToCosm = EEPROM.readByte(EE_COSM_INTERVAL);
  if (intervalSendToCosm == 0xFF || rst) 
    EEPROM.updateByte(EE_COSM_INTERVAL, intervalSendToCosm = 0);
  
  if (EEPROM.readByte(EE_COSM_FEED) == 0xFF || rst) 
    EEPROM.updateByte(EE_COSM_FEED, 0);

  if ( EEPROM.readByte(EE_COSM_KEY) == 0xFF || rst)
    EEPROM.updateByte(EE_COSM_KEY, 0);
  
  // Make sure the strings are termintaed
  EEPROM.updateByte(19, 0);
  EEPROM.updateByte(149, 0);
}

void setup(){
  // use pin 8 to indicate normal operation (blink)
  // use pin 9 to indicate setup
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);

  // Check if to reset to default (SW1/D4)
  pinMode(4, INPUT_PULLUP);
  delay(50);
  boolean rst = (digitalRead(4) == LOW);

  setupEEPROM(rst);

  // Start the onw wire bus
  sensors.begin();
  sensors.setResolution(12);
  sensors.setWaitForConversion(false);

  // Start the ethernet with CS on pin 12 for JY-MCU
  byte mymac[] = ETHER_MAC;
  if (ether.begin(sizeof Ethernet::buffer, mymac, 12) == 0) 
    errorLoop(8, 100);

  // Setup DHCP
  byte myip[] = ETHER_IP;
  byte gwip[] = ETHER_GW;
  if (!ether.dhcpSetup()) {
    // Use static setup if no DHCP (LED7/D9) on)
    ether.staticSetup(myip, gwip);
    digitalWrite(9, HIGH);
  }
  
  // Get COSM ip address
  if (!ether.dnsLookup(PSTR("api.cosm.com")))
      errorLoop(8, 100);
      
  // Setup timers
  timerTempConv = millis();
  timerSendToCosm = millis() + intervalSendToCosm * 60000;

  // finish setup
  digitalWrite(9, LOW);
}

// store the position of the response content to calculate content length
static word contentStart;

// write HTTP headers and store the content position
void writeHeaders(BufferFiller& buf, PGM_P headers) {
  buf.emit_p(PSTR(
    "HTTP/1.1 200 OK\r\n"
    "Cache-Control: no-cache\r\n"
    "Content-Length: 000\r\n"
    "Connection: close\r\n"
    "$F\r\n"), headers);
  contentStart = buf.position();
}

// Use the content posision to calculate content length and write it
void writeContentLength(BufferFiller& buf)
{
  word length = buf.position() - contentStart;   
  writeContentLengthValue(buf.buffer(), length);
}

// write the content lenght value into the header placeholder (": 000\r\n")
void writeContentLengthValue(uint8_t* data, word length)
{
  // Find the place to write
  while (*data && !(*(data-4)==':' && *(data-3)==' ' && *(data-2)=='0' && *(data-1)=='0' && *data=='0' && *(data+1)=='\r' && *(data+2)=='\n')) data++;
  
  while (*data && *data == '0')
  {
    *data = (length%10 + '0');
    data--;
    length /= 10;
  }
}

void respondToHTTP() {
  // Process TCP packets
  word pos = ether.packetLoop(ether.packetReceive());
  // check if valid tcp data is received
  if (pos) {
    bfill = ether.tcpOffset();
    char* data = (char *) Ethernet::buffer + pos;

    // receive buf hasn't been clobbered by reply yet
    if (checkUrl(PSTR("GET / "), data))
      homePage(bfill);
    else if (checkUrl(PSTR("GET /main.css "), data))
      mainCss(bfill);
    else if (checkUrl(PSTR("GET /list.json "), data))
      listJson(bfill);     
    else if (checkUrl(PSTR("GET /main.js "), data))
      mainJs(bfill);
    else if (checkUrl(PSTR("GET /conf.htm "), data))
      confPage(bfill);     
    else if (checkUrl(PSTR("GET /conf.json "), data))
      confJson(bfill);     
    else if (checkUrl(PSTR("GET /conf.js"), data))
      confJs(bfill);     
    else if (checkUrl(PSTR("POST /conf.htm "), data))
      confPost(data, bfill);     
    else
      bfill.print(F(
        "HTTP/1.0 404 Not Found\r\n"
        "Content-Type: text/html\r\n"
        "\r\n"
        "<h1>404 Not Found</h1>"));  
    ether.httpServerReply(bfill.position()); // send web page data
  }
}

// wrtie the home page HTML
void homePage(BufferFiller& buf) {
  writeHeaders(buf, PSTR("Content-Type: text/html\r\n"));
  buf.print(F(
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'/>"
    "<title>TempServer</title>"
    "<link rel='stylesheet' href='//code.jquery.com/mobile/1.2.0/jquery.mobile-1.2.0.min.css'/>"
    "<link rel='stylesheet' href='main.css'/>"
    "<script src='//code.jquery.com/jquery-1.8.2.min.js'></script>"
    "<script src='//code.jquery.com/mobile/1.2.0/jquery.mobile-1.2.0.min.js'></script>"
    "<script src='//stevenlevithan.com/assets/misc/date.format.js'></script>"
    "<script src='main.js'></script>"
    "</head><body><div id='main' data-role='page'>"
    "<div data-role='header' data-position='fixed'><h3>Is it hot?</h3>"
    "<a href='conf.htm' data-icon='gear' data-iconpos='notext' class='ui-btn-right' data-transition='slideup'></div>"
    "<div data-role='content'>"
    "<ul id='list' data-role='listview' data-inset='true'></ul>"
    "<p id='info'></p></div></div></body></html>"));
  writeContentLength(buf);
}

// write the homepage CSS
void mainCss(BufferFiller& buf) {
  writeHeaders(buf, PSTR("Content-Type: text/css\r\n"));
  buf.print(F(
    ".ui-li-aside{font-weight:bold;font-size:xx-large;}"
    ".ui-li-aside > sup{font-size:large;}"
    "#info{margin-top:10px;text-align:center;font-size:small;}"));
  writeContentLength(buf);
}

// write the homepage JS
void mainJs(BufferFiller& buf) {
  writeHeaders(buf, PSTR("Content-Type: application/javascript\r\n"));
  buf.print(F(
    "function refresh(){if(_t+1e4>new Date||$.mobile.activePage.attr('id')!='main')return;_t=(new Date).getTime();"
    "$.getJSON('list.json',function(e){var t=[];$.each(e.list,function(e,n){t.push('<li id=\"'+n.id+'\">"
    "<a><h3>'+n.name+'</h3><p>'+n.id+'</p><p class=\"ui-li-aside\">'+n.val.toFixed(1)+'<sup>&deg;C</sup></p></a></li>')});"
    "$('#list').html(t.join('')).trigger('create').listview('refresh');"
    "$('#info').text('Updated: '+(new Date).format('HH:MM'));"
    "if(e.feed)$('#info').append('<br/><br/><a href=\"//cosm.com/feeds/'+e.feed+'\" target=\"blank\">Open feed in COSM</a>')})}"
    "$(document).ready(function(){setInterval(refresh,500)});var _t=0;"));
  writeContentLength(buf);
}

// write the list.json
void listJson(BufferFiller& buf) {
  writeHeaders(buf, PSTR("Content-Type: application/json\r\n"));
  buf.emit_p(PSTR("{\"list\":["));

  int index = 1;
 
  // Make sure readings are ready
  while (watingForTemp());

  // loop all devices
  DeviceAddress addr;
  oneWire.reset_search();
  while (oneWire.search(addr)) {
    if (index != 1) buf.write(',');
    float tempC = sensors.getTempC(addr);
    buf.emit_p(PSTR("{\"id\":\"$H$H$H$H$H$H$H$H\",\"name\":\"Sensor $D\",\"val\":$T}")
      , addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7], index, tempC);
    index++;
  }

  buf.emit_p(PSTR("],\"free\":$D,\"intr\":$D,\"feed\":\"$E\",\"cosm\":\"$D.$D.$D.$D\"}"), freeRam(), intervalSendToCosm, (byte*)EE_COSM_FEED, ether.hisip[0], ether.hisip[1], ether.hisip[2], ether.hisip[3]);

  writeContentLength(buf);
}

// write configuration HTML
void confPage(BufferFiller& buf) {
  writeHeaders(buf, PSTR("Content-Type: text/html\r\n"));
  buf.print(F(
    "<!DOCTYPE html><html><body><div id='conf' data-role='page'>"
    "<script src='conf.js'></script><form method='post' action='conf.htm'>"
    "<div data-role='header' data-position='fixed'>"
    "<a href='/' data-rel='back' data-theme='b'>Cancel</a><h3>COSM Settings</h3>"
    "<button type='submit' data-theme='b'>Save</button></div>"
    "<div data-role='content'>"
    "<div data-role='fieldcontain'><label for='intr'>Send Every</label>"
    "<select name='intr' id='intr'></select></div>"
    "<div data-role='fieldcontain'><label for='feed'>Feed ID:</label>"
    "<input type='text' name='feed' id='feed' maxlength='18'/></div>"
    "<div data-role='fieldcontain'><label for='key'>API Key:</label>"
    "<input type='text' name='key' id='key' maxlength='129'/></div>"
    "</div></form></div></body></html>"));
  writeContentLength(buf);
}

// write configuration JS
void confJs(BufferFiller& buf) {
  writeHeaders(buf, PSTR("Content-Type: application/javascript\r\n"));
  buf.print(F(
    "$(document).ready(function(){for(var i=0;i<61;i+=i<15?5:15)$('#intr')"
    ".append($(document.createElement('option')).val(i).text(i>0?i+' min':'Never'));"
    "$.getJSON('conf.json',function(d){$('#intr').val(d.intr).selectmenu('refresh');"
    "$('#feed').val(d.feed);$('#key').val(d.key)});"
    "$('#conf > form').submit(function(){$.post('conf.htm',$(this).serialize())"
    ".fail(function(){alert('Fail to save!')})"
    ".done(function(){_t=0;history.back()});return false})});"));
  writeContentLength(buf);
}

// write the conf.json
void confJson(BufferFiller& buf) {
  writeHeaders(buf, PSTR("Content-Type: application/json\r\n"));

  buf.emit_p(PSTR("{\"intr\":\"$D\",\"feed\":\"$E\",\"key\":\"$E\"}"), intervalSendToCosm, (byte*)EE_COSM_FEED, (byte*)EE_COSM_KEY);

  writeContentLength(buf);
}

// process the post data of configuration form
void confPost(const char* data, BufferFiller& buf) {
  // Skip headers
  data = findContent(data);

  // Store values in EEPROM  
  intervalSendToCosm = saveByteValInEE(PSTR("intr"), data, EE_COSM_INTERVAL);
  saveStrValInEE(PSTR("feed"), data, EE_COSM_FEED, 19);
  saveStrValInEE(PSTR("key"), data, EE_COSM_KEY, 130);
  
  // Write empty response
  writeHeaders(buf, PSTR("Content-Type: text/html\r\n"));
  writeContentLength(buf);
}

// check the request URL, return true if match 
boolean checkUrl(PGM_P url, const char* data) {
  while (true) {
    char c = pgm_read_byte(url++);
    if (c == 0) break;
    if (*data != c) return false;
    data++;
  }
  return true;
}

// convert a single hex digit character to its integer value
unsigned char hexToInt(char c)
{
    if (c >= '0' && c <='9') return((unsigned char)c - '0');
    if (c >= 'a' && c <='f') return((unsigned char)c - 'a' + 10);
    if (c >= 'A' && c <='F') return((unsigned char)c - 'A' + 10);
    return(0);
}

// finds the value of the specified key in data, return zero if not found
const char* findKeyVal(PGM_P key, const char *data)
{
  PGM_P pkey = key;
  while(*data && *data!=' ' && *data!='\r' && *data!='\n'){
    if (*data == pgm_read_byte(pkey)){
      pkey++;
      if (pgm_read_byte(pkey) == '\0'){
        data++;
        pkey = key;
        if (*data == '=') return data+1;
      }
    }
    else{
      pkey = key;
    }
    data++;
  }
  return 0;
}

// store the decoded value of the specified key in the EEPROM address, returns the string length
int saveStrValInEE(PGM_P key, const char *data, int address, int maxlen)
{
  const char* value = findKeyVal(key, data);
  int i = 0;
  byte h = 0;
  
  if (value){
    // copy the decoded value to the EE and terminate it with '\0'
    while(*value && *value!=' ' && *value!='\r' && *value!='\n' && *value!='&' && i<maxlen-1){
      char c = *value;
      if (c == '+') c = ' ';
      if (c == '%') h = 3; // decode the next 2 chars as hex
      if (h == 1) c = (hexToInt(*(value-1)) << 4) | hexToInt(c);
      if (h != 0) h--;
      if (h == 0) {
        EEPROM.updateByte(address, c);
        i++;      
        address++;
      }
      value++;
    }
    EEPROM.updateByte(address, 0);
  }
  // return the length of the value
  return(i);
}

// store the byte value of the specified key in the EEPROM address, returns the value or zero
byte saveByteValInEE(PGM_P key, const char *data, int address)
{
  const char* value = findKeyVal(key, data);
  byte result = 0;
  
  if (value){
    // loop over digits
    while(*value && *value >= '0' && *value <= '9'){
      result = (result * 10) + ((unsigned char)*value - '0');
      value++;
    }
  }
  // store in EEPROM
  EEPROM.updateByte(address, result);
  // return the length of the value
  return(result);
}

// finds the content of the http request
const char* findContent(const char* data) {
  // Progress data the content (skip headeres)
  data += 4; // skip the first 4 bytes
  while (*data && !(*(data-1)=='\n' && *(data-2)=='\r' && *(data-3)=='\n' && *(data-4)=='\r')) data++;
  return data;
}

void loop(){
  // Blink a led to show that loop is running
  digitalWrite(8, millis()/1000%2 ? HIGH : LOW);

  // Send request for temp conversion.
  requestTempConv();

  // Handle HTTP requests
  respondToHTTP();
  
  // Post data to COSM
  postDataToCosm();
}

