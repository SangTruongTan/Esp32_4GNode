/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Sang Tan Truong.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by PIO under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <SoftwareSerial.h>
#include <base64.hpp>
#include <WiFi.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum Stream_control_t {
  NO_CONTROL,
  START_CONTROL,
  STOP_CONTROL,
};

enum Server_state_t {
  WAIT_CLIENT = 0,
  CONNECTED = 1,
  START = 2,
  STREAMING = 3,
  STOP = 4,
  DISCONNECT = 6,
  SERVER_START = 7,
  RECEIVE_FIRST = 8,
  ACK = 8,
  SERVER_STOP = 9,
  RE_CONNECT = 10,
  TIMEOUT = 11,
};

enum System_state_t {
  SYS_INITIALIZING = 0,
  SYS_CONNECTING,
  SYS_CONNECTED,
  SYS_LOSS_CONNECTION,
};

enum System_event_t {
  SYS_INITIALIZED_EVENT = 0,
  SYS_CONNECT_SUCC_EVENT,
  SYS_LOSS_EVENT,
  SYS_RE_CONNECT_EVENT,
  SYS_TIMEOUT_EVENT,
};
struct Network_t {
  String LinkNum;
  String Transport;
  String RemotePort;
  String ServerIP;
  char *buffer;
  size_t size;
};

struct type_process_t {
	String header = "";
	size_t NumberOfAttribute = 0;
	String Attribute[6];
	String Msg = "";
  bool Status = false;
};

struct network_t {
  String Num = "";
  String Transport = "UDP";
  String IpServer = "171.247.236.117";
  String RemotePort = "8001";
  String LocalPort = "8001";
};

struct camera_control_t {
  bool status = false;
  bool control = false;
};

struct system_spec_t {
  HardwareSerial *Serial;
  HardwareSerial *SimSerial;
  WiFiClient *Client;
  WiFiServer *Server;
  network_t *Network_t;
  System_state_t *SystemState;
  Server_state_t *ServerState;
  network_t *Udp;
  network_t *Tcp;
  String *GlobalBuffer;
  camera_control_t CameraControl;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mySerial Serial1
#define RX1 26
#define TX1 27
#define DEBUG ((size_t) 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//Stream Message
String StreamStart = "START STREAMING\r";
String StreamACK = "ACKNOWLEDGEMENT\r";
String StreamStop = "STOP STREAMING\r";

//WiFi Information
const char* ssid = "Esp32-Hostpot";
const char* password = "12345678";
const uint16_t port = 8000;
String RemotePortUDP = "8000";
String LocalPortUDP = "8000";
String ServerIP = "20.210.212.103";
String RemotePortTCP = "8002";
String LocalPortTCP = "8002";
String Hello[2] = {"HELLO", "123"};

//Serial
const int Baudrate = 2000000;
//WiFi Variable
WiFiServer Server;
WiFiClient Client;

//Important header
String HEADER_IPCLOSE = "+IPCLOSE";
String HEADER_CIPOPEN = "+CIPOPEN";
String HEADER_CIPSEND = "+CIPSEND";
String HEADER_CAM = "+CAM";
//Parse string variable
String IgnoreString[2] = { "\r\n", "OK\r\n" };
size_t LengthOfIgnore = 1;
String ImportantHeader[10] = { "AT+CIPRXGET", "+IP ERROR", "ERROR",
                               "+CIPCLOSE", "+IPCLOSE", "+CAM"};
size_t LengthOfImportantHeader = 6;
//System Variable
Server_state_t ServerState;
Stream_control_t StreamControl;
System_state_t SystemState;
system_spec_t SystemSpec;

//Global Buffer for commands processing
String GlobalBuffer = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
void sim_init(HardwareSerial *SimSerial);
void forward_serial2(HardwareSerial *SimSerial, String Cmd);
void forward_buffer(HardwareSerial *SimSerial);
void at_netopen(HardwareSerial *SimSerial);
void at_netclose(HardwareSerial *SimSerial);
void at_cipopen(network_t *Network, HardwareSerial *SimSerial);
void at_sendat(HardwareSerial *SimSerial);
void at_send_buffer(HardwareSerial *SimSerial, Network_t Network);
void client_send_message(WiFiClient *Client, String Msg);
void update_server_state(Server_state_t *State,
                                    Server_state_t Action);
void server_state_debug(Server_state_t state);
uint16_t read_send_to_server(HardwareSerial *SimSerial, WiFiClient *Client);
uint32_t receive_and_stream_cam(WiFiClient *Client, HardwareSerial *SimSerial);

//String Parse function
String parse_white_space(String* Info);
size_t parse_tail(String Info, String* Attr);
String parse_buffer_on_terminate(String* Info);
bool check_unexpected_message(String Info);
type_process_t process_parse(String* InfoBuffer);

//System Function Prototype
System_state_t update_system_state(System_event_t Event);
void system_state_debug (System_state_t state);
void System_initializing (system_spec_t *SysSpec);
void System_connecting (system_spec_t *SystemSpec);
void System_connected (system_spec_t *SystemSpec);
void System_loss_connection (system_spec_t *SystemSpec);
void process_command (system_spec_t *SystemSpec);
//Support Function
String read_all_buffer (HardwareSerial *Serial);
bool check_connection_request (HardwareSerial *SimSerial);
bool check_match_important_header (type_process_t Process);
void re_connect_network (system_spec_t *SystemSpec);
//Temporary function
void wifi_initialize (void);
void serial_initialize (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval None
  */

void setup() {
  // put your setup code here, to run once:
  static network_t Udp;
  static network_t Tcp;
  //Config all attribute for system
  SystemSpec.Serial = &Serial;
  SystemSpec.SimSerial = &mySerial;
  SystemSpec.Client = &Client;
  SystemSpec.Server = &Server;
  SystemSpec.ServerState = &ServerState;
  SystemSpec.SystemState = &SystemState;
  //Config for UDP
  Udp.Num = "3";
  Udp.Transport = "UDP";
  Udp.LocalPort = "8000";
  SystemSpec.Udp = &Udp;
  //Config for TCP
  Tcp.Num = "1";
  Tcp.Transport = "TCP";
  Tcp.LocalPort = LocalPortTCP;
  Tcp.RemotePort = RemotePortTCP;
  Tcp.IpServer = ServerIP;
  SystemSpec.Tcp = &Tcp;
  SystemSpec.ServerState = &ServerState;
  SystemSpec.SystemState = &SystemState;
  SystemSpec.GlobalBuffer = &GlobalBuffer;
  System_initializing(&SystemSpec);
}

/**
  * @brief  The application loop.
  * @retval None
  */

void loop() {
  // put your main code here, to run repeatedly:
  //Process the command
  process_command(&SystemSpec);
  //Check System state
  if(SystemState == SYS_CONNECTING) {
    System_connecting(&SystemSpec);
  }
  if(SystemState == SYS_CONNECTED) {
    System_connected(&SystemSpec);
  }
  if(SystemState == SYS_LOSS_CONNECTION) {
    System_loss_connection(&SystemSpec);
  }
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/**
  * @brief  Connected system state.
  * @param SystemSpec The specification of the system.
  * @retval None
  */
void System_connected (system_spec_t *SystemSpec) {
  if(ServerState == WAIT_CLIENT) {
    wifi_initialize();
    //Update to Connected state
    update_server_state(&ServerState, CONNECTED);
  }
  if(ServerState == CONNECTED) {
    if(SystemSpec->CameraControl.control == true) {
      SystemSpec->CameraControl.status = true;
      update_server_state(&ServerState, SERVER_START); //Update to Start State
    }
  }
  if(ServerState == START) {
    uint16_t FrameLength = 0;
    if(SystemSpec->CameraControl.status == true) {
      client_send_message(&Client, StreamStart);
      FrameLength = receive_and_stream_cam (&Client, &mySerial);
      if(FrameLength == 0) {
        //Disconnected State
        update_server_state(&ServerState, TIMEOUT);    //Update to Disconnect
        Client.stop();
      } else {
        update_server_state(&ServerState, RECEIVE_FIRST);  //Update to Streaming State
      }
    }
  }
  if(ServerState == STREAMING) {
    uint16_t FrameLength = 0;
    if(SystemSpec->CameraControl.status == true) {
      FrameLength = receive_and_stream_cam(&Client, &mySerial);
      if(FrameLength == 0) {
        //Disconnected State
        update_server_state(&ServerState, TIMEOUT);  //Update to Disconnect
        Client.stop();
      }
    }
    if(SystemSpec->CameraControl.control == false) {
      SystemSpec->CameraControl.status = false;
      client_send_message(&Client, StreamStop);
      //Update to STOP State
      update_server_state(SystemSpec->ServerState, SERVER_STOP);
    }
  }
  if(ServerState == STOP) {
    if(SystemSpec->CameraControl.control == true) {
      SystemSpec->CameraControl.status = true;
      update_server_state(&ServerState, SERVER_START); //Update to Start State
    }
  }
  if(ServerState == DISCONNECT) {
    //Wifi wait to connect from client
    if(WiFi.softAPgetStationNum()) {
      //Wait to receive start
      if(!Client.connected()) {
        Client = Server.available();      //listen for incoming clients
      } else {
        Client.setTimeout(10);
        update_server_state(&ServerState, RE_CONNECT);  //Update to Connected state
        Serial.println("Client re-connect successful");
      }
    }
  }
}

/**
  * @brief  Loss connection system state.
  * @param SystemSpec The specification of the system.
  * @retval None
  */
void System_loss_connection (system_spec_t *SystemSpec) {
  at_netclose(SystemSpec->SimSerial);
  delay(100);
  at_netopen(SystemSpec->SimSerial);
  delay(100);
  //Reconnect to network
  re_connect_network(SystemSpec);
  //Revert Server state to WAIT_CLIENT
  ServerState = WAIT_CLIENT;
  //Update to Connecting state
  SystemState = update_system_state(SYS_RE_CONNECT_EVENT);
}
/**
  * @brief  Connecting system state.
  * @param SystemSpec The specification of the system.
  * @retval None
  */
void System_connecting (system_spec_t *SystemSpec) {
  static bool decision = false;
  type_process_t process;
  String Buffer = "";
  if(!decision) {
    Buffer = read_all_buffer(SystemSpec->SimSerial);
    check_match_important_header(process);
    while(Buffer.indexOf('\n') != (size_t)(-1)) {
      process = process_parse(&Buffer);
      if(process.header == "+CIPOPEN" && process.NumberOfAttribute == 2 &&
        process.Attribute[1] == "0") {
        decision = true;
        if(DEBUG == 1)
        Serial.print(process.Msg);
      }
      if(process.header == "+CIPOPEN" && process.NumberOfAttribute == 2 &&
         process.Attribute[1] != "0") {
           //Update to Loss_Connection state
           SystemState = update_system_state(SYS_TIMEOUT_EVENT);
         }
    }
  }
  if(decision) {
    //Update to Connected state
    SystemState = update_system_state(SYS_CONNECT_SUCC_EVENT);
    decision = false;
  }
}
/**
  * @brief  Initialized the system.
  * @param SystemSpec The specification of the system.
  * @retval None
  */
void System_initializing (system_spec_t *SystemSpec) {
  //Start Serial
  WiFi.mode(WIFI_MODE_STA);
  serial_initialize();
  //Interface with module SIM
  sim_init(SystemSpec->SimSerial);

  delay(100);
  check_connection_request(SystemSpec->SimSerial);
  //Setup network
  re_connect_network(SystemSpec);
  //Update to Connecting state
  SystemState = update_system_state(SYS_INITIALIZED_EVENT);
}

/**
  * @brief  Process all commands.
  * @param SystemSpec The specification of the system.
  * @retval None
  */
void process_command (system_spec_t *SystemSpec) {
  String TempBuffer = "";
  type_process_t process;
  TempBuffer = read_all_buffer(SystemSpec->SimSerial);
  *(SystemSpec->GlobalBuffer) += TempBuffer;
  process = process_parse(SystemSpec->GlobalBuffer);
  //Server close connection
  if(process.header == HEADER_IPCLOSE) {
    //Update to Loss_connection state
    *(SystemSpec->SystemState) = update_system_state(SYS_LOSS_EVENT);
    if(DEBUG == 1)
      Serial.print(process.Msg);
  }
  if(process.header == HEADER_CIPOPEN) {
    if(process.header == "+CIPOPEN" && process.NumberOfAttribute == 2 &&
      process.Attribute[1] == "0") {
      //Update to Connected state
      *(SystemSpec->SystemState) = update_system_state(SYS_CONNECT_SUCC_EVENT);
      if(DEBUG == 1)
        Serial.print(process.Msg);
    }
    if(process.header == "+CIPOPEN" && process.NumberOfAttribute == 2 &&
        process.Attribute[1] != "0") {
          //Update to Loss_Connection state
          SystemState = update_system_state(SYS_TIMEOUT_EVENT);
    }
  }
  if(process.header == HEADER_CAM) {
    if(process.Attribute[0] == "START") {
      SystemSpec->CameraControl.control = true;
      if(DEBUG == 1)
        SystemSpec->Serial->print(process.Msg);
    }
    if(process.Attribute[0] == "STOP") {
      SystemSpec->CameraControl.control = false;
      if(DEBUG == 1)
        SystemSpec->Serial->print(process.Msg);
    }
  }
}
/**
  * @brief  Re-connect UDP and TCP.
  * @param SystemSpec The pointer of the system Specification.
  * @retval None
  */
void re_connect_network (system_spec_t *SystemSpec) {
  bool decision = false;
  //Setup UDP connection
  do {
    at_cipopen(SystemSpec->Udp, SystemSpec->SimSerial);
    delay(1000);
    decision = check_connection_request(SystemSpec->SimSerial);
  } while (!decision);
  //Setup TCP connecting and don't wait for successful
  at_cipopen(SystemSpec->Tcp, SystemSpec->SimSerial);
}
/**
  * @brief  Check connection open successfully.
  * @param SimSerial The Serial of the SIM module.
  * @retval bool the result of the request
  */
bool check_connection_request (HardwareSerial *SimSerial) {
  bool retval = false;
  type_process_t process;
  String Buffer = "";
  Buffer = read_all_buffer(SimSerial);
  check_match_important_header(process);
  while(Buffer.indexOf('\n') != (size_t)(-1)) {
    process = process_parse(&Buffer);
    if(process.header == "+CIPOPEN" && process.NumberOfAttribute == 2 &&
      process.Attribute[1] == "0") {
      retval = true;
      if(DEBUG == 1)
      Serial.print(process.Msg);
    }
  }
  return retval;
}
/**
  * @brief  Check the message match the important header.
  * @param Process The process of the buffer.
  * @retval bool the result
  */
bool check_match_important_header (type_process_t Process) {
  bool retval = false;
  for(size_t i = 0; i < LengthOfImportantHeader; i ++) {
    if(Process.header == ImportantHeader[i]) {
      retval = true;
      GlobalBuffer = GlobalBuffer + Process.Msg;
    }
  }
  return retval;
}
/**
  * @brief  Read all buffer function.
  * @param Serial The Serial want to read buffer.
  * @retval String The value of the buffer
  */
String read_all_buffer (HardwareSerial *Serial) {
  String retval = "";
  while(Serial->available() >= 2) {
    retval += Serial->readStringUntil('\n') + '\n';
  }
  return retval;
}

/**
  * @brief  The function using update system FSM.
  * @param Event The event of the system
  * @retval System_state_t Put the pointer of the current state
  */
System_state_t update_system_state(System_event_t Event) {
  static System_state_t retval = SYS_INITIALIZING;
  switch(retval) {
    case SYS_INITIALIZING:
      if(Event == SYS_INITIALIZED_EVENT) {
        retval = SYS_CONNECTING;
      }
      break;
    case SYS_CONNECTING:
      if(Event == SYS_CONNECT_SUCC_EVENT) {
        retval = SYS_CONNECTED;
      }
      if(Event == SYS_TIMEOUT_EVENT) {
        retval = SYS_LOSS_CONNECTION;
      }
      break;
    case SYS_CONNECTED:
      if(Event == SYS_LOSS_EVENT) {
        retval = SYS_LOSS_CONNECTION;
      }
      break;
    case SYS_LOSS_CONNECTION:
      if(Event == SYS_RE_CONNECT_EVENT) {
        retval = SYS_CONNECTING;
      }
      break;
    default:
    break;
  }
  system_state_debug(retval);
  return retval;
}

/**
  * @brief  Debug state.
  * @retval void
  */
void system_state_debug (System_state_t state) {
  String StateMsg = "";
  switch (state)
  {
  case SYS_INITIALIZING:
    StateMsg = "System Initializing state";
    break;
  case SYS_CONNECTING:
    StateMsg = "System Connecting state";
    break;
  case SYS_CONNECTED:
    StateMsg = "System Connected state";
    break;
  case SYS_LOSS_CONNECTION:
    StateMsg = "System Loss Connection state";
    break;
  default:
    break;
  }
  Serial.print("New state is:");
  Serial.println(StateMsg);
}

/**
  * @brief  The parse string processing function.
  * @param InfoBuffer Put the buffer of commands
  * @retval type_process_t
  */
type_process_t process_parse(String* InfoBuffer) {
	type_process_t retval;
	String tempString = "";
	bool Unexpected;
  retval.header = "";
	tempString = parse_buffer_on_terminate(InfoBuffer);
	retval.Msg = tempString;
	Unexpected = check_unexpected_message(tempString);
	if (Unexpected == false) {
		retval.header = parse_white_space(&tempString);
		if (tempString != "") {
			retval.NumberOfAttribute = parse_tail(tempString, retval.Attribute);
		}
	}
	return retval;
}

/**
  * @brief  Check the unexpected messages.
  * @param Info Put the message to compare
  * @retval bool
  */
bool check_unexpected_message(String Info) {
	bool retval = false;
	for (size_t i = 0; i < LengthOfIgnore; i++) {
		if (Info == IgnoreString[i])
			retval = true;
	}
	return retval;
}

/**
  * @brief  The function to parse the command from the buffer.
  * @param Info The buffer
  * @retval String The command
  */
String parse_buffer_on_terminate(String* Info) {
	String temp = "";
	size_t pos = Info->indexOf("\r");
	if (pos != (size_t)(-1)) {
		temp = Info->substring(0, pos + 2);
		*(Info) = Info->substring(pos + 2, Info->length());
	}
	return temp;
}

/**
  * @brief  The function parse the header.
  * @param Info The command.
  * @retval String the header.
  */
String parse_white_space(String *Info) {
	String temp = "";
	size_t checkColon = Info->indexOf(":");
	if (checkColon != (size_t)(-1)) {
		temp = Info->substring(0, Info->indexOf(":"));
		*(Info) = Info->substring(Info->indexOf(":") + 2, Info->length());
	}
	else {
		temp = Info->substring(0, Info->indexOf("\r"));
		*(Info) = "";
	}
	return temp;
}

/**
  * @brief  The function parse the tail of the command.
  * @param Info The buffer.
  * @param Attr The pointer of the Attribute.
  * @retval size_t The number of Attributes.
  */
size_t parse_tail(String Info, String *Attr) {
	size_t Number = 0;
	while (Info.indexOf(",") != (size_t)(-1) ) {
		Attr[Number] = Info.substring(0, Info.indexOf(","));
		Info = Info.substring(Info.indexOf(",") + 1, Info.length());
		Number++;
	}
	if (Info.indexOf("\r") != 0) {
		Attr[Number] = Info.substring(0, Info.indexOf("\r"));
		Number++;
	}
	return Number;
}

/**
  * @brief  Serial Initialization.
  * @retval None
  */
void serial_initialize (void) {
  Serial.begin(115200);
  Serial.setTimeout(5000);
  // while(!Serial);
  Serial.println("Hello World ESP32");

  Serial.println("Wait setup the 4G Module");
  for(int i = 0; i < 10; i++) {
    Serial.print(F("..."));
    delay(1000);
  }
  Serial.println();
}
/**
  * @brief  Wifi Initialization.
  * @retval None
  */
void wifi_initialize (void) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println();
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP Ipv4 Address is: ");
  Serial.println(IP);       //Default IP 192.168.4.1
  Server.begin(8000);

  //Wifi wait to connect from client
  while(!WiFi.softAPgetStationNum());
  Serial.println("We have the connected");
  //Wait to receive start
  while(!Client.connected()) {
    Client = Server.available();      //listen for incoming clients
  }
  Client.setTimeout(5);
  Serial.println("Client connect successful");
}
/**
  * @brief  Receive and stream data from Camera.
  * @retval The length of the frame
  */
uint32_t receive_and_stream_cam(WiFiClient *Client, HardwareSerial *SimSerial) {
  uint16_t FrameLength = 0;
  // int64_t fr_start = esp_timer_get_time();
  FrameLength = read_send_to_server(SimSerial, Client);
  client_send_message(Client, StreamACK);
  // int64_t fr_stop = esp_timer_get_time();
  // Serial.printf("The frame length is:%u\r\n", FrameLength);
  // Serial.printf("Take %u ms, %f fps\r\n",(uint32_t) (fr_stop - fr_start)/1000,
  //               (float)1000000/(fr_stop - fr_start));
  return FrameLength;
}
/**
  * @brief  Initialize the module SIM.
  * @retval None
  */
void sim_init (HardwareSerial *SimSerial) {
  String Cmd = "AT+IPREX=3000000";
  SimSerial->begin(3000000, SERIAL_8N1, RX1, TX1);
  at_sendat(SimSerial);
  // SimSerial->println(Cmd);
  // SimSerial->begin(3000000);
  // at_sendat(SimSerial);
  forward_serial2(SimSerial, "AT&F");
  forward_serial2(SimSerial, "ATE0");
  forward_buffer(SimSerial);
  at_netclose(SimSerial);
  delay(2000);
  at_netopen(SimSerial);
}
/**
  * @brief  Read Frame and Send to Server.
  * @retval Length of data read
  */
uint16_t read_send_to_server (HardwareSerial *SimSerial, WiFiClient *Client) {
  bool endframe = false;
  uint16_t FrameLength = 0;
  char* Buffer;
  size_t ReceiveLength = 0;
  Network_t Network;
  Buffer = new char[1081];
  Network.LinkNum = "3";
  Network.RemotePort = "8000";
  Network.ServerIP = ServerIP;
  Network.Transport = "UDP";
  while(endframe == false) {
    ReceiveLength = Client->readBytesUntil('\0', Buffer, 1080);
    FrameLength += ReceiveLength;
    if(ReceiveLength == 0) {
      endframe = true;
    }
    else {
      Network.buffer = Buffer;
      if(ReceiveLength < 1080) {
        Buffer[ReceiveLength] = '\0';
        Network.size = ReceiveLength + 1;
        endframe = true;
      } else {
        Network.size = ReceiveLength;
      }
      at_send_buffer(SimSerial, Network);
    }
  }
  delete[] Buffer;
  return FrameLength;
}
/**
  * @brief  Update Server State function.
  * @retval None
  */
void update_server_state(Server_state_t *State,
                                   Server_state_t Action) {
  Server_state_t retval = *(State);
  switch (*(State))
  {
    case WAIT_CLIENT:
      if(Action == CONNECTED) {
        retval = CONNECTED;
      }
      break;
    case CONNECTED:
      if(Action == SERVER_START) {
        retval = START;
      }
      break;
    case START:
      switch (Action)
      {
        case RECEIVE_FIRST:
          retval = STREAMING;
          break;
        case TIMEOUT:
          retval = DISCONNECT;
        default:
          break;
      }
      break;
    case STREAMING:
      switch (Action) {
        case ACK:
          retval = STREAMING;
        break;
        case TIMEOUT:
          retval = DISCONNECT;
        break;
        case SERVER_STOP:
          retval = STOP;
        break;
        default:
        break;
      }
    break;
    case STOP:
      if(Action == SERVER_START) {
        retval = START;
      }
    break;
    case DISCONNECT:
      if(Action == RE_CONNECT) {
        retval = CONNECTED;
      }
    break;
    default:
      break;
  }
  server_state_debug(retval);
  *(State) = retval;
}
/**
  * @brief  Debug state.
  * @retval void
  */
void server_state_debug (Server_state_t state) {
  String StateMsg = "";
  switch (state)
  {
  case WAIT_CLIENT:
    StateMsg = "wait client";
    break;
  case CONNECTED:
    StateMsg = "connected";
    break;
  case START:
    StateMsg = "start";
    break;
  case STREAMING:
    StateMsg = "Streaming";
    break;
  case STOP:
    StateMsg = "Stop";
    break;
  case DISCONNECT:
    StateMsg = "Disconnect";
    break;
  default:
    break;
  }
  Serial.print("New state is:");
  Serial.println(StateMsg);
}
/**
  * @brief  Send message to Client.
  * @retval void
  */
void client_send_message(WiFiClient *Client, String Msg) {
  Client->print(Msg);
}
void forward_serial2(HardwareSerial *SimSerial, String Cmd) {
  String character;
  int times = 2;
  SimSerial->println(Cmd);
  Serial.print(Cmd);
  for(int i = 0; i < times; i++) {
    character = SimSerial->readStringUntil('\n');
    Serial.println(character);
  }
  delay(100);
}

void forward_buffer(HardwareSerial *SimSerial) {
  if(SimSerial->available()) {
    while(SimSerial->available())
      Serial.write(SimSerial->read());
  }
}
void at_netopen(HardwareSerial *SimSerial) {
  String Cmd = "AT+NETOPEN";
  String character;
  SimSerial->println(Cmd);
  for(int i = 0; i < 2; i++) {
    character = SimSerial->readStringUntil('\n');
    Serial.println(character);
  }
  Serial.println(Cmd);
  delay(100);
}

void at_cipopen(network_t *Network, HardwareSerial *SimSerial) {
  String Cmd = "";
  String character;
  if(Network->Transport == "UDP") {
    Cmd = "AT+CIPOPEN=" + Network->Num + ",\"" + Network->Transport + "\",,," +
          Network->LocalPort;
  }
  else {
    Cmd = "AT+CIPOPEN=" + Network->Num + ",\"" + Network->Transport + "\",\"" +
          Network->IpServer + "\"," + Network->RemotePort + "," +
          Network->LocalPort;
  }
  if(DEBUG == 1)
    Serial.println(Cmd);
  SimSerial->println(Cmd);
}

void at_send_buffer(HardwareSerial *SimSerial, Network_t Network) {
  String Ip = Network.ServerIP;
  String Link = Network.LinkNum;
  String Port = Network.RemotePort;
  String SimBuffer = "";
  type_process_t process;
  char *Buffer = Network.buffer;
  size_t Byte = Network.size;
  String Cmd = "AT+CIPSEND=" + Link + ",,\"" + Ip + "\"," + Port;
  SimSerial->println(Cmd);
  SimBuffer = SimSerial->readStringUntil('>');
  while(SimBuffer != "") {
    process = process_parse(&SimBuffer);
    check_match_important_header(process);
  }
  SimSerial->write(Buffer, Byte);
  SimSerial->print('\x1A');
  while(1) {
    SimBuffer = SimSerial->readStringUntil('\n') + "\n";
    process = process_parse(&SimBuffer);
    check_match_important_header(process);
    if(process.header == HEADER_CIPSEND)
      break;
  }
}
void at_netclose(HardwareSerial *SimSerial) {
  String Cmd = "AT+NETCLOSE";
  String character;
  SimSerial->println(Cmd);
  delay(2000);
  forward_buffer(SimSerial);
}
void at_sendat (HardwareSerial *SimSerial) {
  String Cmd = "AT";
  String character;
  while(1) {
    SimSerial->println(Cmd);
    character = SimSerial->readStringUntil('\n');
    Serial.println(character);
    character = SimSerial->readStringUntil('\n');
    Serial.println(character);
    if(character == "OK\r") {
      Serial.println("SIM is ready");
      break;
    }
    Serial.println(character);
    Serial.println("Connecting with SIM failed, try again...");
    delay(2000);
  }
}

/* USER CODE END 1 */
