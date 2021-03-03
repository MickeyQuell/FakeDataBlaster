//#include <TinyPICO.h>
#include <ArduinoJson.h>

/////////////////////////////////////////////////////////////////////
/////////////////////////////    API   //////////////////////////////
// {"msg":"reset","verbose":"1"}
// {"msg":"setDevice","id":"123456","type":"l"}
// {"msg":"test","string":"ABC123"}
// {"msg":"getstatus"}

// {"msg":"start","rate":"20"}
// {"msg":"stop"}


// {"msg":"setDevice","id":"123456","type":"l","baud":"256000"}
// {"msg":"echo","string":"test123"}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#define MaxReadBytes 64
void GetMessageFromPort(String& s);
void PrintMessage(const char* message, int verbose);
void SendMessageToPort(const char* message, int verbose);

int Verbosity = 1;

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

class MessageReceiver // interface
{
  public:
    virtual  bool HandleMessage(const StaticJsonDocument<200>& doc, const String& msgString ) = 0;
};

class MartinListener : public MessageReceiver
{
  int startFrame;
  bool isRunning;
  unsigned long timeStamp;
  float rate;
  
  public:
    MartinListener(): startFrame(0), isRunning(false), rate(1) {}
  public:
    bool HandleMessage(const StaticJsonDocument<200>& doc, const String& msgString) override
    {
       if(msgString.compareTo("start") == 0)
        {
          PrintMessage("start", 1);
          float r = doc["rate"];
          if(r)
            rate = r;
          startFrame = 0;
          isRunning = true;
          timeStamp = millis();
          return true;
        }
        if(msgString.compareTo("stop") == 0)
        {
          PrintMessage("stop", 1);
          startFrame = 0;
          isRunning = false;
          return true;
        }
        return false;
    }
  
    bool IsRunning() const {return isRunning;}
    void GetLine()
    {
      if(isRunning == false)
        return;

      unsigned long currentTime = millis();
      float timeElapsed = (float)(currentTime - timeStamp);
      timeStamp = currentTime;

      float millisPer = 1000.0f / rate;
      float num = timeElapsed / millisPer;
      for(float i=0; i< num; i++)
      {
        PrintSingleSample();
      }
    }

    int GetBaudRate() { return 256000;}

    void PrintSingleSample()
    {
      //f[00001]{0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,01234} 
      Serial.print("f[");
      Serial.printf("%05d", startFrame++);      
      Serial.print("]{");
      Serial.printf("%04d,%04d,%04d,%04d,%04d,%04d,", 0, 0, 0, 0, 0, 0);  // left
      Serial.printf("%04d,%04d,%04d,%04d,%04d,%04d,", 0, 0, 0, 0, 0, 0); // center 
      Serial.printf("%04d,%04d,%04d,%04d,%04d,%04d,", 0, 0, 0, 0, 0, 0); // right
      Serial.printf("%04d,", 0); // barometer
      Serial.printf("%05d", 1234); // controlls
      Serial.println("}");
    }
};

/////////////////////////////////////////////////////////////////////

template<typename T>
struct CallbackArray
{
  static const int maxNum = 10;
  T array[maxNum];
  int index;
  CallbackArray(): index(0){}
  void Add(T& t) 
  {
    if(index >= maxNum)
    {
      PrintMessage("too many elements in container", 1);
      return;
    }
    array[index++] = t;
  }
  bool HandleMessage(const StaticJsonDocument<200>& doc, const String& msgType)
  {
    for(int i=0; i<index; i++)
    {
      auto var = array[i];
      bool result = var->HandleMessage(doc, msgType );
      if(result == true)
        return true;
    }
    return false;
  }
};

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

class CommsStatus
{
  public:
  enum Status{init, resetting, awaiting_validation, connected, invalid};
  enum DeviceType{left, right, pod};
  
    String id;
    Status status;
    int deviceType;
    CallbackArray<MessageReceiver*> callbacks;
    int baudRate;
    
  public:
    CommsStatus(): status(Status::init), deviceType(DeviceType::left), baudRate(9600){}
    void ResetStats(){}
    void SetBaudRate(int baud) {baudRate = baud;}
    bool IsConnected(){return false;}
    void Add(MessageReceiver* callback){ callbacks.Add(callback);}
    bool HandleMessage(String& json)
    {
        String msgString;
       StaticJsonDocument<200> doc;
       if(Deserialize(json, doc, msgString) == false)
       {
          return false;
       }

       if(HandleStatus(msgString))
       {
          return true;
       }
       if(HandleReset(msgString, doc) == true)
       {
          return true;
       }
       if(status == invalid) {OutputStatus();return false;}
       if(status == Status::connected)
       {
         callbacks.HandleMessage(doc, msgString);
          return true;
       }
       if(HandleDeviceSettingMessages(msgString, doc) == true) { return true; }
       if(HandleAwaitingValidation(msgString, doc) == true) { return true; }
    
       return true;
    }
    
  private:
    bool Deserialize(const String& json, StaticJsonDocument<200>& doc, String& msgString)
    {
       DeserializationError error = deserializeJson(doc, json);
       if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.f_str());
          return false;
        }
       const char* messageType = doc["msg"];
       if(messageType == nullptr)
       {
        Serial.print(F("CommsStatus rejects msg"));
        return false;
       }
       msgString = messageType;
       return true;
    }
    bool HandleStatus(String& msgString)
    {
      if(msgString.compareTo("getstatus") == 0)
       {
        OutputStatus();
        return true;
       }
       return false;
    }
    void OutputStatus()
    {
      switch(status)
        {
          case init: 
            PrintMessage("status: init", 0);
            break;
           case resetting: 
            PrintMessage("status: resetting",0);
            break;
           case awaiting_validation: 
            PrintMessage("status: awaiting_validation",0);
            break;
           case connected: 
            {
              SendMessageToPort("status: connected",0);
              PrintMessage("Baud: ",0);
              Serial.println(baudRate, DEC);
            }
            break;
           case invalid:
            SendMessageToPort("status: invalid", 0);
            break;
           default:
            SendMessageToPort("status: unknown", 0);
            break;
        }
    }
    bool HandleReset(String& msgString, StaticJsonDocument<200>& doc)
    {
      if(msgString.compareTo("reset") == 0)
       {
        status = Status::resetting;
        int verbose = doc["verbose"];
        if(verbose)
        {
          Verbosity = verbose;
        }
        PrintMessage("CommsStatus resets", 0);
        return true;
       }
       return false;
    }
    bool HandleDeviceSettingMessages(String& msgString, StaticJsonDocument<200>& doc)
    { 
      if(msgString.compareTo("setDevice") != 0)
        return false;
      
      if(status != Status::resetting)
      {
        PrintMessage("HandleDeviceSettingMessages bad state",0);
        status = Status::invalid;
        OutputStatus();
        return false;
      }
      
      const char* _id = doc["id"];
      id = _id;
      deviceType = doc["type"];
      PrintMessage("CommsStatus setDevice", 1);
      char buff[50];
      sprintf(buff, "d", deviceType);
      SendMessageToPort(buff, 1);
      Serial.print(deviceType, DEC);
      
      int baud = doc["baud"];
      if(baud > 0)
         baudRate = baud;

      StaticJsonDocument<100> ack;
      ack.clear();
      ack["ack"] = "setDevice";
      ack["id"] = id;
      if(baudRate != 0)
        ack["baud"] = baudRate;
      serializeJson(ack, Serial);
      status = Status::awaiting_validation;
    /*  if(baudRate != 0)
      {
        Serial.print(F("resetting baud rate"));
        Serial.begin(baudRate);
      }*/
      return true;
    }

    bool HandleAwaitingValidation(String& msgString, StaticJsonDocument<200>& doc)
    {
      if(msgString.compareTo("test") != 0)
      {
        return false;
      }
      if(status != Status::awaiting_validation)
      {
        SendMessageToPort("HandleAwaitingValidation bad state",0);
        status = Status::invalid;
        OutputStatus();
        return false;
      }
      
      const char* randomString = doc["string"];
      SendMessageToPort("CommsStatus HandleAwaitingValidation",1);

      StaticJsonDocument<100> ack;
      ack.clear();
      ack["ack"] = "test";
      ack["string"] = randomString;
      ack["id"] = id;
      serializeJson(ack, Serial);
      status = Status::connected;
      return true;      
    }    
};

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

/// global status ///
CommsStatus status;
MartinListener listener;

void setup() 
{
  Serial.begin(listener.GetBaudRate());
  status.SetBaudRate(listener.GetBaudRate());
  status.Add(&listener);
}


////////////////////////////////////////////////////////////
void loop() 
{
  String message;
  GetMessageFromPort(message);
  if(message.length() > 0)
  {
    Serial.println(message.c_str());    
    status.HandleMessage(message);
    
  }
  listener.GetLine();
  delay(100);
}

////////////////////////////////////////////////////////////

void GetMessageFromPort(String& s)
{
  if(s.length() != 0)
  {
    s = "";
  }
  int numBytes = Serial.available();
  if(numBytes >0)
  {
    char buffer[MaxReadBytes+1];// max is 64 bytes on a read
   //Serial.readBytes(buffer, numBytes);
   s = Serial.readString();
  }
}

void PrintMessage(const char* message, int verbose)
{
  if(verbose <= Verbosity)
    Serial.print(message); 
}
void SendMessageToPort(const char* message, int verbose)
{
  if(verbose <= Verbosity)
      Serial.println(message); 
}

////////////////////////////////////////////////////////////

 
////////////////////////////////////////////////////////////
