const int  IBUS_MONITOR_PIN = 3;


typedef byte IbusId;
const byte MAX_IBUS_MSG_SIZE = 64;
const byte MIN_IBUS_MSG_SIZE = 5;
const int  IBUS_FREE = LOW;
const int  IBUS_BUSY = HIGH;

struct IbusMsg {
  IbusId txId;
  byte len;
  IbusId rxId;
  byte command;
  byte data[MAX_IBUS_MSG_SIZE];
  byte checkSum;

  boolean operator==(IbusMsg &other) {return (txId != other.txId) && (rxId == other.rxId) && (command == other.command);}

  inline byte dataLength() {return len - 3;} // - (COMMAND + CHKSUM + RXID)

  inline byte fullLength() {return len + 2;} // + (TXID + LEN)

  byte computeChkSum() {
    byte chk = txId ^ len ^ rxId ^ command;
    int lngth = dataLength();
    for (int i = 0; i < lngth; i++) 
        chk ^= data[i];
    return chk;
  }
};

class Ibus {
  
  private:
    enum IbusParseResult {
      IPR_OK,
      IPR_CHECKSUM_MISMATCH,
      IPR_NOT_ENOUGH
    };

    class Buffer {
      private:
        static const int BUFFER_SIZE = 256;
        byte buff[BUFFER_SIZE];
        byte start, end = 0;

      public:
        inline boolean put(byte b) {
          if (end + 1 == start) return false;
          buff[end++] = b;
          return true;
        }

        inline void clear() {start = end = 0;}

        inline void drop(byte n) {n < available() ? start += n : start = end = 0;}

        inline byte available() {return end - start;}

        inline byte& operator[](byte i) {return buff[start + i];}
    };

    class IbusQueue {
      static const int QUEUE_SIZE = 16;
      IbusMsg queue[QUEUE_SIZE];
      int start, end = 0; 

      inline int next(int i) {
        return (i + 1) % QUEUE_SIZE;
      }
      
      public:
        boolean isEmpty() {
          return start == end;
        }

        // need to check isEmpty() before!!
        IbusMsg& peek() {
           return queue[start];
        }

        //return true when queue was not empty
        boolean remove() {
           if (start == end) return false;
           start = next(start);
        }

        boolean push(IbusMsg& ibusMsg) {
          if (next(end) == start) return false;
          end = next(end);
          queue[end] = ibusMsg;
        }

        void clear() {
          start = end = 0;  
        }
    };

    volatile boolean isWriting; //in case we use interrupts
    IbusQueue queue;
    Buffer buffer;
    
    int statusPin;
    int statusLed;
    //int writeStatusLed;
    void (*onMsg)(IbusMsg&);
    
    HardwareSerial& serial;

    void scanIbusBuffer(Buffer& buff) {
      IbusMsg ibusMsg;
      while (buff.available() >= MIN_IBUS_MSG_SIZE) {
        switch (parseIbusMsg(ibusMsg, buff)) {
          case IPR_OK:
            buffer.drop(ibusMsg.fullLength());

            if (!queue.isEmpty() && ibusMsg == queue.peek()) {
              //we recieve our message, we can finally dequeue it
              queue.remove();
              isWriting = false;
            }

            (*onMsg)(ibusMsg);
            break;
          case IPR_NOT_ENOUGH:
            return;
          case IPR_CHECKSUM_MISMATCH:
            buff.drop(1);
            break;
        }
      }
    }

    IbusParseResult parseIbusMsg(IbusMsg &msg, Buffer &buff) {
      
      byte fullLength = buff[1] + 2;
      if (fullLength > MAX_IBUS_MSG_SIZE || fullLength < MIN_IBUS_MSG_SIZE) return IPR_CHECKSUM_MISMATCH;
      if (fullLength > buff.available()) return IPR_NOT_ENOUGH;
      
      msg.txId     = buff[0];
      msg.len      = buff[1];
      msg.rxId     = buff[2];
      msg.command  = buff[3];
      msg.checkSum = buff[msg.len + 1];
      
      byte lngth = msg.dataLength();
      for (byte i = 0; i < lngth; i++)
        msg.data[i] = buff[i + 4];
      
      if (msg.computeChkSum() != msg.checkSum) return IPR_CHECKSUM_MISMATCH;
      return IPR_OK;
    }

    void write(IbusMsg &msg) {
      serial.write(msg.txId);
      serial.write(msg.len);
      serial.write(msg.rxId);
      serial.write(msg.command);
      serial.write(msg.data, msg.dataLength());
      serial.write(msg.checkSum);
    }

  public:
    Ibus(HardwareSerial& s, void (*handler)(IbusMsg&), int sp, int sl):
      serial(s),
      statusPin(sp),
      statusLed(sl),
      onMsg(handler)
    {}

    void init() {
      pinMode(statusPin, INPUT);
      pinMode(statusLed, OUTPUT);
    
      //wait until ibus free to begin serial
      while (digitalRead(statusPin) == IBUS_BUSY) {}
      serial.begin(9600);
    }
      
    // will be nice to attach interrupt
    void routine() {
      int ibusState = digitalRead(IBUS_MONITOR_PIN);
      digitalWrite(statusLed, ibusState);

      if (ibusState == IBUS_BUSY) {
        isWriting = false;
      } else {
        if (!isWriting && !queue.isEmpty()) {
          IbusMsg& ibusMsg = queue.peek();
          write(ibusMsg); //supposed to be non blocking.
          isWriting = true;
        }
      }
    }

    void onBufferOverflowed() {
      buffer.clear();
      Serial.println("Ibus buffer overflowed");
    }

    void onSerial() {
      while (serial.available()) {
        if (!buffer.put(serial.read())) onBufferOverflowed();
      }
      scanIbusBuffer(buffer);
    }

    boolean send(IbusMsg& msg) {
      boolean res = queue.push(msg);
      if (!res) return false;
      routine();
      return true;
    }
};

//===== INITIALIZATION =====
void onIbusMsg(IbusMsg &ibusMsg);

Ibus ibus(Serial1, onIbusMsg, IBUS_MONITOR_PIN, LED_BUILTIN);

void serialEvent1() {
  ibus.onSerial();
}

void setup() {
  ibus.init();
  Serial.begin(9600);
}

void loop() {
  ibus.routine();
}

//===== APPLICATION =====
#define IBUS_IDS(XX) \
  XX(0x68, RAD) \
  XX(0xF0, MON) \
  XX(0x3b, GRAPH) \
  XX(0x80, IKE) \
  XX(0x50, MFL) \
  XX(0xED, VID) \
  XX(0x3F, DIA) \
  XX(0xD0, LCM) \
  XX(0xE7, OBC)


#define XX(code, name) IBUS_##name = code,
enum ECU {
  IBUS_IDS(XX)
};
#undef XX

void onIbusMsg(IbusMsg &msg) {
  if ((msg.rxId == IBUS_IKE || msg.txId == IBUS_IKE)
   && !(msg.command == 1 || msg.command == 2)){
    printlnMsg(Serial, msg);
  }
}

void printlnMsg(HardwareSerial& serial, IbusMsg &msg) {
  serial.print(getEcuName(msg.txId));
  serial.print("->");
  serial.print(getEcuName(msg.rxId));
  serial.write(':');
  serial.print(getCommandName(msg.rxId, msg.command));
  int lngth = msg.dataLength();
  
  if (lngth > 0) {
    serial.write(':');
    if (lngth == 1)
      serial.print(hex(msg.data[0]));
    else {
      serial.write(msg.data, lngth);
    }
  }
  serial.println();
}

String hex(byte b) {
  String code = String(b, HEX);
  return  (b < 16) ? "0" +  code : code;
}

void printRawMsg(HardwareSerial& serial, IbusMsg &msg) {
  serial.print(hex(msg.txId));
  serial.print(hex(msg.len));
  serial.print(hex(msg.rxId));
  serial.print(hex(msg.command));
  for (int i = 0; i < msg.dataLength(); i++) {
    serial.print(hex(msg.data[i]));
  }
  serial.print(hex(msg.checkSum));
}

String getCommandName(IbusId rxId, byte code) {
  switch (code) {
    case 0x01: return "Status?";
    case 0x02: return "Ready";
    default: return hex(code);
  }
}

String getEcuName(IbusId code) {
#define XX(code, name) case ##name: return #name;
  switch (code) {
      IBUS_IDS(XX);
    default: return "*" + hex(code);
  }
#undef XX
}
