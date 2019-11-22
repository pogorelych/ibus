typedef byte IbusId;

struct IbusMsg {
  IbusId tx;
  byte len;
  IbusId rx;
  byte cmd;
  byte* data;
  byte ck_sum;
  
  IbusMsg() {}
  
  IbusMsg(IbusId tx_, byte len_, IbusId rx_, byte cmd_, byte* data_):
    tx(tx_),
    len(len_),
    rx(rx_),
    cmd(cmd_),
    data(data_)
  {
    ck_sum = computeCkSum();
  }
  
  boolean operator==(IbusMsg &other) {return (tx == other.tx) && (rx == other.rx) && (cmd == other.cmd);}

  inline byte dataLength() {return len - 3;} // - (CMD + CKSUM + RX)

  inline byte fullLength() {return len + 2;} // + (TX + LEN)

  byte computeCkSum() {
    byte chk = tx ^ len ^ rx ^ cmd;
    int lngth = dataLength();
    for (int i = 0; i < lngth; i++) 
        chk ^= data[i];
    return chk;
  }
};

class Ibus {
  private:
    static const int  IBUS_FREE = LOW;
    static const int  IBUS_BUSY = HIGH;
    static const byte MAX_IBUS_DATA_SIZE = 32;
    static const byte MIN_IBUS_MSG_SIZE = 5; //TX + LEN + RX + CMD + CKSUM
    static const byte MAX_IBUS_MSG_SIZE = MAX_IBUS_DATA_SIZE + MIN_IBUS_MSG_SIZE;

    //current buffer content status
    enum IbusState {
      MESSAGE, //we have full message at the beginning of the buffer
      CKSUM_MISMATCH,  //data inconsistency detected
      NOT_ENOUGH //there isn't enough data to parse message
    };

    class Buffer {
      private:
        static const int BUFFER_SIZE = 256;
        byte buff[BUFFER_SIZE];
        byte start = 0;
        byte end = 0;

      public:
        inline boolean put(byte b) {
          if (end + 1 == start) return false;
          buff[end++] = b;
          return true;
        }

        inline void clear() {start = end = 0;}

        inline void remove(byte n) {n < available() ? start += n : start = end = 0;}

        inline byte available() {return end - start;}

        inline byte& operator[](byte i) {return buff[start + i];}
    };

    class IbusQueue {
      static const int QUEUE_SIZE = 16;
      IbusMsg queue[QUEUE_SIZE];
      int start, end = 0; 

      inline int next(int i) {return (i + 1) % QUEUE_SIZE;}
      
      public:
        boolean isEmpty() {return start == end;}

        // need to check isEmpty() before!!
        IbusMsg& peek() {return queue[start];}

        //return true when queue was not empty
        boolean remove() {
           if (start == end) return false;
           start = next(start);
        }

        boolean push(IbusMsg& ibusMsg) {
          if (next(end) == start) return false;
          queue[end] = ibusMsg;
          end = next(end);
        }

        void clear() {
          start = end = 0;  
        }
    };

    volatile boolean isWriting; //in case we use interrupts
    IbusQueue queue;
    Buffer buffer;
    
    const int statusPin;
    const int statusLed;
    void (*onMsg)(IbusMsg&);
    byte data[MAX_IBUS_DATA_SIZE];
    HardwareSerial& serial;

    void scanIbusBuffer() {
      while (buffer.available() >= MIN_IBUS_MSG_SIZE) {
        
        switch (ibusState()) {
          case MESSAGE: {
            IbusMsg ibusMsg = messageFromBuffer(buffer);
            buffer.remove(ibusMsg.fullLength());

            if (!queue.isEmpty() && ibusMsg == queue.peek()) {
              //we recieve our message, we can finally dequeue it
              queue.remove();
              isWriting = false;
            }

            (*onMsg)(ibusMsg);
            break;
          }
          case NOT_ENOUGH:
            return;
          case CKSUM_MISMATCH:
            buffer.remove(1);
            onInvalidByte();
            break;
        }
      }
    }

    IbusState ibusState() {
      byte fullLength = buffer[1] + 2;
      if (fullLength > MAX_IBUS_MSG_SIZE || fullLength < MIN_IBUS_MSG_SIZE) return CKSUM_MISMATCH;
      if (fullLength > buffer.available()) return NOT_ENOUGH;
      byte chk = 0;
      for (byte i = 0; i < fullLength; i++) 
        chk ^= buffer[i];
      if (chk != 0) return CKSUM_MISMATCH;
      return MESSAGE;
    }

    IbusMsg messageFromBuffer(Buffer &buff) {
      IbusMsg msg; 
      msg.tx     = buff[0];
      msg.len    = buff[1];
      msg.rx     = buff[2];
      msg.cmd    = buff[3];
      msg.ck_sum = buff[msg.len + 1];
      
      byte lngth = msg.dataLength();
      for (byte i = 0; i < lngth; i++)
        data[i] = buff[i + 4];
      msg.data = data;
      return msg;
    }

    void write(IbusMsg &msg) {
      serial.write(msg.tx);
      serial.write(msg.len);
      serial.write(msg.rx);
      serial.write(msg.cmd);
      serial.write(msg.data, msg.dataLength());
      serial.write(msg.ck_sum);
    }

    void onBufferOverflowed() {
    }

    void onInvalidByte() {
    }

  public:
    Ibus(HardwareSerial& s, void (*handler)(IbusMsg&), int sp, int sl):
      serial(s),
      statusPin(sp),
      statusLed(sl),
      onMsg(handler)
    {}

    // call it in setup(). 
    // bind led indicator, status pin and waits until bus is free to begin listening
    void init() {
      pinMode(statusPin, INPUT);
      pinMode(statusLed, OUTPUT);
      while (digitalRead(statusPin) == IBUS_BUSY) {}
      serial.begin(9600, SERIAL_8E1);
    }
      
    // do it the main loop. 
    // sets appropiate led indicator, check bus state(FIXME will be nice to attach interrupt)
    // tries to send message from queue
    void routine() {
      int ibusState = digitalRead(statusPin);
      digitalWrite(statusLed, ibusState);
      
      if (ibusState == IBUS_BUSY) {
        isWriting = false;
      } else {
        if (!isWriting && !queue.isEmpty()) {
          IbusMsg& ibusMsg = queue.peek();
          //supposed to be non-blocking since serial buffer is supposed to be empty
          //because ibusState = IBUS_FREE, and isWriting=false
          //but there is no guaranties, since we can't check buffer availability.
          write(ibusMsg); 
          isWriting = true;
        }
      }
    }

    void onSerial() {
      while (serial.available()) {
        if (!buffer.put(serial.read())) onBufferOverflowed();
      }
      scanIbusBuffer();
    }

    boolean send(IbusMsg& msg) {
      boolean res = queue.push(msg);
      if (!res) return false;
      routine();
      return true;
    }
};

