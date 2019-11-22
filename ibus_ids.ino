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

String hex(int b) {
  String code = String(b, HEX);
  return  (b < 16) ? "0" +  code : code;
}
#define XX(code, name) case code: return #name;
String getEcuName(IbusId code) {
  switch (code) {
  IBUS_IDS(XX)
  default: return "*" + hex(code);
  }
};
#undef XX

String getCommandName(IbusId rx, byte code) {
  switch (code) {
    case 0x01: return "'Status?'";
    case 0x02: return "'Ready'";
    default: return hex(code);
  }
}

//prints ibus message in human-readable format
//examples:
//IKE->MON:Status?:0
//*F0->*D0:AB:13:'  Text format '
//*F0->*D0:AB:4:AA B0 CD 10
void printlnMsg(HardwareSerial& serial, IbusMsg &msg) {
  serial.print(getEcuName(msg.tx));
  serial.print("->");
  serial.print(getEcuName(msg.rx));
  serial.write(':');
  serial.print(getCommandName(msg.rx, msg.cmd));
  int lngth = msg.dataLength();
  serial.write(':');
  serial.print(msg.len); 
  if (lngth > 0) {
    serial.write(':');
    if (lngth < 8)
      for (int i =0; i < lngth; i ++) {
        serial.print(hex(msg.data[i]));
        serial.write(' ');
     }
    else {
      serial.write('\'');
      serial.write(msg.data, lngth);
      serial.write('\'');
    }
  }
  serial.println();
}


void printRawMsg(HardwareSerial& serial, IbusMsg &msg) {
  serial.print(hex(msg.tx));
  serial.print(hex(msg.len));
  serial.print(hex(msg.rx));
  serial.print(hex(msg.cmd));
  for (int i = 0; i < msg.dataLength(); i++) {
    serial.print(hex(msg.data[i]));
  }
  serial.print(hex(msg.ck_sum));
}


