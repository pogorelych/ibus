const int  IBUS_MONITOR_PIN = 3;

//===== INITIALIZATION =====
void onIbusMsg(IbusMsg &ibusMsg);

Ibus ibus(Serial1, onIbusMsg, IBUS_MONITOR_PIN, LED_BUILTIN);

byte vlDnData[1] = {0x01};
byte *blabla = "A0HELLO                        ";

IbusMsg vlDn(0x50, 4, 0x68, 0x3b, vlDnData);
byte vlUpData[1] = {0x21};
IbusMsg vlUp(0x50, 4, 0x68, 0x3b, vlUpData);
IbusMsg ike(0x68, 23, 0x80, 0x23, blabla);

void serialEvent1() {
  ibus.onSerial();
}

void setup() {
  ibus.init();
  Serial.begin(9600);
//  blabla[0] = 0xA0;
}

void loop() {
  ibus.routine();
}

//===== APPLICATION =====
void printlnMsg(HardwareSerial&, IbusMsg&);

void onIbusMsg(IbusMsg &msg) {
//  if ((msg.rx == IBUS_IKE || msg.tx == IBUS_IKE)
   if (!(msg.cmd == 1 || msg.cmd == 2))
//   && (msg.tx == 0xD0))
      printlnMsg(Serial, msg);

}

void serialEvent() {
  if (Serial.available()) {
    char r = Serial.read();
    if (r == 'a') {
      ibus.send(vlDn);
    }
    if (r == 's') {
      ibus.send(vlUp);
    }
    if (r == 'd') {
      ibus.send(ike);
    }
  }
}

