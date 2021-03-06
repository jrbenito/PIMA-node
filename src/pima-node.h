void txRadio(Message * mess);
void readUptime(Message *mess);
void readTXInt(Message *mess);
void writeTXInt(const Message *mess);
void readRSSI(Message *mess);
void readVoltage(Message *mess);
void readACK(Message *mess);
void writeACK(const Message *mess);
void readToggle(Message *mess);
void writeToggle(const Message *mess);
void sleep();
void watchdogSetup(void);
uint8_t bcd2dec(uint8_t n);

typedef struct {
    int32_t pwrActivDir;
    int32_t pwrReactInd;
    int32_t pwrReactCap;
    int32_t pwrActivRev;
} Power;

// devices in the node
void readLED(Message *mess);
void writeLED(const Message *mess);
void readPwrADir(Message *mess);
void readPwrRInd(Message *mess);
void readPwrRCap(Message *mess);
void readPwrARev(Message *mess);
void receivePIMAPacket();
void processPIMAPacket();
