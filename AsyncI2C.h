#ifndef ASYNC_I2C_h
#define ASYNC_I2C_h

#include <Wire.h>

#define ASYNC_ANTRY 5

class AsyncI2C
{
public:
  volatile bool done = true;
  volatile uint32_t async_tries;
  //-----------------------------------------------------------------------------------------------------------------------
  AsyncI2C( const uintptr_t _portAddr, const TwoWire::I2C_Hardware_t& myhardware,
    TwoWire* _w, IntervalTimer* _timer, void (* _isr)(), void (* _tsr)() ):
    portAddr(_portAddr), hardware(myhardware), twowire(_w), isr(_isr), usr(yield), tsr(_tsr)
  {
    port = (IMXRT_LPI2C_t*)portAddr;
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void begin()
  {
    // end
    end();
    // begin
    twowire->begin();
    twowire->setClock(400000);
    // setup
    attachInterruptVector(hardware.irq_number, isr);
    NVIC_SET_PRIORITY(hardware.irq_number, 150); // Wire isr uses 144
    NVIC_ENABLE_IRQ(hardware.irq_number);
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void end()
  {
    port->MCR = (LPI2C_MCR_RST | LPI2C_MCR_RRF | LPI2C_MCR_RTF);
    port->MCR = 0;
    NVIC_DISABLE_IRQ(hardware.irq_number);
    attachInterruptVector(hardware.irq_number, nullptr);
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void bid(bool send_stop = 1)
  {
    bound = 0;
    _send_stop = send_stop;
    async_tries = 0;
    tsr();
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void bidask(uint8_t n, uint8_t business, size_t length, bool send_stop = 1)
  {
    bound = 1;
    SO[0] = n;
    SO[1] = business;
    SO[2] = length;
    _send_stop = send_stop;
    async_tries = 0;
    tsr();
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void ask(uint8_t n, size_t length, bool send_stop = 1)
  {
    bound = 2;
    SO[0] = n;
    SO[2] = length;
    _send_stop = send_stop;
    async_tries = 0;
    tsr();
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void start()
  {
    if     (bound == 0)
    {
      // send stop?
      if(_send_stop){ port->MCFGR1 |= LPI2C_MCFGR1_AUTOSTOP;  }
      else          { port->MCFGR1 &= ~LPI2C_MCFGR1_AUTOSTOP; }
      // anset leaf
      SO_m = 0;
      // first words
      port->MTDR = LPI2C_MTDR_CMD_START | ((SO[SO_m++] & 0x7F) << 1);
      while(SO_m < 4 && SO_m < SO_l){ port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | SO[SO_m++]; }
      // next words
      port->MFCR = LPI2C_MFCR_TXWATER(SO_m < SO_l ? 1 : 0); // TD flag: fifo <= water
      // wait
      port->MIER = (LPI2C_MIER_PLTIE | LPI2C_MIER_FEIE | LPI2C_MIER_ALIE | LPI2C_MIER_NDIE) | LPI2C_MIER_TDIE;
    }
    else if(bound == 1)
    {
      // send stop?
      if(_send_stop){ port->MCFGR1 |= LPI2C_MCFGR1_AUTOSTOP; }
      else          { port->MCFGR1 &= ~LPI2C_MCFGR1_AUTOSTOP; }
      // anset leaf
      SI_m = 0;
      SI_l = SO[2];
      for(size_t i=0; i<SI_l; ++i){ SI[i] = 0; }
      // first words
      port->MTDR = LPI2C_MTDR_CMD_START | ((SO[0] & 0x7F) << 1);
      port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | SO[1];
      // next words
      port->MFCR = LPI2C_MFCR_TXWATER(0); // TD flag: fifo <= water
      port->MFCR = LPI2C_MFCR_RXWATER((SI_l - 1) > 3 ? 3 : (SI_l - 1)); // RD flag: fifo > water
      // wait
      port->MIER = (LPI2C_MIER_PLTIE | LPI2C_MIER_FEIE | LPI2C_MIER_ALIE | LPI2C_MIER_NDIE) | LPI2C_MIER_TDIE;
    }
    else if(bound == 2)
    {
      // send stop?
      if(_send_stop){ port->MCFGR1 |= LPI2C_MCFGR1_AUTOSTOP;  }
      else          { port->MCFGR1 &= ~LPI2C_MCFGR1_AUTOSTOP; }
      // anset leaf
      SI_m = 0;
      SI_l = SO[2];
      for(size_t i=0; i<SI_l; ++i){ SI[i] = 0; }
      // first words
      port->MTDR = LPI2C_MTDR_CMD_START | 1 | ((SO[0] & 0x7F) << 1);
      port->MTDR = LPI2C_MTDR_CMD_RECEIVE | (SO[2] - 1);
      // next words
      port->MFCR = LPI2C_MFCR_RXWATER((SI_l - 1) > 3 ? 3 : (SI_l - 1)); // RD flag: fifo > water
      // wait
      port->MIER = (LPI2C_MIER_PLTIE | LPI2C_MIER_FEIE | LPI2C_MIER_ALIE | LPI2C_MIER_NDIE) | LPI2C_MIER_RDIE;
    }
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void ISR()
  {
    uint32_t status = port->MSR; // MCR: pg 2813, MSR: pg 2814
    // mishaps
    if(status & (LPI2C_MSR_ALF | LPI2C_MSR_FEF | LPI2C_MSR_NDF | LPI2C_MSR_PLTF))
    {
      giveup();
		}
    // bid
    if     (bound == 0)
    {
      // how many words to say
      size_t fifo_m = SO_m + (port->MFSR & 0x07);
      // speak
      while(SO_m < fifo_m && SO_m < SO_l){ port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | SO[SO_m++]; }
      if(SO_m == SO_l) // bid done?
      {
        if((port->MFSR & 0x7) == 0) // all words reached?
        {
          anset();
          usr();
        }
        else // wait till all words reached
        {
          port->MFCR = LPI2C_MFCR_TXWATER(0);
        }
      }
    }
    // bid & ask
    else if((bound == 1) && ((port->MFSR & 0x7) == 0))
    {
      port->MCR |= LPI2C_MCR_RTF | LPI2C_MCR_RRF; // clear FIFOs
      // ask
      bound = 2;
      // auto stop
      port->MCFGR1 |= LPI2C_MCFGR1_AUTOSTOP;
      // first words
      port->MTDR = LPI2C_MTDR_CMD_START | 1 | ((SO[0] & 0x7F) << 1);
      port->MTDR = LPI2C_MTDR_CMD_RECEIVE | (SO[2] - 1);
      // wait
      port->MIER = (LPI2C_MIER_PLTIE | LPI2C_MIER_FEIE | LPI2C_MIER_ALIE | LPI2C_MIER_NDIE) | LPI2C_MIER_RDIE;
    }
    // ask
    else if(bound == 2)
    {
      // how many words to hear
      size_t fifo_m = SI_m + ((port->MFSR >> 16) & 0x07);
      // listen
      while(SI_m < fifo_m && SI_m < SI_l ){ SI[SI_m++] = port->MRDR; }
      // next words
      fifo_m = SI_l - SI_m - 1;
      port->MFCR = LPI2C_MFCR_RXWATER(fifo_m > 3 ? 3 : fifo_m); // RD flag: fifo > water
      if(SI_m == SI_l) // all words heard?
      {
        anset();
        SI_m = 0;
        usr();
      }
    }
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void USR(void (* _usr)())
  {
    usr = _usr;
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void name(const uint8_t& n)
  {
    SO[0] = n;
    SO_l = 1;
  }
  //-----------------------------------------------------------------------------------------------------------------------
  template<typename T> void write(const T& w)
  {
    size_t l = sizeof(T);
    if(SO_l + l <= S_LENGTH)
    {
      const uint8_t* p = reinterpret_cast<const uint8_t*>(&w);
      for(size_t i=0; i<l; ++i){ SO[SO_l++] = p[i]; } // p[l-1-i]
    }
  }
  //-----------------------------------------------------------------------------------------------------------------------
  template<typename T> void read(T& w)
  {
    size_t l = sizeof(T);
    if(SI_m + l <= S_LENGTH)
    {
      uint8_t* p = reinterpret_cast<uint8_t*>(&w);
      for(size_t i=0; i<l; ++i){ p[i] = SI[SI_m++]; } // p[l-1-i]
    }
  }
  //-----------------------------------------------------------------------------------------------------------------------
  template<typename T> inline T read()
  {
    T w{};
    read(w);
    return w;
  }
  //-----------------------------------------------------------------------------------------------------------------------
private:
	const uintptr_t portAddr;
  IMXRT_LPI2C_t* port;
	const TwoWire::I2C_Hardware_t &hardware;
  TwoWire* twowire;
  IntervalTimer* timer;

  volatile bool _send_stop;

  static constexpr size_t S_LENGTH = 64;
  volatile uint8_t SI[S_LENGTH];
  volatile uint8_t SO[S_LENGTH];
  volatile size_t SI_l;
  volatile size_t SI_m;
  volatile size_t SO_l;
  volatile size_t SO_m;
  
  void (* isr)(); // interrupt
  void (* usr)(); // user
  void (* tsr)(); // try
  volatile uint8_t bound = 0; // 0 = bid, 1 = bid and ask, 2 = ask
  //-----------------------------------------------------------------------------------------------------------------------
  bool idle()
  {
    if(!done) return false;
    if((port->MSR & LPI2C_MSR_BBF) && !(port->MSR & LPI2C_MSR_MBF)) return false; // bus busy && not ours
    port->MSR = 0x00007F00; // clear all prior flags
    return true;
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void anset()
  {
    port->MIER = 0;
    port->MCR |= LPI2C_MCR_RTF | LPI2C_MCR_RRF; // clear FIFOs
    done = 1;
  }
  //-----------------------------------------------------------------------------------------------------------------------
  void giveup()
  {
    port->MIER = 0;
    port->MCR |= LPI2C_MCR_RTF | LPI2C_MCR_RRF; // clear FIFOs
		uint32_t status = port->MSR; // MCR: pg 2813, MSR: pg 2814
    //if(((status & LPI2C_MSR_MBF) && !(status & LPI2C_MSR_SDF)) || (status & LPI2C_MSR_NDF) || (status & LPI2C_MSR_PLTF))
    if((status & LPI2C_MSR_MBF) && !(status & LPI2C_MSR_SDF))
    {
      port->MTDR = LPI2C_MTDR_CMD_STOP;
    }
    done = 1;
  }
  friend void AsyncWireTry(); 
};

IntervalTimer AsyncWireAntryTimer;
void AsyncWireISR();
void AsyncWireTry();

AsyncI2C AsyncWire(IMXRT_LPI2C1_ADDRESS, TwoWire::i2c1_hardware, &Wire, &AsyncWireAntryTimer, AsyncWireISR, AsyncWireTry);
void AsyncWireISR()
{
  AsyncWire.ISR();
}
void AsyncWireTry()
{
  if(AsyncWire.idle())
  {
    AsyncWireAntryTimer.end();
    AsyncWire.done = 0;
    AsyncWire.start();
    return;
  }
  if     (AsyncWire.async_tries == 0)
  {
    AsyncWireAntryTimer.begin(AsyncWireTry, 2);
  }
  else if(AsyncWire.async_tries > ASYNC_ANTRY)
  {
    AsyncWireAntryTimer.end();
    AsyncWire.done = 1;
    return;
  }
  ++AsyncWire.async_tries;
}

#endif