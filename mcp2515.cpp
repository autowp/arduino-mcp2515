#include "Arduino.h"
#include "mcp2515.h"

const struct MCP2515::TXBn_REGS MCP2515::TXB[MCP2515::N_TXBUFFERS] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};

const struct MCP2515::RXBn_REGS MCP2515::RXB[N_RXBUFFERS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};

MCP2515::MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK, SPIClass * _SPI)
{
    if (_SPI != nullptr) {
        SPIn = _SPI;
    }
    else {
        SPIn = &SPI;
        SPIn->begin();
    }

    SPICS = _CS;
    SPI_CLOCK = _SPI_CLOCK;
    pinMode(SPICS, OUTPUT);
    digitalWrite(SPICS, HIGH);
}

void MCP2515::startSPI() {
    SPIn->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(SPICS, LOW);
}

void MCP2515::endSPI() {
    digitalWrite(SPICS, HIGH);
    SPIn->endTransaction();
}

MCP2515::ERROR MCP2515::reset(void)
{
    startSPI();
    SPIn->transfer(INSTRUCTION_RESET);
    endSPI();

    delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    setRegisters(MCP_TXB0CTRL, zeros, 14);
    setRegisters(MCP_TXB1CTRL, zeros, 14);
    setRegisters(MCP_TXB2CTRL, zeros, 14);

    setRegister(MCP_RXB0CTRL, 0);
    setRegister(MCP_RXB1CTRL, 0);

    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++) {
        bool ext = (i == 1);
        ERROR result = setFilter(filters[i], ext, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++) {
        ERROR result = setFilterMask(masks[i], true, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    return ERROR_OK;
}

uint8_t MCP2515::readRegister(const REGISTER reg)
{
    startSPI();
    SPIn->transfer(INSTRUCTION_READ);
    SPIn->transfer(reg);
    uint8_t ret = SPIn->transfer(0x00);
    endSPI();

    return ret;
}

void MCP2515::readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n)
{
    startSPI();
    SPIn->transfer(INSTRUCTION_READ);
    SPIn->transfer(reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i=0; i<n; i++) {
        values[i] = SPIn->transfer(0x00);
    }
    endSPI();
}

void MCP2515::setRegister(const REGISTER reg, const uint8_t value)
{
    startSPI();
    SPIn->transfer(INSTRUCTION_WRITE);
    SPIn->transfer(reg);
    SPIn->transfer(value);
    endSPI();
}

void MCP2515::setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n)
{
    startSPI();
    SPIn->transfer(INSTRUCTION_WRITE);
    SPIn->transfer(reg);
    for (uint8_t i=0; i<n; i++) {
        SPIn->transfer(values[i]);
    }
    endSPI();
}

void MCP2515::modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data)
{
    startSPI();
    SPIn->transfer(INSTRUCTION_BITMOD);
    SPIn->transfer(reg);
    SPIn->transfer(mask);
    SPIn->transfer(data);
    endSPI();
}

uint8_t MCP2515::getStatus(void)
{
    startSPI();
    SPIn->transfer(INSTRUCTION_READ_STATUS);
    uint8_t i = SPIn->transfer(0x00);
    endSPI();

    return i;
}

MCP2515::ERROR MCP2515::setConfigMode()
{
    return setMode(CANCTRL_REQOP_CONFIG);
}

MCP2515::ERROR MCP2515::setListenOnlyMode()
{
    return setMode(CANCTRL_REQOP_LISTENONLY);
}

MCP2515::ERROR MCP2515::setSleepMode()
{
    return setMode(CANCTRL_REQOP_SLEEP);
}

MCP2515::ERROR MCP2515::setLoopbackMode()
{
    return setMode(CANCTRL_REQOP_LOOPBACK);
}

MCP2515::ERROR MCP2515::setNormalMode()
{
    return setMode(CANCTRL_REQOP_NORMAL);
}

MCP2515::ERROR MCP2515::setMode(const CANCTRL_REQOP_MODE mode)
{
    modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

    unsigned long endTime = millis() + 10;
    bool modeMatch = false;
    while (millis() < endTime) {
        uint8_t newmode = readRegister(MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == mode;

        if (modeMatch) {
            break;
        }
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;
}

MCP2515::ERROR MCP2515::setBitrate(const CAN_SPEED canSpeed)
{
    return setBitrate(canSpeed, MCP_16MHZ);
}

MCP2515::ERROR MCP2515::setBitrate(const CAN_SPEED canSpeed, const CAN_CLOCK canClock)
{
    /* For CNF1, see Register 5-1 on Page 44 of the MCP2515 datasheet */
    /* For CNF2, see Register 5-2 on Page 44 of the MCP2515 datasheet */
    /* For CNF3, see Register 5-3 on Page 45 of the MCP2515 datasheet */

    struct {
        uint8_t cnf1, cnf2, cnf3;
    } setting[][3] = {       /*      20MHz     */  /*      16MHz     */  /*      8MHz      */
        [ CAN_5KBPS    ] = { { 0x00, 0x00, 0x00 }, { 0x3F, 0xFF, 0x87 }, { 0x1F, 0xBF, 0x87 }, },
        [ CAN_10KBPS   ] = { { 0x00, 0x00, 0x00 }, { 0x1F, 0xFF, 0x87 }, { 0x0F, 0xBF, 0x87 }, },
        [ CAN_20KBPS   ] = { { 0x00, 0x00, 0x00 }, { 0x0F, 0xFF, 0x87 }, { 0x07, 0xBF, 0x87 }, },
        [ CAN_31K25BPS ] = { { 0x00, 0x00, 0x00 }, { 0x00, 0x00, 0x00 }, { 0x07, 0xA4, 0x84 }, },
        [ CAN_33KBPS   ] = { { 0x0B, 0xFF, 0x87 }, { 0x4E, 0xF1, 0x85 }, { 0x47, 0xE2, 0x85 }, },
        [ CAN_40KBPS   ] = { { 0x09, 0xFF, 0x87 }, { 0x07, 0xFF, 0x87 }, { 0x03, 0xBF, 0x87 }, },
        [ CAN_50KBPS   ] = { { 0x09, 0xFA, 0x87 }, { 0x07, 0xFA, 0x87 }, { 0x03, 0xB4, 0x86 }, },
        [ CAN_80KBPS   ] = { { 0x04, 0xFF, 0x87 }, { 0x03, 0xFF, 0x87 }, { 0x01, 0xBF, 0x87 }, },
        [ CAN_83K3BPS  ] = { { 0x04, 0xFE, 0x87 }, { 0x03, 0xBE, 0x07 }, { 0x00, 0x00, 0x00 }, },
        [ CAN_95KBPS   ] = { { 0x00, 0x00, 0x00 }, { 0x03, 0xAD, 0x07 }, { 0x00, 0x00, 0x00 }, },
        [ CAN_100KBPS  ] = { { 0x04, 0xFA, 0x87 }, { 0x03, 0xFA, 0x87 }, { 0x01, 0xB4, 0x86 }, },
        [ CAN_125KBPS  ] = { { 0x03, 0xFA, 0x87 }, { 0x03, 0xF0, 0x86 }, { 0x01, 0xB1, 0x85 }, },
        [ CAN_200KBPS  ] = { { 0x01, 0xFF, 0x87 }, { 0x01, 0xFA, 0x87 }, { 0x00, 0xB4, 0x86 }, },
        [ CAN_250KBPS  ] = { { 0x41, 0xFB, 0x86 }, { 0x41, 0xF1, 0x85 }, { 0x00, 0xB1, 0x85 }, },
        [ CAN_500KBPS  ] = { { 0x00, 0xFA, 0x87 }, { 0x00, 0xF0, 0x86 }, { 0x00, 0x90, 0x82 }, },
        [ CAN_1000KBPS ] = { { 0x00, 0xD9, 0x82 }, { 0x00, 0xD0, 0x82 }, { 0x00, 0x80, 0x80 }, },
    };

    ERROR error = setConfigMode();
    if (error != ERROR_OK) {
        return error;
    }

    // If cnf2 is 0, that indicates an invalid setting, and will return ERROR_FAIL
    // Note that cnf1 can properly be 0.
    bool set = !!setting[canSpeed][canClock].cnf2;
    if ( set  ) {
        setRegister(MCP_CNF1, setting[canSpeed][canClock].cnf1);
        setRegister(MCP_CNF2, setting[canSpeed][canClock].cnf2);
        setRegister(MCP_CNF3, setting[canSpeed][canClock].cnf3);
    }

    return set? ERROR_OK : ERROR_FAIL;
}

MCP2515::ERROR MCP2515::setClkOut(const CAN_CLKOUT divisor)
{
    if (divisor == CLKOUT_DISABLE) {
        /* Turn off CLKEN */
        modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

        /* Turn on CLKOUT for SOF */
        modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);
        return ERROR_OK;
    }

    /* Set the prescaler (CLKPRE) */
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

    /* Turn on CLKEN */
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

    /* Turn off CLKOUT for SOF */
    modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
    return ERROR_OK;
}

void MCP2515::prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
    } else {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

MCP2515::ERROR MCP2515::setFilterMask(const MASK mask, const bool ext, const uint32_t ulData)
{
    ERROR res = setConfigMode();
    if (res != ERROR_OK) {
        return res;
    }
    
    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    setRegisters(reg, tbufdata, 4);
    
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::setFilter(const RXF num, const bool ext, const uint32_t ulData)
{
    ERROR res = setConfigMode();
    if (res != ERROR_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);
    setRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

MCP2515::ERROR MCP2515::sendMessage(const TXBn txbn, const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    const struct TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    prepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    setRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);

    modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    uint8_t ctrl = readRegister(txbuf->CTRL);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
        return ERROR_FAILTX;
    }
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::sendMessage(const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};

    for (int i=0; i<N_TXBUFFERS; i++) {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = readRegister(txbuf->CTRL);
        if ( (ctrlval & TXB_TXREQ) == 0 ) {
            return sendMessage(txBuffers[i], frame);
        }
    }

    return ERROR_ALLTXBUSY;
}

MCP2515::ERROR MCP2515::readMessage(const RXBn rxbn, struct can_frame *frame)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5];

    readRegisters(rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id<<8) + tbufdata[MCP_EID8];
        id = (id<<8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return ERROR_FAIL;
    }

    uint8_t ctrl = readRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    readRegisters(rxb->DATA, frame->data, dlc);

    modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return ERROR_OK;
}

MCP2515::ERROR MCP2515::readMessage(struct can_frame *frame)
{
    ERROR rc;
    uint8_t stat = getStatus();

    if ( stat & STAT_RX0IF ) {
        rc = readMessage(RXB0, frame);
    } else if ( stat & STAT_RX1IF ) {
        rc = readMessage(RXB1, frame);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;
}

bool MCP2515::checkReceive(void)
{
    uint8_t res = getStatus();
    return !!( res & STAT_RXIF_MASK );
}

bool MCP2515::checkError(void)
{
    uint8_t eflg = getErrorFlags();
    return !!( eflg & EFLG_ERRORMASK );
}

uint8_t MCP2515::getErrorFlags(void)
{
    return readRegister(MCP_EFLG);
}

void MCP2515::clearRXnOVRFlags(void)
{
	modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t MCP2515::getInterrupts(void)
{
    return readRegister(MCP_CANINTF);
}

void MCP2515::clearInterrupts(void)
{
    setRegister(MCP_CANINTF, 0);
}

uint8_t MCP2515::getInterruptMask(void)
{
    return readRegister(MCP_CANINTE);
}

void MCP2515::clearTXInterrupts(void)
{
    modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP2515::clearRXnOVR(void)
{
    uint8_t eflg = getErrorFlags();
    if (eflg != 0) {
        clearRXnOVRFlags();
        clearInterrupts();
    }
}

void MCP2515::clearMERR()
{
    modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}

void MCP2515::clearERRIF()
{
    modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}

uint8_t MCP2515::errorCountRX(void)                             
{
    return readRegister(MCP_REC);
}

uint8_t MCP2515::errorCountTX(void)                             
{
    return readRegister(MCP_TEC);
}
