#ifndef SAMMODULE_H
#define SAMMODULE_H


#define _SERIAL_PORT_SAM "/dev/ttyUSB0"
#define _SERIAL_BUFF_SIZE    100
//#define _BAUDRATE   B1200
//#define _BAUDRATE   B2400
//#define _BAUDRATE    B4800
//#define _BAUDRATE    B9600
//#define _BAUDRATE    B57600
//#define _BAUDRATE    B115200
//#define _BAUDRATE    B230400
//#define _BAUDRATE    B500000
//#define _BAUDRATE    B921600
#define _BAUDRATE    B1500000
#define PC2MCU_TERMINATOR_  0xFE
#define PC2MCU_HEADER_      0xFF

#define PC_SAM_READ_ALL_POS12_ 0xCC
#define PC_SAM_READ_ALL_POS12_FULL_ 0x99



class SAMmodule
{
private:

    int InitSerial(const char *Serial_Port);

    unsigned char Trans_chr[_SERIAL_BUFF_SIZE];
    unsigned char Recev_chr[_SERIAL_BUFF_SIZE];
    unsigned char Store_chr[_SERIAL_BUFF_SIZE];

    unsigned char flagDataReceived:1;
    unsigned char flagEnableCapute:1;


    unsigned char dataIndex;
    unsigned char NumofSam;
public:
    unsigned char flagDataReceived_readAllPos12:1;
    unsigned char flagDataAvailable_readAllPos12:1;

    unsigned char flagDataReceived_readAllPos8:1;

#define DELTA_SAM_NOISE 100
    unsigned int samPos12[30];
    int pre_samPos12[30];
    unsigned char samPos12Avail[30];

    void Send_Serial_String(int Serial, unsigned char *Trans_chr, int Size);
    int Serial;

    SAMmodule();

    void Recev_Data_hanlder();

    void setSamPos12(unsigned char ID,unsigned int Pos);
    void getSamPos12(unsigned char ID);

    void setSamAverageTorq(unsigned char ID,unsigned int ATorq);
    void getSamAverageTorq(unsigned char ID);

    void setSamPos8(unsigned char ID,unsigned char Pos,unsigned char Mode);
    void getSamPos8(unsigned char ID);
    void setPassive(unsigned char ID);

    void getPID(unsigned char ID);
    void setPID(unsigned char ID,unsigned char Pvalue,unsigned char Ivalue,unsigned char Dvalue);

    void setPDQuick(unsigned char ID,unsigned char Pvalue,unsigned char Dvalue);

    void getAllPos12();
    void getAllPos12Full();
    void getAllPos8Torq8();
    void setAllPassive();
    void SAM_Power_enable(unsigned char state);
    void setAllPos12(unsigned int *Pos,unsigned char numOfSam);
    void setAllAverageTorque(const unsigned int *Atorq,unsigned char numOfSam);
    void getAllAverageTorque();

    void setAllPDQuick(const unsigned char *Pvalue,const unsigned char *Dvalue,unsigned char numOfSam);
     void setAllPIDQuick(const unsigned char *Pvalue,const unsigned char *Dvalue,const unsigned char *Ivalue,unsigned char numOfSam);
    void getAllPDQuick();
};

#endif // SAMMODULE_H
