#include <iostream>
#include <ros/ros.h>

#include <fcntl.h>
#include <termio.h>
#include <sys/stat.h>
using namespace std;

#include "sam_hub/sammodule.h"
#define NUM_OF_SAM_ 25
SAMmodule::SAMmodule()
{
    Serial = InitSerial(_SERIAL_PORT_SAM);
    memset(pre_samPos12,'\0',30);
}

void SAMmodule::Recev_Data_hanlder()
{
    int dataSize=read(Serial, Recev_chr, 60);
    if(dataSize > 0)
    {
        for (unsigned char i=0;i<dataSize;i++)
        {
            if(Recev_chr[i]==PC2MCU_HEADER_){
                dataIndex=0;
                flagEnableCapute=1;
                Store_chr[dataIndex++]=Recev_chr[i];
            }else if((Recev_chr[i]==PC2MCU_TERMINATOR_)&&(flagEnableCapute)){
                flagDataReceived=1;
                flagEnableCapute=0;
                Store_chr[dataIndex++]=Recev_chr[i];

            }else if((flagEnableCapute)&&(dataIndex<_SERIAL_BUFF_SIZE))
            {
                Store_chr[dataIndex++]=Recev_chr[i];
            }else if(dataIndex>=_SERIAL_BUFF_SIZE){
                dataIndex=0;
            }
        }

        if(flagDataReceived)
        {
            flagDataReceived=0;
            memset(Recev_chr, '\0', sizeof(Recev_chr));
            /*
             * begin coding
             */

            if(Store_chr[1]==PC_SAM_READ_ALL_POS12_){
                flagDataReceived_readAllPos12=1;
                memset(samPos12Avail, '\0', sizeof(samPos12Avail));
                //===============================
                NumofSam=(dataIndex-3)/4;
                //                cout<<"read lower pos"<<endl;
                for (unsigned char i=0;i<NumofSam;i++)
                {
                    if(Store_chr[i*4+5]==((Store_chr[i*4+2]^Store_chr[i*4+3]^Store_chr[i*4+4])&0x7F))
                    {
                        unsigned int dataPos=(Store_chr[i*4+4]&0x7F)+((Store_chr[i*4+3]&0x1F)<<7);
                        unsigned char index=Store_chr[i*4+2]&0x1F;

                        //                        samPos12[index]=dataPos;
                        //                        samPos12Avail[index]=1;
                        int delta_noise;
                        delta_noise=(int)dataPos-(int)pre_samPos12[index];
                        if(abs(delta_noise)<DELTA_SAM_NOISE){
                            samPos12[index]=dataPos;
                            samPos12Avail[index]=1;
                        }else{
                            ROS_ERROR("error sam feedback: %d | delta noise: %d",index,delta_noise);
                            samPos12Avail[index]=0;
                        }

                        pre_samPos12[index]=dataPos;


                        //                        samPos12[Store_chr[i*4+2]&0x1F]=(Store_chr[i*4+4]&0x7F)+((Store_chr[i*4+3]&0x1F)<<7);
                        //                        samPos12Avail[Store_chr[i*4+2]&0x1F]=1;
                    }
                    else
                        cout<<"error checksum 1"<<endl;
                }

                for (unsigned char i=0; i<30;i++)
                {
                    if(samPos12Avail[i])
                    {
                        cout<<(int)i<<" : "<<samPos12[i]<<endl;

                    }
                }

            }
            else if(Store_chr[1]==PC_SAM_READ_ALL_POS12_FULL_){
                flagDataReceived_readAllPos12=1;
                memset(samPos12Avail, '\0', sizeof(samPos12Avail));
                //===============================
                NumofSam=(dataIndex-3)/4;
                cout<<"read full pos"<<endl;
                for (unsigned char i=0;i<NumofSam;i++)
                {
                    if(Store_chr[i*4+5]==((Store_chr[i*4+2]^Store_chr[i*4+3]^Store_chr[i*4+4])&0x7F))
                    {
                        samPos12[Store_chr[i*4+2]&0x1F]=(Store_chr[i*4+4]&0x7F)+((Store_chr[i*4+3]&0x1F)<<7);
                        samPos12Avail[Store_chr[i*4+2]&0x1F]=1;
                    }
                    else
                        cout<<"error checksum 1"<<endl;
                }
                for (unsigned char i=0; i<30;i++)
                {
                    if(samPos12Avail[i])
                    {
                        cout<<(int)i<<" : "<<samPos12[i]<<endl;

                    }
                }
            }
            else if(dataIndex==6){
            }
        }
    }
}
void SAMmodule::setSamPos12(unsigned char ID, unsigned int Pos)
{
    unsigned char Mode=8;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+(Pos>>7))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::getSamPos12(unsigned char ID)
{
    unsigned char Mode=7;
    unsigned int Pos=0;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+(Pos>>7))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::setSamAverageTorq(unsigned char ID, unsigned int ATorq)
{
    unsigned char Mode=9;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+(ATorq>>7))&0x7F;
    ba[3] = ATorq&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::getSamAverageTorq(unsigned char ID)
{
    unsigned char Mode=10;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = ((Mode&0x03)<<5);
    ba[3] = 0;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::setSamPos8(unsigned char ID, unsigned char Pos,unsigned char Mode)
{
    if (Mode>4)
        Mode=4;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+((Pos>>7)&0x01))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::getSamPos8(unsigned char ID)
{

    unsigned char  Mode=5;
    unsigned char Pos=0;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+((Pos>>7)&0x01))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::setPassive(unsigned char ID)
{
    unsigned char Mode=6;
    unsigned int Pos=0;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+(Pos>>7))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::getPID(unsigned char ID)
{
    unsigned char ba[4];
    ba[0] = 0xff;
    ba[1] = 0x95;
    ba[2] = ID&0x1F;
    ba[3] =0xfe;
    Send_Serial_String(Serial,ba,4);
}

void SAMmodule::setPID(unsigned char ID, unsigned char Pvalue, unsigned char Ivalue, unsigned char Dvalue)
{
    unsigned char ba[9];
    ba[0] = 0xff;
    ba[1]=0xaa;
    ba[2] = ID&0x1F;
    ba[3] = ((Pvalue&0x80)>>5)+((Ivalue&0x80)>>6)+((Dvalue&0x80)>>7);
    ba[4] = Pvalue&0x7F;
    ba[5] = Ivalue&0x7F;
    ba[6] = Dvalue&0x7F;
    ba[7] = (ba[2]^ba[3]^ba[4]^ba[5]^ba[6])&0x7F;
    ba[8] =0xfe;
    Send_Serial_String(Serial,ba,9);
}

void SAMmodule::setPDQuick(unsigned char ID, unsigned char Pvalue, unsigned char Dvalue)
{
    unsigned char ba[7];
    ba[0] = 0xff;
    ba[1]=0xbb;
    ba[2] = (ID&0x1F)+((Pvalue&0x80)>>1)+((Dvalue&0x80)>>2);
    ba[3] = Pvalue&0x7F;
    ba[4] = Dvalue&0x7F;
    ba[5] = (ba[2]^ba[3]^ba[4])&0x7F;
    ba[6] =0xfe;
    Send_Serial_String(Serial,ba,7);
}

void SAMmodule::getAllPos12()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xcc;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::getAllPos12Full()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0x99;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::getAllPos8Torq8()
{

    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xec;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::setAllPassive()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0x88;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::SAM_Power_enable(unsigned char state)
{
    if(state)
        state=1;
    else
        state=0;
    unsigned char ba[4];
    ba[0] = 0xff;
    ba[1] = 0x81;
    ba[2] = state;
    ba[3] =0xfe;
    Send_Serial_String(Serial,ba,4);
}

void SAMmodule::setAllPos12(unsigned int *Pos, unsigned char *mod,unsigned char numOfSam)
{
    unsigned char ba[numOfSam*4+3];
    ba[0] = 0xff;
    ba[1] = 0xf0;

    unsigned char refIndex=2;
    for(unsigned char i=0; i<numOfSam;i++)
    {
        if((*(Pos+i)>400)&&(*(Pos+i)<3701))
        {
            ba[refIndex++]=(i&0x1F)+(((*(mod+i))&0x0C)<<3);//id
            ba[refIndex++]=((*(Pos+i)>>7)&0x5F)+(((*(mod+i))&0x03)<<5);
            ba[refIndex++]=*(Pos+i)&0x7F;
            ba[refIndex]=(ba[refIndex-3]^ba[refIndex-2]^ba[refIndex-1])&0x7F;
            refIndex++;
            //            cout <<(unsigned int)*(Pos+i)<<":";
        }
    }
    ba[refIndex] =0xfe;
    //    cout<<endl;
    Send_Serial_String(Serial,ba,numOfSam*4+3);
}





void SAMmodule::setAllAverageTorque(const unsigned int *Atorq, unsigned char numOfSam)
{
    unsigned char ba[numOfSam*4+3];
    ba[0] = 0xff;
    ba[1] = 0xbd;

    unsigned char refIndex=2;
    for(unsigned char i=0; i<numOfSam;i++)
    {
        if(*(Atorq+i)<4001)
        {
            ba[refIndex++]=i;//id
            ba[refIndex++]=(*(Atorq+i)>>7)&0x7F;
            ba[refIndex++]=*(Atorq+i)&0x7F;
            ba[refIndex]=(ba[refIndex-3]^ba[refIndex-2]^ba[refIndex-1])&0x7F;
            refIndex++;
        }
    }
    ba[refIndex] =0xfe;
    Send_Serial_String(Serial,ba,numOfSam*4+3);
}

void SAMmodule::getAllAverageTorque()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xbf;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::setAllPDQuick(const unsigned char *Pvalue, const unsigned char *Dvalue, unsigned char numOfSam)
{
    unsigned char ba[numOfSam*4+3];
    ba[0] = 0xff;
    ba[1] = 0xc1;

    unsigned char refIndex=2;
    for(unsigned char i=0; i<numOfSam;i++)
    {

        ba[refIndex++]=(i&0x1F)+(((*(Pvalue+i))&0x80)>>1)+(((*(Dvalue+i))&0x80)>>2);//id
        ba[refIndex++]=(*(Pvalue+i))&0x7F;
        ba[refIndex++]=(*(Dvalue+i))&0x7F;
        ba[refIndex]=(ba[refIndex-3]^ba[refIndex-2]^ba[refIndex-1])&0x7F;
        refIndex++;
    }
    ba[refIndex] =0xfe;
    Send_Serial_String(Serial,ba,numOfSam*4+3);
}

void SAMmodule::setAllPIDQuick(const unsigned char *Pvalue, const unsigned char *Dvalue, const unsigned char *Ivalue, unsigned char numOfSam)
{
    unsigned char ba[numOfSam*5+3];
    ba[0] = 0xff;
    ba[1] = 0xc2;

    unsigned char refIndex=2;
    for(unsigned char i=0; i<numOfSam;i++)
    {

        ba[refIndex++]=(i&0x1F)+(((*(Pvalue+i))&0x80)>>1)+(((*(Dvalue+i))&0x80)>>2);//id
        ba[refIndex++]=(*(Pvalue+i))&0x7F;
        ba[refIndex++]=(*(Dvalue+i))&0x7F;
        ba[refIndex++]=(*(Ivalue+i))&0x7F;
        ba[refIndex]=(ba[refIndex-4]^ba[refIndex-3]^ba[refIndex-2]^ba[refIndex-1])&0x7F;
        refIndex++;
    }
    ba[refIndex] =0xfe;
    Send_Serial_String(Serial,ba,numOfSam*5+3);
}

void SAMmodule::getAllPDQuick()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xc3;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::Send_Serial_String(int Serial, unsigned char *Trans_chr, int Size)
{
    write(Serial, Trans_chr, Size);
}

void My_Send_Serial_String(int Serial, unsigned char *Trans_chr, int Size)
{
    write(Serial, Trans_chr, Size);
}

int SAMmodule::InitSerial(const char *Serial_Port)
{
    termios Serial_Setting;
    if((Serial = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY)) == -1)
    {
        cout << "SERIAL : " << Serial_Port << " Device open error" << endl;
        cout << "SERIAL : " << Serial_Port << " Device permission change progress...." << endl;
        for(int temp = 0; temp < 5; temp++)
        {
            if(chmod(Serial_Port, __S_IREAD | __S_IWRITE) == 0){

                cout << "SERIAL : " << Serial_Port << " Device permission change complete" << endl;
                Serial = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY);

                if(Serial == -1)
                {
                    cout << "SERIAL : " << Serial_Port << " Device Not Found" << endl;
                    return -1;
                }
                else
                    cout << "SERIAL : " << Serial_Port <<" Device open" << endl;
            }
            else
            {
                cout << "SERIAL : " << Serial_Port << " Device permission change error" << endl;
                //return -1;
            }
        }
    }
    else
        cout << "SERIAL : " << Serial_Port << " Device open" << endl;


    memset(&Serial_Setting, 0, sizeof(Serial_Setting));
    Serial_Setting.c_iflag = 0;
    Serial_Setting.c_oflag = 0;
    Serial_Setting.c_cflag = _BAUDRATE | CS8 | CREAD | CLOCAL;
    Serial_Setting.c_lflag = 0;
    Serial_Setting.c_cc[VMIN] = 1;
    Serial_Setting.c_cc[VTIME] = 0;

    cfsetispeed(&Serial_Setting, _BAUDRATE);
    cfsetospeed(&Serial_Setting, _BAUDRATE);

    tcflush(Serial, TCIFLUSH);
    tcsetattr(Serial, TCSANOW, &Serial_Setting);
    return Serial;

}
