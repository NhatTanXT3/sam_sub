#include <iostream>
#include <ros/ros.h>
using namespace std;
#include "sam_hub/sammodule.h"
#include "sam_hub/mainsamhub.h"
#include "sam_hub/SAMJointStateMsg.h"
#include "sam_hub/SAMcmdMsg.h"
#include "sam_hub/SAMJointPos12Msg.h"

SAMmodule  *mySAM;

void sub_function_cmd(const sam_hub::SAMcmdMsg::ConstPtr& msg){
    switch (msg->command){
    case SAM_FB_LOWERLINK_POS12_50HZ:
        ROS_INFO("SAM_FB_LOWERLINK_POS12_50HZ : %d", msg->command);
        sys_flag.getPos12LowerLink_50Hz=1;
        sys_flag.getPos12FullBody_50Hz=0;
        sys_flag.getPos12LowerLink_125Hz=0;
        sys_flag.getPos12FullBody_125Hz=0;
        break;
    case SAM_FB_LOWERLINK_POS12_125HZ:
        ROS_INFO("SAM_FB_LOWERLINK_POS12_125HZ: %d", msg->command);
        sys_flag.getPos12LowerLink_50Hz=0;
        sys_flag.getPos12FullBody_50Hz=0;
        sys_flag.getPos12LowerLink_125Hz=1;
        sys_flag.getPos12FullBody_125Hz=0;
        break;

    case SAM_FB_FULLBODY_POS12_50HZ:
        ROS_INFO("SAM_FB_FULLBODY_POS12_50HZ : %d", msg->command);
        sys_flag.getPos12LowerLink_50Hz=0;
        sys_flag.getPos12FullBody_50Hz=1;
        sys_flag.getPos12LowerLink_125Hz=0;
        sys_flag.getPos12FullBody_125Hz=0;
        break;
    case SAM_FB_FULLBODY_POS12_125HZ:
        ROS_INFO("SAM_FB_FULLBODY_POS12_125HZ: %d", msg->command);
        sys_flag.getPos12LowerLink_50Hz=0;
        sys_flag.getPos12FullBody_50Hz=0;
        sys_flag.getPos12LowerLink_125Hz=0;
        sys_flag.getPos12FullBody_125Hz=1;
        break;
    default:
        ROS_INFO("receive msg, turn off sensor : %d", msg->command);
        sys_flag.getPos12LowerLink_50Hz=0;
        sys_flag.getPos12FullBody_50Hz=0;
        sys_flag.getPos12LowerLink_125Hz=0;
        sys_flag.getPos12FullBody_125Hz=0;
        break;
    }
}

unsigned int pos12[25];
void sub_function_pos12(const sam_hub::SAMJointPos12Msg::ConstPtr& msg){
    for (unsigned char i=0; i<12;i++)
    {
        cout<<(int)msg->SAMMode[i]<<" : "<<msg->SAMPos12[i]<<endl;
        pos12[i]=msg->SAMPos12[i];
    }
   mySAM->setAllPos12(pos12,12);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sam_hub");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE_1000Hz_);
    mySAM =new SAMmodule;

    ros::Publisher sam_pub =  n.advertise<sam_hub::SAMJointStateMsg>("sam_pub",1000);
    ros::Subscriber sam_sub=n.subscribe<sam_hub::SAMcmdMsg>("sam_cmd_sub",200,sub_function_cmd);
    ros::Subscriber samPos12_sub=n.subscribe<sam_hub::SAMJointPos12Msg>("sam_pos12_sub",1000,sub_function_pos12);
    sam_hub::SAMJointStateMsg samMsg;

    sys_flag.getPos12LowerLink_50Hz=0;
    sys_flag.getPos12FullBody_50Hz=0;
    sys_flag.getPos12LowerLink_125Hz=0;
    sys_flag.getPos12FullBody_125Hz=0;

    if(mySAM->Serial!= -1)
    {
        while(ros::ok())
        {
            if(FlagTimer.Hz_50)
            {
                FlagTimer.Hz_50=0;
                //================
                //                unsigned char Trans_chr[1] ={ '0'};
                //                mySAM->Send_Serial_String(mySAM->Serial, Trans_chr, 1);
                if(sys_flag.getPos12LowerLink_50Hz)
                {
                    mySAM->getAllPos12();
                }
                else if(sys_flag.getPos12FullBody_50Hz)
                {
                    mySAM->getAllPos12Full();;
                }
            }
            if(FlagTimer.Hz_100)
            {
                FlagTimer.Hz_100=0;
            }
            if(FlagTimer.Hz_125)
            {
                FlagTimer.Hz_125=0;
                //===============
                if(sys_flag.getPos12LowerLink_125Hz)
                {
                    mySAM->getAllPos12();
                }
                else if(sys_flag.getPos12FullBody_125Hz)
                {
                    mySAM->getAllPos12Full();;
                }
            }


            //==========================================
            mySAM->Recev_Data_hanlder();
            if(mySAM->flagDataReceived_readAllPos12){
                mySAM->flagDataReceived_readAllPos12=0;
                //===========================================
                unsigned char Trans_chr[1] ={ '1'};
                mySAM->Send_Serial_String(mySAM->Serial, Trans_chr, 1);

                for(unsigned char i=0; i<25;i++){
                    if(mySAM->samPos12Avail[i])
                    {
                        samMsg.SAMPos12Avail[i]=1;
                        samMsg.SAMPos12[i]=mySAM->samPos12[i];
                    }else{
                        samMsg.SAMPos12Avail[i]=0;
                        samMsg.SAMPos12[i]=0;
                    }
                }
                if(sys_flag.getPos12FullBody_50Hz|sys_flag.getPos12FullBody_125Hz|sys_flag.getPos12LowerLink_50Hz|sys_flag.getPos12LowerLink_125Hz)
                {
                    sam_pub.publish(samMsg);
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
            Timer_handler();
        }
    }else{
        cout << "SERIAL : " << mySAM->Serial << " Connection error." << endl;
    }
    cout << endl;
    cout << "SERIAL : " << mySAM->Serial << " Device close." << endl;
    close(mySAM->Serial);
    cout << "SERIAL : " << "sam_hub node terminate." << endl;
    return 0;
}
