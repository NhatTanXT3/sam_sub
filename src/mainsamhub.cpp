#define SAM_ACTUATOR_
#include <iostream>
#include <ros/ros.h>
using namespace std;
#include "sam_hub/sammodule.h"
#include "sam_hub/mainsamhub.h"
#include "sam_hub/SAMJointStateMsg.h"
#include "sam_hub/SAMcmdMsg.h"
#include "sam_hub/SAMJointPos12Msg.h"
#include "sam_hub/SAMJointPIDMsg.h"
#include "sam_hub/SAMJointTorqueMsg.h"
#include "sam_hub/SAMtfCalMsg.h"

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


void sub_function_pos12(const sam_hub::SAMJointPos12Msg::ConstPtr& msg){
    unsigned int pos12[25];
    for (unsigned char i=0; i<25;i++)
    {
        if(msg->SAMMode[i])
            pos12[i]=msg->SAMPos12[i];
    }
#ifdef SAM_ACTUATOR_
    mySAM->setAllPos12(pos12,12);
#endif

    if(msg->SAMMode[22]){
        usleep(5000);
#ifdef SAM_ACTUATOR_
        mySAM->setSamPos12(22,pos12[22]);
#endif
    }
}

void sub_function_torque(const sam_hub::SAMJointTorqueMsg::ConstPtr& msg){
    unsigned int torque[12];
    for (unsigned char i=0; i<12;i++)
    {
        torque[i]=msg->Torque[i];
        cout<<"torque:"<<(int)torque[i]<<endl;
    }
    mySAM->setAllAverageTorque(torque,12);
}

void sub_function_pid(const sam_hub::SAMJointPIDMsg::ConstPtr& msg){
    unsigned char samP[12];
    unsigned char samD[12];
    for (unsigned char i=0; i<12;i++)
    {
        samP[i]=msg->P[i];
        samD[i]=msg->D[i];
        cout<<"sam P:"<<(int)samP[i]<<"sam D:"<< (int)samD[i]<<endl;
        //        pos12[i]=msg->[i];
    }
    mySAM->setAllPDQuick(samP,samD,12);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sam_hub");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE_1000Hz_);
    mySAM =new SAMmodule;

    ros::Publisher sam_pub =  n.advertise<sam_hub::SAMJointStateMsg>("sam_pub",1000);
    ros::Subscriber sam_sub=n.subscribe<sam_hub::SAMcmdMsg>("sam_cmd_sub",200,sub_function_cmd);
    ros::Subscriber samPos12_sub=n.subscribe<sam_hub::SAMJointPos12Msg>("sam_pos12_sub",1000,sub_function_pos12);
    ros::Subscriber samPID_sub=n.subscribe<sam_hub::SAMJointPIDMsg>("sam_PID",200,sub_function_pid);
    ros::Subscriber samTorque_sub=n.subscribe<sam_hub::SAMJointTorqueMsg>("sam_torque",200,sub_function_torque);

    ros::Publisher sam_tfCal_pub =n.advertise<sam_hub::SAMtfCalMsg>("sam_tfCal_pub",500);
    sam_hub::SAMtfCalMsg samtfCalMsg;
    sam_hub::SAMJointStateMsg samMsg;

    sys_flag.getPos12LowerLink_50Hz=0;
    sys_flag.getPos12FullBody_50Hz=0;
    sys_flag.getPos12LowerLink_125Hz=0;
    sys_flag.getPos12FullBody_125Hz=0;

    if(mySAM->Serial!= -1)
    {
        ROS_INFO("set default parameter (PID, average torque)....");
        mySAM->setAllPIDQuick(default_samP,default_samD,default_samI,25);
        usleep(10000);
        mySAM->setAllAverageTorque(default_averageTorq,25);
        usleep(10000);
        mySAM->setAllPIDQuick(default_samP,default_samD,default_samI,25);
        usleep(10000);
        mySAM->setAllAverageTorque(default_averageTorq,25);
        usleep(10000);
        ROS_INFO("sam_hub ready!");
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
                //                unsigned char Trans_chr[1] ={ '1'};
                //                mySAM->Send_Serial_String(mySAM->Serial, Trans_chr, 1);

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

                unsigned char lowerLinkCount=0;
                for(unsigned char i=0; i<12; i++){
                    if(mySAM->samPos12Avail[i])
                        lowerLinkCount++;
                }

                if(sys_flag.getPos12FullBody_50Hz|sys_flag.getPos12FullBody_125Hz|sys_flag.getPos12LowerLink_50Hz|sys_flag.getPos12LowerLink_125Hz)
                {
                    sam_pub.publish(samMsg);
                    if(lowerLinkCount==12)
                    {
                        /*
                         *  tfCal function
                         */
                        for(unsigned char i=0;i<12;i++)
                        {
                            angle[i]=((double)mySAM->samPos12[i]-(double)samPos12_hardware[i])*pos12bitTorad*angle_sign[i];
                        }
                        double xValue[12],yValue[12],zValue[12];
                        xValue[0]=-0.083*cos(angle[1]);
                        yValue[0]=-0.083*-sin(angle[1])-0.00121;
                        zValue[0]=0;

                        xValue[1]=xValue[0]*cos(-angle[3])+zValue[0]*-sin(-angle[3])-0.208;
                        yValue[1]=yValue[0];
                        zValue[1]=xValue[0]*sin(-angle[3])+zValue[0]*cos(-angle[3])-0.00049844;

                        xValue[2]=yValue[1];
                        yValue[2]=xValue[1]*-sin(angle[5])+zValue[1]*-cos(angle[5])-0.0043;
                        zValue[2]=xValue[1]*-cos(angle[5])+zValue[1]*sin(angle[5])+0.21;


                        xValue[3]=xValue[2];
                        yValue[3]=yValue[2]*-sin(angle[7])+zValue[2]*-cos(angle[7]);
                        zValue[3]=yValue[2]*cos(angle[7])+zValue[2]*-sin(angle[7]);


                        xValue[4]=xValue[3]*sin(angle[9])+yValue[3]*cos(angle[9])-0.063895;
                        yValue[4]=xValue[3]*-cos(angle[9])+yValue[3]*sin(angle[9])+0.0000998;
                        zValue[4]=zValue[3];

                        xValue[5]=yValue[4]*-cos(angle[11])+zValue[4]*sin(angle[11])+0.057;
                        yValue[5]=yValue[4]*-sin(angle[11])+zValue[4]*-cos(angle[11])-0.00034;
                        zValue[5]=xValue[4]-0.06;
//===================================================
                        xValue[6]=0.083*-cos(-angle[0]);
                        yValue[6]=0.083*-sin(-angle[0]);
                        zValue[6]=0;

                        xValue[7]=yValue[6];
                        yValue[7]=xValue[6]*cos(angle[2])+zValue[6]*sin(angle[2])-0.209;
                        zValue[7]=xValue[6]*sin(angle[2])+zValue[6]*-cos(angle[2])+0.00049844;

                        xValue[8]=xValue[7];
                        yValue[8]=yValue[7]*-sin(-angle[4])+zValue[7]*cos(-angle[4])-0.0043;
                        zValue[8]=yValue[7]*-cos(-angle[4])+zValue[7]*-sin(-angle[4])+0.21;



                        xValue[9]=xValue[8];
                        yValue[9]=yValue[8]*-sin(angle[6])+zValue[8]*-cos(angle[6]);
                        zValue[9]=yValue[8]*cos(angle[6])+zValue[8]*-sin(angle[6]);


                        xValue[10]=xValue[9]*sin(angle[8])+yValue[9]*cos(angle[8])-0.063895;
                        yValue[10]=xValue[9]*-cos(angle[8])+yValue[9]*sin(angle[8]);
                        zValue[10]=zValue[9];

                        xValue[11]=yValue[10]*-cos(angle[10])+zValue[10]*sin(angle[10])-0.057;
                        yValue[11]=yValue[10]*-sin(angle[10])+zValue[10]*-cos(angle[10])-0.00034;
                        zValue[11]=xValue[10]-0.06;

      //=========================================
                        samtfCalMsg.CN1=xValue[11];
                        samtfCalMsg.CN2=yValue[11];
                        samtfCalMsg.CN3=zValue[11];
                        samtfCalMsg.CN4=xValue[5];
                        samtfCalMsg.CN6=yValue[5];
                        samtfCalMsg.CN7=zValue[5];
                        sam_tfCal_pub.publish(samtfCalMsg);

                    }
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
