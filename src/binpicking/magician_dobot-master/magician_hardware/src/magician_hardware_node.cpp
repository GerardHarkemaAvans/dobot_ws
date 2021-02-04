

#include <magician_hardware/magician_hardware_interface.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "magician_hardware/Suction.h"
#include "magician_hardware/calibrate.h"
#include "magician_hardware/isMoving.h"
#include "magician_hardware/isDoneExecutingCommand.h"
#include "magician_hardware/isConnected.h"
#include "magician_hardware/reconnect.h"
#include "magician_hardware/moveToPosition.h"
#include "magician_hardware/isOnline.h"
#include "magician_hardware/turnSuc.h"
#include <boost/algorithm/string.hpp>
#include <math.h>
#include <cmath>
#include <stdlib.h>   



using namespace std;




typedef struct{
    controller_manager::ControllerManager *manager;
    magician_hardware::MagicianHWInterface *interface;
}ArgsForThread;

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

static boost::mutex reset_pose_mutex;
static boost::mutex stop_update_mutex;
static bool reseting_pose;
static bool stopping_update;

static std::string usb;
static bool isConnectedVar;

static controller_manager::ControllerManager* ctlr_maganer_ptr;
static magician_hardware::MagicianHWInterface* interface_ptr;


bool magician_hardware::MagicianHWInterface::ResetPose(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    std::vector<std::string> start_list;
    std::vector<std::string> stop_list;

    start_list.clear();
    stop_list.clear();
    stop_list.push_back("magician_arm_controller");

    ctlr_maganer_ptr->switchController(start_list, stop_list, 2);

    boost::mutex::scoped_lock reset_pose_lock(reset_pose_mutex);
    reseting_pose=true;
    reset_pose_lock.unlock();

    ros::Rate r(10);
    r.sleep();

    bool stop_update=false;
    boost::mutex::scoped_lock stop_update_lock(stop_update_mutex);
    stop_update=stopping_update;
    stop_update_lock.unlock();

    if(!stop_update)
    {
        resp.message="Failed, robot may be moving";
        resp.success=false;
        boost::mutex::scoped_lock reset_pose_lock(reset_pose_mutex);
        reseting_pose=false;
        reset_pose_lock.unlock();

        stop_list.clear();
        start_list.push_back("magician_arm_controller");

        ctlr_maganer_ptr->switchController(start_list, stop_list, 2);

        return true;
    }

    std::vector<double> jnt_values;
    bool result=magician_device_->ResetPose(req, resp, jnt_values);

    if(!resp.success)
    {
        boost::mutex::scoped_lock reset_pose_lock(reset_pose_mutex);
        reseting_pose=false;
        reset_pose_lock.unlock();

        stop_list.clear();
        start_list.push_back("magician_arm_controller");

        ctlr_maganer_ptr->switchController(start_list, stop_list, 2);

        return true;
    }

    if(!reinitPose(jnt_values))
    {
        resp.message="Failed, the length of joint_values is wrong";
        resp.success=false;
        boost::mutex::scoped_lock reset_pose_lock(reset_pose_mutex);
        reseting_pose=false;
        reset_pose_lock.unlock();

        stop_list.clear();
        start_list.push_back("magician_arm_controller");

        ctlr_maganer_ptr->switchController(start_list, stop_list, 2);

        return true;
    }
    else
    {
        boost::mutex::scoped_lock reset_pose_lock(reset_pose_mutex);
        reseting_pose=false;
        reset_pose_lock.unlock();

        stop_list.clear();
        start_list.push_back("magician_arm_controller");

        ctlr_maganer_ptr->switchController(start_list, stop_list, 2);

        return true;
    }
}

void *update_loop(void *threadarg)
{
    ArgsForThread *arg=(ArgsForThread *)threadarg;
    controller_manager::ControllerManager *manager=arg->manager;
    magician_hardware::MagicianHWInterface *interface=arg->interface;
    ros::Duration d(0.016);
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    //time for checking overrun
    struct timespec before;
    double overrun_time;

    bool reset_pose=false;
    boost::mutex::scoped_lock reset_pose_lock(reset_pose_mutex);
    reset_pose_lock.unlock();
    boost::mutex::scoped_lock stop_update_lock(stop_update_mutex);
    stop_update_lock.unlock();

    while(ros::ok())
    {
        ros::Time this_moment(tick.tv_sec, tick.tv_nsec);

        //check if robot is connected
        uint32_t time ;
        int i = GetDeviceTime(&time);
        if (i == 2)
        {
            isConnectedVar = false;
            DisconnectDobot();
        }

        reset_pose_lock.lock();
        reset_pose=reseting_pose;
        reset_pose_lock.unlock();

        if(reset_pose)
        {
            clock_nanosleep(CLOCK_REALTIME/10, TIMER_ABSTIME, &tick, nullptr);
            if(interface->isMoving())
            {
                reset_pose=false;
            }
        }

        if(!reset_pose)
        {
            interface->read(this_moment, d);
            manager->update(this_moment, d);
            interface->write(this_moment, d);
        }
        else
        {
            stop_update_lock.lock();
            stopping_update=true;
            stop_update_lock.unlock();
        }

        timespecInc(tick, d.nsec);
        // check overrun
        clock_gettime(CLOCK_REALTIME, &before);
        overrun_time = (before.tv_sec + double(before.tv_nsec)/1e+9) -  (tick.tv_sec + double(tick.tv_nsec)/1e+9);
        if(overrun_time > 0.0)
        {
            tick.tv_sec=before.tv_sec;
            tick.tv_nsec=before.tv_nsec;
            //std::cout<<"overrun_time: "<<overrun_time<<std::endl;
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, nullptr);
    }
}

/*
    Function for turning the suction cup on/off
*/
bool suctionFunction(magician_hardware::Suction::Request  &req, magician_hardware::Suction::Response &res)
{
    uint64_t index;
    SetEndEffectorSuctionCup(req.on,req.on,false, &index);
    res.completed = true;
    return true;
}

/*
    Function for automaticly calibrate
*/
bool calibrate(magician_hardware::calibrate::Request  &req, magician_hardware::calibrate::Response &res)
{
    SetQueuedCmdClear();
    SetQueuedCmdStartExec();
    HOMECmd home;
    uint64_t homeIndex;
    int i = SetHOMECmd(&home,false, &homeIndex);
    res.index = homeIndex;
    
    PTPCommonParams param4;
    param4.velocityRatio = 300;
    param4.accelerationRatio = 300;
    uint64_t index5;
    SetPTPCommonParams(&param4,true,&index5);

    res.completed = true;
    //std::cout << i << + " + " << DobotCommunicate_BufferFull << std::endl;
    return true;
}

/*
    Function for automaticly calibrate
*/
bool isMoving(magician_hardware::isMoving::Request  &req, magician_hardware::isMoving::Response &res)
{
    res.isMoving = interface_ptr->isMoving();
    return true;
}

/*
    Function to check if the given command has been executed
*/
bool isDoneExecutingCommand(magician_hardware::isDoneExecutingCommand::Request  &req, magician_hardware::isDoneExecutingCommand::Response &res)
{
    uint64_t queuedCmdCurrentIndex;
    GetQueuedCmdCurrentIndex(&queuedCmdCurrentIndex);
    if(req.index < queuedCmdCurrentIndex)
    {
        res.isDoneExecutingCommand = true;
    }
    return true;
}

bool moveToPosition(magician_hardware::moveToPosition::Request &req,magician_hardware::moveToPosition::Response &res)
{
    CPCmd cmd;
    cmd.cpMode = CPAbsoluteMode;

    //cout <<"identi" <<  pow(req.position.x,2) + pow(req.position.y,2) << " "  << pow(0.33,2) << endl;
    if( pow(req.position.x,2) + pow(req.position.y,2) < pow(0.35,2) && 
        pow(req.position.x,2) + pow(req.position.y,2) > pow(0.17,2))
    {
        cmd.x = req.position.y * 1000;
        cmd.y = req.position.x * 1000 * -1;
        cmd.z = req.position.z * 1000 - 65;

        uint64_t index;
        SetCPCmd(&cmd,false,&index );
        res.succes = true;
    }else {
        res.succes = false;
    }
    
    
    return true;
}

bool isConnected(magician_hardware::isConnected::Request &req,magician_hardware::isConnected::Response &res)
{
    res.isConnected = isConnectedVar;
    return true;
}

bool reconnect(magician_hardware::reconnect::Request &req,magician_hardware::reconnect::Response &res)
{
    
    DisconnectDobot();
    int result = ConnectDobot(usb.c_str(), 115200, 0, 0);
    if(result == DobotConnect_NoError) {
        res.succes = true;
    }else res.succes = false;
    isConnectedVar = true;
    SetQueuedCmdClear();
    SetQueuedCmdStartExec();

    PTPJointParams p;
    GetPTPJointParams(&p);
    PTPCoordinateParams param2;
        param2.xyzVelocity = 1500;
            param2.xyzAcceleration = 1500;
            param2.rVelocity = 1500;
            param2.rAcceleration = 1500;
    uint64_t index3;
    SetPTPCoordinateParams(&param2,true,&index3);

    PTPJumpParams param3;
    param3.jumpHeight = 1500;
    param3.zLimit = 1500;
    uint64_t index4;
    SetPTPJumpParams(&param3,true,&index4);

    PTPCommonParams param4;
    param4.velocityRatio = 1500;
    param4.accelerationRatio = 1500;
    uint64_t index5;
    SetPTPCommonParams(&param4,true,&index5);

    CPParams cppcm;
    uint64_t cppcm_index;
    GetCPParams(&cppcm);
    cppcm.planAcc = 1500;
    cppcm.juncitionVel = 1500;
    cppcm.acc = 1500;
    SetCPParams(&cppcm,false,&cppcm_index);

    return true;
}

bool isOnline(magician_hardware::isOnline::Request &req,magician_hardware::isOnline::Response &res)
{
    try {
        char *list;
        SearchDobot(list, 1000);
        string str(list);
        res.isOnline = !str.empty();
    } catch (const std::exception& e) {
        ROS_ERROR(e.what());
    }
    
    return true;
}

bool turnSuc(magician_hardware::turnSuc::Request &req,magician_hardware::turnSuc::Response &res)
{
    if(abs(req.degrees) < 150)
    {
        try{
            Pose pose;
            GetPose(&pose);
            uint64_t indexp;
            PTPCmd cmd2;
            cmd2.ptpMode = PTPMOVJXYZMode;
            //std::cout << "Turnpose: " << pose.x << " - " << pose.y << " - " << pose.z << " - " << req.degrees << std::endl;
            cmd2.x = pose.x;
            cmd2.y = pose.y;
            cmd2.z = pose.z;
            cmd2.r = req.degrees;
            SetPTPCmd(&cmd2, true, &indexp);
            res.succes = true;
        }catch(const std::exception& e) {
            ROS_ERROR(e.what());
            res.succes = false;
        }
    }else res.succes = false;
    
    
    return true;
}

int main(int argc, char** argv)
{
    usb = std::string(argv[1]);
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot not found!");
            return -2;
        break;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            return -3;
        break;
        default:
        break;
    }
    std::cout << "test" << std::endl;
    
    
    const char *deviceName = "binpicking";
    SetDeviceName(deviceName);

    isConnectedVar = true;
    
    ros::init(argc, argv, "magician_hardware_node", ros::init_options::AnonymousName);

    std::cout << "test2" << std::endl;

    uint32_t i = 4100;
    SetCmdTimeout(i);

    std::cout << "test3" << std::endl;

    //Init params for moving

    //Set the velocity and acceleration
    PTPJointParams param;   
    param.velocity[0] = 1500; 
    param.velocity[1] = 1500;
    param.velocity[2] = 1500;
    param.velocity[3] = 1500; 
    param.acceleration[0] = 1500; 
    param.acceleration[1] = 1500;
    param.acceleration[2] = 1500;
    param.acceleration[3] = 1500; 
    uint64_t index2;
    int l = SetPTPJointParams( &param,true, &index2);

    PTPJointParams p;
    GetPTPJointParams(&p);
    PTPCoordinateParams param2;
        param2.xyzVelocity = 1500;
            param2.xyzAcceleration = 1500;
            param2.rVelocity = 1500;
            param2.rAcceleration = 1500;
    uint64_t index3;
    SetPTPCoordinateParams(&param2,true,&index3);

    PTPJumpParams param3;
    param3.jumpHeight = 1500;
    param3.zLimit = 1500;
    uint64_t index4;
    SetPTPJumpParams(&param3,true,&index4);

    PTPCommonParams param4;
    param4.velocityRatio = 1500;
    param4.accelerationRatio = 1500;
    uint64_t index5;
    SetPTPCommonParams(&param4,true,&index5);

    CPParams cppcm;
    uint64_t cppcm_index;
    GetCPParams(&cppcm);
    cppcm.planAcc = 1500;
    cppcm.juncitionVel = 1500;
    cppcm.acc = 1500;
    SetCPParams(&cppcm,false,&cppcm_index);
        std::cout << "test" << std::endl;

    magician_hardware::MagicianHWInterface magician_hw_interface;
    controller_manager::ControllerManager ctlr_manager(&magician_hw_interface);

    ctlr_maganer_ptr=&ctlr_manager;

    ros::NodeHandle n1, n2("~"),suctionNode,calibrateNode,isMovingNode,isDoneExecutingCommandNode,moveToPositionNode,
    checkConnNode,reconnNode,isOnlineNode,turnSucNode;
    magician_hw_interface.init(n1, n2);
    std::cout << "test" << std::endl;

    //Create the service for turning the suctioncup on/off
    ros::ServiceServer suctionService = suctionNode.advertiseService("magician/suction", suctionFunction);
    ROS_INFO("Started suction service");
    ros::ServiceServer calibrateService = calibrateNode.advertiseService("magician/calibrate", calibrate);
    ROS_INFO("Started calibrate service");
    ros::ServiceServer isMovingService = isMovingNode.advertiseService("magician/isMoving", isMoving);
    ROS_INFO("Started isMoving service");
    ros::ServiceServer isDoneExecutingCommandService = isDoneExecutingCommandNode.advertiseService("magician/isDoneExecutingCommand", isDoneExecutingCommand);
    ROS_INFO("Started isDoneExecutingCommand service");

    ros::ServiceServer isConnectedService = checkConnNode.advertiseService("magician/isConnected", isConnected);
    ROS_INFO("Started isConnected service");
    ros::ServiceServer reconnectService = reconnNode.advertiseService("magician/reconnect", reconnect);
    ROS_INFO("Started reconnect service");
    ros::ServiceServer isOnlineService = isOnlineNode.advertiseService("magician/isOnline", isOnline);
    ROS_INFO("Started isOnline service");

    ros::ServiceServer turnSucService = turnSucNode.advertiseService("magician/turnSuc", turnSuc);
    ROS_INFO("Started turnSuc service");

    ros::ServiceServer moveToPositionService = moveToPositionNode.advertiseService("magician/moveToPosition", moveToPosition);
    ROS_INFO("Started moveToPosition service");
    
    reseting_pose=false;
    stopping_update=false;

    ros::ServiceServer reset_pose_server=n2.advertiseService("reset_pose", &magician_hardware::MagicianHWInterface::ResetPose, &magician_hw_interface);

    pthread_t tid;
    ArgsForThread *thread_arg=new ArgsForThread();
    thread_arg->manager=&ctlr_manager;
    thread_arg->interface=&magician_hw_interface;
    interface_ptr = &magician_hw_interface;

    pthread_create(&tid, nullptr, update_loop, thread_arg);

    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

