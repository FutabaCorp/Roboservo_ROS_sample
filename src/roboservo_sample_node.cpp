/**********************************************************************
 *    includes
 *********************************************************************/
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>
#include <canopen_chain_node/GetObject.h>
#include <canopen_chain_node/SetObject.h>
#include <roboservo_sample/roboservo_paramConfig.h>

/**********************************************************************
 *    defines
 *********************************************************************/
#define VEL_UNIT 10

/**********************************************************************
 *    includes
 *********************************************************************/
bool get_obj(const std::string &node_name, const std::string &object_id, std::string &output_value);
bool set_obj(const std::string &node_name, const std::string &object_id, const std::string &input_value);
std::string convert_double2str(double input_double);
double convert_str2double(std::string input_str);
void wait_sec(int wait);


void callback(roboservo_sample::roboservo_paramConfig &config, uint32_t level)
{
    set_obj(config.Joint_name, "6081#0", convert_double2str(config.Profile_velocity * VEL_UNIT));
    set_obj(config.Joint_name, "6083#0", convert_double2str(config.Profile_acceleration * VEL_UNIT));
    set_obj(config.Joint_name, "6084#0", convert_double2str(config.Profile_deceleration * VEL_UNIT));
}

bool get_obj(const std::string &node_name, const std::string &object_id, std::string &output_value)
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<canopen_chain_node::GetObject>("/driver/get_object");
    canopen_chain_node::GetObject args;

    args.request.node = node_name;
    args.request.object = object_id;
    args.request.cached = false;
    ROS_INFO("GetObject request: %s, %s, %s", args.request.node.c_str(), args.request.object.c_str(), args.request.cached ? "True" : "False");
    
    if(client.call(args))
    {
        ROS_INFO("GetObject response: %s, %s, %s", args.response.success ? "True" : "False", args.response.message.c_str(), args.response.value.c_str());
        if(args.response.success == true)
        {
            output_value = args.response.value;
            return true;
        }
    }
    else
    {
        ROS_INFO("The service can't call.");
    }
    return false;
}

bool set_obj(const std::string &node_name, const std::string &object_id, const std::string &input_value)
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<canopen_chain_node::SetObject>("/driver/set_object");
    canopen_chain_node::SetObject args;

    args.request.node = node_name;
    args.request.object = object_id;
    args.request.value = input_value;
    args.request.cached = false;
    ROS_INFO("SetObject request: %s, %s, %s, %s", args.request.node.c_str(), args.request.object.c_str(), args.request.value.c_str(), args.request.cached ? "True" : "False");
    
    if(client.call(args))
    {
        ROS_INFO("SetObject response: %s, %s", args.response.success ? "True" : "False", args.response.message.c_str());
        if(args.response.success == true)
        {
            return true;
        }
    }
    else
    {
        ROS_INFO("The service can't call.");
    }
    return false;
}


std::string convert_double2str(double input_double)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(1) << input_double << std::endl;
    return os.str();
}

double convert_str2double(std::string input_str)
{
    double output_double;
    std::istringstream os(input_str);
    os >> output_double;
    return output_double;
}

void wait_sec(int wait)
{
    ros::Rate rate(1);
    while (wait > 0)
    {
        wait--;
        rate.sleep();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboservo_sample_node");

    // set server
    boost::recursive_mutex config_mutex;
    dynamic_reconfigure::Server<roboservo_sample::roboservo_paramConfig> server;
    
    // wait 5 seconds
    wait_sec(5);

    // get parameters of Roboservo
    std::string output_string = "0";
    roboservo_sample::roboservo_paramConfig config;
    config.Joint_name = "roboservo_1";
    bool result = false;
    if(get_obj(config.Joint_name, "6081#0", output_string))
    {
        double output_double = convert_str2double(output_string) / VEL_UNIT;
        config.Profile_velocity = output_double;
        if(get_obj(config.Joint_name, "6083#0", output_string))
        {
            output_double = convert_str2double(output_string) / VEL_UNIT;
            config.Profile_acceleration = output_double;
            if(get_obj(config.Joint_name, "6084#0", output_string))
            {
                output_double = convert_str2double(output_string) / VEL_UNIT;
                config.Profile_deceleration = output_double;
                result = true;
            }
        }
    }
 
    // update parameters
    if(result == true)
    {
        server.updateConfig(config);
    }
    
    // set callback
    dynamic_reconfigure::Server<roboservo_sample::roboservo_paramConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}
