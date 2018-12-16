#define DEVICE_NAME "1" 
#include <DynamixelWorkbench.h>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#define BAUDRATE  1000000

DynamixelWorkbench dxl_wb;


ros::NodeHandle  nh;

void messageCb( const std_msgs::Int16MultiArray& msg) 
{
    dxl_wb.goalPosition(0, msg.data[0]);
    dxl_wb.goalPosition(1, msg.data[1]);
    dxl_wb.goalPosition(2, msg.data[2]);
    dxl_wb.goalPosition(3, msg.data[3]);
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("ServoPoseSerial", messageCb );

void setup() 
{
    Serial.begin(57600);
    while(!Serial); // If this line is activated, you need to open Serial Terminal.

    dxl_wb.begin(DEVICE_NAME, BAUDRATE);
    dxl_wb.ping(0);
    dxl_wb.jointMode(0);
    dxl_wb.ping(1);
    dxl_wb.jointMode(1);
    dxl_wb.ping(2);
    dxl_wb.jointMode(2);
    dxl_wb.ping(3);
    dxl_wb.jointMode(3);

    nh.initNode();
    nh.subscribe(sub);
}

void loop() 
{
    nh.spinOnce();
}
