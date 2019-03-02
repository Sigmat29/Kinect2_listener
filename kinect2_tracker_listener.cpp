#include "MyFunctions.h"
#include "kinect_v2_class.h"


int main(int argc, char** argv)
{


ros::init(argc, argv, "kinect_v2_listener");

ros::NodeHandle node;
kinect_v2_class ic;
  dynamic_reconfigure::Server<kinect2_tracker::set_kinect_v2Config> server;
  dynamic_reconfigure::Server<kinect2_tracker::set_kinect_v2Config>::CallbackType f;
  f = boost::bind(&kinect_v2_class::reconfigure, boost::ref(ic), _1);
  server.setCallback(f);


// publisher declaration
ros::Publisher neck_joint = node.advertise<geometry_msgs::Point>("neck_joint", 1);
ros::Publisher head_joint = node.advertise<geometry_msgs::Point>("head_joint", 1);
ros::Publisher torso_joint = node.advertise<geometry_msgs::Point>("torso_joint", 1);
ros::Publisher left_shoulder_joint = node.advertise<geometry_msgs::Point>("left_shoulder_joint", 1);
ros::Publisher left_elbow_joint = node.advertise<geometry_msgs::Point>("left_elbow_joint", 1);
ros::Publisher left_hand_joint = node.advertise<geometry_msgs::Point>("left_hand_joint", 1);
ros::Publisher right_shoulder_joint = node.advertise<geometry_msgs::Point>("right_shoulder_joint", 1);
ros::Publisher right_elbow_joint = node.advertise<geometry_msgs::Point>("right_elbow_joint", 1);
ros::Publisher right_hand_joint = node.advertise<geometry_msgs::Point>("right_hand_joint", 1);
ros::Publisher left_hip_joint = node.advertise<geometry_msgs::Point>("left_hip_joint", 1);
ros::Publisher left_knee_joint = node.advertise<geometry_msgs::Point>("left_knee_joint", 1);
ros::Publisher left_foot_joint = node.advertise<geometry_msgs::Point>("left_foot_joint", 1);
ros::Publisher right_hip_joint = node.advertise<geometry_msgs::Point>("right_hip_joint", 1);
ros::Publisher right_knee_joint = node.advertise<geometry_msgs::Point>("right_knee_joint", 1);
ros::Publisher right_foot_joint = node.advertise<geometry_msgs::Point>("right_foot_joint", 1);

// listener
tf::TransformListener listener;

ros::Rate rate(30.0); // frequency of operation
//DRONE PART
ros::init(argc, argv, "drone");
ros::NodeHandle n;
topictakeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1,true);
topiclanding = n.advertise<std_msgs::Empty>("/ardrone/land",1,true);
emergency= n.advertise<std_msgs::Empty>("/ardrone/reset",1,true);
cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
serviceflattrim = n.serviceClient<std_srvs::Empty>("/ardrone/flattrim");
cout<<"Dron zaincjalizowany. W celu wyboru interfejsu sterowania wcisnij"<<endl;
cout<<"2 - sterowanie dlonia"<<endl;
push_key=getch();


if(push_key=='2')//dlon

{


    cout<<"Wybrano sterowanie dlonia."<<endl<<""<<endl<<endl;

    adjust();
    printf("Calibration\n");
    takeoff();
    ////////////////////////////////////////CZESC ODPOWIEDZIALNA ZA STEROWANIE DLONIA//////////////////////////////////////////////////////

while (node.ok())
{

  ros::spinOnce();

    // Transforms declared for each joint
    tf::StampedTransform transform_neck, transform_head, transform_torso,
                            transform_left_shoulder, transform_left_elbow, transform_left_hand,
                                transform_right_shoulder, transform_right_elbow, transform_right_hand,
                                    transform_left_hip, transform_left_knee, transform_left_foot,
                                        transform_right_hip, transform_right_knee, transform_right_foot;
    try
    {
        // each joint frame to reference frame transforms
        listener.lookupTransform("kinect/user_1/neck", "kinect_link",ros::Time(0), transform_neck);
        listener.lookupTransform("kinect/user_1/head", "kinect_link",ros::Time(0), transform_head);
        listener.lookupTransform("kinect/user_1/torso", "kinect_link",ros::Time(0), transform_torso);
        listener.lookupTransform("kinect/user_1/left_shoulder", "kinect_link",ros::Time(0), transform_left_shoulder);
        listener.lookupTransform("kinect/user_1/left_elbow", "kinect_link",ros::Time(0), transform_left_elbow);
        listener.lookupTransform("kinect/user_1/left_hand", "kinect_link",ros::Time(0), transform_left_hand);
        listener.lookupTransform("kinect/user_1/right_shoulder", "kinect_link",ros::Time(0), transform_right_shoulder);
        listener.lookupTransform("kinect/user_1/right_elbow", "kinect_link",ros::Time(0), transform_right_elbow);
        listener.lookupTransform("kinect/user_1/right_hand", "kinect_link",ros::Time(0), transform_right_hand);
        //listener.lookupTransform("kinect/user_1/left_hip", "kinect_link",ros::Time(0), transform_left_hip);
        //listener.lookupTransform("kinect/user_1/left_knee", "kinect_link",ros::Time(0), transform_left_knee);
        //listener.lookupTransform("kinect/user_1/left_foot", "kinect_link",ros::Time(0), transform_left_foot);
        //listener.lookupTransform("kinect/user_1/right_hip", "kinect_link",ros::Time(0), transform_right_hip);
        //listener.lookupTransform("kinect/user_1/right_knee", "kinect_link",ros::Time(0), transform_right_knee);
        //listener.lookupTransform("kinect/user_1/right_foot", "kinect_link",ros::Time(0), transform_right_foot);

    }
        catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.10).sleep();
        continue;
    }

    // geometry points declaration for storing 3D coordinates of joints and then published later
    geometry_msgs::Point neck_pose, head_pose, torso_pose,
                            left_shoulder_pose, left_elbow_pose, left_hand_pose,
                                right_shoulder_pose, right_elbow_pose, right_hand_pose,
                                    left_hip_pose, left_knee_pose, left_foot_pose,
                                        right_hip_pose, right_knee_pose, right_foot_pose;

    // joint position extraction and store
    // neck joint
    neck_pose.x = transform_neck.getOrigin().x();
    neck_pose.y = transform_neck.getOrigin().y();
    neck_pose.z = transform_neck.getOrigin().z();
    // head joint
    head_pose.x = transform_head.getOrigin().x();
    head_pose.y = transform_head.getOrigin().y();
    head_pose.z = transform_head.getOrigin().z();
    // torso joint
    torso_pose.x = transform_torso.getOrigin().x();
    torso_pose.y = transform_torso.getOrigin().y();
    torso_pose.z = transform_torso.getOrigin().z();
    // left shoulder joint
    left_shoulder_pose.x = transform_left_shoulder.getOrigin().x();
    left_shoulder_pose.y = transform_left_shoulder.getOrigin().y();
    left_shoulder_pose.z = transform_left_shoulder.getOrigin().z();
    // left elbow joint
    left_elbow_pose.x = transform_left_elbow.getOrigin().x();
    left_elbow_pose.y = transform_left_elbow.getOrigin().y();
    left_elbow_pose.z = transform_left_elbow.getOrigin().z();
    // left hand joint
    left_hand_pose.x = -transform_left_hand.getOrigin().x();
    left_hand_pose.y = -transform_left_hand.getOrigin().y();
    left_hand_pose.z = transform_left_hand.getOrigin().z();
    // right shoulder joint
    right_shoulder_pose.x = transform_right_shoulder.getOrigin().x();
    right_shoulder_pose.y = transform_right_shoulder.getOrigin().y();
    right_shoulder_pose.z = transform_right_shoulder.getOrigin().z();
    // right elbow joint
    right_elbow_pose.x = transform_right_elbow.getOrigin().x();
    right_elbow_pose.y = transform_right_elbow.getOrigin().y();
    right_elbow_pose.z = transform_right_elbow.getOrigin().z();
    // right hand joint
    right_hand_pose.x = -transform_right_hand.getOrigin().x();
    right_hand_pose.y = -transform_right_hand.getOrigin().y();
    right_hand_pose.z = transform_right_hand.getOrigin().z();
    // left hip joint
    left_hip_pose.x = transform_left_hip.getOrigin().x();
    left_hip_pose.y = transform_left_hip.getOrigin().y();
    left_hip_pose.z = transform_left_hip.getOrigin().z();
    // left knee joint
    left_knee_pose.x = transform_left_knee.getOrigin().x();
    left_knee_pose.y = transform_left_knee.getOrigin().y();
    left_knee_pose.z = transform_left_knee.getOrigin().z();
    // left foot joint
    left_foot_pose.x = transform_left_foot.getOrigin().x();
    left_foot_pose.y = transform_left_foot.getOrigin().y();
    left_foot_pose.z = transform_left_foot.getOrigin().z();
    // right hip joint
    right_hip_pose.x = transform_right_hip.getOrigin().x();
    right_hip_pose.y = transform_right_hip.getOrigin().y();
    right_hip_pose.z = transform_right_hip.getOrigin().z();
    // right knee joint
    right_knee_pose.x = transform_right_knee.getOrigin().x();
    right_knee_pose.y = transform_right_knee.getOrigin().y();
    right_knee_pose.z = transform_right_knee.getOrigin().z();
    // right foot joint
    right_foot_pose.x = transform_right_foot.getOrigin().x();
    right_foot_pose.y = transform_right_foot.getOrigin().y();
    right_foot_pose.z = transform_right_foot.getOrigin().z();

    // joint positions publish
    neck_joint.publish(neck_pose);
    head_joint.publish(head_pose);
    torso_joint.publish(torso_pose);
    left_shoulder_joint.publish(left_shoulder_pose);
    left_elbow_joint.publish(left_elbow_pose);
    left_hand_joint.publish(left_hand_pose);
    right_shoulder_joint.publish(right_shoulder_pose);
    right_elbow_joint.publish(right_elbow_pose);
    right_hand_joint.publish(right_hand_pose);
    left_hip_joint.publish(left_hip_pose);
    left_knee_joint.publish(left_knee_pose);
    left_foot_joint.publish(left_foot_pose);
    right_hip_joint.publish(right_hip_pose);
    right_knee_joint.publish(right_knee_pose);
    right_foot_joint.publish(right_foot_pose);

    if(push_key_hand==50 && marker1==1) //prawa dlon
    {
        if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y>minY && right_hand_pose.y<maxY && right_hand_pose.z>minZ  && right_hand_pose.z<maxZ)
        {
            stop();
            cout<<"stop"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y>maxY && right_hand_pose.z>minZ  && right_hand_pose.z<maxZ)
        {
            up();
            cout<<"gora"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;

        }
        else if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y<minY && right_hand_pose.z>minZ && right_hand_pose.z<maxZ)
        {
            down();
            cout<<"dol"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x>maxX && right_hand_pose.y>minY && right_hand_pose.y<maxY && right_hand_pose.z>minZ  && right_hand_pose.z<maxZ)
        {
            right();
            cout<<"prawo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x<minX && right_hand_pose.y>minY && right_hand_pose.y<maxY && right_hand_pose.z>minZ  && right_hand_pose.z<maxZ)
        {
            left();
            cout<<"lewo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y>minY && right_hand_pose.y<maxY && right_hand_pose.z<minZ)
        {
            back();
            cout<<"tyl"<<"     X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y>minY && right_hand_pose.y<maxY && right_hand_pose.z>maxZ)
        {
            forward();
            cout<<"przod"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(right_hand_pose.x>maxX && right_hand_pose.y>maxY && right_hand_pose.z>minZ && right_hand_pose.z<maxZ)
        {
            up_right();
            cout<<"gora_prawo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x<minX && right_hand_pose.y>maxY && right_hand_pose.z>minZ  && right_hand_pose.z<maxZ)
        {
            up_left();
            cout<<"gora_lewo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;

        }
        else if(right_hand_pose.x>maxX && right_hand_pose.y<minY && right_hand_pose.z>minZ  && right_hand_pose.z<maxZ)
        {
            down_right();
            cout<<"dol_prawo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x<minX && right_hand_pose.y<minY && right_hand_pose.z>minZ && right_hand_pose.z<maxZ)
        {
            down_left();
            cout<<"dol_lewo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(right_hand_pose.x>maxX && right_hand_pose.y>minY && right_hand_pose.y<maxY  && right_hand_pose.z>maxZ)
        {
            forward_right();
            cout<<"przod_prawo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x<minX && right_hand_pose.y>minY && right_hand_pose.y<maxY  && right_hand_pose.z>maxZ)
        {
            forward_left();
            cout<<"przod_lewo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x>maxX && right_hand_pose.y>minY && right_hand_pose.y<maxY  && right_hand_pose.z<minZ)
        {
            back_right();
            cout<<"tyl_prawo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;

        }
        else if(right_hand_pose.x<minX && right_hand_pose.y>minY && right_hand_pose.y<maxY  && right_hand_pose.z<minZ)
        {
            back_left();
            cout<<"tyl_lewo"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y>maxY && right_hand_pose.z>maxZ)
        {
            up_forward();
            cout<<"gora_przod"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y>maxY && right_hand_pose.z<minZ)
        {
            up_back();
            cout<<"gora_tyl"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;

        }
        else if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y<minY && right_hand_pose.z>maxZ)
        {
            down_forward();
            cout<<"dol_przod"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;
        }
        else if(right_hand_pose.x>minX && right_hand_pose.x<maxX && right_hand_pose.y<minY && right_hand_pose.z<minZ)
        {
            down_back();
            cout<<"dol_tyl"<<"    X: "<<right_hand_pose.x<<"    Y: "<<right_hand_pose.y<<"    Z : "<<right_hand_pose.z<<endl;

        }
    }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    else if (push_key_hand==49 && marker1==0) //wczytanie aktualnej pozycji LEWEJ DLONI
    {

        zerox=left_hand_pose.x;
        zeroy=left_hand_pose.y;
        zeroz=left_hand_pose.z;
        strefa=0.2;

        maxX=zerox+strefa;
        minX=zerox-strefa;

        maxY=zeroy+strefa;
        minY=zeroy-strefa;

        maxZ=zeroz+strefa;
        minZ=zeroz-strefa;



        cout<<endl<<endl<<"Sterowanie lewa dlonia rozpoczete!"<<endl<<endl;
        marker1=1;


    }




    else if (push_key_hand==50 && marker1==0) //wczytanie aktualnej pozycji PRAWEJ DLONI
    {

        zerox=right_hand_pose.x;
        zeroy=right_hand_pose.y;
        zeroz=right_hand_pose.z;
        strefa=0.2;

        maxX=zerox+strefa;
        minX=zerox-strefa;

        maxY=zeroy+strefa;
        minY=zeroy-strefa;

        maxZ=zeroz+strefa;
        minZ=zeroz-strefa;


        cout<<endl<<endl<<"Sterowanie prawa dlonia rozpoczete!"<<endl<<endl;
        marker1=1;
    }


    else if(push_key_hand==49 && marker1==1) // lewa dlon
    {
        if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y>minY && left_hand_pose.y<maxY && left_hand_pose.z>minZ  && left_hand_pose.z<maxZ)
        {
            stop();
            cout<<"stop"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y>maxY && left_hand_pose.z>minZ  && left_hand_pose.z<maxZ)
        {
            up();
            cout<<"gora"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;

        }
        else if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y<minY && left_hand_pose.z>minZ && left_hand_pose.z<maxZ)
        {
            down();
            cout<<"dol"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x>maxX && left_hand_pose.y>minY && left_hand_pose.y<maxY && left_hand_pose.z>minZ  && left_hand_pose.z<maxZ)
        {
            right();
            cout<<"prawo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x<minX && left_hand_pose.y>minY && left_hand_pose.y<maxY && left_hand_pose.z>minZ  && left_hand_pose.z<maxZ)
        {
            left();
            cout<<"lewo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y>minY && left_hand_pose.y<maxY && left_hand_pose.z<minZ)
        {
            back();
            cout<<"tyl"<<"     X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y>minY && left_hand_pose.y<maxY && left_hand_pose.z>maxZ)
        {
            forward();
            cout<<"przod"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(left_hand_pose.x>maxX && left_hand_pose.y>maxY && left_hand_pose.z>minZ && left_hand_pose.z<maxZ)
        {
            up_right();
            cout<<"gora_prawo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x<minX && left_hand_pose.y>maxY && left_hand_pose.z>minZ  && left_hand_pose.z<maxZ)
        {
            up_left();
            cout<<"gora_lewo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;

        }
        else if(left_hand_pose.x>maxX && left_hand_pose.y<minY && left_hand_pose.z>minZ  && left_hand_pose.z<maxZ)
        {
            down_right();
            cout<<"dol_prawo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x<minX && left_hand_pose.y<minY && left_hand_pose.z>minZ && left_hand_pose.z<maxZ)
        {
            down_left();
            cout<<"dol_lewo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(left_hand_pose.x>maxX && left_hand_pose.y>minY && left_hand_pose.y<maxY  && left_hand_pose.z>maxZ)
        {
            forward_right();
            cout<<"przod_prawo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x<minX && left_hand_pose.y>minY && left_hand_pose.y<maxY  && left_hand_pose.z>maxZ)
        {
            forward_left();
            cout<<"przod_lewo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x>maxX && left_hand_pose.y>minY && left_hand_pose.y<maxY  && left_hand_pose.z<minZ)
        {
            back_right();
            cout<<"tyl_prawo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;

        }
        else if(left_hand_pose.x<minX && left_hand_pose.y>minY && left_hand_pose.y<maxY  && left_hand_pose.z<minZ)
        {
            back_left();
            cout<<"tyl_lewo"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y>maxY && left_hand_pose.z>maxZ)
        {
            up_forward();
            cout<<"gora_przod"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y>maxY && left_hand_pose.z<minZ)
        {
            up_back();
            cout<<"gora_tyl"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;

        }
        else if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y<minY && left_hand_pose.z>maxZ)
        {
            down_forward();
            cout<<"dol_przod"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;
        }
        else if(left_hand_pose.x>minX && left_hand_pose.x<maxX && left_hand_pose.y<minY && left_hand_pose.z<minZ)
        {
            down_back();
            cout<<"dol_tyl"<<"    X: "<<left_hand_pose.x<<"    Y: "<<left_hand_pose.y<<"    Z : "<<left_hand_pose.z<<endl;

        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //  cout<<"zerox:"<<zerox<<"     zeroy:"<<zeroy<<"     zeroz:"<<zeroz<<endl;

    }
    else
    {

        cout<<endl<<"Wybierz, ktora dlonia chcesz sterowac dronem."<<endl<<"UWAGA! Po dokonaniu wyboru masz trzy sekundy na ustawienie reki w dogodnej pozycji."<<endl<<"1 - lewa dlon"<<endl<<"2 - prawa dlon"<<endl<<endl;
       // marker1=0;
        push_key_hand=getch();


        cout<<">>>>>3<<<<<"<<endl;
        sleep(1);
        cout<<">>>>>2<<<<<"<<endl;
        sleep(1);
        cout<<">>>>>1<<<<<"<<endl;
        sleep(1);
    }


    ///////////////////////////////////////KONIEC CZESCI ODPOWIEDZIALNEJ ZA STEROWANIE DLONIA/////////////////////////////////////////////

    //cout<<left_hand_pose.x<<endl<<left_hand_pose.y<<endl<<left_hand_pose.z<<endl<<endl;
    //cout<<right_hand_pose.x<<"<<endl<<right_hand_pose.y<<endl<<right_hand_pose.z<<endl<<endl;

}






    //rate.sleep();


   // cout<<left_hand_pose.x<<endl<<left_hand_pose.y<<endl<<left_hand_pose.z<<endl<<endl;
}

return 0;
}
