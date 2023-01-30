% This is the edited file for this week
%Initialization of all the components
myev3=legoev3('usb');
%Motor_A for the Gripper
mymotor_A = motor(myev3, 'A');

%Motor_B for middle motorf
mymotor_B = motor(myev3,'B');
mytouchsensor_3=touchSensor(myev3,3);


%Motor_c for Base
mymotor_C=motor(myev3,'C');
mytouchsensor_1=touchSensor(myev3,1);


syms base_angle_rot elbow_angle_rot;
%Length og the links (all dimensions are in mm)
l1 = 50;
l2 = 95;
l3 = 185;
l4 = 110;
z2 = 70;
Homing(mymotor_B, mymotor_C,mytouchsensor_1, mytouchsensor_3)
Gripper_Close(mymotor_A);
B_Touchvalue=readTouch(mytouchsensor_3);
B_Encrotation=readRotation(mymotor_B);
C_Touchvalue=readTouch(mytouchsensor_1);
C_Encrotation = readRotation(mymotor_C);
%A_Encrotation=readRotation(mymotor_A);

%Function Declaration for Homing Operation

function f= Homing(mymotor_B, mymotor_C,mytouchsensor_1, mytouchsensor_3)
    start(mymotor_B);
    readTouch(mytouchsensor_3);
    readRotation(mymotor_B);
    while(readTouch(mytouchsensor_3) == 0)
        readTouch(mytouchsensor_3);
        if (readTouch(mytouchsensor_3)==0)%(B_Touchvalue ~= 0)
            mymotor_B.Speed = -25;
        else
            mymotor_B.Speed = 0;
            break;
        end
    end
    pause(1);
    resetRotation(mymotor_B);
    start(mymotor_C);
    readTouch(mytouchsensor_1);
    readRotation(mymotor_C);
    while(readTouch(mytouchsensor_1) == 0)
        readTouch(mytouchsensor_1);
        if (readTouch(mytouchsensor_1) == 0)
            mymotor_C.Speed = 25;
        else
            mymotor_C.Speed = 0;
            
            break;
        end
    end
    pause(1);
    resetRotation(mymotor_C); 
end
%Function for gripper close:
function B = Gripper_Close(mymotor_A)
start(mymotor_A);
mymotor_A.Speed=-20;
pause(1.5);
mymotor_A.Speed=0;
stop(mymotor_A);
end


% 167 for B_Encrotation to avoid obstacle(7cm)
% 313 for B_Encrotation touch ground.