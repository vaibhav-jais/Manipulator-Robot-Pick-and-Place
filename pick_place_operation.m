%MECHATRONICS GROUP-3 PROJECT:

%MAIN FILE:
%Initialization of all the components:
%5myev3=legoev3('usb');

%FOR MOTOR A:
%Initializzation for Motor_A(Gripper):
mymotor_A = motor(myev3, 'A');

%FOR MOTOR B:
%Initialization for middle motor B:
mymotor_B = motor(myev3,'B');
%Touch sensor value for motor B:
%5mytouchsensor_3=touchSensor(myev3,3);


%FOR MOTOR C:
%Initialization for middle motor C:
mymotor_C=motor(myev3,'C');
%Touch sensor value for motor C:
%5mytouchsensor_1=touchSensor(myev3,1);


%Dimension of the links (all dimensions are in mm):
global l1 l2 l3 l4;
l1 = 50;
l2 = 95;
l3 = 185;
l4 = 110;

%Coordinates for position B(0,103,-70) %-70 beacause the reference
%coordinates is taken at the base motor:
x1 = 0;
y1 = 103;
z1 = -70;

z_upper=0; %position of z in order to avoid obstacles:

%Coordinates for position C(-103,0,-70) %-70 beacause the reference
%coordinates is taken at the base motor:
x2 = -103;
y2 = 0;
z2 = -70;

%Coordinates for position A(110,0,-40) %-70 beacause the reference
%coordinates is taken at the base motor:
x3 = 110;
y3 = 0;
z3 = -40;

%Error to control motor B while going down till the table:
E1 = -230;
%Error to control motor B while going up till it overcomes obstacle height:
E2 = 65;
%Error to control motor B while going up till the platform of position A:
E3 = -105;

%Function call;



%MOVING FROM HOMING TO B AND PICK AND PLACE FROM POSITION B TO C:
Desired_Motor_C_EncValue=theta_1(x1,y1); %To call theta 1 value
Control_C(Desired_Motor_C_EncValue,mymotor_C); %To control motor C to go to position B
Gripper_Open(mymotor_A);
Desired_Motor_B_EncValue=theta_2(z1); %To call theta 2 value
Control_B(E1,Desired_Motor_B_EncValue,mymotor_B); %To control motor B to go down
Gripper_Close(mymotor_A); %Closing the gripper to grab the ball
Desired_Motor_B_EncValue=theta_2(z_upper); %Theta2 for maintaining position avoiding obstacles
Control_B(E2,Desired_Motor_B_EncValue,mymotor_B); %Control of motor B For lifting ball
Desired_Motor_C_EncValue=theta_1(x2,y2); %Theta1 for position C
Control_C(Desired_Motor_C_EncValue,mymotor_C); %Control of motor C to go to position C
Desired_Motor_B_EncValue=theta_2(z2); %Theta2 for maintaining position avoiding obstacles
Control_B(E1,Desired_Motor_B_EncValue,mymotor_B); %To control motor B to place the ball at position C
Gripper_Open(mymotor_A); %To open the gripper to place ball at position C
pause(0.2);
Desired_Motor_B_EncValue=theta_2(z_upper); %Theta2 for maintaining position avoiding obstacles
Control_B(E2,Desired_Motor_B_EncValue,mymotor_B); %Control of motor B to avoid obstacle
pause(2);

%PICK AND PLACE FROM POSITION C TO A: 
Desired_Motor_B_EncValue=theta_2(z2); %To call theta 2 value
Control_B(E1,Desired_Motor_B_EncValue,mymotor_B); %To control motor B to go down
Gripper_Close(mymotor_A); %To close gripper to catch the ball
Desired_Motor_B_EncValue=theta_2(z_upper); %Theta2 for maintaining position avoiding obstacles
Control_B(E2,Desired_Motor_B_EncValue,mymotor_B); %Control of motor B For lifting ball
Desired_Motor_C_EncValue = theta_1(x3,y3); %Theta1 for position A
Control_C(Desired_Motor_C_EncValue,mymotor_C); %Control of motor C to go to position A
Desired_Motor_B_EncValue=theta_2(z3); %To call theta 2 value for position at A
Control_B(E3,Desired_Motor_B_EncValue,mymotor_B); %To control motor B to go down at platform A
pause(1);
Gripper_Open(mymotor_A); %To open the gripper to place the ball
Desired_Motor_B_EncValue=theta_2(z_upper); %Theta2 for maintaining position avoiding obstacles
Control_B(E2,Desired_Motor_B_EncValue,mymotor_B); %Control of motor B for going up
pause(1);

%PICK AND PLACE FROM A TO B:
Desired_Motor_B_EncValue=theta_2(z3); %To call theta 2 value
Control_B(E3,Desired_Motor_B_EncValue,mymotor_B); %Control of motor B to go to platform at A
Gripper_Close(mymotor_A); %Closing the gripper to grab the ball
Desired_Motor_B_EncValue=theta_2(z_upper); %Theta2 for maintaining position avoiding obstacles
Control_B(E2,Desired_Motor_B_EncValue,mymotor_B); %Control of motor B For lifting ball
Desired_Motor_C_EncValue=theta_1(x1,y1); %To call theta 1 value to go to position B
Control_C(Desired_Motor_C_EncValue,mymotor_C); %To control motor C to reach position B
Desired_Motor_B_EncValue=theta_2(z1); %To call theta 2 value to go down at position B
Control_B(E1,Desired_Motor_B_EncValue,mymotor_B); %Control of motor B to place the ball at position B
Gripper_Open(mymotor_A); %To open the gripper to place the ball at position B 



%Initialization for rotation angle of motor C and motor B: 
syms base_angle_rot elbow_angle_rot;

%Function for calculation of Desired motor C encoder value:
function Desired_Motor_C_EncValue=theta_1(x,y)
    % Calculation of base angle rotation
    if x < 0 && y >= 0
        base_angle_rot = atand(y/x) + 180;
    else 
        base_angle_rot = atand(y/x);
    end
    Desired_Motor_C_EncValue = base_angle_rot*3;   %270
end

%Function for calculation of Desired motor B encoder value:
function Desired_Motor_B_EncValue=theta_2(z)
    % calling global variable
    global l1 l2 l3 l4
    % Calculation of elbow angle rotation
    elbow_angle_rot = acosd(((l2/1.414)+l1-l4-z)/l3) - 45;
    Desired_Motor_B_EncValue = elbow_angle_rot*5;
end

%Controlling of motor C rotation:
function C = Control_C(Desired_Motor_C_EncValue,mymotor_C)
Control_motor_C = 0 ; 
% mechanical friction resist the encoder cabel tention 
    while(Control_motor_C ~= -48)
        C_Encrotation = readRotation(mymotor_C);
        Control_motor_C = Desired_Motor_C_EncValue + (C_Encrotation);   %270-300=-30
        if Control_motor_C > -48
            mymotor_C.Speed = -20;
            start(mymotor_C);
        elseif Control_motor_C > -40
            mymotor_C.Speed = -8;
            start(mymotor_C);
        elseif Control_motor_C < -48
            mymotor_C.Speed = 20;
            start(mymotor_C);
        elseif Control_motor_C < -50
            mymotor_C.Speed = 8;
            start(mymotor_C);
        elseif Control_motor_C == -48
            mymotor_C.Speed = 0;
        end
    end
end

 %Controlling of motor B rotation:
 function G = Control_B(E,Desired_Motor_B_EncValue,mymotor_B)
  Control_motor_B = 0;
    while(Control_motor_B ~= E) 
        B_Encrotation = readRotation(mymotor_B);
        Control_motor_B = Desired_Motor_B_EncValue - B_Encrotation;
        if Control_motor_B > E
            mymotor_B.Speed = 20;
        elseif Control_motor_B > E+8
            mymotor_B.Speed = 15;
        elseif Control_motor_B < E
            mymotor_B.Speed = -25;
        elseif Control_motor_B < E-8
            mymotor_B.Speed = -20;s
        elseif Control_motor_B == E
            mymotor_B.Speed = 0;
        end
    end
 end

%Function for gripper open:
function A = Gripper_Open(mymotor_A)
start(mymotor_A);
mymotor_A.Speed=30;
pause(0.2);
mymotor_A.Speed=0;
end

%Function for gripper close:
function B = Gripper_Close(mymotor_A)
start(mymotor_A);
mymotor_A.Speed = -25;
pause(1.5);
mymotor_A.Speed=0;
stop(mymotor_A);
end
 
 %END OF THE PROJECT
 %THANK YOU

