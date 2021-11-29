function plot
clc;
clear all;
close all;
data=importdata('./file1.txt'); 
exp_p=data(:,1);
LL=length(exp_p);
ST=0.8;
ADD=0.9;
ADDS=460/LL;
ED=ST+ADDS;
if ED>1
    ED=1;
end
data=importdata('./file1.txt'); 
exp_p=data(:,1);
now_p=data(:,2);

exp_r=data(:,3);
now_r=data(:,4);

exp_y=data(:,5);
now_y=data(:,6);

now_pr=data(:,7);
now_rr=data(:,8);
now_yr=data(:,9);

now_cog_xr=limit(data(:,10),-0.5,0.5);
now_cog_x=limit(data(:,11),-0.5,0.5);
now_cog_dx=limit(data(:,12),-0.5,0.5);

now_cog_zr=limit(data(:,13),-0.5,0.5);
now_cog_z=limit(data(:,14),-0.5,0.5);
now_cog_dz=limit(data(:,15),-0.5,0.5);

g_pit=limit(data(:,16),-0.5,0.5);
g_rol=limit(data(:,17),-0.5,0.5);

tar_spdx=limit(data(:,18),-0.5,0.5);
tar_spdz=limit(data(:,19),-0.5,0.5);

%   
data=importdata('./file2.txt'); %足底力
exp_fx0=data(:,1);
now_fx0=data(:,2);
exp_fz0=data(:,3);
now_fz0=data(:,4);

exp_fx1=data(:,5);
now_fx1=data(:,6);
exp_fz1=data(:,7);
now_fz1=data(:,8);

exp_fx2=data(:,9);
now_fx2=data(:,10);
exp_fz2=data(:,11);
now_fz2=data(:,12);

exp_fx3=data(:,13);
now_fx3=data(:,14);
exp_fz3=data(:,15);
now_fz3=data(:,16);

g_flag0=data(:,17);
touch_flag0=data(:,18);
g_flag1=data(:,19);
touch_flag1=data(:,20);
g_flag2=data(:,21);
touch_flag2=data(:,22);
g_flag3=data(:,23);
touch_flag3=data(:,24);

trig_state0=data(:,25);
trig_state1=data(:,26);
trig_state2=data(:,27);
trig_state3=data(:,28);

leg0_q0_exp=    data(:,29);
leg0_q1_exp=    data(:,30);
leg0_q0_now=    data(:,31);
leg0_q1_now=    data(:,32);
leg0_posx_h_exp=data(:,33);
leg0_posz_h_exp=data(:,34);
leg0_posx_h_now=data(:,35);
leg0_posz_h_now=data(:,36);

leg1_q0_exp=    data(:,37);
leg1_q1_exp=    data(:,38);
leg1_q0_now=    data(:,39);
leg1_q1_now=    data(:,40);
leg1_posx_h_exp=data(:,41);
leg1_posz_h_exp=data(:,42);
leg1_posx_h_now=data(:,43);
leg1_posz_h_now=data(:,44);

leg2_q0_exp=    data(:,45);
leg2_q1_exp=    data(:,46);
leg2_q0_now=    data(:,47);
leg2_q1_now=    data(:,48);
leg2_posx_h_exp=data(:,49);
leg2_posz_h_exp=data(:,50);
leg2_posx_h_now=data(:,51);
leg2_posz_h_now=data(:,52);

leg3_q0_exp=    data(:,53);
leg3_q1_exp=    data(:,54);
leg3_q0_now=    data(:,55);
leg3_q1_now=    data(:,56);
leg3_posx_h_exp=data(:,57);
leg3_posz_h_exp=data(:,58);
leg3_posx_h_now=data(:,59);
leg3_posz_h_now=data(:,60);


data=importdata('./file3.txt'); %足底力
temp0=data(:,1);
temp1=data(:,2);
temp2=data(:,3);
temp3=data(:,4);
temp4=data(:,5);
temp5=data(:,6);
temp6=data(:,7);
temp7=data(:,8);
temp8=data(:,9);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L=length(now_cog_dz);
P_ST=int16(ST*L);%选择显示区域
P_END=int16(ED*L);
g_flag_4all=data(:,60);
g_flag_0all=data(:,60);
for i=1:L
    if g_flag0(i)+g_flag1(i)+g_flag2(i)+g_flag3(i)>=3
         g_flag_4all(i)=g_flag0(i)+g_flag1(i)+g_flag2(i)+g_flag3(i);
    else
         g_flag_4all(i)=0;
    end
     if g_flag0(i)==0 &&g_flag1(i)==0 &&g_flag2(i)==0 &&g_flag3(i)==0
         g_flag_0all(i)=-2;
    else
         g_flag_0all(i)=0;
    end
end
figure('NumberTitle', 'off', 'Name', '姿态')
subplot(3,3,1);
plot(exp_p(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(now_p(P_ST:P_END),'-k');
grid on;
ylabel('Pitch');
subplot(3,3,4);
plot(exp_r(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(now_r(P_ST:P_END),'-k');
grid on;
ylabel('Roll');
subplot(3,3,7);
plot(exp_y(P_ST:P_END),'-.r');
hold on;
plot(now_y(P_ST:P_END));
grid on;
ylabel('Yaw');
subplot(3,3,2);
plot(now_pr(P_ST:P_END));
hold on;
grid on;
ylabel('Dpitch');
subplot(3,3,5);
plot(now_rr(P_ST:P_END));
hold on;
grid on;
ylabel('Droll');
subplot(3,3,8);
plot(now_yr(P_ST:P_END));grid on;
hold on;
ylabel('Dyaw');
subplot(3,3,3);
plot(g_pit(P_ST:P_END));grid on;
hold on;
ylabel('Gpitch');
subplot(3,3,6);
plot(g_rol(P_ST:P_END));grid on;
hold on;
ylabel('Groll');
subplot(3,3,9);
plot(g_flag0(P_ST:P_END),':k','LineWidth',2);
hold on;
plot(g_flag1(P_ST:P_END),'-r','LineWidth',1);
hold on;
plot(g_flag2(P_ST:P_END),'-b','LineWidth',1);
hold on;
plot(g_flag3(P_ST:P_END),':m','LineWidth',2);
hold on;
plot(g_flag_4all(P_ST:P_END),'-r','LineWidth',2);
hold on;
plot(g_flag_0all(P_ST:P_END),':b','LineWidth',2);
hold on;
ylabel('G_Flag');
grid on;

figure('NumberTitle', 'off', 'Name', '质心')
subplot(2,2,1);
plot(now_cog_xr(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(now_cog_x(P_ST:P_END),'-k');
grid on;
ylabel('X方向Cog');
subplot(2,2,2);
plot(trig_state0(P_ST:P_END)/200+now_cog_z(P_ST),'LineWidth',1);
hold on;
plot(trig_state1(P_ST:P_END)/200+now_cog_z(P_ST),'LineWidth',1);
hold on;
plot(trig_state2(P_ST:P_END)/200+now_cog_z(P_ST),'-.r','LineWidth',1);
hold on;
plot(trig_state3(P_ST:P_END)/200+now_cog_z(P_ST),'-.b','LineWidth',1);
hold on;
plot(now_cog_zr(P_ST:P_END),'-.r','LineWidth',1.5);
hold on;
plot(now_cog_z(P_ST:P_END),'-k','LineWidth',1.5);
grid on;
ylabel('Z方向Cog');
subplot(2,2,3);
plot(tar_spdx(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(now_cog_dx(P_ST:P_END),'-k');
ylim([-0.5,0.5]);
hold on;
grid on;
ylabel('X方向速度');
subplot(2,2,4);
plot(now_cog_dz(P_ST:P_END),'-k');
hold on;
plot(trig_state0(P_ST:P_END)/10+now_cog_dz(P_ST),'LineWidth',2);
hold on;
plot(trig_state1(P_ST:P_END)/10+now_cog_dz(P_ST),'LineWidth',2);
ylim([-0.5,0.5]);
hold on;
ylabel('Z方向速度');
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('NumberTitle', 'off', 'Name', '右前0腿足底力')
subplot(4,2,1);
plot(g_flag0(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(touch_flag0(P_ST:P_END),'-.r');
grid on;
ylabel('着地状态');
subplot(4,2,3);
plot(now_fx0(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fx0(P_ST:P_END),'-.r','LineWidth',2);
grid on;
ylabel('X方向力');
%ylim([-50,50]);
subplot(4,2,5);
plot(now_fz0(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fz0(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state0(P_ST:P_END)*20,'LineWidth',1);
grid on;
ylabel('Z方向力');
ylim([-50,90]);
subplot(4,2,7);
plot(trig_state0(P_ST:P_END),'LineWidth',2);
ylim([0,4]);
ylabel('摆动相序');
grid on;
subplot(4,2,2);
plot(leg0_q0_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_q0_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q0');
grid on;
subplot(4,2,4);
plot(leg0_q1_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_q1_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q1');
grid on;
subplot(4,2,6);
plot(leg0_posx_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_posx_h_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Hip X');
grid on;
subplot(4,2,8);
plot(leg0_posz_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg0_posz_h_exp(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state0(P_ST:P_END)/100+leg0_posz_h_now(P_ST),'LineWidth',2);
ylabel('Hip Z');
%ylim([-0.2,-0.1]);
grid on;



figure('NumberTitle', 'off', 'Name', '右后1腿足底力')
subplot(4,2,1);
plot(g_flag1(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(touch_flag1(P_ST:P_END),'-.r');
grid on;
ylabel('着地状态');
subplot(4,2,3);
plot(now_fx1(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fx1(P_ST:P_END),'-.r','LineWidth',2);
grid on;
ylabel('X方向力');
%ylim([-50,50]);
subplot(4,2,5);
plot(now_fz1(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fz1(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state1(P_ST:P_END)*20,'LineWidth',1);
grid on;
ylabel('Z方向力');
ylim([-50,90]);
subplot(4,2,7);
plot(trig_state1(P_ST:P_END),'LineWidth',2);
ylim([0,4]);
ylabel('摆动相序');
grid on;
subplot(4,2,2);
plot(leg1_q0_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg1_q0_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q0');
grid on;
subplot(4,2,4);
plot(leg1_q1_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg1_q1_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q1');
grid on;
subplot(4,2,6);
plot(leg1_posx_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg1_posx_h_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Hip X');
grid on;
subplot(4,2,8);
plot(leg1_posz_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg1_posz_h_exp(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state1(P_ST:P_END)/100+leg1_posz_h_now(P_ST),'LineWidth',2);
ylabel('Hip Z');
%ylim([-0.2,-0.1]);
grid on;

figure('NumberTitle', 'off', 'Name', '左前2足底力')
subplot(4,2,1);
plot(g_flag2(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(touch_flag2(P_ST:P_END),'-.r');
grid on;
ylabel('着地状态');
subplot(4,2,3);
plot(now_fx2(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fx2(P_ST:P_END),'-.r','LineWidth',2);
grid on;
ylabel('X方向力');
%ylim([-50,50]);
subplot(4,2,5);
plot(now_fz2(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fz2(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state2(P_ST:P_END)*20,'LineWidth',1);
grid on;
ylabel('Z方向力');
ylim([-50,90]);
subplot(4,2,7);
plot(trig_state2(P_ST:P_END),'LineWidth',2);
ylim([0,4]);
ylabel('摆动相序');
grid on;
subplot(4,2,2);
plot(leg2_q0_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg2_q0_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q0');
grid on;
subplot(4,2,4);
plot(leg2_q1_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg2_q1_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q1');
grid on;
subplot(4,2,6);
plot(leg2_posx_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg2_posx_h_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Hip X');
grid on;
subplot(4,2,8);
plot(leg2_posz_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg2_posz_h_exp(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state2(P_ST:P_END)/100+leg2_posz_h_now(P_ST),'LineWidth',2);
ylabel('Hip Z');
%ylim([-0.2,-0.1]);
grid on;
% 

figure('NumberTitle', 'off', 'Name', '左后3足底力')
subplot(4,2,1);
plot(g_flag3(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(touch_flag3(P_ST:P_END),'-.r');
grid on;
ylabel('着地状态');
subplot(4,2,3);
plot(now_fx3(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fx3(P_ST:P_END),'-.r','LineWidth',2);
grid on;
ylabel('X方向力');
%ylim([-50,50]);
subplot(4,2,5);
plot(now_fz3(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(exp_fz3(P_ST:P_END),'-.r','LineWidth',2);
grid on;
hold on;
plot(trig_state3(P_ST:P_END)*20,'LineWidth',1);
ylabel('Z方向力');
ylim([-50,90]);
subplot(4,2,7);
plot(trig_state3(P_ST:P_END),'-r','LineWidth',2);
hold on;
plot(trig_state2(P_ST:P_END),':b','LineWidth',2);
ylim([0,4]);
ylabel('摆动相序');
grid on;
subplot(4,2,2);
plot(leg3_q0_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg3_q0_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q0');
grid on;
subplot(4,2,4);
plot(leg3_q1_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg3_q1_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Q1');
grid on;
subplot(4,2,6);
plot(leg3_posx_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg3_posx_h_exp(P_ST:P_END),'-.r','LineWidth',2);
ylabel('Hip X');
grid on;
subplot(4,2,8);
plot(leg3_posz_h_now(P_ST:P_END),'-k','LineWidth',2);
hold on;
plot(leg3_posz_h_exp(P_ST:P_END),'-.r','LineWidth',2);
hold on;
plot(trig_state3(P_ST:P_END)/100+leg3_posz_h_now(P_ST),'LineWidth',2);
ylabel('Hip Z');
%ylim([-0.2,-0.1]);
grid on;



figure('NumberTitle', 'off', 'Name', 'TEMP')
subplot(2,2,1);
plot(temp0(P_ST:P_END),'-k','LineWidth',1.5);
hold on;
plot(temp1(P_ST:P_END),'-.r','LineWidth',1.5);
hold on;
plot(temp2(P_ST:P_END),':b','LineWidth',1.5);
grid on;
subplot(2,2,2);
plot(temp3(P_ST:P_END),'-k','LineWidth',1.5);
hold on;
plot(temp4(P_ST:P_END),'-.r','LineWidth',1.5);
hold on;
plot(temp5(P_ST:P_END),':b','LineWidth',1.5);
grid on;
subplot(2,2,3);
plot(temp6(P_ST:P_END),'-k','LineWidth',1.5);
hold on;
plot(temp7(P_ST:P_END),'-.r','LineWidth',1.5);
hold on;
plot(temp8(P_ST:P_END),':b','LineWidth',1.5);
grid on;
end

function out=limit(in,min,max)
out=in;
if in<min
    out=min;
end
if in>max
    out=max;
end
end
