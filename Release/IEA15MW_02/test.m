copyfile('D:\WETI\4_Forschung\28_ACC2023\build\ROSCO\Release\discon.dll','ROSCO_v2d6.dll')
RunExample
R_FB          = ReadROSCOtextIntoStruct([SimulationName,'.RO.dbg']);
% figure
% plot(R_FB.Time, 	rad2deg(R_FB.Fl_PitCom))

% figure
% hold on
% plot(FB.Time,rad2deg(FB.PTCHLI));

% figure
% hold on
% plot(FB.Time,FB.NcIMURAys);



%%
omega = 0.213000;  
D= 1.000000; 

dt = 1/80;
LP2 = tf(omega^2,[1 2*D*omega omega^2]);
LP2d = c2d(LP2,dt,'Tustin');

FA_Acc = FB.NcIMURAys;
FA_Acc(1) = 0;

B =     LP2d.Numerator{:};
A =     LP2d.Denominator{:};
% FA_AccF = filter(B,A,FA_Acc);

n_t = length(FB.Time);
% FA_AccF(1)= 0;
% FA_AccF(2)= B(1)*FA_Acc(2);
% % FA_AccF = zeros(1,n_t);
% for i_t=3:n_t
%     FA_AccF(i_t) = B(1)*FA_Acc(i_t)+B(2)*FA_Acc(i_t-1)+B(3)*FA_Acc(i_t-2)-A(2)*FA_AccF(i_t-1)-A(3)*FA_AccF(i_t-2);
% end

NacIMU_FA_vel = cumsum(FA_Acc)*dt;
NacIMU_FA_vel_f = filter(B,A,NacIMU_FA_vel);

figure
hold on
plot(FB.Time,NacIMU_FA_vel_f);
plot(R_FB.Time, 	rad2deg(R_FB.Fl_PitCom))
% xlim([0 20])
 %%
% NacIMU_FA_vel = rad2deg(R_FB.Fl_PitCom);
% NacIMU_FA_vel_f = filter(B,A,NacIMU_FA_vel);
% 
% figure
% hold on
% % plot(FB.Time,NacIMU_FA_vel);
% plot(FB.Time,NacIMU_FA_vel_f);
% xlim([0 20])

