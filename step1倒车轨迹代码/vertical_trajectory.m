%%
clc
clear
close
%车辆参数   
L = 3.95;  
W = 1.97; 
l = 2.48;   
lf = 0.8;    
lr = 0.67;   
delta_f = 0.524;  
omiga_f = 0.524;   
Rmin = 4.3; 
h = 4;    
Lp = 7; 
Wp = 2.2;  

xM1 = 4.5;
yM1 = 0.8;

xM2 = 4.5;
yM2 = 4.5;

xM3 = 9.5;
yM3 = 9.5;     

DX32 = xM3-xM2;
DY32 = yM3-yM2;
DY21 = yM2-yM1;    
%首先定义各个变量
k = 1;
ya = [];yb = [];yc = [];yd = [];
xa = [];xb = [];xc = [];xd = [];
xmr = [];ym = [];ymr = [];
phi = [];d_ym = [];d2_ym = [];
tho = [];delta_fr = [];tmp = [];
for xm = 0:0.01:DY21
    xmr = [xmr;xM1];
    ymr = [ymr;yM1+xm];
    phi = [phi;pi/2];
    d_ym = [d_ym;inf];
    d2_ym = [d2_ym;0];
    tho = [tho;d2_ym(k)/((1+d_ym(k)^2)^(3/2))];
    delta_fr = [delta_fr;atan(l*(d2_ym(k))/((1+d_ym(k)^2)^(3/2)))];
  
    xa=[xa;xmr(k)-lr*cos(phi(k))+W/2*sin(phi(k))];
    xb=[xb;xmr(k)+(L-lr)*cos(phi(k))+W/2*sin(phi(k))];
    xc=[xc;xmr(k)+(L-lr)*cos(phi(k))-W/2*sin(phi(k))];
    xd=[xd;xmr(k)-lr*cos(phi(k))-W/2*sin(phi(k))];
    ya=[ya;ymr(k)-lr*sin(phi(k))-W/2*cos(phi(k))];
    yb=[yb;ymr(k)+(L-lr)*sin(phi(k))-W/2*cos(phi(k))];
    yc=[yc;ymr(k)+(L-lr)*sin(phi(k))+W/2*cos(phi(k))];
    yd=[yd;ymr(k)-lr*sin(phi(k))+W/2*cos(phi(k))];
    k = k+1;
end

x0 = 9.5;
y0 = 4.5;
R = 5;
for xm = 0:0.01:DX32
    xmr = [xmr;xM1+xm];
    ymr = [ymr;y0+(R^2-(xmr(k)-x0)^2)^(1/2)];
    phi = [phi;atan(-(xmr(k)-x0)/(ymr(k)-y0))];
    d_ym = [d_ym;-(xmr(k)-x0)/(ymr(k)-y0)];
    d2_ym = [d2_ym;(-1*(ymr(k)-y0)+d_ym(k)*(xmr(k)-x0))/(ymr(k)-y0)^2];
    tho = [tho;d2_ym(k)/((1+d_ym(k)^2)^(3/2))];
    delta_fr = [delta_fr;atan(l*(d2_ym(k))/((1+d_ym(k)^2)^(3/2)))];
    xa=[xa;xmr(k)-lr*cos(phi(k))+W/2*sin(phi(k))];
    xb=[xb;xmr(k)+(L-lr)*cos(phi(k))+W/2*sin(phi(k))];
    xc=[xc;xmr(k)+(L-lr)*cos(phi(k))-W/2*sin(phi(k))];
    xd=[xd;xmr(k)-lr*cos(phi(k))-W/2*sin(phi(k))];
    
    ya=[ya;ymr(k)-lr*sin(phi(k))-W/2*cos(phi(k))];
    yb=[yb;ymr(k)+(L-lr)*sin(phi(k))-W/2*cos(phi(k))];
    yc=[yc;ymr(k)+(L-lr)*sin(phi(k))+W/2*cos(phi(k))];
    yd=[yd;ymr(k,1)-lr*sin(phi(k,1))+W/2*cos(phi(k,1))];
    k = k+1;
end
omiga_fr = [0;diff(d2_ym)/0.01];
%%

figure
box off
set(0,'defaultfigurecolor','w')
hold on

color01 = [1,0.27,0];
color03 = [0,1,1];
color04 = [0.5,1,0];
color05 = [0.5,0.5,0.5];

plot(xa,ya,'color',color01,'linewidth',2)
plot(xb,yb,'b','linewidth',2)
plot(xc,yc,'color',color03,'linewidth',2)
plot(xd,yd,'color',color04,'linewidth',2)
plot(xmr,ymr,'k','linewidth',2)



for i = 1:20:length(xmr)
    Car=[xa(i),xb(i),xc(i),xd(i),xa(i);
        ya(i),yb(i),yc(i),yd(i),ya(i)];
    plot(Car(1,:),Car(2,:),'color',[0.75,0.75,0.75],'linewidth',1)
    hold on 
end
plot(xa,ya,'color',color01,'linewidth',2)
plot(xb,yb,'b','linewidth',2)
plot(xc,yc,'color',color03,'linewidth',2)
plot(xd,yd,'color',color04,'linewidth',2)
plot(xmr,ymr,'k','linewidth',2)

Car=[0,3.2,3.2,5.8,5.8,14;
       4,4,0,0,4,4];
plot(Car(1,:),Car(2,:),'-r','LineWidth',2)

axis([-1 14 -1 7]); 

set(gca,'LineWidth',2)

xlabel('X(m)');
ylabel('Y(m)'); 

annotation('arrow',[0.13 0.13],[0.06 0.99],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);
annotation('arrow',[0.1 0.95],[0.126 0.126],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);

text(9.5,9.5,'{\itM_1}','FontSize',20)
text(4.5,4.5,'{\itM_2}','FontSize',20)
text(4.5,0.8,'{\itM_3}','FontSize',20)
scatter(9.5,9.5,'k','filled')
scatter(4.5,4.5,'k','filled')
scatter(4.5,0.8,'k','filled')

legend({'a trajectory','b trajectory','c trajectory','d trajectory','axles trajectory'},'FontSize',12,'Location','best')

legend('boxoff')

% %%
% %路径曲率
% figure
% plot(xmr,tho,'k','LineWidth',2)
% 
% %上下限
% hold on
% plot([0,9.9],[1/Rmin,1/Rmin],'--r','LineWidth',2)
% plot([0,9.9],[-1/Rmin,-1/Rmin],'--r','LineWidth',2)
% 
% box off
% set(gca,'FontSize',40);
% %坐标轴范围
% axis([0 10 -0.3 0.3]); 
% %坐标轴粗细
% set(gca,'LineWidth',2)
% %轴的名称
% xlabel('X(m)');% x轴名称
% ylabel('路径曲率(m^{-1})'); 
% 
% %修改XY轴的刻度
% set(gca,'XTick',0:2:10)
% set(gca,'XTickLabel',{0:2:10})
% set(gca,'YTick',-0.3:0.1:0.3)
% set(gca,'YTickLabel',{-0.3:0.1:0.3})

% %%
% %等效前轮转角
% figure
% plot(xmr,delta_fr,'k','LineWidth',2)
% 
% %上下限
% hold on
% plot([0,9.9],[0.524,0.524],'--r','LineWidth',2)
% plot([0,9.9],[-0.524,-0.524],'--r','LineWidth',2)
% 
% box off
% set(gca,'FontSize',40);
% %坐标轴范围
% axis([0 10 -0.6 0.6]); 
% %坐标轴粗细
% set(gca,'LineWidth',2)
% %轴的名称
% xlabel('X(m)');% x轴名称
% ylabel('等效前轮转角(rad)'); 
% %修改XY轴的刻度
% set(gca,'XTick',0:2:10)
% set(gca,'XTickLabel',{0:2:10})
% set(gca,'YTick',-0.6:0.2:0.6)
% set(gca,'YTickLabel',{-0.6:0.2:0.6})


% %%
% %等效前轮转角转速
% figure
% plot(xmr,omiga_fr,'k','LineWidth',2)
% 
% %上下限
% hold on
% plot([0,9.9],[0.524,0.524],'--r','LineWidth',2)
% 
% box off
% set(gca,'FontSize',40);
% %坐标轴范围
% axis([0 10 -0.4 0.6]); 
% %坐标轴粗细
% set(gca,'LineWidth',2)
% %轴的名称
% xlabel('X(m)');% x轴名称
% ylabel('等效前轮转角转速(rad/s)'); 
% %修改XY轴的刻度
% set(gca,'XTick',0:2:10)
% set(gca,'XTickLabel',{0:2:10})
% set(gca,'YTick',-0.4:0.2:0.6)
% set(gca,'YTickLabel',{-0.4:0.2:0.6})